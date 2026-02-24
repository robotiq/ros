/******************************************************************************************
//PollData V4.0 : Package for the new tactile sensor as per date of October 12th 2016
//Author: Jean-Philippe Roberge Ing, M.Sc.A.
//Creation Date: April 2nd 2015
//Affiliations: Laboratoire de commande et de robotique (École de technologie supérieure)
//
//Description:  PollData4.cpp - Source code of the ros package responsible for extracting
//              and publishing the static, dynamic and IMU data from tactile sensors. The
//              results are then published on /TactileSensor3/StaticData ,
//              /TactileSensor3/DynamicData or /TactileSensor3/DynamicAndIMUData
//              respectively. The static data are published in format
//              std_msgs::Int32MultiArray while dynamic and imu data are published in
//              format std_msgs::Float64.
//
//Synopsis:
rosrun tactilesensors PollData [-device PATH_TO_DEV]
//
//              Where [OPTIONS]:
//                  -device PATH_TO_DEV is the path to the device (Note that by default
//                  "/dev/ttyACM0" is considered to be the proper path.
//                  -data static | dynamic | dyna+imu' specifies if we want to extract
//                  "static" data, "dynamic" data or "dynamic" data along with the "imu"
//                  data. By default, this node will extract static data.
//
//Comments:     1) Magnetometers are currently unsupported, their values are thus set to
//              0 by default.
//              2) The First time one queries the imu(s), please be aware that
//              "BIASCalculationIterations" milliseconds of waiting time will be
//              required. "BIASCalculationIterations" is a global variable defined just
//              below.
//              3) During the biases calculations, it is mandatory that the imus remain
//              still, i.e.: the sensors sould not be moving / vibrating at all.
//
//
//Examples:     1)  rosrun tactilesensors PollData -sensor 1,2,5
//              -This will publish static data from sensors 1,2 and 5 to the topic
//              /TactileSensor/StaticData
//              2)  rosrun tactilesensors PollData -sensors 3:7 -data dynamic
//              -This will publish dynamic data from sensors 3, 4, 5, 6 and 7 to the
//              topic /TactileSensor/DynamicData
//
//______________________________________________________________________________________
//Version 1.0 : April 2nd 2015 - Initial release
//Version 1.1 : June 9th 2015  - Modified to include acquisition of multiple sensors at
//                               the same time --> e.g. by adding option -sensor 1:10
//                               on the command line
//
//Version 2.0 : --- n/a
//
//Version 3.0 : December 8th 2015 - Modifications to comply with the new sensor hardware
//                                  structure.
//
//Version 3.1 : December 15th 2015 - Modifications to add IMU data acquisition and
//                                   processing.
//
//Version 3.2 : June 2nd 2016 - Modifications to add combined dynamic data acquisition
//                              and processing, to fix jitter issued in dynamic and imu
//                              signals and to fix euler angle calculations. Biases
//                              estimation for accelerometers and gyroscopes are now
//                              included since it is not done on the PSoC side anymore.
//
//Version 4.0 : October 12th 2016 - Major modifications to comply with the new
//                                  communication protocol and data format. The sensor,
//                                  which has almost the same hardware as in v3.X, has a
//                                  different PSoC program which includes new and improved
//                                  communication protocol and data format. As of today,
//                                  all the data is acquired at the same time without
//                                  causing any noise problem on the dynamic channel as
//                                  before.
******************************************************************************************/

#include <stdint.h>
#include "ros/ros.h"
#include "robotiq_tsf/TactileSensors.h"
#include "robotiq_tsf/Sensor.h"
#include "robotiq_tsf/Accelerometer.h"
#include "robotiq_tsf/Dynamic.h"
#include "robotiq_tsf/EulerAngle.h"
#include "robotiq_tsf/Gyroscope.h"
#include "robotiq_tsf/Magnetometer.h"
#include "robotiq_tsf/Quaternion.h"
#include "robotiq_tsf/StaticData.h"
#include "MadgwickAHRS.h" // Quaternion calculation for 1st IMU
#include "MadgwickAHRS2.h" // Quaternion calculation for 2nd IMU
#include <stdio.h>      // standard input / output functions
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <algorithm>
#include <tf/transform_datatypes.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>
#include <cmath>

//Using std namespace
using namespace std;

#define READ_DATA_PERIOD_MS 1
#define FINGER_COUNT 2
#define FINGER_STATIC_TACTILE_ROW 4
#define FINGER_STATIC_TACTILE_COL 7
#define FINGER_STATIC_TACTILE_COUNT (FINGER_STATIC_TACTILE_ROW * FINGER_STATIC_TACTILE_COL)
#define FINGER_DYNAMIC_TACTILE_COUNT 1

//Global Variables:
bool IMUCalibrationMode = false;
bool StopSensorDataAcquisition=false;
bool ModeHasChanged=true;
//struct timespec th, stop;
double EllapsedMicroSeconds;
//IMU Biases:
const int BIASCalculationIterations=5000; // Number of samples we want to collect to compute IMU biases
int BIASCalculationIterator=0;
float norm_bias1=0,norm_bias2=0;
float ax1_bias=0,ay1_bias=0,az1_bias=0,ax2_bias=0,ay2_bias=0,az2_bias=0;
float gx1_bias=0,gy1_bias=0,gz1_bias=0,gx2_bias=0,gy2_bias=0,gz2_bias=0;
float mx1_bias=0,my1_bias=0,mz1_bias=0,mx2_bias=0,my2_bias=0,mz2_bias=0; // For future implementation (magnetometers are not currently acquired)

enum UsbPacketSpecial
{
    USB_PACKET_START_BYTE = 0x9A
};

enum UsbCommands
{
    USB_COMMAND_READ_SENSORS = 0x61,
    USB_COMMAND_AUTOSEND_SENSORS = 0x58,
    USB_COMMAND_ENTER_BOOTLOADER = 0xE2
};

// Sensor types occupy the higher 4 bits, the 2 bits lower than that identify finger, and the lower 2 bits is used as an index.
enum UsbSensorType
{
    USB_SENSOR_TYPE_STATIC_TACTILE = 0x10,
    USB_SENSOR_TYPE_DYNAMIC_TACTILE = 0x20,
    USB_SENSOR_TYPE_ACCELEROMETER = 0x30,
    USB_SENSOR_TYPE_GYROSCOPE = 0x40,
    USB_SENSOR_TYPE_MAGNETOMETER = 0x50,
    USB_SENSOR_TYPE_TEMPERATURE = 0x60
};

struct UsbPacket
{
    uint8_t start_byte;
    uint8_t crc8;           // over command, data_length and data
    uint8_t command;        // 4 bits of flag (MSB) and 4 bits of command (LSB)
    uint8_t data_length;
    uint8_t data[60];
};

struct FingerData
{
    uint16_t staticTactile[FINGER_STATIC_TACTILE_COUNT];
    int16_t dynamicTactile[FINGER_DYNAMIC_TACTILE_COUNT];
    int16_t accelerometer[3];
    int16_t gyroscope[3];
    int16_t magnetometer[3];
    int16_t temperature;
};

struct Fingers
{
    int64_t timestamp;
    FingerData finger[FINGER_COUNT];
};

//Function prototypes:
bool cmdOptionExists(char** begin, char** end, const string& option);
char* getCmdOption(char ** begin, char ** end, const string & option);
bool OpenAndConfigurePort(int *USB, char const *TheDevice);
static void usbSend(int *USB, UsbPacket *packet);
static uint8_t calcCrc8(uint8_t *data, size_t len);
static bool usbReadByte(UsbPacket *packet, unsigned int *readSoFar, uint8_t d);
static bool parseSensors(UsbPacket *packet, Fingers *fingers, robotiq_tsf::Sensor *SensorsData);
static inline uint16_t parseBigEndian2(uint8_t *data);
static uint8_t extractUint16(uint16_t *to, uint16_t toCount, uint8_t *data, unsigned int size);


//Callbacks:
bool TactileSensorServiceCallback(robotiq_tsf::TactileSensors::Request  &req, robotiq_tsf::TactileSensors::Response &res)
{
    ROS_INFO("The Tactile Sensors Service has received a request: [%s]",req.Request.data());
    ModeHasChanged=true;
    if(strcmp(req.Request.data(),"start")==0 || strcmp(req.Request.data(),"Start")==0)
    {
        StopSensorDataAcquisition=false;
        res.Response=true;
        return 1;
    }
    else if(strcmp(req.Request.data(),"stop")==0 || strcmp(req.Request.data(),"Stop")==0)
    {
        StopSensorDataAcquisition=true;
        res.Response=true;
        return 1;
    }
    else
    {
        ROS_WARN("The Tactile Sensor service has received an invalid request.");
        res.Response=false;
        return 0;
    }
    return true;
}

//Main
int main(int argc, char **argv)
{
    //Variable declarations:
    ros::init(argc, argv, "PollData");
    ros::NodeHandle n;
    ros::ServiceServer TactileSensorService=n.advertiseService("Tactile_Sensors_Service", TactileSensorServiceCallback);
    robotiq_tsf::Sensor SensorsData;
    robotiq_tsf::Quaternion TheQuaternions;
    ros::Publisher StaticData_pub = n.advertise<robotiq_tsf::StaticData>("TactileSensor/StaticData", 1000);
    ros::Publisher Dynamic_pub = n.advertise<robotiq_tsf::Dynamic>("TactileSensor/Dynamic", 1000);
    ros::Publisher Accelerometer_pub = n.advertise<robotiq_tsf::Accelerometer>("TactileSensor/Accelerometer",1000);
    ros::Publisher EulerAngle_pub = n.advertise<robotiq_tsf::EulerAngle>("TactileSensor/EulerAngle",1000);
    ros::Publisher Gyroscope_pub = n.advertise<robotiq_tsf::Gyroscope>("TactileSensor/Gyroscope",1000);
    ros::Publisher Magnetometer_pub = n.advertise<robotiq_tsf::Magnetometer>("TactileSensor/Magnetometer",1000);
    // I decided not to publish the quaternions for now (since I'm not sure it would be useful for anyone...:
    //    ros::Publisher Quaternion = n.advertise<robotiq_tsf::Quaternion>("TactileSensor4/Quaternion",1000);

    int USB, n_read;
    char const * TheDevice = "/dev/ttyACM0"; // By default, the device descriptor is set to "/dev/ttyACM0", rq_tsf85_0 not working as expected
    UsbPacket send, recv;
    unsigned int recvSoFar=0;
    std::vector<char> receiveBuffer(4096);
    tf::Quaternion q_1, q_2;
    tf::Matrix3x3 m1, m2;
    float ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2;
    const float aRes = 2.0/32768.0; // Accelerometers MPU9250 set resolution
    const float gRes = 250.0/32768.0; //Gyroscope set resolution (250 Degrees-Per-Second)


    if(cmdOptionExists(argv, argv+argc, "-device"))
    {
        char * filename = getCmdOption(argv, argv + argc, "-device");
        if (filename)
        {
            TheDevice=filename;
        }
    }

    if(!OpenAndConfigurePort(&USB,TheDevice)) return 1;

    // Gathered data
    Fingers fingers = {0};

    send.command=USB_COMMAND_AUTOSEND_SENSORS;
    send.data_length = 1;
    send.data[0] = READ_DATA_PERIOD_MS;
    usbSend(&USB, &send);

    //Now we enter the ros loop where we will acquire and publish data
    while (ros::ok())
    {

        //Read the answer, i.e. the bytes received from the sensor:
        n_read = read(USB, receiveBuffer.data(), receiveBuffer.size());

        //   Parse packets and store sensor values
        for (int64_t i = 0; i < n_read; ++i)
        {
            if (usbReadByte(&recv, &recvSoFar, receiveBuffer[i]))
            {
                bool newSetOfData = parseSensors(&recv, &fingers, &SensorsData);

                // Many messages can arrive in the same millisecond, so let the data accumulate and store it only when a whole set is complete
                if (newSetOfData)
                {
                    // We copy the dynamic values for both sensors:
                    SensorsData.dynamic.data[0].value=fingers.finger[0].dynamicTactile[0]; 
                    SensorsData.dynamic.data[1].value=fingers.finger[1].dynamicTactile[0];

                    // Then we copy static values for both sensors:
                    memcpy(&SensorsData.staticdata.taxels[0].values,&fingers.finger[0].staticTactile,sizeof(fingers.finger[0].staticTactile));
                    memcpy(&SensorsData.staticdata.taxels[1].values,&fingers.finger[1].staticTactile,sizeof(fingers.finger[1].staticTactile));

                    // Then we copy accelerometer values:
                    memcpy(&SensorsData.accelerometer.data[0].values,&fingers.finger[0].accelerometer,sizeof(fingers.finger[0].accelerometer));
                    memcpy(&SensorsData.accelerometer.data[1].values,&fingers.finger[1].accelerometer,sizeof(fingers.finger[1].accelerometer));

                    // Then we copy gyroscope values:
                    memcpy(&SensorsData.gyroscope.data[0].values,&fingers.finger[0].gyroscope,sizeof(fingers.finger[0].gyroscope));
                    memcpy(&SensorsData.gyroscope.data[1].values,&fingers.finger[1].gyroscope,sizeof(fingers.finger[1].gyroscope));

                    // Then we copy magnetometer values:
                    memcpy(&SensorsData.magnetometer.data[0].values,&fingers.finger[0].magnetometer,sizeof(fingers.finger[0].magnetometer));
                    memcpy(&SensorsData.magnetometer.data[1].values,&fingers.finger[1].magnetometer,sizeof(fingers.finger[1].magnetometer));


                    // Euler angles (and quaternions) computation:
                    if(BIASCalculationIterator>BIASCalculationIterations)
                    {
                        //Write the accel and gyro data to the topic Struct with the computed sensor biases
                        ax1=fingers.finger[0].accelerometer[0]*aRes-ax1_bias;
                        ay1=fingers.finger[0].accelerometer[1]*aRes-ay1_bias;
                        az1=fingers.finger[0].accelerometer[2]*aRes-az1_bias;
                        ax2=fingers.finger[1].accelerometer[0]*aRes-ax2_bias;
                        ay2=fingers.finger[1].accelerometer[1]*aRes-ay2_bias;
                        az2=fingers.finger[1].accelerometer[2]*aRes-az2_bias;
                        gx1=fingers.finger[0].gyroscope[0]*gRes-gx1_bias;
                        gy1=fingers.finger[0].gyroscope[1]*gRes-gy1_bias;
                        gz1=fingers.finger[0].gyroscope[2]*gRes-gz1_bias;
                        gx2=fingers.finger[1].gyroscope[0]*gRes-gx2_bias;
                        gy2=fingers.finger[1].gyroscope[1]*gRes-gy2_bias;
                        gz2=fingers.finger[1].gyroscope[2]*gRes-gz2_bias;

                        MadgwickAHRSupdateIMU(gx1*M_PI/180,gy1*M_PI/180,gz1*M_PI/180,ax1,ay1,az1); // 6-axis IMU
                        MadgwickAHRSupdateIMU2(gx2*M_PI/180,gy2*M_PI/180,gz2*M_PI/180,ax2,ay2,az2); // 6-axis IMU

                        TheQuaternions.data[0].values[0]=q0;
                        TheQuaternions.data[0].values[1]=q1;
                        TheQuaternions.data[0].values[2]=q2;
                        TheQuaternions.data[0].values[3]=q3;
                        TheQuaternions.data[1].values[0]=q0new;
                        TheQuaternions.data[1].values[1]=q1new;
                        TheQuaternions.data[1].values[2]=q2new;
                        TheQuaternions.data[1].values[3]=q3new;

                        SensorsData.eulerangle.data[0].values[0]=atan2(2.0f*(q0*q1+q2*q3),q0*q0-q1*q1-q2*q2+q3*q3)*180/M_PI;
                        SensorsData.eulerangle.data[0].values[1]=-asin(2.0f*(q1*q3-q0*q2))*180/M_PI;
                        SensorsData.eulerangle.data[0].values[2]=atan2(2.0f*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3)*180/M_PI;

                        SensorsData.eulerangle.data[1].values[0]=atan2(2.0f*(q0new*q1new+q2new*q3new),q0new*q0new-q1new*q1new-q2new*q2new+q3new*q3new)*180/M_PI;
                        SensorsData.eulerangle.data[1].values[1]=-asin(2.0f*(q1new*q3new-q0new*q2new))*180/M_PI;
                        SensorsData.eulerangle.data[1].values[2]=atan2(2.0f*(q1new*q2new+q0new*q3new),q0new*q0new+q1new*q1new-q2new*q2new-q3new*q3new)*180/M_PI;
                    }
                    else if (BIASCalculationIterator==BIASCalculationIterations)
                    {
                        gx1_bias/=BIASCalculationIterations;
                        gy1_bias/=BIASCalculationIterations;
                        gz1_bias/=BIASCalculationIterations;
                        gx2_bias/=BIASCalculationIterations;
                        gy2_bias/=BIASCalculationIterations;
                        gz2_bias/=BIASCalculationIterations;
                        ax1_bias/=BIASCalculationIterations;
                        ay1_bias/=BIASCalculationIterations;
                        az1_bias/=BIASCalculationIterations;
                        ax2_bias/=BIASCalculationIterations;
                        ay2_bias/=BIASCalculationIterations;
                        az2_bias/=BIASCalculationIterations;

                        norm_bias1=sqrtf(pow(ax1_bias,2)+pow(ay1_bias,2)+pow(az1_bias,2))-1;
                        norm_bias2=sqrtf(pow(ax2_bias,2)+pow(ay2_bias,2)+pow(az2_bias,2))-1;

                        ax1_bias*=norm_bias1/(ax1_bias+ay1_bias+az1_bias);
                        ay1_bias*=norm_bias1/(ax1_bias+ay1_bias+az1_bias);
                        az1_bias*=norm_bias1/(ax1_bias+ay1_bias+az1_bias);
                        ax2_bias*=norm_bias2/(ax2_bias+ay2_bias+az2_bias);
                        ay2_bias*=norm_bias2/(ax2_bias+ay2_bias+az2_bias);
                        az2_bias*=norm_bias2/(ax2_bias+ay2_bias+az2_bias);
                        BIASCalculationIterator++;
                    }
                    else if (BIASCalculationIterator<BIASCalculationIterations)
                    {
                        gx1_bias+=fingers.finger[0].gyroscope[0]*gRes;gy1_bias+=fingers.finger[0].gyroscope[1]*gRes;gz1_bias+=fingers.finger[0].gyroscope[2]*gRes;
                        gx2_bias+=fingers.finger[1].gyroscope[0]*gRes;gy2_bias+=fingers.finger[1].gyroscope[1]*gRes;gz2_bias+=fingers.finger[1].gyroscope[2]*gRes;
                        ax1_bias+=fingers.finger[0].accelerometer[0]*aRes;ay1_bias+=fingers.finger[0].accelerometer[1]*aRes;az1_bias+=fingers.finger[0].accelerometer[2]*aRes;
                        ax2_bias+=fingers.finger[1].accelerometer[0]*aRes;ay2_bias+=fingers.finger[1].accelerometer[1]*aRes;az2_bias+=fingers.finger[1].accelerometer[2]*aRes;
                        BIASCalculationIterator++;
                    }

                    if (!StopSensorDataAcquisition)
                    {
                        // We publish everything:
                        Dynamic_pub.publish(SensorsData.dynamic);
                        StaticData_pub.publish(SensorsData.staticdata);
                        Accelerometer_pub.publish(SensorsData.accelerometer);
                        Gyroscope_pub.publish(SensorsData.gyroscope);
                        Magnetometer_pub.publish(SensorsData.magnetometer);

                        // If we had the time to compute the IMU biases, then we can publish the Euler angles:
                        if (BIASCalculationIterator>BIASCalculationIterations)
                            EulerAngle_pub.publish(SensorsData.eulerangle);
                    }
                }
            }
        }

        ros::spinOnce();    //Refresh publishing buffers
    }

    // Stop auto-send message
    send.command = USB_COMMAND_AUTOSEND_SENSORS;
    send.data_length = 1;
    send.data[0] = 0;
    usbSend(&USB, &send);

    close(USB);  // close and free /dev/ttyACM0 peripheral
    return 0;
}

/*****************************************************************************************
//Function: cmdOptionExists
//
//Description:  This function check a string starting at **begin up to **end and tries
//              to find the string defined by the argument &option. If it finds it, then
//              it returns true, otherwise it will return false.
//
*****************************************************************************************/
bool cmdOptionExists(char** begin, char** end, const string& option)
{
    return find(begin, end, option) != end;
}

/****************************************************************************************
//Function: getCmdOption
//
//Description:  This function check a string starting at **begin up to **end and tries
//              to find the string defined by the argument &option. If it finds it, then
//              it returns a pointer pointing just after the string that was found.
//
****************************************************************************************/
char* getCmdOption(char ** begin, char ** end, const string & option)
{
    char ** itr = find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

/****************************************************************************************
//Function: OpenAndConfigurePort
//
//Description:  This function opens the /dev/ttyUSB0 port with read and write access
//              rights (that suppose that the user has done "sudo chmod 777 /dev/ttyUSB0"
//              before. If it succeeds, it returns true, otherwise it returns false.
//
//Arguments:    int *USB: A pointer thats points file descriptor attached to ttyUSB0
//                          device.
//              char const * TheDevice: The string of the device we want to open.
//                          (e.g. /dev/ttyACM0).
//              int DesiredVMIN: is an integer used to determine how many byte we need to
//                          receive before we consider the message/line has ended. If set
//                          to 0, read will be non-blocking. See Termios documentation
//                          for more details.
//              int DesiredVTIME: is an integer representing the number of deciseconds
//                          before we consider that a timeout has occurred. If set to 0,
//                          then we will do pure data polling and usually, in this latter
//                          case, a timeout loop still need to be coded manually. See the
//                          Termios documentation for more details.
****************************************************************************************/
bool OpenAndConfigurePort(int *USB, char const * TheDevice)
{
    // All written by Jean-Philippe Roberge on April 2015, reviewed by Jean-Philippe Roberge in June 2016
    /* Open File Descriptor */
    //            *USB = open( "/tmp/interceptty", O_RDWR| O_NOCTTY ); // For debugging purposes (JP)
    *USB = open(TheDevice, O_RDWR| O_NOCTTY );
    /* Error Handling */
    if ( (*USB) < 0 )
    {
        cout << "Error " << errno << " opening " << TheDevice << ": " << strerror (errno) << endl;
        return false;
    }

    /**** Configure Port ****/
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if ( tcgetattr ( (*USB), &tty ) != 0 ) {
        cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
        return false;
    }

    /* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B115200);
    cfsetispeed (&tty, (speed_t)B115200);

    /* Setting other Port Stuff */
    tty.c_cflag     &=  ~PARENB;            // Make 8n1
    tty.c_cflag     &=  ~CSTOPB;
    tty.c_cflag     &=  ~CSIZE;
    tty.c_lflag     &=  ~ICANON;            // Remove canonical mode
    tty.c_cflag     |=  CS8;
    tty.c_cflag     &=  ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN]   =  0;        // read doesn't block
    tty.c_cc[VTIME]  =  0;       // 0.0 seconds read timeout between each byte max
    tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

    /* Flush Port, then applies attributes */
    tcflush( (*USB), TCIFLUSH );
    if ( tcsetattr ( (*USB), TCSANOW, &tty ) != 0) {
        cout << "Error " << errno << " from tcsetattr" << endl;
        return false;
    }

    return true;
}

static void usbSend(int *USB, UsbPacket *packet)
{
    uint8_t *p = (uint8_t *)packet;
    int n_written;

    packet->start_byte = USB_PACKET_START_BYTE;
    packet->crc8 = calcCrc8(p + 2, packet->data_length + 2);

    n_written = write( (*USB), (char *)p, packet->data_length + 4 );
}

static uint8_t calcCrc8(uint8_t *data, size_t len)
{
    // TODO: calculate CRC8
    return data[-1];
}

static bool usbReadByte(UsbPacket *packet, unsigned int *readSoFar, uint8_t d)
{
    uint8_t *p = (uint8_t *)packet;

    // Make sure start byte is seen
    if (*readSoFar == 0 && d != USB_PACKET_START_BYTE)
        return false;

    // Buffer the byte (making sure not to overflow the packet)
    if (*readSoFar < 64)
        p[*readSoFar] = d;
    ++*readSoFar;

    // If length is read, stop when done
    if (*readSoFar > 3 && *readSoFar >= (unsigned)packet->data_length + 4)
    {
        *readSoFar = 0;

        // If CRC is ok, we have a new packet!  Return it.
        if (packet->crc8 == calcCrc8(p + 2, packet->data_length + 2))
            return true;

        // If CRC is not ok, find the next start byte and shift the packet back in hopes of getting back in sync
        for (unsigned int i = 1; i < (unsigned)packet->data_length + 4; ++i)
            if (p[i] == USB_PACKET_START_BYTE)
            {
                memmove(p, p + i, packet->data_length + 4 - i);
                *readSoFar = packet->data_length + 4 - i;
                break;
            }
    }

    return false;
}


static bool parseSensors(UsbPacket *packet, Fingers *fingers, robotiq_tsf::Sensor *SensorsData)
{
    bool sawDynamic = false;
    for (unsigned int i = 0; i < packet->data_length;)
    {
        uint8_t sensorType = packet->data[i] & 0xF0;
        uint8_t f= packet->data[i] >> 2 & 0x03;
        ++i;

        uint8_t *sensorData = packet->data + i;
        unsigned int sensorDataBytes = packet->data_length - i;

        switch (sensorType)
        {
        case USB_SENSOR_TYPE_DYNAMIC_TACTILE:
            i += extractUint16((uint16_t *)fingers->finger[f].dynamicTactile, FINGER_DYNAMIC_TACTILE_COUNT, sensorData, sensorDataBytes);
            sawDynamic = true;
            break;
        case USB_SENSOR_TYPE_STATIC_TACTILE:
            i += extractUint16(fingers->finger[f].staticTactile, FINGER_STATIC_TACTILE_COUNT, sensorData, sensorDataBytes);
            break;
        case USB_SENSOR_TYPE_ACCELEROMETER:
            i += extractUint16((uint16_t *)fingers->finger[f].accelerometer, 3, sensorData, sensorDataBytes);
            break;
        case USB_SENSOR_TYPE_GYROSCOPE:
            i += extractUint16((uint16_t *)fingers->finger[f].gyroscope, 3, sensorData, sensorDataBytes);
            break;
        case USB_SENSOR_TYPE_MAGNETOMETER:
            i += extractUint16((uint16_t *)fingers->finger[f].magnetometer, 3, sensorData, sensorDataBytes);
            break;
        case USB_SENSOR_TYPE_TEMPERATURE:
            i += extractUint16((uint16_t *)&fingers->finger[f].temperature, 1, sensorData, sensorDataBytes);
            break;
        default:
            // Unknown sensor, we can't continue parsing anything from here on
            return sawDynamic;
        }
    }

    /*
     * Return true every time dynamic data is read.  This is used to identify when a whole set of data has
     * arrived and needs to be processed.
     */
    return sawDynamic;
}

static inline uint16_t parseBigEndian2(uint8_t *data)
{
    return (uint16_t)data[0] << 8 | data[1];
}

static uint8_t extractUint16(uint16_t *to, uint16_t toCount, uint8_t *data, unsigned int size)
{
    unsigned int cur;

    // Extract 16-bit values.  If not enough data, extract as much data as available
    for (cur = 0; 2 * cur + 1 < size && cur < toCount; ++cur)
        to[cur] = parseBigEndian2(&data[2 * cur]);

    // Return number of bytes read
    return cur * 2;
}
