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
//Comments:     1) The First time one queries the imu(s), please be aware that
//              "BIASCalculationIterations" milliseconds of waiting time will be
//              required. "BIASCalculationIterations" is a global variable defined just
//              below.
//              2) During the biases calculations, it is mandatory that the imus remain
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

#include "rclcpp/rclcpp.hpp"
#include "robotiq_tsf/msg/accelerometer.hpp"
#include "robotiq_tsf/msg/dynamic.hpp"
#include "robotiq_tsf/msg/euler_angle.hpp"
#include "robotiq_tsf/msg/gyroscope.hpp"
#include "robotiq_tsf/msg/quaternion.hpp"
#include "robotiq_tsf/msg/sensor.hpp"
#include "robotiq_tsf/msg/static_data.hpp"
#include "robotiq_tsf/msg/timestamp.hpp"
#include "robotiq_tsf/srv/tactile_sensors.hpp"
#include "robotiq_tsf/MadgwickAHRS.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <glob.h>
#include <math.h>
#include <memory>
#include <stdio.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <strings.h>

// Using std namespace
using namespace std;

#define NODE_LOGGER (g_node ? g_node->get_logger() : rclcpp::get_logger("poll_data4"))

namespace msg = robotiq_tsf::msg;
namespace srv = robotiq_tsf::srv;

std::shared_ptr<rclcpp::Node> g_node;
rclcpp::Publisher<msg::StaticData>::SharedPtr g_static_pub;
rclcpp::Publisher<msg::Dynamic>::SharedPtr g_dynamic_pub;
rclcpp::Publisher<msg::Accelerometer>::SharedPtr g_accel_pub;
rclcpp::Publisher<msg::Gyroscope>::SharedPtr g_gyro_pub;
rclcpp::Publisher<msg::EulerAngle>::SharedPtr g_euler_pub;
rclcpp::Publisher<msg::Quaternion>::SharedPtr g_quat_pub;
rclcpp::Publisher<msg::Timestamp>::SharedPtr g_timestamp_pub;
rclcpp::Service<srv::TactileSensors>::SharedPtr g_service;

#define READ_DATA_PERIOD_MS 1
#define FINGER_COUNT 2
#define FINGER_STATIC_TACTILE_ROW 7
#define FINGER_STATIC_TACTILE_COL 4
#define FINGER_STATIC_TACTILE_COUNT (FINGER_STATIC_TACTILE_ROW * FINGER_STATIC_TACTILE_COL)
#define FINGER_DYNAMIC_TACTILE_COUNT 1

// Global Variables:
[[maybe_unused]] bool IMUCalibrationMode = false;
bool StopSensorDataAcquisition = false;
[[maybe_unused]] bool ModeHasChanged = true;
// struct timespec th, stop;
double EllapsedMicroSeconds;
// IMU Biases:
const int BIASCalculationIterations = 5000;  // Number of samples we want to collect to compute IMU biases
int BIASCalculationIterator = 0;
float ax1_bias = 0, ay1_bias = 0, az1_bias = 0, ax2_bias = 0, ay2_bias = 0, az2_bias = 0;
float gx1_bias = 0, gy1_bias = 0, gz1_bias = 0, gx2_bias = 0, gy2_bias = 0, gz2_bias = 0;

// AHRS filter state — one instance per finger, owned by the IO thread.
MadgwickFilter g_filter[FINGER_COUNT];

// Per-finger MCU timestamp (milliseconds, as exposed by the firmware) of the
// last sample we used to integrate the filter. 0 means "not yet seeded after
// calibration".
uint64_t g_last_ts_ms[FINGER_COUNT] = { 0, 0 };

// Tunables, populated from ROS parameters in main().
struct AhrsConfig
{
  float beta = 0.041f;
  float accel_gate_lo = 0.85f;
  float accel_gate_hi = 1.15f;
  float bias_learn_rate = 0.0005f;    // EMA step toward residual gyro when still
  float still_gyro_eps_deg_s = 0.8f;  // |omega| below this counts as still
  float still_accel_eps_g = 0.05f;    // ||a| - 1g| below this counts as still
  float dt_clamp_lo = 1e-4f;          // 0.1 ms
  float dt_clamp_hi = 0.1f;           // 100 ms
};
AhrsConfig g_ahrs_cfg;

enum UsbPacketSpecial
{
  USB_PACKET_START_BYTE = 0x9A,
  USB_PACKET_HEADER_SIZE = 4,
  USB_PACKET_MAX_DATA_SIZE = 256,
  USB_PACKET_MAX_SIZE = USB_PACKET_HEADER_SIZE + USB_PACKET_MAX_DATA_SIZE,
};

enum UsbCommands
{
  USB_COMMAND_READ_SENSORS = 0x61,
  USB_COMMAND_AUTOSEND_SYNC_SENSORS = 0x58,
  USB_COMMAND_AUTOSEND_ASYNC_SENSORS = 0x59,
  USB_COMMAND_ENTER_BOOTLOADER = 0xE2,
  USB_COMMAND_GET_VERSION = 0xE3,
};

// Sensor types occupy the higher 4 bits, the 2 bits lower than that identify finger, and the lower 2 bits is used as an index.
enum UsbSensorType
{
  USB_SENSOR_TYPE_STATIC_TACTILE = 0x10,
  USB_SENSOR_TYPE_DYNAMIC_TACTILE = 0x20,
  USB_SENSOR_TYPE_ACCELEROMETER = 0x30,
  USB_SENSOR_TYPE_GYROSCOPE = 0x40,
  USB_SENSOR_TYPE_TEMPERATURE = 0x60,
  USB_SENSOR_TYPE_TIMESTAMP = 0x70,
};

struct UsbPacket
{
  uint8_t start_byte;
  uint8_t crc8;     // over command, data_length and data
  uint8_t command;  // 4 bits of flag (MSB) and 4 bits of command (LSB)
  uint8_t data_length;
  uint8_t data[USB_PACKET_MAX_DATA_SIZE];
};

struct FingerData
{
  bool newDataAvailable;

  uint16_t staticTactile[FINGER_STATIC_TACTILE_COUNT];
  int16_t dynamicTactile[FINGER_DYNAMIC_TACTILE_COUNT];
  int16_t accelerometer[3];
  int16_t gyroscope[3];
  int16_t temperature;
  uint64_t timestamp;

  uint16_t baseline[FINGER_STATIC_TACTILE_COUNT];
};

struct Fingers
{
  int64_t timestamp;
  FingerData finger[FINGER_COUNT];
};

// Function prototypes:
bool cmdOptionExists(char** begin, char** end, const string& option);
char* getCmdOption(char** begin, char** end, const string& option);
bool OpenAndConfigurePort(int* USB, char const* TheDevice);
static void usbSend(int* USB, UsbPacket* packet);
static uint8_t calcCrc8(uint8_t* data, size_t len);
static bool usbReadByte(UsbPacket* packet, unsigned int* readSoFar, uint8_t d);
static void parseSensors(UsbPacket* packet, Fingers* fingers);
static inline uint16_t parseBigEndian2(uint8_t* data);
static inline uint64_t parseBigEndian8(uint8_t* data);
static uint8_t extractUint16(uint16_t* to, uint16_t toCount, uint8_t* data, unsigned int size);
static unsigned int extractUint64(uint64_t* to, unsigned int toCount, uint8_t* data, unsigned int size);
static std::string trimWhitespace(const std::string& value);
static bool matchesKnownUsbString(const std::string& value);
static bool readSysfsEntry(const std::string& path, std::string& value);
static bool deviceMatchesKnownSensor(const std::string& device_path);
static std::string autoDetectSensorDevice();
static void forceReconnectSensor(int* USB);

constexpr const char* kDefaultDevice = "/dev/rq_tsf85_0";

// Callbacks:
void TactileSensorServiceCallback(const std::shared_ptr<srv::TactileSensors::Request> req,
                                  std::shared_ptr<srv::TactileSensors::Response> res)
{
  RCLCPP_INFO(NODE_LOGGER, "The Tactile Sensors Service has received a request: [%s]", req->request.c_str());
  ModeHasChanged = true;
  if (strcasecmp(req->request.c_str(), "start") == 0)
  {
    StopSensorDataAcquisition = false;
    res->response = true;
    return;
  }
  if (strcasecmp(req->request.c_str(), "stop") == 0)
  {
    StopSensorDataAcquisition = true;
    res->response = true;
    return;
  }

  RCLCPP_WARN(NODE_LOGGER, "The Tactile Sensor service has received an invalid request.");
  res->response = false;
}

// Main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  g_node = std::make_shared<rclcpp::Node>("poll_data4");
  g_node->declare_parameter<std::string>("device", kDefaultDevice);

  // AHRS tuning parameters — see AhrsConfig for what each one does.
  g_node->declare_parameter<double>("madgwick.beta", g_ahrs_cfg.beta);
  g_node->declare_parameter<double>("madgwick.accel_gate_lo", g_ahrs_cfg.accel_gate_lo);
  g_node->declare_parameter<double>("madgwick.accel_gate_hi", g_ahrs_cfg.accel_gate_hi);
  g_node->declare_parameter<double>("madgwick.bias_learn_rate", g_ahrs_cfg.bias_learn_rate);
  g_node->declare_parameter<double>("madgwick.still_gyro_eps_deg_s", g_ahrs_cfg.still_gyro_eps_deg_s);
  g_node->declare_parameter<double>("madgwick.still_accel_eps_g", g_ahrs_cfg.still_accel_eps_g);

  g_ahrs_cfg.beta = static_cast<float>(g_node->get_parameter("madgwick.beta").as_double());
  g_ahrs_cfg.accel_gate_lo = static_cast<float>(g_node->get_parameter("madgwick.accel_gate_lo").as_double());
  g_ahrs_cfg.accel_gate_hi = static_cast<float>(g_node->get_parameter("madgwick.accel_gate_hi").as_double());
  g_ahrs_cfg.bias_learn_rate = static_cast<float>(g_node->get_parameter("madgwick.bias_learn_rate").as_double());
  g_ahrs_cfg.still_gyro_eps_deg_s =
      static_cast<float>(g_node->get_parameter("madgwick.still_gyro_eps_deg_s").as_double());
  g_ahrs_cfg.still_accel_eps_g = static_cast<float>(g_node->get_parameter("madgwick.still_accel_eps_g").as_double());

  for (int f = 0; f < FINGER_COUNT; ++f)
  {
    g_filter[f].setBeta(g_ahrs_cfg.beta);
    g_filter[f].setAccelGate(g_ahrs_cfg.accel_gate_lo, g_ahrs_cfg.accel_gate_hi);
  }

  g_service = g_node->create_service<srv::TactileSensors>("tactile_sensors_service", &TactileSensorServiceCallback);

  auto sensor_qos = rclcpp::SensorDataQoS();
  auto orientation_qos = sensor_qos;
  g_static_pub = g_node->create_publisher<msg::StaticData>("TactileSensor/StaticData", sensor_qos);
  g_dynamic_pub = g_node->create_publisher<msg::Dynamic>("TactileSensor/Dynamic", sensor_qos);
  g_accel_pub = g_node->create_publisher<msg::Accelerometer>("TactileSensor/Accelerometer", sensor_qos);
  g_euler_pub = g_node->create_publisher<msg::EulerAngle>("TactileSensor/EulerAngle", orientation_qos);
  g_gyro_pub = g_node->create_publisher<msg::Gyroscope>("TactileSensor/Gyroscope", sensor_qos);
  g_quat_pub = g_node->create_publisher<msg::Quaternion>("TactileSensor/Quaternion", orientation_qos);
  g_timestamp_pub = g_node->create_publisher<msg::Timestamp>("TactileSensor/Timestamp", sensor_qos);

  std::string device = g_node->get_parameter("device").as_string();
  bool cliDeviceOverride = false;
  if (cmdOptionExists(argv, argv + argc, "-device"))
  {
    if (char* filename = getCmdOption(argv, argv + argc, "-device"))
    {
      device = filename;
      cliDeviceOverride = true;
      g_node->set_parameter(rclcpp::Parameter("device", device));
    }
  }

  const bool paramOverridesDefault = device != kDefaultDevice;
  if (!cliDeviceOverride && !paramOverridesDefault)
  {
    const std::string detected = autoDetectSensorDevice();
    if (!detected.empty())
    {
      RCLCPP_INFO(NODE_LOGGER, "Auto-detected tactile sensor on %s", detected.c_str());
      device = detected;
      g_node->set_parameter(rclcpp::Parameter("device", device));
    }
    else
    {
      RCLCPP_WARN(NODE_LOGGER, "Could not auto-detect tactile sensor device; falling back to %s", device.c_str());
    }
  }

  int USB;
  if (!OpenAndConfigurePort(&USB, device.c_str()))
  {
    RCLCPP_FATAL(NODE_LOGGER, "Failed to open device: %s", device.c_str());
    rclcpp::shutdown();
    return 1;
  }

  forceReconnectSensor(&USB);

  Fingers fingers = { 0 };
  msg::Sensor sensors_data;
  msg::Quaternion quaternions;
  UsbPacket send{}, recv{};
  unsigned int recvSoFar = 0;
  std::vector<char> receiveBuffer(4096);
  float ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2;
  const float aRes = 2.0F / 32768.0F;
  const float gRes = 250.0F / 32768.0F;

  send.command = USB_COMMAND_AUTOSEND_SYNC_SENSORS;
  send.data_length = 1;
  send.data[0] = READ_DATA_PERIOD_MS;
  usbSend(&USB, &send);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(g_node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  while (rclcpp::ok())
  {
    int n_read = read(USB, receiveBuffer.data(), receiveBuffer.size());
    if (n_read < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      RCLCPP_ERROR(NODE_LOGGER, "Read error on %s: %s", device.c_str(), strerror(errno));
      break;
    }

    for (int64_t i = 0; i < n_read; ++i)
    {
      if (usbReadByte(&recv, &recvSoFar, static_cast<uint8_t>(receiveBuffer[i])))
      {
        // The firmware no longer emits a single "end-of-frame" sensor type; instead
        // each finger advertises whether it received new data in this packet.
        if (recv.command != USB_COMMAND_AUTOSEND_SYNC_SENSORS && recv.command != USB_COMMAND_AUTOSEND_ASYNC_SENSORS &&
            recv.command != USB_COMMAND_READ_SENSORS)
        {
          continue;
        }

        parseSensors(&recv, &fingers);

        bool newSetOfData = false;
        for (int f = 0; f < FINGER_COUNT; ++f)
        {
          if (fingers.finger[f].newDataAvailable)
          {
            newSetOfData = true;
            break;
          }
        }

        if (newSetOfData)
        {
          sensors_data.dynamic.data[0].value = fingers.finger[0].dynamicTactile[0];
          sensors_data.dynamic.data[1].value = fingers.finger[1].dynamicTactile[0];

          std::memcpy(sensors_data.staticdata.taxels[0].values.data(), fingers.finger[0].staticTactile,
                      sizeof(fingers.finger[0].staticTactile));
          std::memcpy(sensors_data.staticdata.taxels[1].values.data(), fingers.finger[1].staticTactile,
                      sizeof(fingers.finger[1].staticTactile));

          std::memcpy(sensors_data.accelerometer.data[0].values.data(), fingers.finger[0].accelerometer,
                      sizeof(fingers.finger[0].accelerometer));
          std::memcpy(sensors_data.accelerometer.data[1].values.data(), fingers.finger[1].accelerometer,
                      sizeof(fingers.finger[1].accelerometer));

          std::memcpy(sensors_data.gyroscope.data[0].values.data(), fingers.finger[0].gyroscope,
                      sizeof(fingers.finger[0].gyroscope));
          std::memcpy(sensors_data.gyroscope.data[1].values.data(), fingers.finger[1].gyroscope,
                      sizeof(fingers.finger[1].gyroscope));

          sensors_data.timestamp.values[0] = fingers.finger[0].timestamp;
          sensors_data.timestamp.values[1] = fingers.finger[1].timestamp;

          if (BIASCalculationIterator > BIASCalculationIterations)
          {
            // Apply bias correction. ax/ay/az in g, gx/gy/gz in deg/s.
            ax1 = fingers.finger[0].accelerometer[0] * aRes - ax1_bias;
            ay1 = fingers.finger[0].accelerometer[1] * aRes - ay1_bias;
            az1 = fingers.finger[0].accelerometer[2] * aRes - az1_bias;
            ax2 = fingers.finger[1].accelerometer[0] * aRes - ax2_bias;
            ay2 = fingers.finger[1].accelerometer[1] * aRes - ay2_bias;
            az2 = fingers.finger[1].accelerometer[2] * aRes - az2_bias;
            gx1 = fingers.finger[0].gyroscope[0] * gRes - gx1_bias;
            gy1 = fingers.finger[0].gyroscope[1] * gRes - gy1_bias;
            gz1 = fingers.finger[0].gyroscope[2] * gRes - gz1_bias;
            gx2 = fingers.finger[1].gyroscope[0] * gRes - gx2_bias;
            gy2 = fingers.finger[1].gyroscope[1] * gRes - gy2_bias;
            gz2 = fingers.finger[1].gyroscope[2] * gRes - gz2_bias;

            constexpr float deg_to_rad = static_cast<float>(M_PI / 180.0f);

            const float ax_per_finger[FINGER_COUNT] = { ax1, ax2 };
            const float ay_per_finger[FINGER_COUNT] = { ay1, ay2 };
            const float az_per_finger[FINGER_COUNT] = { az1, az2 };
            const float gx_per_finger[FINGER_COUNT] = { gx1, gx2 };
            const float gy_per_finger[FINGER_COUNT] = { gy1, gy2 };
            const float gz_per_finger[FINGER_COUNT] = { gz1, gz2 };
            float* const gx_bias_per_finger[FINGER_COUNT] = { &gx1_bias, &gx2_bias };
            float* const gy_bias_per_finger[FINGER_COUNT] = { &gy1_bias, &gy2_bias };
            float* const gz_bias_per_finger[FINGER_COUNT] = { &gz1_bias, &gz2_bias };

            for (int f = 0; f < FINGER_COUNT; ++f)
            {
              const float ax = ax_per_finger[f];
              const float ay = ay_per_finger[f];
              const float az = az_per_finger[f];
              const float gx = gx_per_finger[f];
              const float gy = gy_per_finger[f];
              const float gz = gz_per_finger[f];

              // Measured dt from MCU per-finger timestamp (ms).
              const uint64_t ts = fingers.finger[f].timestamp;
              float dt = 0.0f;
              if (g_last_ts_ms[f] != 0 && ts > g_last_ts_ms[f])
              {
                dt = static_cast<float>(ts - g_last_ts_ms[f]) * 1e-3f;
                if (dt < g_ahrs_cfg.dt_clamp_lo)
                  dt = g_ahrs_cfg.dt_clamp_lo;
                if (dt > g_ahrs_cfg.dt_clamp_hi)
                  dt = g_ahrs_cfg.dt_clamp_hi;
              }
              g_last_ts_ms[f] = ts;

              if (dt > 0.0f)
              {
                g_filter[f].updateIMU(gx * deg_to_rad, gy * deg_to_rad, gz * deg_to_rad, ax, ay, az, dt);
              }

              // Online gyro-bias trim when stationary. The residual gx/gy/gz
              // (post-bias-subtraction) should be ~0 if the sensor really is
              // still and the bias is still right; if it's not, slowly drift
              // the stored bias toward the current reading.
              const float gyroMag = std::sqrt(gx * gx + gy * gy + gz * gz);
              const float accelNorm = std::sqrt(ax * ax + ay * ay + az * az);
              const bool still = (gyroMag < g_ahrs_cfg.still_gyro_eps_deg_s) &&
                                 (std::fabs(accelNorm - 1.0f) < g_ahrs_cfg.still_accel_eps_g);
              if (still)
              {
                const float alpha = g_ahrs_cfg.bias_learn_rate;
                *gx_bias_per_finger[f] += alpha * gx;
                *gy_bias_per_finger[f] += alpha * gy;
                *gz_bias_per_finger[f] += alpha * gz;
              }

              // Absolute filter quaternion sources both topics.
              // Published Euler is bounded to ±180° (ZYX Tait-Bryan) and will
              // wrap discretely at the boundary; consumers that need continuous
              // angles should subscribe to TactileSensor/Quaternion and decode
              // it themselves.
              float qRaw[4];
              g_filter[f].getQuaternion(qRaw[0], qRaw[1], qRaw[2], qRaw[3]);
              quaternions.data[f].values[0] = qRaw[0];
              quaternions.data[f].values[1] = qRaw[1];
              quaternions.data[f].values[2] = qRaw[2];
              quaternions.data[f].values[3] = qRaw[3];

              float roll, pitch, yaw;
              quatToEulerDeg(qRaw, roll, pitch, yaw);

              sensors_data.eulerangle.data[f].values[0] = roll;
              sensors_data.eulerangle.data[f].values[1] = pitch;
              sensors_data.eulerangle.data[f].values[2] = yaw;
            }
          }
          else if (BIASCalculationIterator == BIASCalculationIterations)
          {
            gx1_bias /= BIASCalculationIterations;
            gy1_bias /= BIASCalculationIterations;
            gz1_bias /= BIASCalculationIterations;
            gx2_bias /= BIASCalculationIterations;
            gy2_bias /= BIASCalculationIterations;
            gz2_bias /= BIASCalculationIterations;
            ax1_bias /= BIASCalculationIterations;
            ay1_bias /= BIASCalculationIterations;
            az1_bias /= BIASCalculationIterations;
            ax2_bias /= BIASCalculationIterations;
            ay2_bias /= BIASCalculationIterations;
            az2_bias /= BIASCalculationIterations;

            // The calibration-time accel mean *is* gravity expressed in the
            // body frame at rest — it's the signal, not a bias to subtract.
            // Seed each filter's quaternion from it, then null out the accel
            // bias so the runtime path stops subtracting a phantom offset.
            g_filter[0].initFromAccel(ax1_bias, ay1_bias, az1_bias);
            g_filter[1].initFromAccel(ax2_bias, ay2_bias, az2_bias);
            ax1_bias = ay1_bias = az1_bias = 0.0f;
            ax2_bias = ay2_bias = az2_bias = 0.0f;

            g_last_ts_ms[0] = 0;
            g_last_ts_ms[1] = 0;

            ++BIASCalculationIterator;
          }
          else if (BIASCalculationIterator < BIASCalculationIterations)
          {
            gx1_bias += fingers.finger[0].gyroscope[0] * gRes;
            gy1_bias += fingers.finger[0].gyroscope[1] * gRes;
            gz1_bias += fingers.finger[0].gyroscope[2] * gRes;
            gx2_bias += fingers.finger[1].gyroscope[0] * gRes;
            gy2_bias += fingers.finger[1].gyroscope[1] * gRes;
            gz2_bias += fingers.finger[1].gyroscope[2] * gRes;
            ax1_bias += fingers.finger[0].accelerometer[0] * aRes;
            ay1_bias += fingers.finger[0].accelerometer[1] * aRes;
            az1_bias += fingers.finger[0].accelerometer[2] * aRes;
            ax2_bias += fingers.finger[1].accelerometer[0] * aRes;
            ay2_bias += fingers.finger[1].accelerometer[1] * aRes;
            az2_bias += fingers.finger[1].accelerometer[2] * aRes;
            BIASCalculationIterator++;
          }

          if (!StopSensorDataAcquisition)
          {
            g_dynamic_pub->publish(sensors_data.dynamic);
            g_static_pub->publish(sensors_data.staticdata);
            g_accel_pub->publish(sensors_data.accelerometer);
            g_gyro_pub->publish(sensors_data.gyroscope);
            g_timestamp_pub->publish(sensors_data.timestamp);

            if (BIASCalculationIterator > BIASCalculationIterations)
            {
              g_euler_pub->publish(sensors_data.eulerangle);
              g_quat_pub->publish(quaternions);
            }
          }

          for (int f = 0; f < FINGER_COUNT; ++f)
            fingers.finger[f].newDataAvailable = false;
        }
      }
    }
  }

  send.command = USB_COMMAND_AUTOSEND_SYNC_SENSORS;
  send.data_length = 1;
  send.data[0] = 0;
  usbSend(&USB, &send);

  close(USB);
  executor.cancel();
  spin_thread.join();

  rclcpp::shutdown();
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
char* getCmdOption(char** begin, char** end, const string& option)
{
  char** itr = find(begin, end, option);
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
bool OpenAndConfigurePort(int* USB, char const* TheDevice)
{
  // All written by Jean-Philippe Roberge on April 2015, reviewed by Jean-Philippe Roberge in June 2016
  /* Open File Descriptor */
  //            *USB = open( "/tmp/interceptty", O_RDWR| O_NOCTTY ); // For debugging purposes (JP)
  *USB = open(TheDevice, O_RDWR | O_NOCTTY);
  /* Error Handling */
  if ((*USB) < 0)
  {
    cout << "Error " << errno << " opening " << TheDevice << ": " << strerror(errno) << endl;
    return false;
  }

  /**** Configure Port ****/
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr((*USB), &tty) != 0)
  {
    cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
    return false;
  }

  /* Set Baud Rate */
  cfsetospeed(&tty, (speed_t)B115200);
  cfsetispeed(&tty, (speed_t)B115200);

  /* Setting other Port Stuff */
  tty.c_cflag &= ~PARENB;  // Make 8n1
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_lflag &= ~ICANON;  // Remove canonical mode
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;        // no flow control
  tty.c_cc[VMIN] = 0;             // read doesn't block
  tty.c_cc[VTIME] = 0;            // 0.0 seconds read timeout between each byte max
  tty.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines

  /* Flush Port, then applies attributes */
  tcflush((*USB), TCIFLUSH);
  if (tcsetattr((*USB), TCSANOW, &tty) != 0)
  {
    cout << "Error " << errno << " from tcsetattr" << endl;
    return false;
  }
  return true;
}

static void usbSend(int* USB, UsbPacket* packet)
{
  uint8_t* p = (uint8_t*)packet;
  int n_written;

  packet->start_byte = USB_PACKET_START_BYTE;
  packet->crc8 = calcCrc8(p + 2, packet->data_length + 2);

  n_written = write((*USB), (char*)p, packet->data_length + 4);
}

static uint8_t calcCrc8(uint8_t* data, size_t len)
{
  // TODO: calculate CRC8
  return data[-1];
}

static bool usbReadByte(UsbPacket* packet, unsigned int* readSoFar, uint8_t d)
{
  uint8_t* p = (uint8_t*)packet;

  // Make sure start byte is seen
  if (*readSoFar == 0 && d != USB_PACKET_START_BYTE)
    return false;

  // Buffer the byte (making sure not to overflow the packet)
  if (*readSoFar < USB_PACKET_MAX_SIZE)
    p[*readSoFar] = d;
  ++*readSoFar;

  // If length is read, stop when done
  if (*readSoFar > 3 && *readSoFar >= (unsigned)packet->data_length + USB_PACKET_HEADER_SIZE)
  {
    *readSoFar = 0;

    // If CRC is ok, we have a new packet!  Return it.
    if (packet->crc8 == calcCrc8(p + 2, packet->data_length + 2))
      return true;

    // If CRC is not ok, find the next start byte and shift the packet back in hopes of getting back in sync
    for (unsigned int i = 1; i < (unsigned)packet->data_length + USB_PACKET_HEADER_SIZE; ++i)
      if (p[i] == USB_PACKET_START_BYTE)
      {
        memmove(p, p + i, packet->data_length + USB_PACKET_HEADER_SIZE - i);
        *readSoFar = packet->data_length + USB_PACKET_HEADER_SIZE - i;
        break;
      }
  }

  return false;
}

static void parseSensors(UsbPacket* packet, Fingers* fingers)
{
  for (unsigned int i = 0; i < packet->data_length;)
  {
    uint8_t sensorType = packet->data[i] & 0xF0;
    uint8_t f = packet->data[i] >> 2 & 0x03;
    ++i;

    if (f >= FINGER_COUNT)
      continue;

    uint8_t* sensorData = packet->data + i;
    unsigned int sensorDataBytes = packet->data_length - i;

    switch (sensorType)
    {
      case USB_SENSOR_TYPE_DYNAMIC_TACTILE:
        i += extractUint16((uint16_t*)fingers->finger[f].dynamicTactile, FINGER_DYNAMIC_TACTILE_COUNT, sensorData,
                           sensorDataBytes);
        fingers->finger[f].newDataAvailable = true;
        break;
      case USB_SENSOR_TYPE_STATIC_TACTILE:
        i += extractUint16(fingers->finger[f].staticTactile, FINGER_STATIC_TACTILE_COUNT, sensorData, sensorDataBytes);
        fingers->finger[f].newDataAvailable = true;
        break;
      case USB_SENSOR_TYPE_ACCELEROMETER:
        i += extractUint16((uint16_t*)fingers->finger[f].accelerometer, 3, sensorData, sensorDataBytes);
        fingers->finger[f].newDataAvailable = true;
        break;
      case USB_SENSOR_TYPE_GYROSCOPE:
        i += extractUint16((uint16_t*)fingers->finger[f].gyroscope, 3, sensorData, sensorDataBytes);
        fingers->finger[f].newDataAvailable = true;
        break;
      case USB_SENSOR_TYPE_TEMPERATURE:
        i += extractUint16((uint16_t*)&fingers->finger[f].temperature, 1, sensorData, sensorDataBytes);
        fingers->finger[f].newDataAvailable = true;
        break;
      case USB_SENSOR_TYPE_TIMESTAMP:
        i += extractUint64(&fingers->finger[f].timestamp, 1, sensorData, sensorDataBytes);
        fingers->finger[f].newDataAvailable = true;
        break;
      default:
        // Unknown sensor type — skip the marker byte and keep parsing so
        // forward-compatible additions don't kill the rest of the packet.
        break;
    }
  }
}

static inline uint16_t parseBigEndian2(uint8_t* data)
{
  return (uint16_t)data[0] << 8 | data[1];
}

static inline uint64_t parseBigEndian8(uint8_t* data)
{
  return (uint64_t)data[0] << 56 | (uint64_t)data[1] << 48 | (uint64_t)data[2] << 40 | (uint64_t)data[3] << 32 |
         (uint64_t)data[4] << 24 | (uint64_t)data[5] << 16 | (uint64_t)data[6] << 8 | (uint64_t)data[7];
}

static uint8_t extractUint16(uint16_t* to, uint16_t toCount, uint8_t* data, unsigned int size)
{
  unsigned int cur;

  // Extract 16-bit values.  If not enough data, extract as much data as available
  for (cur = 0; 2 * cur + 1 < size && cur < toCount; ++cur)
    to[cur] = parseBigEndian2(&data[2 * cur]);

  // Return number of bytes read
  return cur * 2;
}

static unsigned int extractUint64(uint64_t* to, unsigned int toCount, uint8_t* data, unsigned int size)
{
  unsigned int cur;

  for (cur = 0; 8 * cur + 7 < size && cur < toCount; ++cur)
    to[cur] = parseBigEndian8(&data[8 * cur]);

  return cur * 8;
}

static void forceReconnectSensor(int* USB)
{
  if (USB == nullptr || *USB < 0)
  {
    return;
  }

  RCLCPP_INFO(NODE_LOGGER, "Forcing tactile sensor reconnect sequence");

#ifdef TIOCM_DTR
  int modem_bits = 0;
  if (ioctl(*USB, TIOCMGET, &modem_bits) == -1)
  {
    RCLCPP_WARN(NODE_LOGGER, "Failed to read modem bits for %d: %s", *USB, strerror(errno));
  }
  else
  {
    int cleared = modem_bits & ~TIOCM_DTR;
    if (ioctl(*USB, TIOCMSET, &cleared) == -1)
    {
      RCLCPP_WARN(NODE_LOGGER, "Failed to clear DTR on %d: %s", *USB, strerror(errno));
    }
    usleep(100000);

    int set_bits = cleared | TIOCM_DTR;
    if (ioctl(*USB, TIOCMSET, &set_bits) == -1)
    {
      RCLCPP_WARN(NODE_LOGGER, "Failed to set DTR on %d: %s", *USB, strerror(errno));
    }
    usleep(100000);
  }
#endif

  UsbPacket stop_packet{};
  stop_packet.command = USB_COMMAND_AUTOSEND_SYNC_SENSORS;
  stop_packet.data_length = 1;
  stop_packet.data[0] = 0;
  usbSend(USB, &stop_packet);

  tcflush(*USB, TCIOFLUSH);
  usleep(100000);
}

static std::string trimWhitespace(const std::string& value)
{
  const auto start = value.find_first_not_of(" \t\r\n");
  if (start == std::string::npos)
  {
    return std::string();
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(start, end - start + 1);
}

static bool matchesKnownUsbString(const std::string& value)
{
  if (value.empty())
  {
    return false;
  }

  static const char* kKnownStrings[] = { "CoRo Tactile Sensor", "Cypress USB UART" };

  for (const char* expected : kKnownStrings)
  {
    if (strcasecmp(value.c_str(), expected) == 0)
    {
      return true;
    }
  }
  return false;
}

static bool readSysfsEntry(const std::string& path, std::string& value)
{
  std::ifstream file(path.c_str());
  if (!file.is_open())
  {
    return false;
  }
  std::string line;
  std::getline(file, line);
  value = trimWhitespace(line);
  return !value.empty();
}

static bool deviceMatchesKnownSensor(const std::string& device_path)
{
  const auto slash = device_path.find_last_of('/');
  const std::string dev_name = (slash == std::string::npos) ? device_path : device_path.substr(slash + 1);

  const std::vector<std::string> candidate_files = {
    std::string("/sys/class/tty/") + dev_name + "/device/../product",
    std::string("/sys/class/tty/") + dev_name + "/device/product",
    std::string("/sys/class/tty/") + dev_name + "/device/../manufacturer",
    std::string("/sys/class/tty/") + dev_name + "/device/manufacturer"
  };

  for (const auto& file : candidate_files)
  {
    std::string entry;
    if (readSysfsEntry(file, entry) && matchesKnownUsbString(entry))
    {
      return true;
    }
  }
  return false;
}

static std::string autoDetectSensorDevice()
{
  // Prefer udev symlinks created by sensor_install.sh
  {
    glob_t glob_result{};
    const int glob_ret = glob("/dev/rq_tsf85_*", 0, nullptr, &glob_result);
    if (glob_ret == 0 && glob_result.gl_pathc > 0)
    {
      std::string found = glob_result.gl_pathv[0];
      globfree(&glob_result);
      return found;
    }
    globfree(&glob_result);
  }

  const char* patterns[] = { "/dev/ttyACM*", "/dev/ttyUSB*" };
  for (const char* pattern : patterns)
  {
    glob_t glob_result{};
    const int glob_ret = glob(pattern, 0, nullptr, &glob_result);
    if (glob_ret != 0)
    {
      globfree(&glob_result);
      continue;
    }

    for (size_t i = 0; i < glob_result.gl_pathc; ++i)
    {
      const std::string candidate = glob_result.gl_pathv[i];
      if (deviceMatchesKnownSensor(candidate))
      {
        globfree(&glob_result);
        return candidate;
      }
    }
    globfree(&glob_result);
  }
  return std::string();
}
