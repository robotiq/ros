#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string.h>
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <boost/thread/thread.hpp>
#include "robotiq_tsf/StaticData.h"
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"

bool GripperIsAlive=false, TactileSensorsNeedToBeInitialized = false, StaticDataReadingRequested=false;
bool NewTactileSensorDataHasArrived=false;
#define NUM_OF_TAXELS 28
#define NUM_OF_TACTILE_SENSOR_INITIALIZATION_ITER 1000
std::vector<double> TactileSensor1(28),TactileSensor2(28);
std::vector<double> TactileSensor1Bias(28),TactileSensor2Bias(28);
std::vector<double> TactileSensor1RealValues(28),TactileSensor2RealValues(28);
int NumOfTactileSensorsInitializationIterations=0, TactileSensor1Sum=0, TactileSensor2Sum=0;
float avgsum1, avgsum2, avgforce;
float force_guage;
int largest_taxe_S1=0;
int largest_taxe_S2=0;
void InitializeTactileSensors();

void staticdataCallback(const robotiq_tsf::StaticData::ConstPtr &msg)
{

  if (TactileSensorsNeedToBeInitialized && NumOfTactileSensorsInitializationIterations<NUM_OF_TACTILE_SENSOR_INITIALIZATION_ITER)
  {
      std::cout << "INTI to WIN IT!" << std::endl;

    for (int i=0;i<msg->taxels.data()->values.size();i++)
    {
      TactileSensor1.at(i)=TactileSensor1.at(i)+msg->taxels[0].values[i];
      TactileSensor2.at(i)=TactileSensor2.at(i)+msg->taxels[1].values[i];
    }
    NumOfTactileSensorsInitializationIterations++;
    std::cout << "NumOfTactileSensorsInitializations = " << NumOfTactileSensorsInitializationIterations << std::endl;
  }
  else if (NumOfTactileSensorsInitializationIterations==NUM_OF_TACTILE_SENSOR_INITIALIZATION_ITER)
  {
    for (int i=0;i<msg->taxels.data()->values.size();i++)
    {
      TactileSensor1Bias.at(i)=TactileSensor1.at(i)/NUM_OF_TACTILE_SENSOR_INITIALIZATION_ITER;
      TactileSensor2Bias.at(i)=TactileSensor2.at(i)/NUM_OF_TACTILE_SENSOR_INITIALIZATION_ITER;
      std::cout << "TactileSensor1Bias: " << TactileSensor1Bias.at(i) << std::endl;
      std::cout << "TactileSensor2Bias: " << TactileSensor2Bias.at(i) << std::endl;
    }
    TactileSensorsNeedToBeInitialized=false;
    NumOfTactileSensorsInitializationIterations++; // Just so we show this message only once
  }
  else
  {
    TactileSensor1Sum=0; TactileSensor2Sum=0;
    for (int i=0;i<msg->taxels.data()->values.size();i++)
    {
      TactileSensor1RealValues.at(i)=(int)(msg->taxels[0].values[i]-TactileSensor1Bias.at(i));
      TactileSensor2RealValues.at(i)=(int)(msg->taxels[1].values[i]-TactileSensor2Bias.at(i));
      TactileSensor1Sum+=TactileSensor1RealValues.at(i); TactileSensor2Sum+=TactileSensor2RealValues.at(i);
    }

    if (StaticDataReadingRequested)
    {
      std::cout << "Sum of the taxels located in the middle of sensor #1: " << TactileSensor1RealValues.at(9)+TactileSensor1RealValues.at(10)+TactileSensor1RealValues.at(13)+TactileSensor1RealValues.at(14)+TactileSensor1RealValues.at(17)+TactileSensor1RealValues.at(18) << std::endl;
      std::cout << "Sum of the taxels located in the middle of sensor #2: " << TactileSensor2RealValues.at(9)+TactileSensor2RealValues.at(10)+TactileSensor2RealValues.at(13)+TactileSensor2RealValues.at(14)+TactileSensor2RealValues.at(17)+TactileSensor2RealValues.at(18) << std::endl;
      StaticDataReadingRequested=false;
    }
    NewTactileSensorDataHasArrived=true;
  }
}

void ForceCallback(const std_msgs::Float64::ConstPtr &msg)
{
   force_guage = msg->data;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tacile_sum");
  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
  ros::Subscriber sub_static = n->subscribe("/TactileSensor4/StaticData", 1000, staticdataCallback);
  ros::Subscriber sub_force_guage = n->subscribe("/force_gauge_reading", 1000, ForceCallback);

  ros::Rate loop_rate(60);


      /******************** TACTILE SENSORS SETUP *******************/
  std::cout << "Tactile Sensors Initialization will now proceed, please allow 1-2 seconds to complete. TO INIT" << std::endl;
  NumOfTactileSensorsInitializationIterations = 0;
  InitializeTactileSensors();
  std::cout << "Tactile Sensors Initialization completed." << std::endl;
    /******************** TACTILE *******************/
  int count =5;
  while (ros::ok())
  {
    //lets sum this bitch

    avgsum1=0;
    avgsum2=0;
    //avgforce = 0;

/*
    for(int i=0; i<count;i++)
    {
        avgsum1+=TactileSensor1Sum;
        avgsum2+=TactileSensor2Sum;
        //avgforce+=force_guage;
        ros::spinOnce();
        loop_rate.sleep();

    }*/

    largest_taxe_S1=0;
    largest_taxe_S2=0;
    for (int i=0;i<TactileSensor1RealValues.size();i++)
    {
      if(i>0)
      {
          if(TactileSensor1RealValues.at(i)>TactileSensor1RealValues.at(largest_taxe_S1))
          {
              largest_taxe_S1=i;
          }
         if(TactileSensor2RealValues.at(i)>TactileSensor2RealValues.at(largest_taxe_S2))
         {
              largest_taxe_S2=i;
         }
      }

    }
      std::cout<<"force,"<<force_guage<< ",taxels2,"<<TactileSensor2RealValues.at(largest_taxe_S2)<< ","<<largest_taxe_S2<< std::endl; //<< ", force_guage: "<<avgforce/count
  //<<",taxels1," <<largest_taxe_S1 << ","<<TactileSensor2RealValues.at(largest_taxe_S1)
    //std::cout << "AverageT1: " <<avgsum1/count<< ", AverageT2: " <<avgsum2/count  <<  std::endl; //<< ", force_guage: "<<avgforce/count

   if(force_guage>50)
   {
       break;
   }
   ros::spinOnce();
   loop_rate.sleep();
  }

  std::cout<< "BiasS2," << TactileSensor2Bias.at(largest_taxe_S2) << ","<<largest_taxe_S2<<std::endl;


}


void InitializeTactileSensors()
{
  TactileSensorsNeedToBeInitialized=true;
  while(TactileSensorsNeedToBeInitialized)
  {
    ros::spinOnce();
  }
  std::cout << "Tactile Sensors have been initialized!" << std::endl;
}

/*
std::cout<< "force"<< force_guage << "S1," << TactileSensor1RealValues.at(0) << "," << TactileSensor1RealValues.at(1) << "," << TactileSensor1RealValues.at(2) << "," << TactileSensor1RealValues.at(3) << "," << TactileSensor1RealValues.at(4) << "," << TactileSensor1RealValues.at(5) << "," << TactileSensor1RealValues.at(6) << "," << +
                    TactileSensor1RealValues.at(7) << "," << TactileSensor1RealValues.at(8) << "," << TactileSensor1RealValues.at(9) << "," << TactileSensor1RealValues.at(10) << "," << TactileSensor1RealValues.at(11) << "," << TactileSensor1RealValues.at(12) << "," << TactileSensor1RealValues.at(13) << "," << +
                    TactileSensor1RealValues.at(14) << "," << TactileSensor1RealValues.at(15) << "," << TactileSensor1RealValues.at(16) << "," << TactileSensor1RealValues.at(17) << "," << TactileSensor1RealValues.at(18) << "," << TactileSensor1RealValues.at(19) << "," << TactileSensor1RealValues.at(20) << "," << +
                    TactileSensor1RealValues.at(21) << "," << TactileSensor1RealValues.at(22) << "," << TactileSensor1RealValues.at(23) << "," << TactileSensor1RealValues.at(24) << "," << TactileSensor1RealValues.at(25) << "," << TactileSensor1RealValues.at(26) << "," << TactileSensor1RealValues.at(27) << "," << +
            "S2," << TactileSensor2RealValues.at(0) << "," << TactileSensor2RealValues.at(1) << "," << TactileSensor2RealValues.at(2) << "," << TactileSensor2RealValues.at(3) << "," << TactileSensor2RealValues.at(4) << "," << TactileSensor2RealValues.at(5) << "," << TactileSensor2RealValues.at(6) << "," << +
                    TactileSensor2RealValues.at(7) << "," << TactileSensor2RealValues.at(8) << "," << TactileSensor2RealValues.at(9) << "," << TactileSensor2RealValues.at(10) << "," << TactileSensor2RealValues.at(11) << "," << TactileSensor2RealValues.at(12) << "," << TactileSensor2RealValues.at(13) << "," << +
                    TactileSensor2RealValues.at(14) << "," << TactileSensor2RealValues.at(15) << "," << TactileSensor2RealValues.at(16) << "," << TactileSensor2RealValues.at(17) << "," << TactileSensor2RealValues.at(18) << "," << TactileSensor2RealValues.at(19) << "," << TactileSensor2RealValues.at(20) << "," << +
                    TactileSensor2RealValues.at(21) << "," << TactileSensor2RealValues.at(22) << "," << TactileSensor2RealValues.at(23) << "," << TactileSensor2RealValues.at(24) << "," << TactileSensor2RealValues.at(25) << "," << TactileSensor2RealValues.at(26) << "," << TactileSensor2RealValues.at(27) << "," << +
            "TS1," << TactileSensor1Sum << "," << "TS2," << TactileSensor2Sum << std::endl;
ros::spinOnce();
loop_rate.sleep();
}


std::cout<< "BiasS1," << TactileSensor1Bias.at(0) << "," << TactileSensor1Bias.at(1) << "," << TactileSensor1Bias.at(2) << "," << TactileSensor1Bias.at(3) << "," << TactileSensor1Bias.at(4) << "," << TactileSensor1Bias.at(5) << "," << TactileSensor1Bias.at(6) << "," << +
         TactileSensor1Bias.at(7) << "," << TactileSensor1Bias.at(8) << "," << TactileSensor1Bias.at(9) << "," << TactileSensor1Bias.at(10) << "," << TactileSensor1Bias.at(11) << "," << TactileSensor1Bias.at(12) << "," << TactileSensor1Bias.at(13) << "," << +
         TactileSensor1Bias.at(14) << "," << TactileSensor1Bias.at(15) << "," << TactileSensor1Bias.at(16) << "," << TactileSensor1Bias.at(17) << "," << TactileSensor1Bias.at(18) << "," << TactileSensor1Bias.at(19) << "," << TactileSensor1Bias.at(20) << "," << +
         TactileSensor1Bias.at(21) << "," << TactileSensor1Bias.at(22) << "," << TactileSensor1Bias.at(23) << "," << TactileSensor1Bias.at(24) << "," << TactileSensor1Bias.at(25) << "," << TactileSensor1Bias.at(26) << "," << TactileSensor1Bias.at(27) << "," << +
         "BiasS2," << TactileSensor2Bias.at(0) << "," << TactileSensor2Bias.at(1) << "," << TactileSensor2Bias.at(2) << "," << TactileSensor2Bias.at(3) << "," << TactileSensor2Bias.at(4) << "," << TactileSensor2Bias.at(5) << "," << TactileSensor2Bias.at(6) << "," << +
         TactileSensor2Bias.at(7) << "," << TactileSensor2Bias.at(8) << "," << TactileSensor2Bias.at(9) << "," << TactileSensor2Bias.at(10) << "," << TactileSensor2Bias.at(11) << "," << TactileSensor2Bias.at(12) << "," << TactileSensor2Bias.at(13) << "," << +
         TactileSensor2Bias.at(14) << "," << TactileSensor2Bias.at(15) << "," << TactileSensor2Bias.at(16) << "," << TactileSensor2Bias.at(17) << "," << TactileSensor2Bias.at(18) << "," << TactileSensor2Bias.at(19) << "," << TactileSensor2Bias.at(20) << "," << +
         TactileSensor2Bias.at(21) << "," << TactileSensor2Bias.at(22) << "," << TactileSensor2Bias.at(23) << "," << TactileSensor2Bias.at(24) << "," << TactileSensor2Bias.at(25) << "," << TactileSensor2Bias.at(26) << "," << TactileSensor2Bias.at(27) << "," << std::endl;
         */
