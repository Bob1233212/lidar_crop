#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <iostream>
#include<string>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <vector>
#include<tf/tf.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

class LidarCrop_test{
  public:
     double CropAngle;
     std::string SubTopic;
     std::string PubTopic;
     ros::Publisher PubCropedPoint;
     ros::Subscriber SubOrignPoint;
     void callbackfunction(const sensor_msgs::LaserScan::ConstPtr &msg){
          sensor_msgs::LaserScan msg1;
          msg1.header=msg->header;
          msg1.angle_min=-CropAngle/360*3.1415926;
          msg1.angle_max=CropAngle/360*3.1415926;
          msg1.angle_increment=msg->angle_increment;
          msg1.time_increment=msg->time_increment;
          msg1.scan_time=msg->scan_time;
          msg1.range_max=msg->range_max;
          msg1.range_min=msg->range_min;
          msg1.ranges.resize(msg->ranges.size());
          msg1.intensities.resize(msg->intensities.size());
          int count=0;
          int num=CropAngle/2;
          int num1=msg1.ranges.size();
          
          for(int i=num1-num;i<num1;i++){
               msg1.ranges[i-num1+num]=msg->ranges[i];
               if(msg->intensities.size()>i){
                  msg1.intensities[i-num1+num]=msg->intensities[i];
               }
               count++;
          }
         
         for(int i=0;i<num;i++){
               msg1.ranges[i+num]=msg->ranges[i];
               if(msg->intensities.size()>i){
                  msg1.intensities[i+num]=msg->intensities[i];
               }
               count++;
            }
           msg1.ranges.resize(count);

        if(msg->intensities.size() >= count){
         msg1.intensities.resize(count);}
         
         PubCropedPoint.publish(msg1);     
   }    
   void callbackfunctionsimulation(const sensor_msgs::LaserScan::ConstPtr &msg){
          sensor_msgs::LaserScan msg1;
          msg1.header=msg->header;
          msg1.header.frame_id="laser";
          msg1.angle_min=msg->angle_min;
          msg1.angle_max=msg->angle_max;
          msg1.angle_increment=msg->angle_increment;
          msg1.time_increment=msg->time_increment;
          msg1.scan_time=msg->scan_time;
          msg1.range_max=msg->range_max;
          msg1.range_min=msg->range_min;
          msg1.ranges.resize(msg->ranges.size());
          msg1.intensities.resize(msg->intensities.size());
          double num=CropAngle/180*3.1415926;
          int num1=msg1.ranges.size();
          int start=(-CropAngle/360*3.1415926-msg1.angle_min)/msg->angle_increment;
          int end=num/msg->angle_increment;
          for(int i=start;i<(start+end+1);i++){
               msg1.ranges[i]=msg->ranges[i];
               if(msg->intensities.size()>i){
               msg1.intensities[i]=msg->intensities[i];}
          }
         PubCropedPoint.publish(msg1);     
   }    
   
   void callbackfunctionMultiEcho(const sensor_msgs::MultiEchoLaserScan::ConstPtr &msg){
          sensor_msgs::MultiEchoLaserScan msg1;
          msg1.header=msg->header;
          msg1.angle_min=-CropAngle/360*3.1415926;
          msg1.angle_max=CropAngle/360*3.1415926;
          msg1.angle_increment=msg->angle_increment;
          msg1.time_increment=msg->time_increment;
          msg1.scan_time=msg->scan_time;
          msg1.range_max=msg->range_max;
          msg1.range_min=msg->range_min;
          msg1.ranges.resize(msg->ranges.size());
          msg1.intensities.resize(msg->intensities.size());
          int num1=msg->ranges.size();
          int count=0;
          for(int i=0;i<num1;i++){
            if((msg->angle_min+i*msg->angle_increment>=msg1.angle_min)&&(msg->angle_min+i*msg->angle_increment<=msg1.angle_max)){
                   msg1.ranges[count]=msg->ranges[i];
                   if(msg->intensities.size()>i){
                  msg1.intensities[count]=msg->intensities[i];
               }
               count++;
            }
          }
          msg1.ranges.resize(count);
        if(msg->intensities.size() >= count){
         msg1.intensities.resize(count);}
         PubCropedPoint.publish(msg1);     
   } 

};

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "lidarcrop");

  ros::NodeHandle n;
  //建立雷达假数据
  /*sensor_msgs::LaserScan msg;
  ros::Publisher marker_pub = n.advertise<sensor_msgs::LaserScan>("laser", 10);
  ros::Rate loop_rate(10);
  msg.header.frame_id="laserscan_link";
  msg.angle_min=0;
  msg.angle_max=3.14159*2;
  msg.angle_increment=0.01745329;
  msg.time_increment=0.0;
  msg.scan_time=0.1;
  msg.range_min=0.5;
  msg.range_max=4;
  int count=0;
  msg.intensities.resize(1000);
  msg.ranges.resize(1000);
  for(int i=0;i<360;i++){
   msg.ranges[i]=1;
   msg.intensities[i]=1;
   count++;
  }
  msg.ranges[1]=3;
  msg.ranges[20]=3;
  msg.ranges.resize(count);
  msg.intensities.resize(count);
  while (ros::ok())
    {
        marker_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep(); //配合执行频率，sleep一段时间，然后进入下一个循环。
    }
  */
  bool multechoflag,simulation;
  int lidarnum;
  n.getParam("lidarnum",lidarnum);
  n.getParam("multechoflag",multechoflag);
  n.getParam("simulation",simulation);
  LidarCrop_test lidarGroup[100];
  for(int i=0;i<lidarnum;i++){
    std::string lidarsubname="subtopicname"+std::to_string(i);
    std::string lidarpubname="pubtopicname"+std::to_string(i);
    std::string lidarangle="angle"+std::to_string(i);
    
    n.getParam(lidarsubname,lidarGroup[i].SubTopic);
    n.getParam(lidarpubname,lidarGroup[i].PubTopic);
    n.getParam(lidarangle,lidarGroup[i].CropAngle);
  }
  for(int i=0;i<lidarnum;i++){
  if(!multechoflag){
      if(simulation){
      lidarGroup[i].PubCropedPoint=n.advertise<sensor_msgs::LaserScan>(lidarGroup[i].PubTopic,10);
      lidarGroup[i].SubOrignPoint=n.subscribe<sensor_msgs::LaserScan>(lidarGroup[i].SubTopic,1000,std::bind(&LidarCrop_test::callbackfunctionsimulation,&lidarGroup[i],std::placeholders::_1));
      }
      else{
      lidarGroup[i].PubCropedPoint=n.advertise<sensor_msgs::LaserScan>(lidarGroup[i].PubTopic,10);
      lidarGroup[i].SubOrignPoint=n.subscribe<sensor_msgs::LaserScan>(lidarGroup[i].SubTopic,1000,std::bind(&LidarCrop_test::callbackfunction,&lidarGroup[i],std::placeholders::_1));
      }}
 else{
      lidarGroup[i].PubCropedPoint=n.advertise<sensor_msgs::MultiEchoLaserScan>(lidarGroup[i].PubTopic,10);
      lidarGroup[i].SubOrignPoint=n.subscribe<sensor_msgs::MultiEchoLaserScan>(lidarGroup[i].SubTopic,1000,std::bind(&LidarCrop_test::callbackfunctionMultiEcho,&lidarGroup[i],std::placeholders::_1));

  } }
   ros::spin();

  return 0;
}

