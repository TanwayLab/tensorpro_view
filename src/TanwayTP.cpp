/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-07-2019
 *  Edited on: 31-05-2020
 *  Author: Elodie Shan
 *  Editor: LF Shen

 *  Node for Tanway Tensor 3D LIDARs   
 *  Function: 20lines one bag-->switch:cout flag state
**************************************************/

#include <ros/ros.h> //generic C++ stuff
#include <math.h>

#include "tensorpro_view/TanwayTensor.h"

using namespace std;
using namespace TensorData;

typedef Tanway_TP_Point PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

TanwayTensor::TanwayTensor()
{
  PointCloudT cloud;
  point_cloud_ptr = cloud.makeShared();
};

TanwayTensor::~TanwayTensor(){};

bool TanwayTensor::initialize(ros::NodeHandle nh, ros::NodeHandle nh_private)
{
  getParam(nh_private);

  pubCloud = nh.advertise<sensor_msgs::PointCloud2> (topic, 1);

    //UDP Sockect Connect
  if (!UDP_.Init(host, port, LiDARhost, LiDARport))
    return false;
  
  return true;
}

void TanwayTensor::getParam(ros::NodeHandle nh_private)
{
  nh_private.param<std::string>("host", host, "192.168.111.204");
  nh_private.param<std::string>("LiDARhost", LiDARhost, "192.168.111.51");
  nh_private.param<std::string>("frame_id", frame_id, "TanwayTP");
  nh_private.param<std::string>("topic", topic, "/tensorpro_cloud");
  nh_private.param<int>("port", port, 5600);
  nh_private.param<int>("LiDARport", LiDARport, 5051);

  nh_private.param<double>("StartAngle", StartAngle, 30);
  nh_private.param<double>("EndAngle", EndAngle, 150);
  nh_private.param<double>("min_range", min_range, 0);
  nh_private.param<double>("max_range", max_range, 200);
  nh_private.param<int>("min_channel", min_channel, 1);
  nh_private.param<int>("max_channel", max_channel, 16);
  nh_private.param<bool>("timestamp_print_switch", timestamp_print_switch, false);
}

float TanwayTensor::getHorizontalAngle(float HextoAngle)
{
  float horizontalAngle = HextoAngle/100000;
  return horizontalAngle;
}

PointT TanwayTensor::getBasicPoint(double x, double y, double z, int ring, float intensity)
{
  PointT basic_point;

  basic_point.x = x;
  basic_point.y = y;
  basic_point.z = z;
  basic_point.ring = ring;
  basic_point.intensity = intensity;
  return basic_point;
}


bool TanwayTensor::processXYZ(float horizontalAngle, int offset)
{ 
  float cos_hA_RA = cos(horizontalAngle * RA);
  float sin_hA_RA = sin(horizontalAngle * RA);

  int seq = 0;
  //Parse the data for each point 
  while (seq < 16) 
  {
    float vA = verticalChannels[seq];
    float cos_vA_RA = cos(vA * RA);

    float hexToInt = TwoHextoInt(buf[offset+seq*4], buf[offset+seq*4+1]); 
    float L = hexToInt*500*c/10.f/16384.f/2;
    ROS_DEBUG_STREAM("seq:"<< seq <<" hA:"<< horizontalAngle <<" L:" <<L); 

    if (L > max_range || L < min_range || (seq+1) < min_channel || (seq+1) > max_channel) 
    { 
      seq++;
      continue;
    }

    //Calculate the coordinate values
    float pulse_intensity = TwoHextoInt(buf[offset+seq*4+2], buf[offset+seq*4+3]);
    pulse_intensity = pulse_intensity*500*c/10.f/16384.f/2;

    float x = L * cos_vA_RA * cos_hA_RA;
    float y = L * cos_vA_RA * sin_hA_RA;
    float z = L * sin(vA * RA);
    point_cloud_ptr->points.push_back(getBasicPoint(x,y,z,(seq+1),pulse_intensity));

    seq++;
  }
  return true;
}

bool TanwayTensor::printTimeStamps(int offset)
{
  cout << "本包尾列GPS时间戳: ";
  cout.fill('0');
  cout.width(2);
  cout << hex << (unsigned int)(unsigned char)buf[offset+68];
  cout  << " ";
  cout.fill('0');
  cout.width(2);
  cout << hex << (unsigned int)(unsigned char)buf[offset+69];
  cout << " ";
  cout.fill('0');
  cout.width(2);
  cout << hex << (unsigned int)(unsigned char)buf[offset+70];
  cout  << " ";
  cout.fill('0');
  cout.width(2);
  cout << hex << (unsigned int)(unsigned char)buf[offset+71];    
  cout << endl;   
  return true; 
}

bool TanwayTensor::fillCloudAttr()
{
  point_cloud_ptr->width = (int) point_cloud_ptr->points.size(); //Number of points in one frame
  point_cloud_ptr->height = 1; // Whether the point cloud is orderly, 1 is disordered
  point_cloud_ptr->header.frame_id = frame_id; //Point cloud coordinate system name
  ROS_DEBUG( "Publish   num: [%d]",(int) point_cloud_ptr->points.size());
  return true;
}

bool TanwayTensor::publishCloud()
{
  fillCloudAttr();
  pcl::toROSMsg(*point_cloud_ptr, ros_cloud); //convert between PCL and ROS datatypes
  ros_cloud.header.stamp = ros::Time::now(); //Get ROS system time
  pubCloud.publish(ros_cloud); //Publish cloud

  PointCloudT cloud;
  point_cloud_ptr = cloud.makeShared();

  return true;
}
bool TanwayTensor::getUDP()
{
  if (UDP_.recvUDP(buf) < 0) //Recieve UDP packets
    return false;
  
  bool status = getPoints();
  return status;
}

bool TanwayTensor::getPoints()
{
  int blocks_num = 0;

  while (blocks_num < 20) 
  {
    int offset = blocks_num * 72;

    if (timestamp_print_switch && blocks_num == 19)
      printTimeStamps(offset);

    int HextoAngle = FourHexToInt(buf[offset+64],buf[offset+65],buf[offset+66],buf[offset+67]);
    float horizontalAngle = getHorizontalAngle(HextoAngle);
    if(horizontalAngle>=StartAngle && horizontalAngle<=EndAngle)
    {
      needPublishCloud = true;
      processXYZ(horizontalAngle,offset);
    }

    else
      ROS_DEBUG("hA = [%f]",horizontalAngle);

    if (horizontalAngle <StartAngle && needPublishCloud)
    {
      publishCloud();
      needPublishCloud = false;
    }
    
    blocks_num++;
  }

  return true;
}

