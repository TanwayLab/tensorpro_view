/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-07-2019
 *  Edited on: 21-12-2019
 *  Author: Elodie Shan
 *  Editor: LF Shen
 *
 *  ROS driver interface for Tensor-Pro 3D LIDARs
**************************************************/


#ifndef TANWAYTENSOR_H_
#define TANWAYTENSOR_H_
#include <ros/ros.h> //generic C++ stuff
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h> //ROS message type to publish a pointCloud
#include <pcl_ros/point_cloud.h> //use these to convert between PCL and ROS datatypes
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <iostream> 
#include <sys/stat.h>
#include "tensorpro_view/network.h"
#include "tensorpro_view/rawdata_utils.h"

#include "tensorpro_view/tanwaytp_point_type.h"
typedef Tanway_TP_Point PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


using namespace std;

class TanwayTensor
{
public:
  TanwayTensor();

  virtual ~TanwayTensor();

  double verticalChannels[16] = { -5.274283f, -4.574258f,
 -3.872861f, -3.1703f, -2.466783f, -1.762521f, -1.057726f, -0.352611f, 0.352611f,
 1.057726f, 1.762521f, 2.466783f, 3.1703f, 3.872861f, 4.574258f, 5.274283f};

  double RA = 0.01745329;//deg to rad. 180/pi
  double c = 2.997924;

  std::string host = "192.168.111.204" ;
  std::string LiDARhost = "192.168.111.51" ;
  std::string frame_id = "TanwayTP" ;
  std::string topic = "/tensorpro_cloud" ;

  int port = 5600;
  int LiDARport = 5050;

  double StartAngle = 30;
  double EndAngle = 150;
  double min_range = 0;
  double max_range = 200;
  int min_channel = 1;
  int max_channel = 16;
  bool timestamp_print_switch = false;
  bool needPublishCloud = true;

  u_char buf[1500];
  
  UDPNetwork UDP_;

  pcl::PointXYZRGB p_nan;

  sensor_msgs::PointCloud2 ros_cloud; 

  PointCloudT::Ptr point_cloud_ptr;
  ros::Publisher pubCloud;

  // @brief Initialization.
  bool initialize(ros::NodeHandle nh, ros::NodeHandle nh_private);

  /** 
  *  @brief use private node handle to get parameters.
  *
  *  @param node ROS handle for calling node.
  *  @param nh_private ROS private handle.
  */
  void getParam(ros::NodeHandle nh_private);

  /** 
  *  @brief calculate the horizontal angle.
  *
  *  @param HextoAngle is the value obtained by parsing the UDP data.
  *  @param ticks number.
  */
  float getHorizontalAngle(float HextoAngle);

  /** 
  *  @brief get basic point.
  *
  *  @param x.
  *  @param y
  *  @param z
  *  @param ring
  *  @param intensity pulse intensity  
  */
  PointT getBasicPoint(double x, double y, double z, int ring, float intensity);

  /** 
  *  @brief Parse the UDP packet and calculate the coordinate values.
  *
  *  @param horizontalAngle.
  *  @param offset of buf.
  */
  virtual bool processXYZ(float horizontalAngle,int offset);

  /** 
  *  @brief fill attributions of Cloud.
  */
  bool fillCloudAttr();

  /** 
  *  @brief Publish point cloud data when conditions are met.
  *
  *  @param point_cloud_ptr Pointer to store point clouds.
  */
  virtual bool publishCloud();
  
  /** 
  *  @brief Print timestamps.
  *
  *  @param offset of buf.
  */
  bool printTimeStamps(int offset);

  /** 
  *  @brief Recieve UDP date.
    */
  bool getUDP();

  /** 
  *  @brief Parse UDP date to get points.
  */
  bool getPoints();


};

#endif
