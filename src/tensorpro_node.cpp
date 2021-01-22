/************************************************
 *  Copyright (C) 2020 Tanway Technology Co., Ltd
 *  License:　BSD 3-Clause License
 *
 *  Created on: 16-07-2019
 *  Edited on: 22-01-2021
 *  Author: Elodie Shan
 *  Editor: LF Shen

 *  Node for Tanway Tensor 3D LIDARs   
**************************************************/

#include <ros/ros.h>
#include <ros/console.h>
#include "tensorpro_view/TanwayTensor.h"



int main(int argc, char** argv) {
  ros::init(argc, argv, "tensorproview"); 
  ros::NodeHandle nh;
  ros::Rate r(20);

  ros::NodeHandle nh_private("~");


  ROS_INFO( "tensorpro viewer for ROS" );
  ROS_INFO( "Version 1.1.7" );
  ROS_INFO( "Update Date: 2021/01/22\n" );
  ROS_INFO( "View in rviz;");
  ROS_INFO( "topic= tensorpro_cloud and fixed frame= TanwayTP");

  TanwayTensor TensorPro;
  if (!TensorPro.initialize(nh,nh_private))
  {
    ros::shutdown();
    return 0;
  }

  while (ros::ok()){
    TensorPro.getUDP();
  }
  return 0;
}
