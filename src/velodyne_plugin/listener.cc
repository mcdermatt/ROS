/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
// #include <gazebo/msgs/laserscan_stamped.proto> //test
#include <gazebo/gazebo_client.hh>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <iostream>


//TODO: write similar function to take in velodyne scan info instead of world stats! 
/////////////////////////////////////////////////
// Function is called everytime a message is received.
// void cb(ConstWorldStatisticsPtr &_msg)
// {
//   // Dump the message contents to stdout.
//   std::cout << _msg->DebugString();
// }

//Try with LaserScanStamped anther
void cb(ConstWorldStatisticsPtr &_msg)
{
  // Print when ready 
  std::cout << _msg->DebugString();
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // ///////////////////////////////////////////////
  // //TEST - set up node to publish information to ROS topic (rather than whatever gazebo passes information on)
  // ros::init(_argc, _argv, "publisher_node");
  // ros::NodeHandle n; //Not sure if I need this here

  // //init publisher
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  // ///////////////////////////////////////////////


  // Listen to Gazebo world_stats topic
  // gazebo::transport::SubscriberPtr sub = node->Subscribe("~/world_stats", cb);
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/my_velodyne/top/sensor/scan", cb);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

    // DEBUG: publish to new "chatter" topic
    // std_msgs::String msg; //declare message object
    // std::stringstream ss;
    // ss << "hello world ";
    // msg.data = ss.str();
    // chatter_pub.publish(msg);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
