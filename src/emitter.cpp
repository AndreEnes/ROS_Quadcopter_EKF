// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <signal.h>
#include "ros/ros.h"

// include files to use services like 'robot_get_time'.
// srv files needed to use webots service can be found in the /srv folder where you found this example.
// for more info on how to create and use services with ROS refer to their website: http://wiki.ros.org/
// here 'webots_ros' is the name of the package used for this node. Replace it by your own package.
#include <webots_ros/robot_get_device_list.h>
#include <webots_ros/StringStamped.h>

#include <webots_ros/get_int.h>
#include <webots_ros/get_string.h>
#include <webots_ros/set_int.h>
#include <webots_ros/set_string.h>
#include <webots_ros/node_get_name.h>
#include <webots_ros/node_get_type.h>
#include <webots_ros/get_uint64.h>
#include <webots_ros/supervisor_get_from_def.h>

// include files to use standard message types in topic
// Webots only use basic messages type defined in ROS library
#include <std_msgs/String.h>

#include <string.h>

#define TIME_STEP 32

static bool callbackCalled = false;
ros::ServiceClient time_step_client;
webots_ros::set_int time_step_srv;


void quit(int sig) 
{
    ROS_INFO("User stopped the 'emitter' node.");
    ros::shutdown();
    exit(0);
}

void receiverCallback(const webots_ros::StringStamped::ConstPtr &value) 
{
    char *message = const_cast<char *>(value->data.c_str());
    ROS_INFO("Received a message %s.", message);
    callbackCalled = true;
}

int main(int argc, char **argv) 
{
  //std::string model_name = "DJI_MAVIC_2_PRO";
  std::string model_name = "rec_generic";
  std::string controllerName;
  std::vector<std::string> deviceList;
  
  // create a node named 'robot_information_parser' on ROS network    
  ros::init(argc, argv, "emitter", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  signal(SIGINT, quit);
  
  // Wait for the `ros` controller.
  //ros::service::waitForService("/robot/time_step");
  ros::spinOnce();
  
  ros::ServiceClient emitter_send_client;
  webots_ros::set_string emitter_send_srv;
  emitter_send_client = n.serviceClient<webots_ros::set_string>(model_name + "/emitter/send");
  int i = 0;
  while(1)
  {
    std::string msg = "abc" + std::to_string(i);
    i++; 
    emitter_send_srv.request.value = "abc" + std::to_string(i);
    
    if (emitter_send_client.call(emitter_send_srv) && emitter_send_srv.response.success)
      ROS_INFO("Emitter has sent data: %s", msg);
    else
      ROS_ERROR("Failed to call service emitter_send to send data.");

    emitter_send_client.shutdown();
    time_step_client.call(time_step_srv);
    
    ros::Duration(0.2).sleep();
    
  }

  emitter_send_client.shutdown();
  time_step_client.call(time_step_srv);

}
