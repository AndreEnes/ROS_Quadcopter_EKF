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
#include <webots_ros/get_float.h>
#include <webots_ros/get_bool.h>

// include files to use standard message types in topic
// Webots only use basic messages type defined in ROS library
#include <std_msgs/String.h>

#define TIME_STEP 32

static bool callbackCalled = false;
ros::ServiceClient time_step_client;
webots_ros::set_int time_step_srv;


void quit(int sig) 
{
    ROS_INFO("User stopped the 'basico' node.");
    ros::shutdown();
    exit(0);
}

//void receiverCallback(const webots_ros::StringStamped::ConstPtr &value) 
//{
//    char *message = const_cast<char *>(value->data.c_str());
//   ROS_INFO("Received a message %s.", message);
//    callbackCalled = true;
//}

int main(int argc, char **argv) 
{
    std::string model_name = "beacon_";
    std::string robot_num = "1";

    //std::string model_name = "rec";
    std::string controllerName;
    std::vector<std::string> deviceList;
    // create a node named 'robot_information_parser' on ROS network    
    ros::init(argc, argv, "basico");//, ros::init_options::AnonymousName);
    ros::NodeHandle n;

    signal(SIGINT, quit);

    // Wait for the `ros` controller.
    //ros::service::waitForService("/robot/time_step");
    ros::spinOnce();

    ros::ServiceClient set_receiver_client;
    webots_ros::set_int receiver_srv;
    ros::Subscriber sub_receiver_32;    
    set_receiver_client = n.serviceClient<webots_ros::set_int>(model_name + robot_num + "/receiver/enable");

    ros::ServiceClient sampling_period_receiver_client;
    webots_ros::get_int sampling_period_receiver_srv;
    sampling_period_receiver_client = n.serviceClient<webots_ros::get_int>(model_name + robot_num + "/receiver/get_sampling_period");

    // test receiver_get_signal_strength
    // An error message will probably appear since no signal has been sent to the receiver.
    ros::ServiceClient receiver_get_signal_strength_client;
    webots_ros::get_float receiver_get_signal_strength_srv;
    receiver_get_signal_strength_client = n.serviceClient<webots_ros::get_float>(model_name + robot_num + "/receiver/get_signal_strength");

    // test receiver_next_packet
    // An error message will probably appear since there is no packet to read
    ros::ServiceClient receiver_next_packet_client;
    webots_ros::get_bool receiver_next_packet_srv;
    receiver_next_packet_client = n.serviceClient<webots_ros::get_bool>(model_name + robot_num + "/receiver/next_packet");

    receiver_srv.request.value = 32;
    if (set_receiver_client.call(receiver_srv) && receiver_srv.response.success) 
    {
        ROS_INFO("Receiver enabled.");
        //sub_receiver_32 = n.subscribe(model_name + robot_num + "/receiver/data", 1, receiverCallback);
        //callbackCalled = false;
        
        //while (sub_receiver_32.getNumPublishers() == 0 || !callbackCalled) 
        //{
        //    ROS_INFO("AQUIIII");
        //    ros::spinOnce();
        //    time_step_client.call(time_step_srv);
        //}
    } 
    else 
    {
        if (!receiver_srv.response.success)
            ROS_ERROR("Sampling period is not valid.");
        
        ROS_ERROR("Failed to enable receiver.");
        
        return 1;
    }
    receiver_next_packet_srv.request.ask = true;

    while (1)
    {
        receiver_get_signal_strength_client.call(receiver_get_signal_strength_srv);
        if (receiver_get_signal_strength_srv.response.value != -1.0)
        {
            ROS_INFO("Strength of the signal is %lf.", receiver_get_signal_strength_srv.response.value);
            if (receiver_next_packet_client.call(receiver_next_packet_srv) && receiver_next_packet_srv.response.value){
                //ROS_INFO("Next packet is ready to be read.");
                ros::Duration(0.1).sleep();	
            }
            else if (!receiver_next_packet_srv.response.value)
            ROS_INFO("No message received by emitter, impossible to get next packet.");
            else
                ROS_ERROR("Failed to call service receiver_next_packet.");
        }
        else{
            ros::Duration(0.1).sleep();	
            //ROS_INFO("No message received by emitter, impossible to get signal strength.");
        }
        ros::spinOnce();
    }

  

  receiver_next_packet_client.shutdown();
  time_step_client.call(time_step_srv);
    
    sub_receiver_32.shutdown();
    set_receiver_client.shutdown();
    time_step_client.call(time_step_srv);


}