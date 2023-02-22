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

void receiverCallback(const webots_ros::StringStamped::ConstPtr &value) 
{
    char *message = const_cast<char *>(value->data.c_str());
    ROS_INFO("Received a message %s.", message);
    callbackCalled = true;
}

int main(int argc, char **argv) 
{
    std::string model_name = "DJI_MAVIC_2_PRO";
    //std::string model_name = "rec";
    std::string controllerName;
    std::vector<std::string> deviceList;
    // create a node named 'robot_information_parser' on ROS network    
    ros::init(argc, argv, "basico");//, ros::init_options::AnonymousName);
    ros::NodeHandle n;

    signal(SIGINT, quit);

    // Wait for the `ros` controller.
    ros::service::waitForService("/robot/time_step");
    ros::spinOnce();

    ROS_INFO("hmmmmmmm");

    // call get_model services to get more general information about the robot
    ros::ServiceClient getModelClient = n.serviceClient<webots_ros::get_string>("/robot/get_model");
    webots_ros::get_string getModelSrv;

    if (getModelClient.call(getModelSrv)) 
    {
        if (!getModelSrv.response.value.empty())
        {
            ROS_INFO("The model of this robot is %s.", getModelSrv.response.value.c_str());
            //model_name = getModelSrv.response.value.c_str();
            time_step_client = n.serviceClient<webots_ros::set_int>(model_name + "/robot/time_step");

        }
        else
            ROS_ERROR("The robot doesn't seem to have a model.");
    } 
    else
        ROS_ERROR("Could not get the model of this robot.");

    // call deviceList service to get the list of the name of the devices available on the controller and print it
    // the deviceListSrv object contains 2 members: request and response. Their fields are described in the corresponding .srv
    // file
    ros::ServiceClient deviceListClient = n.serviceClient<webots_ros::robot_get_device_list>("/robot/get_device_list");
    webots_ros::robot_get_device_list deviceListSrv;

    if (deviceListClient.call(deviceListSrv)) 
    {
        deviceList = deviceListSrv.response.list;
        ROS_INFO("The controller has %lu devices availables:", deviceList.size());
        for (unsigned int i = 0; i < deviceList.size(); i++)
            ROS_INFO("Device [%d]: %s.", i, deviceList[i].c_str());
    } else
        ROS_ERROR("Failed to call service deviceList.");

    ros::ServiceClient set_receiver_client;
    webots_ros::set_int receiver_srv;
    ros::Subscriber sub_receiver_32;    
    set_receiver_client = n.serviceClient<webots_ros::set_int>("/receiver/enable");

    ros::ServiceClient sampling_period_receiver_client;
    webots_ros::get_int sampling_period_receiver_srv;
    sampling_period_receiver_client = n.serviceClient<webots_ros::get_int>(model_name + "/receiver/get_sampling_period");

    ROS_INFO("BLABALBALBa   ");

    receiver_srv.request.value = 32;
    if (set_receiver_client.call(receiver_srv) && receiver_srv.response.success) 
    {
        ROS_INFO("Receiver enabled.");
        sub_receiver_32 = n.subscribe(model_name + "/receiver/data", 1, receiverCallback);
        callbackCalled = false;
        
        while (sub_receiver_32.getNumPublishers() == 0 || !callbackCalled) 
        {
            ros::spinOnce();
            time_step_client.call(time_step_srv);
        }
    } 
    else 
    {
        if (!receiver_srv.response.success)
            ROS_ERROR("Sampling period is not valid.");
        
        ROS_ERROR("Failed to enable receiver.");
        
        return 1;
    }

    sub_receiver_32.shutdown();
    set_receiver_client.shutdown();
    time_step_client.call(time_step_srv);

    while(1);

}