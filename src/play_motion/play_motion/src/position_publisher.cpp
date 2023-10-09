/*
 * Copyright 2021 Edgar Macias, Humanoid Robotics Group, 
 * Automatic Control Laboratory, Centro de Investigación y Estudios 
 * Avanzados del Instituto Politécnico Nacional, Jalisco, Mexico.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

/*
 * @file   yaml_position.cpp
 * @Author Edgar Macias (edgar.macias@cinvestav.mx)
 * @date   April, 2021
 * @brief  This node allows to manipulate the joints of reemC gazebo simulator through a .yaml file.
*/

//Import basic libraries
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <cmath>

//Reemc services
#include "play_motion/move_joint_group.h"
#include <play_motion_msgs/PlayMotionResult.h>

//ROS messages
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/duration.h>

//Additional libraries
#include <yaml-cpp/yaml.h>

//Ros messages
#include <std_msgs/Bool.h>
#include <whole_body_state_msgs/JointState.h>
#include <whole_body_state_msgs/WholeBodyTrajectory.h>
#include <whole_body_state_msgs/WholeBodyState.h>

struct articulation
{
    std::string cont_name;
    play_motion::MoveJointGroup* client;
    std::vector<std::string> names;
    std::vector<double> position;
    std::vector<double> speed;
    std::vector<double> torque;
};


//Globals
std::string node_path;
std::vector<articulation> body_art;
ros::Publisher pos_pub;

//Metodo principal
int main(int argc, char** argv)
{
    //Registar nodo
    ros::init(argc, argv, "position_control");

    //Crear ROS Nodehandle
    ros::NodeHandle n("~");

    //Path params
	n.param<std::string>( "node_path", node_path , "/home/paths");

    //Setup publishers
    pos_pub = n.advertise<whole_body_state_msgs::WholeBodyState>("/reemc/positions", 1);

    //Fijar tasa de publicacion
    ros::Rate loop_rate(10);

    //Ciclo principal
    while(ros::ok())
    {
        //Read new yaml
	    YAML::Node nconf = YAML::LoadFile(node_path + "/config/position_manipulation.yaml");

	    //Get joints number
	    int a_number = nconf["art_number"].as<int>();

        //Init full message
        whole_body_state_msgs::WholeBodyState position_msg;

        //Get parameters for articulations
	    for(int i = 0; i < a_number; i++)
	    {
            //Joints number
            int j_number = nconf["art_" + std::to_string(i)]["joints"].as<int>();


            //Joint loop
            for(int j = 0; j < j_number; j++)
            {
                
                //Init joint element
                whole_body_state_msgs::JointState art_sample;

                //Joint name
                art_sample.name = nconf["art_" + std::to_string(i)]["j_" + std::to_string(j)]["name"].as<std::string>();
        
                //Position value
                art_sample.position = nconf["art_" + std::to_string(i)]["j_" + std::to_string(j)]["position"].as<double>(); 
                
                //effort en lugar de position

                //Save on vector
                position_msg.joints.push_back(art_sample);
            }			
        }

        //Publish position
        pos_pub.publish(position_msg);
        
        //Esperar lazo
        loop_rate.sleep();
    }
}