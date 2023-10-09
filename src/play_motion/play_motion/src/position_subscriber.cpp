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
    bool moving;
};

//Globals
std::string node_path;
std::vector<articulation> body_art;

//Auxiliar functions
void positionCallback(const whole_body_state_msgs::WholeBodyState& msg);

//Metodo principal
int main(int argc, char** argv)
{
    //Registar nodo
    ros::init(argc, argv, "position_control");

    //Crear ROS Nodehandle
    ros::NodeHandle n("~");

    //Path params
	n.param<std::string>( "node_path", node_path , "/home/paths");
 
    //Subscriber
    ros::Subscriber pos_sub = n.subscribe("/reemc/posiciones", 1 , positionCallback);

    //Read YAML file
	YAML::Node econf = YAML::LoadFile(node_path + "/config/position_manipulation.yaml");

    //Get joints number
	int a_number = econf["art_number"].as<int>();

    std::cout << "number of articulations: " << a_number << std::endl;

    //Get parameters for articulations
	for(int i = 0; i < a_number; i++)
	{
        articulation art_sample;

        //Control name
        art_sample.cont_name = econf["art_" + std::to_string(i)]["control_name"].as<std::string>();

        //Joints number
        int j_number = econf["art_" + std::to_string(i)]["joints"].as<int>();

        //Joint loop
        for(int j = 0; j < j_number; j++)
        {
            //Joint name
            std::string name = econf["art_" + std::to_string(i)]["j_" + std::to_string(j)]["name"].as<std::string>();
        
            //Position value
            double position = econf["art_" + std::to_string(i)]["j_" + std::to_string(j)]["position"].as<double>();

            //Save on vector
            art_sample.names.push_back(name);
            art_sample.position.push_back(position);
        }			

        //Init control service
        art_sample.client = new play_motion::MoveJointGroup(art_sample.cont_name, art_sample.names);
    
        //Save articulation
        body_art.push_back(art_sample);
    }

    std::cout << "Configuration ready" << std::endl;

    //Ros rate
    ros::Rate loop_rate(10);

    //Main loop
    while(ros::ok())
    {   
        //Run callbacks
        ros::spinOnce();

        //Setup position
        for(int i = 0; i < body_art.size(); i++)
        {

            if(body_art[i].moving)
            {
                //Init message
                std::vector<trajectory_msgs::JointTrajectoryPoint> traj;
                trajectory_msgs::JointTrajectoryPoint msg_out;

                //Setup message
                msg_out.positions = body_art[i].position;
                msg_out.time_from_start = ros::Duration(0.5);
                traj.push_back(msg_out);

                //Send info
                body_art[i].client->sendGoal(traj);
            }
        }

        //Delay
        loop_rate.sleep();
    }
}

//Callbacks
void positionCallback(const whole_body_state_msgs::WholeBodyState& msg)
{
    //std::cout << msg.joints[k].name << ": " << msg.joints[k].position << std::endl;
    
    //Artic loop
    for(int i = 0; i < body_art.size(); i++) //TODO: Do for body_art.size() quickly
    {
        //Articulation flag
        body_art[i].moving = false;

        //Generate trajectory according yaml
        //std::vector<trajectory_msgs::JointTrajectoryPoint> traj;
        //trajectory_msgs::JointTrajectoryPoint msg_out; 

        for(int j = 0; j < body_art[i].position.size(); j++)
        {
            for(int k = 0; k < msg.joints.size(); k++)
            {
                //Check joint name
                if(msg.joints[k].name == body_art[i].names[j])
                {
                    body_art[i].position[j] = msg.joints[k].position;
                    body_art[i].moving = true;
                    break;
                } 
            }
        }

/*
        //Setup position
        if(moving)
        {
            msg_out.positions = body_art[i].position;
            msg_out.time_from_start = ros::Duration(0.5);
            traj.push_back(msg_out);

            //Send info
            body_art[i].client->sendGoal(traj);
        }
*/
    }
}