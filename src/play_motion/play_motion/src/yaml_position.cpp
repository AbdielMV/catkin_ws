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

//Metodo principal
int main(int argc, char** argv)
{
    //Registar nodo
    ros::init(argc, argv, "position_control");

    //Crear ROS Nodehandle
    ros::NodeHandle n("~");

    //Path params
	n.param<std::string>( "node_path", node_path , "/home/paths");

    //Read YAML file
	YAML::Node econf = YAML::LoadFile(node_path + "/config/position_manipulation.yaml");

    std::cout << node_path + "/config/position_manipulation.yaml" << std::endl;
 
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

    //Fijar tasa de publicacion
    ros::Rate loop_rate(2);

    //Ciclo principal
    while(ros::ok())
    {
        //Read new yaml
	    YAML::Node nconf = YAML::LoadFile(node_path + "/config/position_manipulation.yaml");

        //Get parameters for articulations
        for(int i = 0; i < body_art.size(); i++)
        {
            //Generate trajectory according yaml
            std::vector<trajectory_msgs::JointTrajectoryPoint> traj;
            trajectory_msgs::JointTrajectoryPoint msg; 

            for(int j = 0; j < body_art[i].position.size(); j++)
            {
                //Setup new position
                body_art[i].position[j] = nconf["art_" + std::to_string(i)]["j_" + std::to_string(j)]["position"].as<double>();
            }

            msg.positions = body_art[i].position;
            msg.time_from_start = ros::Duration(0.5);
            traj.push_back(msg);

            //Send info
            body_art[i].client->sendGoal(traj);
        }
        
        //Esperar lazo
        loop_rate.sleep();
    }
}