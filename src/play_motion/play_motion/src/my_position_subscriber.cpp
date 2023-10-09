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
#include <sensor_msgs/JointState.h>

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
void positionCallback(const sensor_msgs::JointState& msg);
void KalmanFilterE();

//Metodo principal
int main(int argc, char** argv)
{
    //Registar nodo
    ros::init(argc, argv, "position_control");

    //Crear ROS Nodehandle
    ros::NodeHandle n("~");
 
    //Subscriber
    ros::Subscriber pos_sub = n.subscribe("/joint_states", 1 , positionCallback);

    std::cout << "Configuration ready" << std::endl;

    //Ros rate
    ros::Rate loop_rate(10);

    //Debug Hello world que se va a quitar despues
    std::cout << "Hello world 1" << std::endl;

    //Main loop
    while(ros::ok())
    {   
        //Run callbacks
        ros::spinOnce();

        //Funcion Auxiliar para algo mas
        KalmanFilterE();

        //Delay
        loop_rate.sleep();
    }

    // ros::spin();

    std::cout << "Hello world 2" << std::endl;
}

//Callbacks
void positionCallback(const sensor_msgs::JointState& msg)
{
    // std::cout << msg.joints[1].name << ": " << msg.joints[1].position << std::endl;
    for (int i = 0; i < msg.name.size(); i++) {
        std::cout << msg.name[i] << ": " << msg.position[i] << "\n";
    }
    //El tamaño de todo es 30
    //Comienza en 0
    //Va de la 16 a la 28 las piernas
}

void KalmanFilterE()
{
    std::cout << "Hello World desde otra funcion" << std::endl;
}
