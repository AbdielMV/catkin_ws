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

//Metodo principal
int main(int argc, char** argv)
{
    //Registar nodo
    ros::init(argc, argv, "position_control");

    //Crear ROS Nodehandle
    ros::NodeHandle n("~");

    //Init controller information
    const std::string cont_name = "left_arm_controller";
    //std::vector<std::string> joint_names; 

    std::cout << "controller name:" << cont_name << std::endl;

    //Get joint names
    const std::vector<std::string> joint_names = std::vector<std::string>{std::string{"arm_left_1_joint"}, 
                                                                          std::string{"arm_left_2_joint"}, 
                                                                          std::string{"arm_left_3_joint"}, 
                                                                          std::string{"arm_left_4_joint"}, 
                                                                          std::string{"arm_left_5_joint"}, 
                                                                          std::string{"arm_left_6_joint"}, 
                                                                          std::string{"arm_left_7_joint"}};

    //Init control service
    play_motion::MoveJointGroup* control_service = new play_motion::MoveJointGroup(cont_name, joint_names);

    //Fijar tasa de publicacion
    ros::Rate loop_rate(2);

    int time_loop = 0;

    //Ciclo principal
    while(ros::ok())
    {
        //Build example trajectory
        std::vector<trajectory_msgs::JointTrajectoryPoint> traj;
        trajectory_msgs::JointTrajectoryPoint msg; 
        msg.positions = {sin(0.1*time_loop), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        msg.time_from_start = ros::Duration(0.5);
        traj.push_back(msg);

        //Send info
        control_service->sendGoal(traj);

        time_loop++;
        
        //Esperar lazo
        loop_rate.sleep();
    }
}