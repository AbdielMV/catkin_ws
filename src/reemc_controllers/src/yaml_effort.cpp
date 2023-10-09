/*
 * Copyright 2020 Edgar Macias, Humanoid Robotics Group, 
 * Automatic Control Laboratory, Research and Advanced Studies Center 
 * of the National Polytechnic Institute, Jalisco, Mexico.
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
 * @file   position_reader.cpp
 * @Author Edgar Macias (edgar.macias@cinvestav.mx)
 * @date   December, 2020
 * @brief  This node allows to read an publish the joint information from a .urdf or .sdf gazebo robot model.
*/

//Ros Libraries
#include <ros/ros.h>

//Standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h> 

//Yaml lecture
#include <yaml-cpp/yaml.h>

//Headers
#include <whole_body_state_msgs/JointState.h>
#include <whole_body_state_msgs/WholeBodyTrajectory.h>
#include <whole_body_state_msgs/WholeBodyState.h>

//Robot articulations
whole_body_state_msgs::WholeBodyState r_joints;
std::vector<std::string> art_names; 
int art_counter;

//Node configuration
std::string node_path;
int mode;

//Init publishers
ros::Publisher effort_pub; 

int main(int argc, char **argv)
{
	std::string effort_topic;

	//Register node
	ros::init(argc, argv, "yaml_effort");
	ros::NodeHandle n;

	//Init gazebo services
	//ros::ServiceClient start_effort_client = n.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort"); 

	//Path params
	n.param<std::string>("yaml_effort/node_path", node_path , "/home/paths");

	//Get node mode
	//n.param("robot_manipulation/mode", mode, 0);

	//Get output topic name
	n.param<std::string>("yaml_effort/robot_configuration/effort_topic", effort_topic, "torques");
	//effort_topic = "/reemc/efforts";

	//Read YAML file
	YAML::Node econf = YAML::LoadFile(node_path + "/config/effort_manipulation.yaml");

	//Processing nodes
	switch(mode)
	{
		case 0:

			//Init publishers
			effort_pub = n.advertise<whole_body_state_msgs::WholeBodyState>(effort_topic, 1);

			//Get joints number
			int j_number = econf["joints"]["number"].as<int>();

			//Get parameters for articulations
			for(int i = 0; i < j_number; i++)
			{
				whole_body_state_msgs::JointState joint;

				//Get joint name
				joint.name = econf["joints"]["j_" + std::to_string(i)]["name"].as<std::string>();

				//Save on vector
				r_joints.joints.push_back(joint);				
			}
	}

	//Ros rate as fps
	ros::Rate loop_rate(100);

	while (ros::ok())
	{
		//Run callbacks
		//ros::spinOnce();

		if(mode == 0)
		{
			for(int i = 0; i < r_joints.joints.size(); i++)
			{
				//Setup effort
				YAML::Node lconf = YAML::LoadFile(node_path + "/config/effort_manipulation.yaml");

				//Setup effort
				r_joints.joints[i].effort = lconf["joints"]["j_" + std::to_string(i)]["effort"].as<float>();
			}

			r_joints.header.stamp = ros::Time::now();
			effort_pub.publish(r_joints);
		}

		//Induced delay
		loop_rate.sleep();
	}
	
	return 0;
}