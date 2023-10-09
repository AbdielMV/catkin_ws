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
#include <vector>
#include <Eigen/Dense>

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

//Globals ROS
std::string node_path;

//Variables para obtener la posicion y nombre de Gazebo
std::vector<std::string> joints_names;
std::vector<double> joints_positions;
std::vector<double> joints_velocities;

//Variables para calcular el Filtro de Kalman
//Parte 1
Eigen::MatrixXd weights_now_1;
Eigen::Matrix2d P_matrix_now_1;
Eigen::Matrix<double, 2, 1> H_1;
Eigen::Matrix<double, 1, 1> R_1;
Eigen::Matrix2d Q_1;
Eigen::MatrixXd M_1_partial;
Eigen::MatrixXd M_1;
Eigen::MatrixXd K_1;
Eigen::MatrixXd weights_next_1;
Eigen::Matrix2d P_matrix_next_1;
Eigen::Matrix2d Q_1_init;
//Parte 2
Eigen::MatrixXd weights_now_2;
Eigen::Matrix2d P_matrix_now_2;
Eigen::Matrix<double, 2, 1> H_2;
Eigen::Matrix<double, 1, 1> R_2;
Eigen::Matrix2d Q_2;
Eigen::MatrixXd M_2_partial;
Eigen::MatrixXd M_2;
Eigen::MatrixXd K_2;
Eigen::MatrixXd weights_next_2;
Eigen::Matrix2d P_matrix_next_2;
Eigen::Matrix2d Q_2_init;

//Variables para calcular los estados estimados
Eigen::Matrix<double, 1, 1> xn_1;
Eigen::Matrix<double, 1, 1> xn_2;
Eigen::Matrix<double, 1, 1> x_1;
Eigen::Matrix<double, 1, 1> x_2;

//Variables para calcular la ley de control
Eigen::Matrix<double, 1, 1> u {1};

//Variable para la funcion de activacion
double Sigmoidal;

//Construccion del mensaje a publicar
sensor_msgs::JointState estimated_states;


//Auxiliar functions
void positionCallback(const sensor_msgs::JointState& msg);
void KalmanFilterE();
double ActivationFunction(Eigen::Matrix<double, 1, 1> estado_actual);
void Emptyvector_Names_Positions();
void RHONN();

//Metodo principal
int main(int argc, char** argv)
{
    //Registar nodo
    ros::init(argc, argv, "identificador_controlador");

    //Crear ROS Nodehandle
    ros::NodeHandle n("~");

    //Publisher
    ros::Publisher pos_pub = n.advertise<sensor_msgs::JointState>("chatter",1000);
 
    //Subscriber
    ros::Subscriber pos_sub = n.subscribe("/joint_states", 1 , positionCallback);

    std::cout << "Configuration ready" << std::endl;

    //Ros rate
    ros::Rate loop_rate(1000);

    //Inicializar variables
    weights_now_1 = Eigen::MatrixXd::Random(2,1);
    weights_now_2 = Eigen::MatrixXd::Random(2,1);
    xn_1 = Eigen::MatrixXd::Random(1,1);
    xn_2 = Eigen::MatrixXd::Random(1,1);
    P_matrix_now_1 << 1,0,
                      0,1;
    Q_1_init << 1,0,
                0,1;
    P_matrix_now_2 << 1,0,
                      0,1;
    Q_2_init << 1,0,
                0,1;

    //Debug Hello world que se va a quitar despues
    std::cout << "Starting loop" << std::endl;

    //Main loop
    while(ros::ok())
    {   
        //Run callbacks
        ros::spinOnce();

        //Publicacion del mensaje por ROS
        pos_pub.publish(estimated_states);

        //Delay
        loop_rate.sleep();
    }

    //Debug Hello world que se va a quitar despues
    std::cout << "\nEnding program" << std::endl;
}

//Callbacks
void positionCallback(const sensor_msgs::JointState& msg)
{
    std::cout << "Callback Suscriber" << std::endl;
    //La resta de -2 es para solamente tomar en consideración los 12 grados de libertad de la pierna
    //En caso de realizar a todo el cuerpo es mejor utilizar el valor completo
    int number_of_joints = 0;
    number_of_joints = msg.name.size()-2;
    for (int i = 16; i < number_of_joints; i++) {
        joints_names.push_back(msg.name[i]);
        joints_positions.push_back(msg.position[i]);
        joints_velocities.push_back(msg.velocity[i]);
    }
    //El tamaño de todo es 30
    //Comienza en 0
    //Va de la 16 a la 28 las piernas

    estimated_states.name = joints_names;
    estimated_states.position = xn_1;
    estimated_states.velocity = xn_2;

/*     for (int i = 0; i < joints_names.size(); i++) {
        x_1(0,0) = joints_positions[i];
        x_2(0,0) = joints_velocities[i];
        KalmanFilterE();
        RHONN();
    }
    std::cout << "Finishing loop for all values saved in joints_names" << std::endl; */

    x_1(0,0) = joints_positions[0];
    x_2(0,0) = joints_velocities[0];
    KalmanFilterE();
    RHONN();
    Emptyvector_Names_Positions();
    std::cout << "Finishing Callback Suscriber" << std::endl;
}

//Funciones auxiliares
void KalmanFilterE()
{
    //Parte 1 para R1 y Q1
    std::cout << "Kalman Filter P1" << std::endl;
    R_1 << 1e1; //Matriz de covarianza ruido del estado
    Q_1 << Q_1_init * 1e1; //Matriz de covarianza ruido de medicion
    //Cuando se usa 1e2 eso es 10, cuando se usa exp(1) es euler elevado a la 1
    H_1(0,0) = ActivationFunction(x_1); //En H(1,0) es fila primero y despues es columna; comienza en (0,0)
    H_1(1,0) = 1;
    M_1_partial = ((H_1.transpose()*P_matrix_now_1*H_1) + R_1);
    M_1 = M_1_partial.inverse();
    K_1 = P_matrix_now_1*H_1*M_1;
    weights_next_1 = weights_now_1 + (0.5*K_1*(x_1 - xn_1)); //Falta el error de identificacion que es el 0.01
    P_matrix_next_1 = P_matrix_now_1 - (K_1*H_1.transpose()*P_matrix_now_1) + Q_1; //Falta Q, P es 2x2 y Q es 1x1

    //Parte 2 para R2 y Q2
    std::cout << "Kalman Filter P2" << std::endl;
    R_2 << 1e1;
    Q_2 << Q_2_init * 1e1;
    H_2(0,0) = ActivationFunction(x_2);
    H_2(1,0) = 1;
    M_2_partial = ((H_2.transpose()*P_matrix_now_2*H_2) + R_2);
    M_2 = M_2_partial.inverse();
    K_2 = P_matrix_now_2*H_2*M_2;
    weights_next_2 = weights_now_2 + (0.5*K_2*(x_2 - xn_2));
    P_matrix_next_2 = P_matrix_now_2 - (K_2*H_2.transpose()*P_matrix_now_2) + Q_2;

    weights_now_1 = weights_next_1;
    P_matrix_now_1 = P_matrix_next_1;
    weights_now_2 = weights_next_2;
    P_matrix_now_2 = P_matrix_next_2;
}

void RHONN()
{
    std::cout << "States Estimated by RHONN" << std::endl;
    Eigen::Matrix<double, 1, 1> weight_fix_1 {0.01}; //Utilizar {} es en filas y = {} es en columnas
    Eigen::Matrix<double, 1, 1> weight_fix_2 {0.01};
    xn_1(0,0) = (weights_now_1(0,0) * ActivationFunction(x_1)) + weights_now_1(1,0) + (weight_fix_1 * x_2);
    xn_2(0,0) = (weights_now_2(0,0) * ActivationFunction(x_2)) + weights_now_2(1,0) + (weight_fix_2 * u);
}

double ActivationFunction(Eigen::Matrix<double, 1, 1> estado_actual)
{
    double sigma = estado_actual(0,0);
    std::cout << "Activation Function" << std::endl;
    float beta = 0.5;
    Sigmoidal = 1/(1 + exp(-beta*sigma));

    return Sigmoidal;
}

void Emptyvector_Names_Positions()
{
    std::cout << "Cleaning vectors" << std::endl;
    joints_names.clear();
    joints_positions.clear();
    joints_velocities.clear();
}
