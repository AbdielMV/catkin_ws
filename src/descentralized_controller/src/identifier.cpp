#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdio.h>
#include <sstream>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>

#include <whole_body_state_msgs/JointState.h>
#include <whole_body_state_msgs/WholeBodyTrajectory.h>
#include <whole_body_state_msgs/WholeBodyState.h>
#include <whole_body_state_msgs/RhonnState.h>

#include "Rhonn.h" // Include the Rhonn header
#include "Efk.h"   // Include the Efk header

//Node class
class MyNode{
  public:
    MyNode(ros::NodeHandle& nh) : loop_rate(100) {
      // Initialize ROS node handle, subscriber, and publisher
      nh_ = nh;
      subscriber = nh_.subscribe("/robot_states", 1, &MyNode::subscriberCallback, this);
      publisher = nh_.advertise<whole_body_state_msgs::WholeBodyState>("/reemc/efforts", 1);

      // Init RHONN values (inputs and weights) C ares inputs and W weights
      Eigen::Matrix<float, 2, 1> C1;
      C1 << 1,
            1;
      Eigen::Matrix<float, 3, 1> C2;
      C2 << 1,
            1,
            1;
      // Eigen::Matrix<float, 2, 1> W1 = C1;
      // Eigen::Matrix<float, 3, 1> W2 = C2;
      Eigen::Matrix<float, 2, 1> W1;
/*       W1 << 3.66483,
            -1.83241; */
      W1 << 0,
            0;
      Eigen::Matrix<float, 3, 1> W2;
/*       W2 << 6.64743,
            -13.3101,
            6.64774; */
      W2 << 0,
            0,
            0;
      position = 0;
      position_y1 = 0;
      position_y = 0;
      position_1 = 0;
      velocity = 0;
      velocity_y1 = 0;
      velocity_y = 0;
      velocity_1 = 0;

      // Init values of EFK trainning (P, Q and R)
/*       Eigen::Matrix<float, 1, 1> R1 = 1e-8*Eigen::Matrix<float, 1, 1>::Identity();
      Eigen::Matrix<float, 2, 2> Q1 = 1e-7*Eigen::Matrix<float, 2, 2>::Identity();
      Eigen::Matrix<float, 2, 2> P1 = 1e-10*Eigen::Matrix<float, 2, 2>::Identity();

      Eigen::Matrix<float, 1, 1> R2 = 1e-8*Eigen::Matrix<float, 1, 1>::Identity();
      Eigen::Matrix<float, 3, 3> Q2 = 1e-7*Eigen::Matrix<float, 3, 3>::Identity();
      Eigen::Matrix<float, 3, 3> P2 = 1e-10*Eigen::Matrix<float, 3, 3>::Identity(); */

      Eigen::Matrix<float, 1, 1> R1 = 1e8*Eigen::Matrix<float, 1, 1>::Identity();
      Eigen::Matrix<float, 2, 2> Q1 = 1e7*Eigen::Matrix<float, 2, 2>::Identity();
      Eigen::Matrix<float, 2, 2> P1 = 1e10*Eigen::Matrix<float, 2, 2>::Identity();

      Eigen::Matrix<float, 1, 1> R2 = 1e3*Eigen::Matrix<float, 1, 1>::Identity();
      Eigen::Matrix<float, 3, 3> Q2 = 1e3*Eigen::Matrix<float, 3, 3>::Identity();
      Eigen::Matrix<float, 3, 3> P2 = 1e10*Eigen::Matrix<float, 3, 3>::Identity();

      // Create object Rhonn
      neuron_1 = Rhonn(C1,W1,1);
      neuron_2 = Rhonn(C2,W2,2);

      // Create an object of Efk and update the value of w_weight (HERE CREATE CLASS EFK)
      efk_object_1 = Efk(R1,P1,Q1);
      efk_object_2 = Efk(R2,P2,Q2);
    }

    void subscriberCallback(const whole_body_state_msgs::WholeBodyState& msg) {
      // Create a message to publish
      whole_body_state_msgs::WholeBodyState position_msg;
      whole_body_state_msgs::JointState joint_estimation;
      whole_body_state_msgs::RhonnState rhonn_estimation;

      //for (size_t i = 0; i < 30; i++)
      //{
            // Process the incoming message
            //arm_right_1 is the 21 of 30 joints
            name = msg.joints[21].name;
            position = (msg.joints[21].position*180)/M_PI;
            velocity = (msg.joints[21].velocity*180)/M_PI;

            position_y = position_y1*0.99+position_1*0.00995;
            velocity_y = velocity_y1*0.99+velocity_1*0.00995;
            position_1 = position;
            velocity_1 = velocity;
            position_y1 = position_y;
            velocity_y1 = velocity_y;

            //Change of variable for use those names
            position = position_y;
            velocity = velocity_y;

            // Create a message to publish
            std::cout << "Neurona 1" << std::endl;
            observer_1_value = neuron_1.observer_state(position,velocity);
            neuron_1_value = neuron_1.prediction_state(position, velocity);
            //rhonn_estimation.error_w1 = efk_object_1.error_estimation(neuron_1_value,position,velocity);
            error_1 = efk_object_1.error_estimation(neuron_1_value,position,velocity,1);
            efk_object_1.calculate_new_weights(neuron_1);

            std::cout << "Neurona 2" << std::endl;
            neuron_2.fx0_value(neuron_1);
            observer_2_value = neuron_2.observer_state(position,velocity);
            neuron_2_value = neuron_2.prediction_state(position, observer_2_value);
            //rhonn_estimation.error_w2 = efk_object_2.error_estimation(neuron_2_value,position,velocity);
            error_2 = efk_object_2.error_estimation(neuron_2_value,position,velocity,2);
            efk_object_2.calculate_new_weights(neuron_2);
            neuron_2.control_law(neuron_1, position, velocity);

            //Construction of Message
            rhonn_estimation.name = name;
            rhonn_estimation.position = neuron_1_value;
            rhonn_estimation.velocity = neuron_2_value;
            rhonn_estimation.error_w1 = error_1;
            rhonn_estimation.error_w2 = error_2;
            rhonn_estimation.obs_position = observer_1_value;
            rhonn_estimation.obs_velocity = observer_2_value;

/*             std::cout << "Building message" << std::endl;
            std::cout << "Valor X1_k+1 = " << neuron_1_value << std::endl;
            std::cout << "Valor X2_k+1 = " << neuron_2_value << std::endl; */

            joint_estimation.name = name;
            joint_estimation.position = position;
            joint_estimation.velocity = velocity;
            joint_estimation.effort = neuron_2.getControlLaw();

            //Has to be JointState (joints) NOT Rhonn (rhonn)
            position_msg.joints.push_back(joint_estimation);
            position_msg.rhonn.push_back(rhonn_estimation);
      //}
      
      // Publish the message
      position_msg.header.stamp = ros::Time::now();
      publisher.publish(position_msg);
    }

    void run() {
      // Main loop
      while (ros::ok()) {
          // Call subscriber callback explicitly
          ros::spinOnce();

          // Sleep to maintain the loop rate
          loop_rate.sleep();
      }
    }

    private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber;
    ros::Publisher publisher;
    ros::Rate loop_rate;
    std::string name;
    float position;
    float position_y1;
    float position_y;
    float position_1;
    float velocity;
    float velocity_y1;
    float velocity_y;
    float velocity_1;
    float neuron_1_value;
    float observer_1_value;
    float neuron_2_value;
    float observer_2_value;
    float error_1;
    float error_2;
    Rhonn neuron_1;
    Rhonn neuron_2;
    Efk efk_object_1;
    Efk efk_object_2;
};

//Main Code Start
int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "identifier");

    //Create a ROS node handle
    ros::NodeHandle nh;

    // Create an instance of MyNode
    MyNode myNode(nh);

    // Run the node
    myNode.run();

    return 0;
}