#include <ros/ros.h>
#include <std_msgs/String.h>

#include <stdio.h>
#include <sstream>
#include <cmath>
#include <Eigen/Dense>

#include <whole_body_state_msgs/JointState.h>
#include <whole_body_state_msgs/WholeBodyTrajectory.h>
#include <whole_body_state_msgs/WholeBodyState.h>
#include <whole_body_state_msgs/Rhonn.h>

#include "Rhonn.h" // Include the Rhonn header
#include "Efk.h"   // Include the Efk header

//Node class
class MyNode{
  public:
    MyNode(ros::NodeHandle& nh) : loop_rate(10000) {
      // Initialize ROS node handle, subscriber, and publisher
      nh_ = nh;
      subscriber = nh_.subscribe("/robot_states", 100, &MyNode::subscriberCallback, this);
      publisher = nh_.advertise<whole_body_state_msgs::WholeBodyState>("/reemc/effort", 100);

      // Init RHONN values (inputs and weights) C ares inputs and W weights
      Eigen::Matrix<float, 2, 1> C1;
      C1 << 1,
            1;
      Eigen::Matrix<float, 3, 1> C2;
      C2 << 1,
            1,
            1;
      Eigen::Matrix<float, 2, 1> W1 = C1;
      Eigen::Matrix<float, 3, 1> W2 = C2;

      // Init values of EFK trainning (P, Q and R)
      Eigen::Matrix<float, 1, 1> R1 = 1e-8*Eigen::Matrix<float, 1, 1>::Identity();
      Eigen::Matrix<float, 2, 2> Q1 = 1e-7*Eigen::Matrix<float, 2, 2>::Identity();
      Eigen::Matrix<float, 2, 2> P1 = 1e-10*Eigen::Matrix<float, 2, 2>::Identity();

      Eigen::Matrix<float, 1, 1> R2 = 1e-8*Eigen::Matrix<float, 1, 1>::Identity();
      Eigen::Matrix<float, 3, 3> Q2 = 1e-7*Eigen::Matrix<float, 3, 3>::Identity();
      Eigen::Matrix<float, 3, 3> P2 = 1e-10*Eigen::Matrix<float, 3, 3>::Identity();

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
      whole_body_state_msgs::Rhonn art_estimation;

      for (size_t i = 0; i < 30; i++)
      {
        // Process the incoming message
        name = msg.joints[i].name;
        position = msg.joints[i].position;
        velocity = msg.joints[i].velocity;

        // Create a message to publish
        std::cout << "Neurona 1" << std::endl;
        neuron_1_value = neuron_1.prediction_state(position, velocity);
        art_estimation.error_w1 = efk_object_1.error_estimation(neuron_1_value,position,velocity);
        efk_object_1.calculate_new_weights(neuron_1);
        art_estimation.name = name;
        art_estimation.position = neuron_1_value;

        std::cout << "Neurona 2" << std::endl;
        neuron_2_value = neuron_2.prediction_state(position, velocity);
        art_estimation.error_w2 = efk_object_2.error_estimation(neuron_2_value,position,velocity);
        efk_object_2.calculate_new_weights(neuron_2);
        art_estimation.velocity = neuron_2_value;

        position_msg.rhonn.push_back(art_estimation);

      }
      
      // Publish the message
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
    float velocity;
    float neuron_1_value;
    float neuron_2_value;
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