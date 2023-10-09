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

//Global variable
std::string name = "leg_left_1_joints";
float position = 0;
float velocity = 0;
float effort = 0;
//Fixed weights (pseudo-control and input control)
float W1_fixed = 0.1;
float W2_fixed = 1;
//Sigmoid function const
float betha = 0.7;

//RHONN class
class Rhonn{
public:
  //Constructor with parameters
  Rhonn(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> c_input, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_input): z_input(c_input), w_now(w_input){
    std::cout << "Object created with values" << std::endl;
  }

  Eigen::Matrix<float, 1, 1> prediction_state (){
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_transposed = w_now.transpose();
    Eigen::Matrix<float, 1, 1> state = w_transposed*z_input;
    return state;
  }

private:
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> z_input;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_now;
};

float activation_function(float state){
  return 1.0 / (1.0 + std::exp(-betha*state));
}

// Callback function for the subscriber
void subscriberCallback(const whole_body_state_msgs::WholeBodyState&  msg) {
  std::cout << "Inside Callback" << std::endl;
  name = msg.joints[0].name;
  position = msg.joints[0].position;
  velocity = msg.joints[0].velocity;
}

int main(int argc, char** argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "identifier");

  // Create a node handle
  ros::NodeHandle nh;

  // Create a subscriber to listen to a topic named "input_topic"
  ros::Subscriber subscriber = nh.subscribe("/robot_states", 10, subscriberCallback);

  // Create a publisher to publish messages on a topic named "output_topic"
  ros::Publisher publisher = nh.advertise<whole_body_state_msgs::WholeBodyState>("/reemc/effort", 10);

  // Set the loop rate (in Hz)
  ros::Rate loop_rate(10);

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

  //Create object Rhonn
  Rhonn neuron_1(C1,W1);

  // Main loop
  while (ros::ok()) {
    // Create a message to publish
    Eigen::Matrix<float, 1, 1>value_prediction_state = neuron_1.prediction_state();
    float value_from_prediction = value_prediction_state(0,0);
    std::cout << "The value of x1 by the rhonn is: " << value_x1_rhonn << std::endl;
    // Publish the message
    //publisher.publish(msg);
    
    // Spin once to process callbacks
    ros::spinOnce();

    // Sleep to maintain the loop rate
    loop_rate.sleep();

    std::cout << "End of while" << std::endl;
  }

  return 0;
}

