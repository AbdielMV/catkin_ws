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

//#include "Rhonn.h" // Include the Rhonn header
// #include "Efk.h"   // Include the Efk header


//Global variable
std::string name = "leg_left_1_joints";
float position = 0;
float velocity = 0;
float effort = 0;
float error_position = 0;
float error_velocity = 0;
//Fixed weights (pseudo-control and input control)
float W1_fixed = 1;
float W2_fixed = 1;
//Sigmoid function const
float betha = 0.7;
//Design factor
float etha = 0.6;
//Control law
float u = 1;

////TEMPORAL BORRAR/////////////
//float e_position_grados = 0;//
//////////////////////////////

//Function declaration
float activation_function(float state);
void subscriberCallback(const whole_body_state_msgs::WholeBodyState& msg);

//RHONN class
class Rhonn{
public:
  //Constructor with parameters
  Rhonn(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> c_input, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_input, int number): z_input(c_input), w_weight(w_input), neuron(number){
    std::cout << "Object RHONN created with values" << std::endl;
  }

  float prediction_state (){
    float fixed_result;
    float state_final_prediction;
    update_input();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_transposed = w_weight.transpose();
    Eigen::Matrix<float, 1, 1> state = w_transposed*z_input;
    float value_from_prediction = state(0,0);
    if (neuron == 1)
    {
      fixed_result = W1_fixed*velocity;
      state_final_prediction = value_from_prediction+fixed_result;
    }else{
      fixed_result = W2_fixed*u;
      state_final_prediction = value_from_prediction+fixed_result;
    }
    return state_final_prediction;
  }

  void update_input(){
    if (neuron == 1)
    {
      z_input(0,0) = activation_function(position);
    }else{
      z_input(0,0) = activation_function(velocity);
      z_input(1,0) = activation_function(velocity);
    }
  }

  void updateWeights(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& new_weights){
    w_weight = new_weights;
  }

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> getInputs(){
    return z_input;
  }

  int getNeuron(){
    return neuron;
  }

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> getWeights(){
      return w_weight;
    }

private:
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> z_input;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_weight;
  int neuron;
};

// Another class (or any external code) that wants to modify w_weight HERE GOES THE EFK
class Efk {
public:
  Efk(Eigen::Matrix<float, 1, 1> r_value, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> p_value, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> q_value): r(r_value), p_now(p_value), q_now(q_value){
    std::cout << "Object EFK created with values" << std::endl;
  }

  void calculate_new_weights(Rhonn& rhonn_object) {
    h = rhonn_object.getInputs();
    neuron = rhonn_object.getNeuron();
    w_now = rhonn_object.getWeights();
    m = (r + (h.transpose() * p_now * h)).inverse();
    k = p_now*h*m;
    if (neuron == 1)
    {
      w_next = w_now + (etha*k*e(0,0));
    }else{
      w_next = w_now + (etha*k*e(0,1));
    }
    p_next = p_now - (k*h.transpose()*p_now) + q_now;
    update_values();
    // Use the public member function to update w_weight of the Rhonn object
    rhonn_object.updateWeights(w_now);
  }

  // void weights_prediction(){
  //   m = (r + (h.transpose() * p_now * h)).inverse();
  // }

  void error_estimation(float rhonn_state_value){ //Se calcula el valor del error y se coloca en vector (esto pudiera no ser necesario)
    if (neuron == 1)
    {
      Eigen::Matrix<float, 1, 1> e_position = (position*Eigen::Matrix<float, 1, 1>::Identity()) - (rhonn_state_value*Eigen::Matrix<float, 1, 1>::Identity());
      float e_position_grados = (e_position(0,0)*180)/M_PI;
      std::cout << "Error position: " << e_position_grados << std::endl;
      error_position = e_position_grados;
      e(0,0) = e_position(0,0);
    }else{
      Eigen::Matrix<float, 1, 1> e_velocity = (velocity*Eigen::Matrix<float, 1, 1>::Identity()) - (rhonn_state_value*Eigen::Matrix<float, 1, 1>::Identity());
      float e_velocity_grados = (e_velocity(0,0)*180)/M_PI;
      std::cout << "Error velocity: " << e_velocity_grados << std::endl;
      error_velocity = e_velocity_grados;
      e(0,1) = e_velocity(0,0);
    }
    //std::cout << "Error matrix: " << e << std::endl;
  }

  void update_values(){
    p_now = p_next;
    w_now = w_next;
  }

private:
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> p_now;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> p_next;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> q_now;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> h;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> m;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> k;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_now;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_next;
  Eigen::Matrix<float, 1, 2> e;
  Eigen::Matrix<float, 1, 1> r;
  int neuron;
};

//Main Code Start

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

  // Init values of EFK trainning (P, Q and R)
  Eigen::Matrix<float, 1, 1> R1 = 1e8*Eigen::Matrix<float, 1, 1>::Identity();
  Eigen::Matrix<float, 2, 2> Q1 = 1e7*Eigen::Matrix<float, 2, 2>::Identity();
  Eigen::Matrix<float, 2, 2> P1 = 1e10*Eigen::Matrix<float, 2, 2>::Identity();

  Eigen::Matrix<float, 1, 1> R2 = 1e8*Eigen::Matrix<float, 1, 1>::Identity();
  Eigen::Matrix<float, 3, 3> Q2 = 1e7*Eigen::Matrix<float, 3, 3>::Identity();
  Eigen::Matrix<float, 3, 3> P2 = 1e10*Eigen::Matrix<float, 3, 3>::Identity();

  // Create object Rhonn
  Rhonn neuron_1(C1,W1,1);
  Rhonn neuron_2(C2,W2,2);

  // Create an object of Efk and update the value of w_weight (HERE CREATE CLASS EFK)
  Efk efk_object_1(R1, P1, Q1);
  Efk efk_object_2(R2, P2, Q2);

  //Init values of RHONN
  float neuron_1_value = 0;
  float neuron_2_value = 0; 

  // Main loop
  while (ros::ok()) {

    whole_body_state_msgs::WholeBodyState position_msg;

    whole_body_state_msgs::Rhonn art_estimation;

    // Create a message to publish
    //efk_object.error_estimation(neuron_1_value);
    neuron_1_value = neuron_1.prediction_state();
    //std::cout << "Value of state rhonn x1 -> " << neuron_1_value << std::endl;
    efk_object_1.error_estimation(neuron_1_value);
    efk_object_1.calculate_new_weights(neuron_1);
    //std::cout << "Value of state real x1 -> " << position << std::endl;
    art_estimation.name = name;
    art_estimation.position = neuron_1_value;
    art_estimation.error_w1 = error_position;

    neuron_2_value = neuron_2.prediction_state();
    //std::cout << "Value of state rhonn x2 -> " << neuron_2_value << std::endl;
    efk_object_2.error_estimation(neuron_2_value);
    efk_object_2.calculate_new_weights(neuron_2);
    //std::cout << "Value of state real x2 -> " << velocity << std::endl;
    art_estimation.velocity = neuron_2_value;
    art_estimation.error_w2 = error_velocity;

    position_msg.rhonn.push_back(art_estimation);

    // Publish the message
    publisher.publish(position_msg);
    
    // Spin once to process callbacks
    ros::spinOnce();

    // Sleep to maintain the loop rate
    loop_rate.sleep();
  }

  return 0;
}


//Function definition
float activation_function(float state){
  return 1.0 / (1.0 + std::exp(-betha*state));
}

// Callback function for the subscriber
void subscriberCallback(const whole_body_state_msgs::WholeBodyState& msg) {
  name = msg.joints[0].name;
  position = msg.joints[0].position;
  velocity = msg.joints[0].velocity;
}

