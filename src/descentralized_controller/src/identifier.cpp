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


//Global variable
//std::string name = "leg_left_1_joints";
//float position = 0;
//float velocity = 0;
//float error_position = 0;
//float error_velocity = 0;
//Fixed weights (pseudo-control and input control)
//Sigmoid function const
//Design factor
//Control law
//float u = 1;

//Function declaration
// float activation_function(float state);
// void subscriberCallback(const whole_body_state_msgs::WholeBodyState& msg);

//RHONN class
/* class Rhonn{
public:
  //Default constructor
  Rhonn(){
    //WTF?
  }
  //Constructor with parameters
  Rhonn(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> c_input, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_input, int number): z_input(c_input), w_weight(w_input), neuron(number){
    std::cout << "Object RHONN created with values" << std::endl;
  }

  float prediction_state (float position, float velocity){
    W1_fixed = 0.0001;
    W2_fixed = 1;
    u = 0.0001;
    float fixed_result;
    float state_final_prediction;
    update_input(position,velocity);
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

  void update_input(float position, float velocity){
    if (neuron == 1)
    {
      z_input(0,0) = activation_function(position);
    }else{
      z_input(0,0) = activation_function(velocity);
      z_input(1,0) = activation_function(position);
      z_input(2,0) = activation_function(velocity);
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
  float W1_fixed;
  float W2_fixed;
  float u;
}; */

// Another class (or any external code) that wants to modify w_weight HERE GOES THE EFK
/* class Efk {
public:
  Efk(){
    //WTF? x2
  }
  Efk(Eigen::Matrix<float, 1, 1> r_value, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> p_value, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> q_value): r(r_value), p_now(p_value), q_now(q_value){
    std::cout << "Object EFK created with values" << std::endl;
  }

  void calculate_new_weights(Rhonn& rhonn_object) {
    etha = 0.75;
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

  float error_estimation(float rhonn_state_value, float position, float velocity){ //Se calcula el valor del error y se coloca en vector (esto pudiera no ser necesario)
    if (neuron == 1)
    {
      Eigen::Matrix<float, 1, 1> e_position = (position*Eigen::Matrix<float, 1, 1>::Identity()) - (rhonn_state_value*Eigen::Matrix<float, 1, 1>::Identity());
      float e_position_grados = (e_position(0,0)*180)/M_PI;
      error_return = e_position_grados;
      e(0,0) = e_position(0,0);
    }else{
      Eigen::Matrix<float, 1, 1> e_velocity = (velocity*Eigen::Matrix<float, 1, 1>::Identity()) - (rhonn_state_value*Eigen::Matrix<float, 1, 1>::Identity());
      float e_velocity_grados = (e_velocity(0,0)*180)/M_PI;
      error_return = e_velocity_grados;
      e(0,1) = e_velocity(0,0);
    }
    return error_return;
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
  float etha;
  float error_return;
}; */

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

/* int main(int argc, char** argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "identifier");

  // Create a node handle
  ros::NodeHandle nh;

  // Create a subscriber to listen to a topic named "input_topic"
  ros::Subscriber subscriber = nh.subscribe("/robot_states", 10, subscriberCallback);

  // Create a publisher to publish messages on a topic named "output_topic"
  ros::Publisher publisher = nh.advertise<whole_body_state_msgs::WholeBodyState>("/reemc/effort", 10);

  // Set the loop rate (in Hz)
  ros::Rate loop_rate(10000);

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
    neuron_1_value = neuron_1.prediction_state();
    efk_object_1.error_estimation(neuron_1_value);
    efk_object_1.calculate_new_weights(neuron_1);
    art_estimation.name = name;
    art_estimation.position = neuron_1_value;
    art_estimation.error_w1 = error_position;

    neuron_2_value = neuron_2.prediction_state();
    efk_object_2.error_estimation(neuron_2_value);
    efk_object_2.calculate_new_weights(neuron_2);
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
} */

/* // Callback function for the subscriber
void subscriberCallback(const whole_body_state_msgs::WholeBodyState& msg) {
  name = msg.joints[0].name;
  position = msg.joints[0].position;
  velocity = msg.joints[0].velocity;
} */

