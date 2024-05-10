#include "Rhonn.h"

//Function declaration
float activation_function(float state);

//RHONN class implementation
//Default constructor
Rhonn::Rhonn(){

}
//Constructor with parameters
Rhonn::Rhonn(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> c_input, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_input, int number): z_input(c_input), w_weight(w_input), neuron(number){
    std::cout << "Object RHONN created with values" << std::endl;
    // f_x << 0,
    //        0;
    fx_1 = 0; //Just exist in neuron 1
    fx_2 = 0; //Just exist in neuron 2
    W1_fixed = 0.001;
    W2_fixed = 1;
    u = 0;
}
//Prediction state
float Rhonn::prediction_state (float position, float velocity){
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
      fx_1 = state_final_prediction;
    }else{
      fixed_result = W2_fixed*u;
      state_final_prediction = value_from_prediction+fixed_result;
      fx_2 = value_from_prediction;
      control_law(fx_1,fx_2);
    }
    std::cout << "Valor fx1: " << fx_1 << std::endl;
    std::cout << "Valor fx2: " << fx_2 << std::endl;
    std::cout << "Ley de control u: " << u << std::endl;
    return state_final_prediction;
}

void Rhonn::update_input(float position, float velocity){
    if (neuron == 1)
    {
      z_input(0,0) = activation_function(position);
    }else{
      z_input(0,0) = activation_function(velocity);
      z_input(1,0) = activation_function(position);
      z_input(2,0) = activation_function(velocity);
    }
}

void Rhonn::updateWeights(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& new_weights){
    w_weight = new_weights;
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Rhonn::getInputs(){
  return z_input;
}

int Rhonn::getNeuron(){
    return neuron;
}

Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Rhonn::getWeights(){
    return w_weight;
}

void Rhonn::fx1_value(float x_neuron_1){
  fx_1 = x_neuron_1;
}

float Rhonn::getControlLaw(){
  return u;
}

//Function definition
float activation_function(float state){
  float betha = 0.7;
  return 1.0 / (1.0 + std::exp(-betha*state));
}

void Rhonn::control_law(float x_1, float x_2){
  float set_point = 1.10;
  float error_1 = x_1 - set_point;
  std::cout << "Error de seguimiento (st-x): " << error_1 << std::endl;
  float error_2 = x_2 - 0;
  int sign_e1;
  int sign_e2;
  if (error_1 < 0)
  {
    sign_e1 = -1;
  }else{
    sign_e1 = 1;
  }
  if (error_2 < 0)
  {
    sign_e2 = -1;
  }else{
    sign_e2 = 1;
  }
  double k1 = 0.7;
  double k2 = 0.5;
  u = k1*std::abs(error_1)*sign_e1 + k2*std::abs(error_2)*sign_e2;
}