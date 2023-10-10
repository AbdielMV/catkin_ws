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
    f_x << 0,
           0;
}
//Prediction state
float Rhonn::prediction_state (float position, float velocity){
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
      f_x(0,0) = state_final_prediction;
    }else{
      fixed_result = W2_fixed*u;
      state_final_prediction = value_from_prediction+fixed_result;
      f_x(1,0) = value_from_prediction;
    }
    std::cout << "Valor f(x): " << f_x << std::endl;
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

//Function definition
float activation_function(float state){
  float betha = 0.7;
  return 1.0 / (1.0 + std::exp(-betha*state));
}