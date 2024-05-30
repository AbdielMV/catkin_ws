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
    fx_0 = 0; //Just exist in neuron 1
    fx_1 = 0; //Just exist in neuron 2
    W1_fixed = 0.001;
    W2_fixed = 1;
    u = 0;
    error_x1_old = 0;
    sign_ex0 = 0;
    sign_ex1 = 0;
    counter = 0;
    observer_x0_prediction = -0.878902;
    observer_x1_prediction = -0.076326;
}
//Observer filter
float Rhonn::observer_state(float position, float velocity){
  float k1 = 0.01;
  float k2 = 0.001;
  observer_x0_actual = observer_x0_prediction;
  observer_x1_actual = observer_x1_prediction;
  error_obs1 = position - observer_x0_actual;
  error_obs2 = velocity - observer_x1_actual;
  if (error_obs1 < 0)
    {
      sign_eobs1 = -1;
    }else{
      sign_eobs1 = 1;
    }
  if (error_obs2 < 0)
    {
      sign_eobs2 = -1;
    }else{
      sign_eobs2 = 1;
    }
 /*  float v1_equival = (error_obs1)*0.99;
  float v2_equival = (error_obs2)*0.99;
  float u0 = 100; //cota de la superficie
  if (abs(v1_equival) < u0)
  {
    v1 = v1_equival;
  }else{
    v1 = (u0*v1_equival)/abs(v1_equival);
  }
  if (abs(v2_equival) < u0)
  {
    v2 = v2_equival;
  }else{
    v2 = (u0*v1_equival)/abs(v2_equival);
  } */
  v1 = -k1*sqrt(abs(error_obs1))*sign_eobs1;
  v2 = -k2*sign_eobs1;
  observer_x0_prediction = position + v1;
  observer_x1_prediction = velocity + v2;
  if (neuron == 1)
  {
    return observer_x0_prediction;
  }else{
    return observer_x1_prediction;
  }
  
}

//Prediction state
float Rhonn::prediction_state (float position, float velocity){
    float fixed_result;
    float state_final_prediction;
    update_input(position,velocity);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_transposed = w_weight.transpose();
    std::cout << "Valor de z_input: " << z_input << std::endl;
    std::cout << "Valor de w_transposed: " << w_transposed << std::endl;
    Eigen::Matrix<float, 1, 1> state = w_transposed*z_input;
    float value_from_prediction = state(0,0);
    if (neuron == 1)
    {
      fixed_result = W1_fixed*velocity;
      state_final_prediction = value_from_prediction+fixed_result;
      fx_0 = state_final_prediction;
    }else{
      fixed_result = W2_fixed*u;
      state_final_prediction = value_from_prediction+fixed_result;
      fx_1 = value_from_prediction;
      control_law(fx_0,fx_1);
    }
    // std::cout << "Ley de control u: " << u << std::endl;
    return state_final_prediction;
}

void Rhonn::update_input(float position, float velocity){
    if (neuron == 1)
    {
      z_input(0,0) = activation_function(position);
    }else{
      z_input(0,0) = activation_function(velocity);
      z_input(1,0) = activation_function(velocity);
      //z_input(2,0) = activation_function(velocity);
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

void Rhonn::fx0_value(float x_neuron_1){
  fx_0 = x_neuron_1;
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

    float set_point = 1.79538302136;
/*     error_x1_old = error_x1;
    error_x0 = x_1 - set_point;
    error_x1 = x_2 - 0;
    if (error_x0 < 0)
    {
      sign_ex0 = -1;
    }else{
      sign_ex0 = 1;
    }
    if (error_x1 < 0)
    {
      sign_ex1 = -1;
    }else{
      sign_ex1 = 1;
    }
    double k1 = 1.2;
    double k2 = 1.5; */
    float angulo_temporal = counter*M_PI/180;
    std::cout << "El angulo es: " << angulo_temporal << std::endl;
    if (angulo_temporal > 360){
      counter = 0;  
    }
    u = 10*sin(angulo_temporal*0.1); //Valor maximo de 10 a 14
    if (u < -10)
    {
      u = -10;
    }
    counter = counter + 1;

}