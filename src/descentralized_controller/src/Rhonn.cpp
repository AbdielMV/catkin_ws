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
    fx_0 = 0; //Just exist in neuron 1
    fx_1 = 0; //Just exist in neuron 2
    W1_fixed = 0.001; //0.001
    W2_fixed = 0.1; //0.1
    u = 0;
    ueq = 0;
    error_x1_old = 0;
    sign_ex0 = 0;
    sign_ex1 = 0;
    counter = 0;
    observer_x0_prediction = -0.878902;
    observer_x1_prediction = -0.076326;
    set_point = 45;
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
    /* float fixed_result;
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
    return state_final_prediction; */

    update_input(position,velocity);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_transposed = w_weight.transpose();
    //std::cout << "Neurona numero: " << neuron << std::endl;
    if (neuron == 1)
    {
      fx_11 = w_transposed(0,0)*z_input(0,0);
      fx_12 = w_transposed(0,1)*z_input(1,0);
      fixed_result = W1_fixed*velocity;
      state_final_prediction = fx_11+fx_12+fixed_result;
      fx_0 = state_final_prediction;
      std::cout << "fx_11 = " << fx_11 << " fx_12 = " << fx_12 << std::endl;
      std::cout << "X1_(k+1)= " << fx_0 << std::endl;
      return fx_0;
    }else{
      fx_21 = w_transposed(0,0)*z_input(0,0);
      fx_22 = w_transposed(0,1)*z_input(1,0);
      fx_23 = w_transposed(0,2)*z_input(2,0);
      fixed_result = W2_fixed*u;
      state_final_prediction = fx_21+fx_22+fx_23+fixed_result;
      fx_1 = state_final_prediction;
      std::cout << "fx_11 = " << fx_11 << " fx_12 = " << fx_12 << std::endl;
      std::cout << "fx_21 = " << fx_21 << " fx_22 = " << fx_22 << " fx_23 = " << fx_23 << " u = " << u << std::endl; 
      std::cout << "X1_(k+1)= " << fx_0 << std::endl;
      std::cout << "X2_(k+1)= " << fx_1 << std::endl;
      //control_law();
      return fx_1;
    }
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

void Rhonn::fx0_value(Rhonn& rhonn_object){
  fx_0 = rhonn_object.fx_0;
  fx_11 = rhonn_object.fx_11;
  fx_12 = rhonn_object.fx_12;
}

float Rhonn::getControlLaw(){
  return u;
}

//Function definition
float activation_function(float state){
/*   float betha = 0.7;
  return 1.0 / (1.0 + std::exp(-betha*state)); */
  return tanh(state);
}

void Rhonn::control_law(Rhonn& neuron1, float position){
    std::cout << "Value of fx_0= " << fx_0 << std::endl;
    std::cout << "Value of fx_1= " << fx_1 << std::endl;
    error_x1_old = error_x1;
    error_x0 = position - set_point;
    error_x1 = fx_1 - 0;
    
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
    double k1 = 10; //0.0006
    double k2 = 5; //0.0001
    //v = -(k1*pow(abs(error_x1),0.5)*sign_ex1)-(k2*pow(abs(error_x0),0.5/(2-0.5))*sign_ex0);
    v = -(k2*error_x0) - (k1*error_x1);
    std::cout << "Value of V = " << v << std::endl;
    std::cout << "Value of w11 = " << neuron1.w_weight(0,0) << std::endl;
    std::cout << "Value of w12 = " << neuron1.w_weight(1,0) << std::endl;
    std::cout << "Value of w13 = " << neuron1.W1_fixed << std::endl;
    std::cout << "Value of w21 = " << w_weight(0,0) << std::endl;
    std::cout << "Value of w22 = " << w_weight(1,0) << std::endl;
    std::cout << "Value of w23 = " << w_weight(2,0) << std::endl;
    std::cout << "Value of w24 = " << W2_fixed << std::endl;

    ueq = (((v + set_point - neuron1.w_weight(0,0)*activation_function(fx_0)-neuron1.w_weight(1,0))*(1/neuron1.W1_fixed))-fx_21-fx_22-fx_23)*(1/W2_fixed);
    u = ueq * 1e-8;
/*     if (abs(ueq) <= 10)
    {
      u = ueq;
    }else{
      u = (10*(ueq/abs(ueq)));
    }
 */

/*     float angulo_temporal = counter*M_PI/180;
    std::cout << "El angulo es: " << angulo_temporal << std::endl;
    if (angulo_temporal > 360){
      counter = 0;  
    }
    u = 10*sin(angulo_temporal*0.1); //Valor maximo de 10 a 14
    if (u < -10)
    {
      u = -10;
    }
    counter = counter + 1; */
}