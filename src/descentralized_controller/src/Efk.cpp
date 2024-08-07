#include "Efk.h"

//Default constructor
Efk::Efk(){
}
// Constructor for the Efk class
Efk::Efk(Eigen::Matrix<float, 1, 1> r_value, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> p_value, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> q_value): r(r_value), p_now(p_value), q_now(q_value){
    std::cout << "Object EFK created with values" << std::endl;
}

// Some function in the Efk class that takes a Rhonn object as a parameter
void Efk::calculate_new_weights(Rhonn& rhonn_object) {
    etha = 0.7;
    h = rhonn_object.getInputs();
    neuron = rhonn_object.getNeuron();
    w_now = rhonn_object.getWeights();
    m = (r + (h.transpose() * p_now * h)).inverse();
    k = p_now*h*m;
    if (neuron == 1)
    {
      w_next = w_now + (etha*k*error_1);
    }else{
      w_next = w_now + (etha*k*error_2);
    }
    p_next = p_now - (k*h.transpose()*p_now) + q_now;
    update_values();
    // Use the public member function to update w_weight of the Rhonn object
    rhonn_object.updateWeights(w_now);
  }

// Error estimation method
float Efk::error_estimation(float rhonn_state_value, float position, float velocity, int neuron){ //Se calcula el valor del error y se coloca en vector (esto pudiera no ser necesario)
    if (neuron == 1)
    {
      Eigen::Matrix<float, 1, 1> e_position = (position*Eigen::Matrix<float, 1, 1>::Identity()) - (rhonn_state_value*Eigen::Matrix<float, 1, 1>::Identity());
      //float e_position_grados = (e_position(0,0)*180)/M_PI;
      error_1 = e_position(0,0);
      error_return = error_1;
      std::cout << "Error identificacion x1: " << error_1 << std::endl;
    }else{
      Eigen::Matrix<float, 1, 1> e_velocity = (velocity*Eigen::Matrix<float, 1, 1>::Identity()) - (rhonn_state_value*Eigen::Matrix<float, 1, 1>::Identity());
      //float e_velocity_grados = (e_velocity(0,0)*180)/M_PI;
      error_2 = e_velocity(0,0);
      error_return = error_2;
      std::cout << "Error identificacion x2: " << error_2 << std::endl;
    }
    return error_return;
  }

// Update values method
void Efk::update_values(){
    p_now = p_next;
    w_now = w_next;
  }
