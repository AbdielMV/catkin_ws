#pragma once
#include <Eigen/Dense>
#include <iostream>

class Rhonn {
public:
    Rhonn();
    Rhonn(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> c_input, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_input, int number);
    float observer_state(float position, float velocity);
    float prediction_state(float position, float velocity);
    void update_input(float position, float velocity);
    void updateWeights(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& new_weights);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> getInputs();
    int getNeuron();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> getWeights();
    void control_law(Rhonn& rhonn_object, float position, float velocity);
    void fx0_value(Rhonn& rhonn_object); //This is for pass the fx0 value to Neuron 2
    float getControlLaw();

private:
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> z_input;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_weight;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w1_weight;
    int neuron;
    float W1_fixed;
    float W2_fixed;
    float u;
    float ueq;
    float v;
    float fx_0;
    float fx_1;
    float error_x0;
    float error_x1;
    float error_x1_old;
    float counter;
    int sign_ex0;
    int sign_ex1;
    float observer_x0_actual;
    float observer_x1_actual;
    float observer_x0_prediction;
    float observer_x1_prediction;
    float error_obs1;
    float error_obs2;
    int sign_eobs1;
    int sign_eobs2;
    float v1;
    float v2;
    float fixed_result;
    float state_final_prediction;
    float fx_11;
    float fx_12;
    float fx_21;
    float fx_22;
    float fx_23;
    float set_point;
};
