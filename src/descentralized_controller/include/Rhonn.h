#pragma once
#include <Eigen/Dense>
#include <iostream>

class Rhonn {
public:
    Rhonn();
    Rhonn(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> c_input, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_input, int number);
    float prediction_state(float position, float velocity);
    void update_input(float position, float velocity);
    void updateWeights(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& new_weights);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> getInputs();
    int getNeuron();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> getWeights();
    void control_law(float x_1, float x_2);
    void fx1_value(float x_neuron_1); //This is for pass the fx1 value to Neuron 1
    float getControlLaw();

private:
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> z_input;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_weight;
    int neuron;
    float W1_fixed;
    float W2_fixed;
    float u;
    float fx_1;
    float fx_2;
};
