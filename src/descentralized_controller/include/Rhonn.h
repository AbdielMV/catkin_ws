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

private:
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> z_input;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_weight;
    int neuron;
    float W1_fixed;
    float W2_fixed;
    float u;
    Eigen::Matrix<float, 2, 1> f_x;
};
