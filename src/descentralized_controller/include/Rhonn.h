#pragma once
#include <Eigen/Dense>
#include <iostream>

class Rhonn {
public:
    Rhonn(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> c_input, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_input, int number);
    float prediction_state();
    void update_input();
    void updateWeights(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& new_weights);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> getInputs();
    int getNeuron();
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> getWeights();

private:
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> z_input;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> w_weight;
    int neuron;
};
