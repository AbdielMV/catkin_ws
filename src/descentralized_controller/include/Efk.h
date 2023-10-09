#pragma once
#include <Eigen/Dense>
#include <iostream>
#include "Rhonn.h"

class Efk {
public:
    Efk(Eigen::Matrix<float, 1, 1> r_value, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> p_value, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> q_value);

    void someFunction(Rhonn& rhonn_object);
    void error_estimation(float rhonn_state_value);
    void update_values();

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
};
