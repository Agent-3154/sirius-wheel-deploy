#ifndef FILTERS_H
#define FILTERS_H

#include <eigen3/Eigen/Dense>

class FirstOrderLowPassFilter {
private:
    float tau_;
    float dt_;
    float alpha_;
    Eigen::VectorXf output_;
    
public:
    FirstOrderLowPassFilter(float tau, float dt);
    Eigen::VectorXf update(const Eigen::VectorXf& input);
};

class SecondOrderLowPassFilter {
private:
    float tau_;
    float dt_;
    float alpha_;
    Eigen::VectorXf output_;
    Eigen::VectorXf prev_output_;
    
public:
    SecondOrderLowPassFilter(float tau, float dt);
    Eigen::VectorXf update(const Eigen::VectorXf& input);
};

#endif // FILTERS_H