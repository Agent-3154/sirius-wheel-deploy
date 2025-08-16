#include "./filters.h"

// FirstOrderLowPassFilter implementation
FirstOrderLowPassFilter::FirstOrderLowPassFilter(float tau, float dt) : tau_(tau), dt_(dt) {
    alpha_ = dt / (tau + dt);
    output_ = Eigen::VectorXf::Zero(0);
}

Eigen::VectorXf FirstOrderLowPassFilter::update(const Eigen::VectorXf& input) {
    if (output_.size() != input.size()) {
        output_ = Eigen::VectorXf::Zero(input.size());
    }
    Eigen::VectorXf output = output_ * (1 - alpha_) + input * alpha_;
    output_ = output;
    return output;
}

// SecondOrderLowPassFilter implementation
SecondOrderLowPassFilter::SecondOrderLowPassFilter(float tau, float dt) : tau_(tau), dt_(dt) {
    alpha_ = dt / (tau + dt);
    output_ = Eigen::VectorXf::Zero(0);
    prev_output_ = Eigen::VectorXf::Zero(0);
}

Eigen::VectorXf SecondOrderLowPassFilter::update(const Eigen::VectorXf& input) {
    if (output_.size() != input.size()) {
        output_ = Eigen::VectorXf::Zero(input.size());
        prev_output_ = Eigen::VectorXf::Zero(input.size());
    }
    
    Eigen::VectorXf output = alpha_ * alpha_ * input + 
                            2.0f * (1.0f - alpha_) * output_ - 
                            (1.0f - alpha_) * (1.0f - alpha_) * prev_output_;
    
    prev_output_ = output_;
    output_ = output;
    return output;
}
