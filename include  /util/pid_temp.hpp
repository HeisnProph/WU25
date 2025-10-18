#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"

class PIDController {
public:
    // initial PID parameter
    PIDController(double kp, double ki, double kd, double output_min, double output_max)
        : kp_(kp), ki_(ki), kd_(kd), output_min_(output_min), output_max_(output_max),
          integral_(0.0), prev_error_(0.0) {}

    // calculate PID output
    double compute(double setpoint, double current_value, const rclcpp::Duration& dt) {
        double error = setpoint - current_value;
        
        // P term (product)
        double p_term = kp_ * error;

        // I tern (integration 带抗饱和)
        double dt_seconds = dt.seconds();
        integral_ += error * dt_seconds;
        // 限制积分项，防止积分饱和 (Integral Windup)
        if (integral_ > output_max_) integral_ = output_max_;
        if (integral_ < output_min_) integral_ = output_min_;
        double i_term = ki_ * integral_;

        // D term (differential term)
        double derivative = (error - prev_error_) / dt_seconds;
        double d_term = kd_ * derivative;

        // total output
        double output = p_term + i_term + d_term;

        // limit output
        if (output > output_max_) {
            output = output_max_;
        } else if (output < output_min_) {
            output = output_min_;
        }

        // status update
        prev_error_ = error;

        return output;
    }

    // reset PID
    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
    }

private:
    // PID parameter gain
    double kp_; // Proportional gain
    double ki_; // Integral gain
    double kd_; // Derivative gain

    // output limitation
    double output_min_;
    double output_max_;

    // internal status
    double integral_;
    double prev_error_;
};

#endif // PID_CONTROLLER_HPP_
