//
// Created by chh3213 on 2022/11/24.
// Modified by lxy on 2024/4/1.
#pragma once
#ifndef CHHROBOTICS_CPP_PID_CONTROLLER_H
#define CHHROBOTICS_CPP_PID_CONTROLLER_H
#include <iostream>

using namespace std;

/**
 * 位置式PID实现
 */
class PID_controller {
private:
    double kp_, ki_, kd_, target_, output_max_, output_min_;
    double error_=0.0, pre_error_=0.0, integral_=0.0, pre_output_ = 0.0;
    bool first_hit_ = true;
    double integral_max_ = 10000, integral_min_ = -10000;
    double errshreshold_ = 10000;
    double kaw_ = 0.2;
public:
    PID_controller(double kp, double ki, double kd, double target, double output_max, double output_min);

    void setTarget(double target);

    void setK(double kp,double ki,double kd );

    void setBound(double output_max,double output_min, double integral_max, double integral_min, double errshreshold);

    double calOutput(double state);

    void reset();

    void setSumError(double integral);
};


#endif //CHHROBOTICS_CPP_PID_CONTROLLER_H
