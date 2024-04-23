//
// Created by chh3213 on 2022/11/24.
// Modified by lxy on 2024/4/1.
#include "PID_controller.h"




PID_controller::PID_controller(double kp, double ki, double kd, double target, double output_max, double output_min) 
                                                                                                            :kp_(kp),
                                                                                                             ki_(ki),
                                                                                                             kd_(kd),
                                                                                                             target_(target),
                                                                                                             output_max_(output_max),
                                                                                                             output_min_(output_min)
                                                                                                              {}
/**
 * 设置目标
 * @param target
 */
void PID_controller::setTarget(double target) {
    PID_controller::target_ = target;
}
/**
 * 设置pid参数
 * @param kp
 * @param ki
 * @param kd
 */
void PID_controller::setK(double kp, double ki, double kd) {
    this->kp_=kp;
    this->ki_=ki;
    this->kd_=kd;
}

/**
 * 设置控制量边界
 * @param umax
 * @param umin
 */
void PID_controller::setBound(double output_max,double output_min, double integral_max, double integral_min, double errshreshold) {
    this->output_max_=output_max;
    this->output_min_=output_min;
    this->integral_max_ = integral_max;
    this->integral_min_ = integral_min;
    this->errshreshold_ = errshreshold;
}
/**
 * 计算控制输出
 * @param state 当前状态量
 * @return
 */
double PID_controller::calOutput(double state) {
    this->error_ = this->target_ - state;
    double u;

    // 积分分离，判断积分项是否超出限制
    // if(std::abs(this->error_) > this->errshreshold_)
    // {
    //     u  = this->error_*this->kp_+(this->error_-this->pre_error_)*this->kd_;
    // }
    // else
    // {
    //     // 积分限幅
    //     if (this->integral_ > this->integral_max_)
    //     {
    //         u  = this->error_*this->kp_+ this->integral_max_ + (this->error_ - this->pre_error_)*this->kd_;
    //     }
    //     else if(this->integral_ < this->integral_min_)
    //     {
    //         u  = this->error_*this->kp_+ this->integral_min_ + (this->error_ - this->pre_error_)*this->kd_;
    //     }
    //     else
    //     {
    //         u = this->error_*this->kp_+ integral_ + (this->error_ -this->pre_error_)*this->kd_;
    //         integral_ += this->error_ * ki_;
    //     }
    // }

    //IC抗饱和
    // double diff = 0;
    // if(first_hit_)
    // {
    //     first_hit_ = false;
    // }
    // else
    //     diff = this->error_ - this->pre_error_;
    // u = this->error_ * this->kp_ + this->integral_ + this->error_ * ki_ + diff * this->kd_;
    // if((u * this->error_ > 0) && ((u > this->output_max_) || (u < this->output_min_)))
    // {

    // }
    // else
    //     this->integral_ += this->error_ * ki_;
    // this->pre_error_ = this->error_;
    // this->pre_output_ = this->error_ * this->kp_ + this->integral_ + diff * this->kd_;
    // // 输出量（控制量）上下限
    // if(this->pre_output_ < this->output_min_)
    //     this->pre_output_ = this->output_min_;
    // else if(this->pre_output_ > this->output_max_)
    //     this->pre_output_ = this->output_max_;
    // return this->pre_output_;



    //BC抗饱和
    double diff = 0;
    if(first_hit_)
    {
        first_hit_ = false;
    }
    else
    {
        diff = this->error_ - this->pre_error_;
    }

    u = this->error_ * kp_ + integral_ + this->error_ * ki_ + diff * kd_;
    double temp = u;
    if(temp > this->output_max_)
    {
        temp = this->output_max_;
    }
    else if(temp < this->output_min_)
    {
        temp = this->output_min_;
    }
    else
    {

    }
    double aw_term = temp - u;
    integral_ += this->error_ * ki_ + aw_term * kaw_;
    this->pre_output_ = this->error_ * kp_ + integral_ + diff * kd_;
    this->pre_error_ = this->error_;

    // 输出量（控制量）上下限
    if(this->pre_output_ < this->output_min_)
        this->pre_output_ = this->output_min_;
    else if(this->pre_output_ > this->output_max_)
        this->pre_output_ = this->output_max_;
    return this->pre_output_;



}
/**
 * 重置
 */
void PID_controller::reset() {
    error_=0.0,pre_error_=0.0,integral_=0.0;
}

/**
 * 设置累计误差
 * @param sum_error
 */
void PID_controller::setSumError(double integral) {
    this->integral_ = integral;
}
