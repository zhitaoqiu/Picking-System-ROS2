#include "custom_trajectory_core/trapezoidal_planner.hpp"

namespace custom_trajectory {


TrapezoidalPlanner::TrapezoidalPlanner(double s_total, double v_max, double a_max)
    : s_total_(std::abs(s_total)), v_max_(v_max), a_max_(a_max) 
{
    double theoretical_sa = (v_max_ * v_max_) / (2.0 * a_max_);

    if (s_total_ >= 2.0 * theoretical_sa) {
        // 正常梯形
        v_peak_ = v_max_;
        t_acc_ = v_max_ / a_max_;
        s_acc_ = theoretical_sa;
        s_ave_ = s_total_ - 2.0 * s_acc_;
        t_ave_ = s_ave_ / v_max_;
        t_dec_ = t_acc_;
    } else {
        // 退化为三角形
        s_ave_ = 0.0;
        v_peak_ = std::sqrt(2.0 * a_max_ * s_total_);
        t_acc_ = v_peak_ / a_max_;
        s_acc_ = s_total_ / 2.0;
        t_ave_ = 0.0;
        t_dec_ = t_acc_;
    }
}

double TrapezoidalPlanner::getTotalTime() const {
    return t_acc_ + t_ave_ + t_dec_;
}

MotionState TrapezoidalPlanner::getMotionState(double t) const {
    MotionState state = {0.0, 0.0, 0.0};
    
    if (t < 0.0) {
        return state;
    } else if (t < t_acc_) {
        state.a = a_max_;
        state.v = a_max_ * t;
        state.s = 0.5 * a_max_ * t * t;
    } else if (t < t_acc_ + t_ave_) {
        double t_ave = t - t_acc_;
        state.a = 0.0;
        state.v = v_peak_;
        state.s = s_acc_ + v_peak_ * t_ave;
    } else if (t < t_acc_ + t_ave_ + t_dec_) {
        double t_dec = t - t_acc_ - t_ave_;
        state.a = -a_max_;
        state.v = v_peak_ - a_max_ * t_dec;
        state.s = s_acc_ + s_ave_ + v_peak_ * t_dec - 0.5 * a_max_ * t_dec * t_dec;
    } else {
        state.a = 0.0;
        state.v = 0.0;
        state.s = s_total_;
    }
    return state;
}

} // namespace custom_trajectory