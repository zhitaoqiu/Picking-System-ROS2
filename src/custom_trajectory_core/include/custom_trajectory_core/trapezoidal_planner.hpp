#ifndef CUSTOM_TRAJECTORY_CORE_TRAPEZOIDAL_PLANNER_HPP_
#define CUSTOM_TRAJECTORY_CORE_TRAPEZOIDAL_PLANNER_HPP_

#include <cmath>

namespace custom_trajectory {

struct MotionState {
    double v;
    double s;
    double a;
};

class TrapezoidalPlanner {
public:
    TrapezoidalPlanner(double s_total, double v_max, double a_max);
    ~TrapezoidalPlanner() = default;

    double getTotalTime() const;
    MotionState getMotionState(double t) const;

private:
    double s_total_;
    double v_max_;
    double a_max_;
    
    double t_acc_;
    double t_dec_;
    double t_ave_;
    
    double s_acc_;
    double s_ave_;
    
    double v_peak_; 
};

} // namespace custom_trajectory

#endif // CUSTOM_TRAJECTORY_CORE_TRAPEZOIDAL_PLANNER_HPP_