#ifndef DIFF_DRIVE_ROBOT_H
#define DIFF_DRIVE_ROBOT_H

#include <stdlib.h>
#include <vector>

namespace diff_drive_robot{
double wrap_angle(double angle);
struct Goal{
  double x;
  double y;
  double theta;
};
class DiffDriveRobot {
  public:
    DiffDriveRobot (double v_max, double w_max, double a_max, double alpha_max, double dt);
    void set_goals(std::vector<Goal> const &goals);
    void set_pose(double x, double y, double theta);
    double get_v() const {return v_;};
    double get_w() const {return w_;};
    double get_goal_idx() const {return goal_idx_;};
    void spin(void);
    void stop(void);
    Goal get_goal(size_t idx) const;
    size_t goals_count() {return goals_.size();};
  private:
    double distance_to_goal(Goal const &goal) const;
    std::vector<Goal> goals_;
    double x_, y_, theta_;
    double v_, w_;
    double v_max_, w_max_;
    // double a_, alpha_;
    double a_max_, alpha_max_;
    double dt_;
    size_t goal_idx_;
    size_t stop_mode_;
};
}
#endif
