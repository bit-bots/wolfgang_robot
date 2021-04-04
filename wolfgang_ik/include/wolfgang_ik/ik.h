#ifndef WOLFGANG_IK_
#define WOLFGANG_IK_

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>

namespace wolfgang_ik {
class IK {
 public:
  IK();
  bool solve(Eigen::Isometry3d &l_sole_goal, robot_state::RobotStatePtr goal_state);
  static bool findIntersection(const Eigen::Vector3d &p1, const Eigen::Vector3d &v1,
                               const Eigen::Vector3d &p2, const Eigen::Vector3d &v2,
                               Eigen::Vector3d &output);
  Eigen::Quaterniond getQuaternionTwist(const Eigen::Quaterniond& rotation,
                                        const Eigen::Vector3d& direction);
 private:
  robot_model_loader::RobotModelLoader robot_model_loader_;
  robot_state::RobotStatePtr robot_state_;
};
}
#endif
