#ifndef WOLFGANG_IK_
#define WOLFGANG_IK_

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/JointState.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

namespace wolfgang_ik {
class IK {
 public:
  IK();
  bool solve(const Eigen::Isometry3d &l_sole_goal, geometry_msgs::TransformStamped &hip_ry_tf, geometry_msgs::TransformStamped &ankle_ry_tf, robot_state::RobotStatePtr goal_state);
  static bool findIntersection(const Eigen::Vector3d &p1, const Eigen::Vector3d &v1,
                               const Eigen::Vector3d &p2, const Eigen::Vector3d &v2,
                               Eigen::Vector3d &output);
 private:
  robot_model_loader::RobotModelLoader robot_model_loader_;
  robot_state::RobotStatePtr robot_state_;
  // For visualizing things in rviz
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_base_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_goal_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_sole_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_hip_ry_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_ankle_ry_;
};
}


std::ostream &operator<<(std::ostream &out, const Eigen::Quaterniond &quat) {
  Eigen::Vector3d euler = Eigen::Matrix3d(quat).eulerAngles(0, 1, 2);
  out << "Rotation: " << euler.x() << ", " << euler.y() << ", " << euler.z() << std::endl
      << "Rotation (q): " << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << std::endl;
  return out;
}

std::ostream &operator<<(std::ostream &out, const Eigen::Isometry3d &iso) {
  Eigen::Vector3d euler = iso.rotation().eulerAngles(0, 1, 2);
  Eigen::Quaterniond quat(iso.rotation());
  out << "Translation: " << iso.translation().x() << ", " << iso.translation().y() << ", " << iso.translation().z()
      << std::endl
      << "Rotation: " << euler.x() << ", " << euler.y() << ", " << euler.z() << std::endl
      << "Rotation (q): " << quat.x() << ", " << quat.y() << ", " << quat.z() << ", " << quat.w() << std::endl;
  return out;
}

#endif
