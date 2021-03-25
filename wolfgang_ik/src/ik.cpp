#include <wolfgang_ik/ik.h>

std::ostream& operator<<(std::ostream& out, const Eigen::Isometry3d& iso) {
  double r, p, y;
  iso.rotation().eulerAngles(r, p, y);
  out << "Translation: " << iso.translation().x() << ", " << iso.translation().y() << ", " << iso.translation().z()
      << std::endl
      << "Rotation: " << r << ", " << p << ", " << y << std::endl;
  return out;
}


namespace wolfgang_ik {
IK::IK() : robot_model_loader_("robot_description", false) {
  // get robot model state
  robot_model::RobotModelPtr kinematic_model = robot_model_loader_.getModel();
  if (!kinematic_model) {
    ROS_FATAL("No robot model loaded, unable to run IK");
    exit(1);
  }
  robot_state_.reset(new robot_state::RobotState(kinematic_model));
  robot_state_->updateLinkTransforms();
  // compute the necessary link length and save them as class variables

}

bool IK::solve(Eigen::Isometry3d& l_sole_goal, robot_state::RobotStatePtr goal_state) {
  // some naming conventions:
  // hip_RY_intersect: the point where the HipYaw and HipRoll axis intersect
  // ankle_intersect: the point where AnklePitch and AnkleRoll intersect

  // the provided goal points from base_link to sole link
  // first get rid of the static transform between sole and ankle_intersect
  // vector rotate with goal rotation and subtract

  // in our robot, the chain is (alternating link -> joint -> link...), fixed joints are denoted (f)
  // base_link -> base_link_to_base(f) -> torso -> LHipYaw -> l_hip_1 -> LHipRoll -> l_hip_2 -> LHipPitch ->
  // l_upper_leg -> LKnee -> l_lower_leg -> LAnklePitch -> l_ankle -> LAnkleRoll -> l_foot -> l_sole_frame(f) -> l_sole_goal
  // link -> joint transform is always zero, joint -> link is the interesting one

  const moveit::core::JointModel* l_sole_frame = robot_state_->getJointModel("l_sole_frame");
  Eigen::Isometry3d l_sole_to_l_foot = robot_state_->getLinkModel("l_sole")->getJointOriginTransform().inverse();
  Eigen::Isometry3d l_foot_goal = l_sole_goal * l_sole_to_l_foot;
  Eigen::Isometry3d l_foot_to_l_ankle = robot_state_->getLinkModel("l_foot")->getJointOriginTransform().inverse();
  Eigen::Isometry3d l_ankle_pose = l_foot_goal * l_foot_to_l_ankle;
  Eigen::Vector3d ankle_pitch_axis = dynamic_cast<const moveit::core::RevoluteJointModel*>(robot_state_->getJointModel("LAnklePitch"))->getAxis();
  Eigen::Vector3d ankle_roll_axis = dynamic_cast<const moveit::core::RevoluteJointModel*>(robot_state_->getJointModel("LAnkleRoll"))->getAxis();
  Eigen::Vector3d ankle_intersect;
  findIntersection(l_ankle_pose.translation(), ankle_pitch_axis, l_foot_goal.translation(), ankle_roll_axis, ankle_intersect);
  ROS_INFO_STREAM(l_ankle_pose << l_foot_goal);
  ROS_INFO_STREAM(ankle_intersect);

  // get rid of static transform between base_link and hip_RY_intersect
  // directly subtract, no rotation necessary

  // now the goal describes the pose of the ankle_intersect in hip_RY_intersect frame
  // compute AnkleRoll
  // Create triangle in y,z dimension consisting of goal position vector and the y and z axis (in hip_RY_intersect frame)
  // Compute atan(goal.y,goal.z), this gives us the angle of the foot as if the goal rotation would be zero.
  // This is only the angle coming from the goal position. We need to simply add the goal orientation in roll
  // We can compute this joint angles without knowing the HipYaw value, since the position is not influenced by it.
  // Because we use the exact intersection of HipYaw and HipRoll as a basis.

  // Compute HipRoll
  // We can compute this similarly as the AnkleRoll. We can also use the triangle method but don't need to add
  // the goal orientation of the foot.

  // We need to know the position of the HipPitch to compute the rest of the pitch joints.
  // Therefore we need to firstly compute the HipYaw joint, since it influences the position of the HipPitch
  // joint (as their axes do not intersect)
  // First we compute the plane of the leg (the plane in which all pitch joints are).
  // This plane contains the goal position of the ankle_intersect. It can be defined by the normal which is the
  // Y axis in the ankle_intersect frame. //todo nicht sicher ob vorher der roll abgezogen werden muss
  // The hip_RY_intersect is also located on this plane.
  // We take the point where the normal of this plane starting from the ankle_intersect point intersects with
  // the Y axis. The vector (v) from hio_RY_intersect to this point and the gloabl X and Y axis create a triangle.
  // Basically we are interested in the rotation between the leg plane and the global XZ plane.
  // HipYaw equals the atan(v.x,v.y)

  // Represent the goal in the HipPitch frame. Subtract transform from hip_RY_intersect to HipPitch
  // Now we have a triangle of HipPitch, Knee and AnklePitch which we can treat as a prismatic joint.
  // First we can get the knee angle with the rule of cosine

  // Similarly, we can compute HipPitch and AnklePitch, but we need to add half of the knee angle.

  // ankle pitch needs goal pitch

  //todo use markers to debug
  //todo write unit test
  return true;
}

bool IK::findIntersection(const Eigen::Vector3d &p1,
                          const Eigen::Vector3d &v1,
                          const Eigen::Vector3d &p2,
                          const Eigen::Vector3d &v2,
                          Eigen::Vector3d &output) {
  // https://web.archive.org/web/20180324134610/https://mathforum.org/library/drmath/view/62814.html
  Eigen::Vector3d v1_v2 = v1.cross(v2);
  if (v1_v2.norm() == 0) {
    return false;
  }
  Eigen::Vector3d other = (p2 - p1).cross(v2);
  double a = other.norm() / v1_v2.norm();
  if (v1_v2 * a == -other) {
    a *= -1;
  }
  if (v1_v2 * a != other) {
    return false;
  }

  output = p1 + a * v1;
  return true;
}

}
