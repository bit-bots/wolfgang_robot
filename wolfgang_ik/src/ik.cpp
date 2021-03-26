#include <wolfgang_ik/ik.h>

std::ostream& operator<<(std::ostream& out, const Eigen::Isometry3d& iso) {
  Eigen::Vector3d euler = iso.rotation().eulerAngles(0, 1, 2);
  out << "Translation: " << iso.translation().x() << ", " << iso.translation().y() << ", " << iso.translation().z()
      << std::endl
      << "Rotation: " << euler.x() << ", " << euler.y() << ", " << euler.z() << std::endl;
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

  for (auto& joint : goal_state->getJointModelGroup("All")->getJointModels()) {
    double zero = 0;
    goal_state->setJointPositions(joint, &zero);
  }
  Eigen::Isometry3d l_sole_to_l_foot = goal_state->getGlobalLinkTransform("l_sole").inverse() * goal_state->getGlobalLinkTransform("l_foot");
  Eigen::Isometry3d l_sole_to_l_ankle = goal_state->getGlobalLinkTransform("l_sole").inverse() * goal_state->getGlobalLinkTransform("l_ankle");
  // calc ankle_intersection
  Eigen::Vector3d ankle_roll_axis = l_sole_to_l_foot.rotation()
      * dynamic_cast<const moveit::core::RevoluteJointModel *>(goal_state->getJointModel("LAnkleRoll"))->getAxis();
  Eigen::Vector3d ankle_pitch_axis = l_sole_to_l_ankle.rotation()
      * dynamic_cast<const moveit::core::RevoluteJointModel *>(goal_state->getJointModel("LAnklePitch"))->getAxis();
  Eigen::Vector3d ankle_intersection_point;
  bool success = findIntersection(l_sole_to_l_foot.translation(), ankle_roll_axis, l_sole_to_l_ankle.translation(), ankle_pitch_axis, ankle_intersection_point);
  if (!success) {
    return false;
  }
  Eigen::Isometry3d ankle_intersection = l_sole_goal;
  ankle_intersection.translation() += ankle_intersection_point;

  // get rid of static transform between base_link and hip_RY_intersect
  Eigen::Isometry3d base_link_to_hip_1 = goal_state->getGlobalLinkTransform("l_hip_1");
  Eigen::Isometry3d base_link_to_hip_2 = goal_state->getGlobalLinkTransform("l_hip_2");
  // calc ankle_intersection
  Eigen::Vector3d hip_yaw_axis = base_link_to_hip_1.rotation()
      * dynamic_cast<const moveit::core::RevoluteJointModel *>(goal_state->getJointModel("LHipYaw"))->getAxis();
  Eigen::Vector3d hip_roll_axis = base_link_to_hip_2.rotation()
      * dynamic_cast<const moveit::core::RevoluteJointModel *>(goal_state->getJointModel("LHipRoll"))->getAxis();
  Eigen::Vector3d hip_ry_intersection_point;
  success = findIntersection(base_link_to_hip_1.translation(), hip_yaw_axis, base_link_to_hip_2.translation(), hip_roll_axis, hip_ry_intersection_point);
  if (!success) {
    return false;
  }
  Eigen::Isometry3d hip_ry_intersection = Eigen::Isometry3d::Identity();
  hip_ry_intersection.translation() += hip_ry_intersection_point;

  // now the goal describes the pose of the ankle_intersect in hip_RY_intersect frame
  Eigen::Isometry3d goal = hip_ry_intersection.inverse() * ankle_intersection;
  // compute AnkleRoll
  // Create triangle in y,z dimension consisting of goal position vector and the y and z axis (in hip_RY_intersect frame)
  // Compute atan(goal.y,goal.z), this gives us the angle of the foot as if the goal rotation would be zero.
  double ankle_roll = std::atan2(goal.translation().y(), -goal.translation().z());
  // This is only the angle coming from the goal position. We need to simply add the goal orientation in roll
  // We can compute this joint angles without knowing the HipYaw value, since the position is not influenced by it.
  // Because we use the exact intersection of HipYaw and HipRoll as a basis.
  Eigen::Vector3d goal_rpy = l_sole_goal.rotation().eulerAngles(0, 1, 2);
  ankle_roll += goal_rpy.x();
  goal_state->setJointPositions("LAnkleRoll", &ankle_roll);
  std::cout << ankle_roll << std::endl;

  // Compute HipRoll
  // We can compute this similarly as the AnkleRoll. We can also use the triangle method but don't need to add
  // the goal orientation of the foot.
  double hip_roll = std::atan2(goal.translation().y(), -goal.translation().z());
  goal_state->setJointPositions("LHipRoll", &hip_roll);
  std::cout << hip_roll << std::endl;

  // We need to know the position of the HipPitch to compute the rest of the pitch joints.
  // Therefore we need to firstly compute the HipYaw joint, since it influences the position of the HipPitch
  // joint (as their axes do not intersect)
  // First we compute the plane of the leg (the plane in which all pitch joints are).
  // This plane contains the goal position of the ankle_intersect. It can be defined by the normal which is the
  // Y axis in the ankle_intersect frame. //todo nicht sicher ob vorher der roll abgezogen werden muss
  // The hip_RY_intersect is also located on this plane.
  //        We take the point where the normal of this plane starting from the ankle_intersect point intersects with
  //        the Y axis. The vector (v) from hip_RY_intersect to this point and the global X and Y axis create a triangle.
  //        Basically we are interested in the rotation between the leg plane and the global XZ plane.
  //        HipYaw equals the atan(v.x,v.y)
  // We determine the intersection of this plane with the xy plane going through hip_ry_intersect
  // Then, we take the angle between this intersection line and the x axis
  // The intersection line can easily be computed with the cross product.
  ankle_pitch_axis = goal.rotation() * Eigen::Vector3d(0, 1, 0);  // this is the rotational axis of the ankle pitch, todo get from urdf
  Eigen::Vector3d line = ankle_pitch_axis.cross(Eigen::Vector3d(0, 0, 1));  // this is the normal of the xy plane
  double hip_yaw = std::acos(line.dot(Eigen::Vector3d(1, 0, 0)) / line.norm());
  goal_state->setJointPositions("LHipYaw", &hip_yaw);
  std::cout << hip_yaw << std::endl;

  // Represent the goal in the HipPitch frame. Subtract transform from hip_RY_intersect to HipPitch
  goal_state->updateLinkTransforms();
  Eigen::Isometry3d hip_intersect_to_hip_pitch = hip_ry_intersection.inverse() * goal_state->getGlobalLinkTransform("l_upper_leg");
  Eigen::Isometry3d hip_pitch_to_goal = hip_intersect_to_hip_pitch.inverse() * goal;
  // Now we have a triangle of HipPitch, Knee and AnklePitch which we can treat as a prismatic joint.
  // First we can get the knee angle with the rule of cosine
  std::cout<<hip_pitch_to_goal;

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
  double tolerance = 1e-3;
  // The math behind this is from here:
  // https://web.archive.org/web/20180324134610/https://mathforum.org/library/drmath/view/62814.html
  // Basically, it is the following:
  // Equate the line equations: p1 + a * v1 = p2 + b * v2
  // Reorder: a * v1 = (p2 - p1) + b * v2
  // On both sides, take cross product with v2: a * v1 x v2 = (p2 - p1) x v2
  // Now, we can solve for a by dividing the norms of the vectors.
  // There are two cases where this does not work: v1 x v2 is zero or the vectors are not parallel.
  // In these cases, there is no intersection.

  // calculate v1 x v2
  Eigen::Vector3d v1_v2 = v1.cross(v2);
  if (v1_v2.norm() < tolerance) {
    // v1 x v2 is zero, so there is no intersection
    return false;
  }

  // calculate (p2 - p1) x v2
  Eigen::Vector3d other = (p2 - p1).cross(v2);

  // calculate a by dividing the norm
  double a = other.norm() / v1_v2.norm();

  // now, check if they are parallel
  if ((v1_v2 * a + other).norm() < tolerance) {
    // in this case they are in opposite directions, so just switch the sign of a
    a *= -1;
  } else if ((v1_v2 * a - other).norm() > tolerance) {
    // in this case they are not parallel, so there is no solution
    return false;
  }

  // now calculate the point from the line equation
  output = p1 + a * v1;
  return true;
}

}
