#include <wolfgang_ik/ik.h>


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
  visual_tools_base_.reset(new rviz_visual_tools::RvizVisualTools("base_link", "/rviz_visual_markers_base"));
  visual_tools_goal_.reset(new rviz_visual_tools::RvizVisualTools("sole_goal", "/rviz_visual_markers"));
  visual_tools_sole_.reset(new rviz_visual_tools::RvizVisualTools("l_sole", "/rviz_visual_markers_sole"));
  visual_tools_hip_ry_.reset(new rviz_visual_tools::RvizVisualTools("hip_ry", "/rviz_visual_markers_hip_ry"));
  visual_tools_ankle_ry_.reset(new rviz_visual_tools::RvizVisualTools("ankle_ry", "/rviz_visual_markers_ankle_ry"));

  // compute the necessary link length and save them as class variables
}

bool IK::solve(const Eigen::Isometry3d &l_sole_goal, geometry_msgs::TransformStamped &hip_ry_tf, geometry_msgs::TransformStamped &ankle_ry_tf, robot_state::RobotStatePtr goal_state) {
  // some naming conventions:
  // hip_RY_intersect: the point where the HipYaw and HipRoll axis intersect
  // ankle_intersect: the point where AnklePitch and AnkleRoll intersect

  // the provided hip_ry_to_ankle_intersect points from base_link to sole link
  // first get rid of the static transform between sole and ankle_intersect
  // vector rotate with hip_ry_to_ankle_intersect rotation and subtract

  // in our robot, the chain is (alternating link -> joint -> link...), fixed joints are denoted (f)
  // base_link -> base_link_to_base(f) -> torso -> LHipYaw -> l_hip_1 -> LHipRoll -> l_hip_2 -> LHipPitch ->
  // l_upper_leg -> LKnee -> l_lower_leg -> LAnklePitch -> l_ankle -> LAnkleRoll -> l_foot -> l_sole_frame(f) -> l_sole_goal
  // link -> joint transform is always zero, joint -> link is the interesting one

  for (auto &joint : goal_state->getJointModelGroup("All")->getJointModels()) {
    double zero = 0;
    goal_state->setJointPositions(joint, &zero);
  }
  Eigen::Isometry3d l_sole_to_l_foot =
      goal_state->getGlobalLinkTransform("l_sole").inverse() * goal_state->getGlobalLinkTransform("l_foot");
  Eigen::Isometry3d l_sole_to_l_ankle =
      goal_state->getGlobalLinkTransform("l_sole").inverse() * goal_state->getGlobalLinkTransform("l_ankle");
  // calc ankle_intersection
  Eigen::Vector3d ankle_roll_axis = l_sole_to_l_foot.rotation()
      * dynamic_cast<const moveit::core::RevoluteJointModel *>(goal_state->getJointModel("LAnkleRoll"))->getAxis();
  Eigen::Vector3d ankle_pitch_axis = l_sole_to_l_ankle.rotation()
      * dynamic_cast<const moveit::core::RevoluteJointModel *>(goal_state->getJointModel("LAnklePitch"))->getAxis();
  Eigen::Vector3d ankle_intersection_point;
  bool success = findIntersection(l_sole_to_l_foot.translation(),
                                  ankle_roll_axis,
                                  l_sole_to_l_ankle.translation(),
                                  ankle_pitch_axis,
                                  ankle_intersection_point);
  if (!success) {
    return false;
  }
  Eigen::Isometry3d ankle_intersection = l_sole_goal;
  ankle_intersection.translate(ankle_intersection_point);
  visual_tools_goal_->publishSphere(ankle_intersection_point);
  visual_tools_sole_->publishSphere(ankle_intersection_point, rviz_visual_tools::RED);
  ankle_ry_tf.header.frame_id = "base_link";
  ankle_ry_tf.header.stamp = ros::Time::now();
  ankle_ry_tf.child_frame_id = "ankle_ry";
  ankle_ry_tf.transform = tf2::eigenToTransform(ankle_intersection).transform;


  // get rid of static transform between base_link and hip_RY_intersect
  Eigen::Isometry3d base_link_to_hip_1 = goal_state->getGlobalLinkTransform("l_hip_1");
  Eigen::Isometry3d base_link_to_hip_2 = goal_state->getGlobalLinkTransform("l_hip_2");
  // calc ankle_intersection
  Eigen::Vector3d hip_yaw_axis = base_link_to_hip_1.rotation()
      * dynamic_cast<const moveit::core::RevoluteJointModel *>(goal_state->getJointModel("LHipYaw"))->getAxis();
  Eigen::Vector3d hip_roll_axis = base_link_to_hip_2.rotation()
      * dynamic_cast<const moveit::core::RevoluteJointModel *>(goal_state->getJointModel("LHipRoll"))->getAxis();
  Eigen::Vector3d hip_ry_intersection_point;
  success = findIntersection(base_link_to_hip_1.translation(),
                             hip_yaw_axis,
                             base_link_to_hip_2.translation(),
                             hip_roll_axis,
                             hip_ry_intersection_point);
  if (!success) {
    return false;
  }
  Eigen::Isometry3d hip_ry_intersection = Eigen::Isometry3d::Identity();
  hip_ry_intersection.translate(hip_ry_intersection_point);
  visual_tools_base_->publishSphere(hip_ry_intersection_point);
  hip_ry_tf.header.frame_id = "base_link";
  hip_ry_tf.header.stamp = ros::Time::now();
  hip_ry_tf.child_frame_id = "hip_ry";
  hip_ry_tf.transform = tf2::eigenToTransform(hip_ry_intersection).transform;

  // now the hip_ry_to_ankle_intersect describes the pose of the ankle_intersect in hip_RY_intersect frame
  Eigen::Isometry3d hip_ry_to_ankle_intersect = hip_ry_intersection.inverse() * ankle_intersection;
  // todo extracting euler angles do not work: they are not continuous
  Eigen::Vector3d goal_rpy = l_sole_goal.rotation().eulerAngles(0, 1, 2);
  visual_tools_base_->publishLine(hip_ry_intersection.translation(), ankle_intersection.translation());
  visual_tools_hip_ry_->publishAxis(hip_ry_to_ankle_intersect);

  // compute AnkleRoll
  /*// The rolls depend on the position of the ankle relative to the hip_ry_intersect, AFTER the yaw is applied.
  // for that, we can just multiply the yaw because the origin is on the hip yaw rotation axis
  Eigen::Isometry3d goal_with_yaw = Eigen::AngleAxisd(hip_yaw, Eigen::Vector3d::UnitZ()) * hip_ry_to_ankle_intersect;
  // Create triangle in y,z dimension consisting of hip_ry_to_ankle_intersect position vector and the y and z axis (in hip_RY_intersect frame)
  // Compute atan(hip_ry_to_ankle_intersect.y,hip_ry_to_ankle_intersect.z), this gives us the angle of the foot as if the hip_ry_to_ankle_intersect rotation would be zero.
  double ankle_roll = std::atan2(goal_with_yaw.translation().y(), -goal_with_yaw.translation().z());
  // This is only the angle coming from the hip_ry_to_ankle_intersect position. We need to simply add the hip_ry_to_ankle_intersect orientation in roll
  // We can compute this joint angles without knowing the HipYaw value, since the position is not influenced by it.
  // Because we use the exact intersection of HipYaw and HipRoll as a basis.*/

  Eigen::Isometry3d goal_rotated = hip_ry_to_ankle_intersect;
  //Eigen::Vector3d hip_to_ankle_translation = hip_ry_to_ankle_intersect.translation();
  visual_tools_hip_ry_->publishLine({0,0,0}, goal_rotated.translation(), rviz_visual_tools::YELLOW);
  double atan_1 = std::atan2(goal_rotated.translation().y(), -goal_rotated.translation().z());
  double atan_2 = std::atan2(goal_rotated.translation().x(), -goal_rotated.translation().z());
  /*// determine z axis in goal
  Eigen::Vector3d z_in_goal = hip_ry_to_ankle_intersect.rotation() * Eigen::Vector3d::UnitZ();
  // project intersection to intersection onto yz plane
  // normal is x axis in goal
  Eigen::Vector3d x_in_goal = hip_ry_to_ankle_intersect.rotation() * Eigen::Vector3d::UnitX();
  // scalar distance from point to plane along the normal
  double dist = x_in_goal.dot(hip_ry_to_ankle_intersect.inverse().translation());
  // subtract
  Eigen::Vector3d projection = hip_ry_to_ankle_intersect.inverse().translation() - dist * x_in_goal;
  std::cout << z_in_goal << std::endl;
  std::cout << projection << std::endl;
  // get angle between z axis in goal and intersection to intersection projected
  double ankle_roll = std::acos(z_in_goal.dot(projection)/projection.norm());*/
  double goal_yaw = goal_rpy.z();
  double ankle_roll = std::cos(goal_yaw) * atan_1 + std::sin(goal_yaw) * atan_2;

  //double ankle_roll = goal_rpy.x();
  goal_state->setJointPositions("LAnkleRoll", &ankle_roll);

  // We need to know the position of the HipPitch to compute the rest of the pitch joints.
  // Therefore we need to firstly compute the HipYaw joint, since it influences the position of the HipPitch
  // joint (as their axes do not intersect)
  // First we compute the plane of the leg (the plane in which all pitch joints are).
  // This plane contains the hip_ry_to_ankle_intersect position of the ankle_intersect. It can be defined by the normal which is the
  // Y axis in the ankle_intersect frame.
  // The hip_RY_intersect is also located on this plane.
  // We determine the intersection of this plane with the xy plane going through hip_ry_intersect
  // Then, we take the angle between this intersection line and the x axis
  // The intersection line can easily be computed with the cross product.

  // this is the rotational axis of the ankle pitch, todo get from urdf
  ankle_pitch_axis = (Eigen::AngleAxisd(ankle_roll, hip_ry_to_ankle_intersect.rotation() * Eigen::Vector3d::UnitX()) * Eigen::Vector3d::UnitY());
  visual_tools_ankle_ry_->publishABCDPlane(ankle_pitch_axis.x(), ankle_pitch_axis.y(), ankle_pitch_axis.z(), 0);
  ankle_pitch_axis = hip_ry_to_ankle_intersect.rotation() * ankle_pitch_axis;
  visual_tools_hip_ry_->publishLine({0,0,0}, ankle_pitch_axis, rviz_visual_tools::GREEN);
  // project to xy plane
  Eigen::Vector3d line = ankle_pitch_axis.cross(Eigen::Vector3d::UnitZ());  // this is the normal of the xy plane
  double sign = std::copysign(1.0, ankle_pitch_axis.x());  // todo explain why we use x here
  // calculate angle
  double hip_yaw = sign * std::acos(line.dot(Eigen::Vector3d::UnitX()) / line.norm());
  goal_state->setJointPositions("LHipYaw", &hip_yaw);
  visual_tools_hip_ry_->publishABCDPlane(ankle_pitch_axis.x(), ankle_pitch_axis.y(), ankle_pitch_axis.z(), 0);

  // Compute HipRoll
  Eigen::Isometry3d goal_with_yaw = Eigen::AngleAxisd(hip_yaw, Eigen::Vector3d::UnitZ()) * hip_ry_to_ankle_intersect;
  double hip_roll = std::atan2(goal_with_yaw.translation().y(), -goal_with_yaw.translation().z());
  goal_state->setJointPositions("LHipRoll", &hip_roll);

  // Represent the hip_ry_to_ankle_intersect in the HipPitch frame. Subtract transform from hip_RY_intersect to HipPitch
  goal_state->updateLinkTransforms();
  Eigen::Isometry3d base_link_to_hip_pitch = goal_state->getGlobalLinkTransform("l_upper_leg");
  l_sole_to_l_ankle = goal_state->getGlobalLinkTransform("l_sole").inverse() *
      goal_state->getGlobalLinkTransform("l_ankle");
  Eigen::Isometry3d base_link_to_l_ankle = l_sole_goal * l_sole_to_l_ankle;
  Eigen::Isometry3d hip_pitch_to_l_ankle = base_link_to_hip_pitch.inverse() * base_link_to_l_ankle;

  // rotation of hip_pitch_to_goal should be zero
  // Now we have a triangle of HipPitch, Knee and AnklePitch which we can treat as a prismatic joint.
  // First we can get the knee angle with the rule of cosine
  // here we get rid of the axis vertical to the plane that should be ignored
  // todo the axis is still hard coded
  Eigen::Isometry3d hip_pitch_to_knee = goal_state->getGlobalLinkTransform("l_upper_leg").inverse() * goal_state->getGlobalLinkTransform("l_lower_leg");
  double upper_leg_length = std::sqrt(std::pow(hip_pitch_to_knee.translation().x(), 2) + std::pow(hip_pitch_to_knee.translation().y(), 2));
  double upper_leg_length_2 = upper_leg_length * upper_leg_length;
  Eigen::Isometry3d knee_to_ankle_pitch = goal_state->getGlobalLinkTransform("l_lower_leg").inverse() * goal_state->getGlobalLinkTransform("l_ankle");
  double lower_leg_length = std::sqrt(std::pow(knee_to_ankle_pitch.translation().x(), 2) + std::pow(knee_to_ankle_pitch.translation().z(), 2));
  double lower_leg_length_2 = lower_leg_length * lower_leg_length;
  double hip_to_ankle_length = std::sqrt(std::pow(hip_pitch_to_l_ankle.translation().x(), 2) + std::pow(hip_pitch_to_l_ankle.translation().y(), 2));
  double hip_to_ankle_length_2 = hip_to_ankle_length * hip_to_ankle_length;
  double knee_angle = std::acos((upper_leg_length_2 + lower_leg_length_2 - hip_to_ankle_length_2) / (2 * upper_leg_length * lower_leg_length));
  double knee = M_PI - knee_angle;

  // Similarly, we can compute HipPitch and AnklePitch, but we need to add half of the knee angle.
  double hip_pitch = std::acos((upper_leg_length_2 + hip_to_ankle_length_2 - lower_leg_length_2) / (2 * upper_leg_length * hip_to_ankle_length));
  // add pitch of hip_pitch_to_ankle todo not actually hip_pitch_to_l_ankle because this is already rotated?? why??
  double foot_position_angle = std::atan2(-hip_pitch_to_l_ankle.translation().x(), -hip_pitch_to_l_ankle.translation().y());  // todo z axis is y axis?
  hip_pitch += foot_position_angle;

  // todo get from urdf
  knee += 15.0 / 180.0 * M_PI;

  double ankle_pitch = hip_pitch + knee_angle - M_PI;
  // ankle pitch needs hip_ry_to_ankle_intersect pitch
  ankle_pitch += goal_rpy.y();

  goal_state->setJointPositions("LHipPitch", &hip_pitch);
  goal_state->setJointPositions("LKnee", &knee);
  goal_state->setJointPositions("LAnklePitch", &ankle_pitch);

  visual_tools_base_->trigger();
  visual_tools_goal_->trigger();
  visual_tools_sole_->trigger();
  visual_tools_hip_ry_->trigger();
  visual_tools_ankle_ry_->trigger();

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

int main(int argc, char** argv) {
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  ros::Publisher p = nh.advertise<sensor_msgs::JointState>("/config/fake_controller_joint_states", 1, true);
  tf2_ros::TransformBroadcaster br;

  wolfgang_ik::IK ik;

  Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
  goal.translation().x() = std::stod(argv[1]);
  goal.translation().y() = std::stod(argv[2]);
  goal.translation().z() = std::stod(argv[3]);
  Eigen::Quaterniond q = Eigen::AngleAxisd(std::stod(argv[4]), Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(std::stod(argv[5]), Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(std::stod(argv[6]), Eigen::Vector3d::UnitZ());
  goal.matrix().block<3, 3>(0, 0) = Eigen::Matrix3d(q);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description", true);
  const robot_model::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  if (!kinematic_model) {
    ROS_FATAL("No robot model loaded, unable to run IK");
    exit(1);
  }
  robot_state::RobotStatePtr result;
  result.reset(new robot_state::RobotState(kinematic_model));

  geometry_msgs::TransformStamped hip_ry_tf, ankle_ry_tf;
  ik.solve(goal, hip_ry_tf, ankle_ry_tf, result);
  result->updateLinkTransforms();

  Eigen::Isometry3d l_sole = result->getGlobalLinkTransform("l_sole");
  std::cout << l_sole;

  robot_state::RobotStatePtr truth;
  truth.reset(new robot_state::RobotState(kinematic_model));
  robot_model::JointModelGroup* left_leg_group = kinematic_model->getJointModelGroup("LeftLeg");
  truth->setFromIK(left_leg_group, goal, 0.01);
  for (const auto& j : left_leg_group->getJointModels()) {
    std::cout << j->getName() << " should be " << *truth->getJointPositions(j)
              << " but is " << *result->getJointPositions(j) << std::endl;
  }

  sensor_msgs::JointState joint_state;
  joint_state.name = kinematic_model->getJointModelGroup("LeftLeg")->getJointModelNames();
  result->copyJointGroupPositions("LeftLeg", joint_state.position);
  p.publish(joint_state);

  // publish goal as tf frame
  geometry_msgs::TransformStamped goal_tf;
  goal_tf.child_frame_id = "sole_goal";
  goal_tf.header.stamp = ros::Time::now();
  goal_tf.header.frame_id = "base_link";
  goal_tf.transform = tf2::eigenToTransform(goal).transform;
  while (ros::ok()) {
    br.sendTransform(goal_tf);
    br.sendTransform(hip_ry_tf);
    br.sendTransform(ankle_ry_tf);
    ros::Duration(0.01).sleep();
  }
}