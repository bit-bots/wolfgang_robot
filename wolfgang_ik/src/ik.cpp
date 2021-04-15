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
  // richtig!
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

  // falsch!
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

  // falsch!
  // calculate translation from hip_ry_intersect to hip_rp_intersect
  Eigen::Isometry3d base_link_to_upper_leg = goal_state->getGlobalLinkTransform("l_upper_leg");
  Eigen::Vector3d hip_pitch_axis = base_link_to_upper_leg.rotation()
      * dynamic_cast<const moveit::core::RevoluteJointModel *>(goal_state->getJointModel("LHipPitch"))->getAxis();
  Eigen::Vector3d hip_rp_intersection_point;
  success = findIntersection(base_link_to_hip_2.translation(),
                             hip_roll_axis,
                             base_link_to_upper_leg.translation(),
                             hip_pitch_axis,
                             hip_rp_intersection_point);

  if (!success) {
    return false;
  }
  Eigen::Vector3d hip_yaw_pitch_offset = {(hip_rp_intersection_point - hip_ry_intersection_point).norm(), 0, 0};

  // now the hip_ry_to_ankle_intersect describes the pose of the ankle_intersect in hip_RY_intersect frame
  Eigen::Isometry3d hip_ry_to_ankle_intersect = hip_ry_intersection.inverse() * ankle_intersection;
  // todo extracting euler angles do not work: they are not continuous
  Eigen::Vector3d goal_rpy = l_sole_goal.rotation().eulerAngles(0, 1, 2);
  visual_tools_base_->publishLine(hip_ry_intersection.translation(), ankle_intersection.translation());
  visual_tools_hip_ry_->publishAxis(hip_ry_to_ankle_intersect);

  // richtig
  Eigen::Vector3d footLocation = ankle_intersection.translation();
  Eigen::Matrix3d footRot = ankle_intersection.rotation();
  Eigen::Isometry3d hip_pitch_to_knee = goal_state->getGlobalLinkTransform("l_upper_leg").inverse() * goal_state->getGlobalLinkTransform("l_lower_leg");
  double thigh_length = std::sqrt(std::pow(hip_pitch_to_knee.translation().x(), 2) + std::pow(hip_pitch_to_knee.translation().y(), 2));
  double thigh_length_2 = thigh_length * thigh_length;
  Eigen::Isometry3d knee_to_ankle_pitch = goal_state->getGlobalLinkTransform("l_lower_leg").inverse() * goal_state->getGlobalLinkTransform("l_ankle");
  double shank_length = std::sqrt(std::pow(knee_to_ankle_pitch.translation().x(), 2) + std::pow(knee_to_ankle_pitch.translation().z(), 2));
  double shank_length_2 = shank_length * shank_length;
  double length_div = 2.0 * shank_length * thigh_length;

  // everything from here is copied from the nimbro ik
  // https://github.com/NimbRo/nimbro-op-ros/blob/a286de0de30b8e36ba1cbb0d2e4a083d9f1325e2/src/nimbro/tools/trajectory_editor/src/leg_ik.cpp

  Eigen::Vector3d goal = hip_ry_intersection.rotation() * (footLocation - hip_ry_intersection.translation());

  // Rotation semantics: All rotations convert from local coordinates to
  // global coordinates.

  // The general idea is to treat the shank-knee-thigh assembly as a prismatic
  // joint. The knee angle can then be calculated in a last step from the
  // distance between ankle and hip.

  // 1) Determine the roll angle at the foot
  Eigen::Vector3d goal_foot = footRot.transpose() * (-goal);

  if(goal_foot.z() == 0) {
    return false;
  }

  double roll_angle = atan(goal_foot.y() / goal_foot.z());
  Eigen::Matrix3d rot_ankle_roll;
  rot_ankle_roll = Eigen::AngleAxisd(roll_angle, Eigen::Vector3d::UnitX());

  // Eliminate the roll from the following equations
  goal_foot = rot_ankle_roll * goal_foot;

  // Okay, the foot is in a fixed pose and the ankle roll angle is already
  // determined. This means our hip can move on a plane defined by
  // the ankle pitch axis (as the normal).

  // In particular, the hip roll axis needs to lie completely inside the
  // plane. We can use that to calculate the hip yaw.

  // The plane normal
  Eigen::Vector3d normal = footRot * rot_ankle_roll.transpose() * Eigen::Vector3d::UnitY();
  visual_tools_hip_ry_->publishABCDPlane(normal.x(), normal.y(), normal.z(), 0);

  // We are only interested in the direction of the intersection (and we know
  // one exists as we rotated the plane in step 1) to that effect)
  Eigen::Vector3d intersection = normal.cross(Eigen::Vector3d::UnitZ());

  if(intersection.x() == 0) {
    return false;
  }

  // We need to rotate the X axis onto the intersection.
  double yaw_angle = atan2(intersection.y(), intersection.x());

  Eigen::Matrix3d rot_yaw;
  rot_yaw = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

  // Determine the location of the intersection hip_roll/pitch in foot coordinates
  visual_tools_hip_ry_->publishSphere(rot_yaw * hip_yaw_pitch_offset);
  Eigen::Vector3d pitch_goal_foot = rot_ankle_roll * footRot.transpose() * (-goal + rot_yaw * hip_yaw_pitch_offset);
  visual_tools_ankle_ry_->publishLine({0,0,0}, footRot.transpose() * pitch_goal_foot);

  if(pitch_goal_foot.z() == 0) {
    return false;
  }

  double pitch_angle = atan(-pitch_goal_foot.x() / pitch_goal_foot.z());

  Eigen::Matrix3d rot_ankle_pitch;
  rot_ankle_pitch = Eigen::AngleAxisd(pitch_angle, Eigen::Vector3d::UnitY());

  // Determine missing angles in the hip
  auto rot = footRot * rot_ankle_roll.transpose() * rot_ankle_pitch.transpose();
  auto iso = Eigen::Isometry3d::Identity();
  iso.translate(hip_yaw_pitch_offset);
  iso.rotate(rot);
  visual_tools_hip_ry_->publishAxis(iso);
  Eigen::Vector3d euler = rot.eulerAngles(2, 0, 1);
  double hip_pitch_angle = euler(2);
  double hip_roll_angle = euler(1);
  // CHANGE manual correction of euler angles in extreme situations
  std::cout << euler << std::endl;
  if (euler(0) > M_PI_2) {
    hip_pitch_angle = M_PI + hip_pitch_angle;
    hip_roll_angle = M_PI - hip_roll_angle;
  }

  // Now we can replace the prismatic joint with the real knee
  double length = pitch_goal_foot.norm();

  // Calculate the angle between the shank and thigh links (Law of cosines)
  double knee_len = (shank_length_2 + thigh_length_2 - length*length) / length_div;
  double knee_angle = acos(knee_len);
  if(isnan(knee_angle)) {
    return false; // Goal is too far away
  }

  // CHANGE: calculate hip angle in hip-knee-ankle triangle (Law of cosines)
  double hip_angle = acos((thigh_length_2 + length*length - shank_length_2) / (2 * thigh_length * length));

  // CHANGE: ankle roll and hip yaw had to be inverted, 15 degree offset added to knee pitch
  //         calculate hip pitch from hip angle and hip pitch angle
  //         calculate ankle pitch from other angles
  // Hip pitch
  double hip_pitch_motor = hip_angle - hip_pitch_angle;
  // Hip yaw
  double hip_yaw_motor = -yaw_angle;
  // Hip roll
  double hip_roll_motor = hip_roll_angle;
  // Knee pitch
  double knee_pitch_motor = M_PI - knee_angle + 15.0 / 180.0 * M_PI;
  // Ankle roll
  double ankle_roll_motor = -roll_angle;
  // Ankle pitch
  double ankle_pitch_motor = pitch_angle + hip_angle + knee_angle - M_PI;

  goal_state->setJointPositions("LHipRoll", &hip_roll_motor);
  goal_state->setJointPositions("LHipPitch", &hip_pitch_motor);
  goal_state->setJointPositions("LHipYaw", &hip_yaw_motor);
  goal_state->setJointPositions("LKnee", &knee_pitch_motor);
  goal_state->setJointPositions("LAnklePitch", &ankle_pitch_motor);
  goal_state->setJointPositions("LAnkleRoll", &ankle_roll_motor);

  visual_tools_ankle_ry_->trigger();
  visual_tools_hip_ry_->trigger();
  visual_tools_base_->trigger();

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
  std::cout << "FK of bio ik:" << std::endl << truth->getGlobalLinkTransform("l_sole");

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
  br.sendTransform(goal_tf);
  br.sendTransform(hip_ry_tf);
  br.sendTransform(ankle_ry_tf);
  while (ros::ok()) {
    ros::Duration(0.01).sleep();
  }
}