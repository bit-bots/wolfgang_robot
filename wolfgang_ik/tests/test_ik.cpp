#include <wolfgang_ik/ik.h>
#include <gtest/gtest.h>

const double TRANSLATION_TOLERANCE = 0.01;
const double ROTATION_TOLERANCE = 0.001;

bool test_ik(const Eigen::Isometry3d &request, Eigen::Isometry3d &result) {
  wolfgang_ik::IK ik;

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description", false);
  const robot_model::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
  if (!kinematic_model) {
    ROS_FATAL("No robot model loaded, unable to run IK");
    exit(1);
  }
  robot_state::RobotStatePtr state;
  state.reset(new robot_state::RobotState(kinematic_model));
  bool success = ik.solve(request, state);
  state->updateLinkTransforms();

  result = state->getGlobalLinkTransform("l_sole");
  return success;
}

Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw) {
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  return q;
}

TEST(TestIK, test_ik_position) {
  Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
  goal.translation().x() = 0.1;
  goal.translation().y() = 0.08;
  goal.translation().z() = -0.3;
  Eigen::Isometry3d result;
  ASSERT_TRUE(test_ik(goal, result));
  ASSERT_TRUE((result.translation() - goal.translation()).norm() < TRANSLATION_TOLERANCE);
  // if two quaternions are equal, their dot product is 1
  ASSERT_TRUE(std::abs(Eigen::Quaterniond(result.rotation()).dot(Eigen::Quaterniond(goal.rotation())) - 1) < ROTATION_TOLERANCE);
}

TEST(TestIK, test_ik_roll) {
  Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
  goal.translation().x() = 0;
  goal.translation().y() = 0;
  goal.translation().z() = -0.35;
  goal.matrix().block<3, 3>(0, 0) = Eigen::Matrix3d(euler_to_quat(1, 0, 0));
  Eigen::Isometry3d result;
  ASSERT_TRUE(test_ik(goal, result));
  ASSERT_TRUE((result.translation() - goal.translation()).norm() < TRANSLATION_TOLERANCE);
  // if two quaternions are equal, their dot product is 1
  ASSERT_TRUE(std::abs(Eigen::Quaterniond(result.rotation()).dot(Eigen::Quaterniond(goal.rotation())) - 1) < ROTATION_TOLERANCE);
}

TEST(TestIK, test_ik_pitch) {
  Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
  goal.translation().x() = 0;
  goal.translation().y() = 0;
  goal.translation().z() = -0.35;
  goal.matrix().block<3, 3>(0, 0) = Eigen::Matrix3d(euler_to_quat(0.0, 1, 0));
  Eigen::Isometry3d result;
  ASSERT_TRUE(test_ik(goal, result));
  ASSERT_TRUE((result.translation() - goal.translation()).norm() < TRANSLATION_TOLERANCE);
  // if two quaternions are equal, their dot product is 1
  ASSERT_TRUE(std::abs(Eigen::Quaterniond(result.rotation()).dot(Eigen::Quaterniond(goal.rotation())) - 1) < ROTATION_TOLERANCE);
}

TEST(TestIK, test_ik_yaw) {
  Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
  goal.translation().x() = 0;
  goal.translation().y() = 0;
  goal.translation().z() = -0.35;
  goal.matrix().block<3, 3>(0, 0) = Eigen::Matrix3d(euler_to_quat(0, 0, 1));
  Eigen::Isometry3d result;
  ASSERT_TRUE(test_ik(goal, result));
  ASSERT_TRUE((result.translation() - goal.translation()).norm() < TRANSLATION_TOLERANCE);
  // if two quaternions are equal, their dot product is 1
  ASSERT_TRUE(std::abs(Eigen::Quaterniond(result.rotation()).dot(Eigen::Quaterniond(goal.rotation())) - 1) < ROTATION_TOLERANCE);
}

TEST(TestIK, test_ik_pose) {
  Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
  goal.translation().x() = 0.12;
  goal.translation().y() = 0.13;
  goal.translation().z() = -0.3;
  goal.matrix().block<3, 3>(0, 0) = Eigen::Matrix3d(euler_to_quat(0.4, 1, -0.4));
  Eigen::Isometry3d result;
  ASSERT_TRUE(test_ik(goal, result));
  std::cout << result;
  ASSERT_TRUE((result.translation() - goal.translation()).norm() < TRANSLATION_TOLERANCE);
  // if two quaternions are equal, their dot product is 1
  ASSERT_TRUE(std::abs(Eigen::Quaterniond(result.rotation()).dot(Eigen::Quaterniond(goal.rotation())) - 1) < ROTATION_TOLERANCE);
}

TEST(TestIK, test_intersection) {
  Eigen::Vector3d p1(0, 0, 0);
  Eigen::Vector3d p2(0, 0, 0);
  Eigen::Vector3d v1(1, 0, 0);
  Eigen::Vector3d v2(0, 1, 0);
  Eigen::Vector3d res;
  ASSERT_TRUE(wolfgang_ik::IK::findIntersection(p1, v1, p2, v2, res));
  ASSERT_DOUBLE_EQ((Eigen::Vector3d(0, 0, 0) - res).norm(), 0);

  p1 = {1, 1, 1};
  p2 = {0, 2, 0};
  v1 = {1, 0, 0};
  v2 = {0, 1, 0};
  ASSERT_FALSE(wolfgang_ik::IK::findIntersection(p1, v1, p2, v2, res));

  p1 = {1, 1, 1};
  p2 = {0, 2, 0};
  v1 = {1, 0, 0};
  v2 = {1, 0, 0};
  ASSERT_FALSE(wolfgang_ik::IK::findIntersection(p1, v1, p2, v2, res));

  p1 = {1, 1, 1};
  p2 = {0, 2, 0};
  v1 = {1, 1, 1};
  v2 = {0, 1, 0};
  ASSERT_TRUE(wolfgang_ik::IK::findIntersection(p1, v1, p2, v2, res));
  ASSERT_DOUBLE_EQ((Eigen::Vector3d(0, 0, 0) - res).norm(), 0);

  p1 = {10, 5, 1};
  p2 = {-3.5, -2.5, 4};
  v1 = {5, 1, 7};
  v2 = {7, 3, 2.5};
  ASSERT_TRUE(wolfgang_ik::IK::findIntersection(p1, v1, p2, v2, res));
  ASSERT_DOUBLE_EQ((Eigen::Vector3d(17.5, 6.5, 11.5) - res).norm(), 0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}