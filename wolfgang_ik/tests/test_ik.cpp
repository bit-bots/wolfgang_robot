#include <wolfgang_ik/ik.h>
#include <gtest/gtest.h>

TEST(TestIK, test_ik) {
  wolfgang_ik::IK ik;

  Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
  goal.translation().y() = 0.2;
  goal.translation().z() = -0.3;
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description", false);
  const robot_model::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  if (!kinematic_model) {
    ROS_FATAL("No robot model loaded, unable to run IK");
    exit(1);
  }
  robot_state::RobotStatePtr result;
  result.reset(new robot_state::RobotState(kinematic_model));
  ASSERT_TRUE(ik.solve(goal, result));
  result->updateLinkTransforms();
  Eigen::Isometry3d foot_result = result->getGlobalLinkTransform("l_sole");
  ASSERT_TRUE(foot_result.isApprox(goal));
  ASSERT_DOUBLE_EQ((foot_result.translation() - goal.translation()).norm(), 0);
  // if two quaternions are equal, their dot product is 1
  ASSERT_TRUE(foot_result.rotation().isApprox(goal.rotation()));
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