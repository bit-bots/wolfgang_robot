#include "wolfgang_ik/ik.h"

namespace wolfgang_ik {
    IK::IK(){
    // get robot model state
      robot_model_loader_.loadKinematicsSolvers(std::make_shared<kinematics_plugin_loader::KinematicsPluginLoader>());
      robot_model::RobotModelPtr kinematic_model = robot_model_loader_.getModel();
      if (!kinematic_model) {
        ROS_FATAL("No robot model loaded, killing dynup.");
        exit(1);
      }
      robot_state::RobotStatePtr init_state;
      init_state.reset(new robot_state::RobotState(kinematic_model));
      // set elbows to make arms straight, in a stupid way since moveit is annoying
      std::vector<std::string> names_vec = {"LElbow", "RElbow"};
      std::vector<double> pos_vec = {-M_PI/2, M_PI/2};
      init_state->setJointPositions(names_vec[0], &pos_vec[0]);
      init_state->setJointPositions(names_vec[1], &pos_vec[1]);
      init_state->updateLinkTransforms();
      // get shoulder and wrist pose
      geometry_msgs::Pose shoulder_origin, wrist_origin;
      tf2::convert(init_state->getGlobalLinkTransform("l_upper_arm"), shoulder_origin);
      tf2::convert(init_state->getGlobalLinkTransform("l_wrist"), wrist_origin);
      //compute arm length
      double arm_max_length = shoulder_origin.position.z - wrist_origin.position.z;

    // compute the necessary link length and save them as class variables


    }

    bool IK::solve(tf2::Transform goal, std::vector<double> joint_positions){
        // some naming conventions:
        // hip_RY_intersect: the point where the HipYaw and HipRoll axis intersect
        // ankle_intersect: the point where AnklePitch and AnkleRoll intersect

        // the provided goal points from base_link to sole link
        // first get rid of the static transform between sole and ankle_intersect
        // vector rotate with goal rotation and substract

        // get rid of static transform between base_link and hip_RY_intersect
        // directly substract, no rotation necessary

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
    }

}
