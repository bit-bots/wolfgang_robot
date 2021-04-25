import yaml
from controller import Supervisor
import transforms3d
import numpy as np
import math

class PosePlayController:
    def __init__(self, file):
        with open(file) as f:
            p = yaml.load(f, Loader=yaml.Loader)
        self.name = "amy"
        self.poses = {}
        self.poses[self.name] = p
        s = Supervisor()
        self.robot_nodes = {"amy": s.getFromDef("amy")}
        to_delete = []
        print(f"There are {len(p['poses'])} poses in this file")
        input("Enter to start...")
        for n in range(len(p["poses"])):
            self.play_nth(n)
            s.step(int(s.getBasicTimeStep()))
            s.step(int(s.getBasicTimeStep()))
            i = input(f"{n}: {p['poses'][n]['motion']} pose")
            if i == "d":
                to_delete.append(n)
                print(f"will delete {n}")

        new_poses = []
        for n in range(len(p["poses"])):
            if n not in to_delete:
                new_poses.append(p["poses"][n])
        with open(file + "_new", "w") as fout:
            yaml.dump({"poses": new_poses}, fout)



    def play_nth(self, n):
        for joint_name, initial_rot in self.poses[self.name]["initial_rot"].items():
            rot_axis = self.poses[self.name]["axes"][joint_name]
            rotation = self.poses[self.name]["poses"][n][joint_name]
            # todo save to yaml what pose is used for which robot
            initial_rot_mat = transforms3d.axangles.axangle2mat(axis=initial_rot[:3], angle=initial_rot[3])
            joint_rot_mat = transforms3d.axangles.axangle2mat(axis=rot_axis, angle=rotation)
            combined_rot_mat = np.matmul(joint_rot_mat, initial_rot_mat)
            axis, angle = transforms3d.axangles.mat2axangle(combined_rot_mat)
            self.robot_nodes[self.name].getField(joint_name + "Rot").setSFRotation([*axis, angle])
        robot_pose = self.poses[self.name]["poses"][n]["pose"]
        position = [0,0, robot_pose["position"]["z"]]
        orientation = [robot_pose["orientation"]["w"], robot_pose["orientation"]["x"], robot_pose["orientation"]["y"], robot_pose["orientation"]["z"]]
        rpy = transforms3d.euler.quat2euler(orientation)
        self.set_robot_pose_rpy(position, [rpy[0], rpy[1], 0])

    def set_robot_pose_rpy(self, pos, rpy, name="amy"):
        self.set_robot_position(pos, name)
        self.set_robot_rpy(rpy, name)
        self.robot_nodes[name].resetPhysics()

    def set_robot_rpy(self, rpy, name="amy"):
        axis, angle = transforms3d.euler.euler2axangle(rpy[0], rpy[1], rpy[2], axes='sxyz')
        self.set_robot_axis_angle(axis, angle, name)

    def set_robot_axis_angle(self, axis, angle, name="amy"):
        self.robot_nodes[name].getField("rotation").setSFRotation(list(np.append(axis, angle)))

    def set_robot_position(self, pos, name="amy"):
        self.robot_nodes[name].getField("translation").setSFVec3f(list(pos))

p = PosePlayController("capture_kick.yaml_new")