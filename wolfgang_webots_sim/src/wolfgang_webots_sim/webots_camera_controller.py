from controller import Robot, Node, Supervisor, Field

import os

import random
import math
import time

from geometry_msgs.msg import Point, Pose
from skimage.morphology import convex_hull_image

import transforms3d
import numpy as np
import cv2
import yaml

G = 9.81
DARWIN_PITCH = 0.225
LINE_WIDTH = 0.05
LINE_WIDTH_HALF = LINE_WIDTH/2
T_INTERSECTIONS = [[-4.5 + LINE_WIDTH_HALF,  1.5 - LINE_WIDTH_HALF], # check
                   [-4.5 + LINE_WIDTH_HALF, -1.5 + LINE_WIDTH_HALF], # check penalty area home
                   [-4.5 + LINE_WIDTH_HALF,  2.5 - LINE_WIDTH_HALF], # check
                   [-4.5 + LINE_WIDTH_HALF, -2.5 + LINE_WIDTH_HALF], # check goal area home
                   [4.5 - LINE_WIDTH_HALF,  1.5 - LINE_WIDTH_HALF],
                   [4.5 - LINE_WIDTH_HALF, -1.5 + LINE_WIDTH_HALF], # penalty area far
                   [4.5 - LINE_WIDTH_HALF,  2.5 - LINE_WIDTH_HALF],
                   [4.5 - LINE_WIDTH_HALF, -2.5 + LINE_WIDTH_HALF], # goal area far
                   [0, 3 - LINE_WIDTH_HALF], [0, -3 + LINE_WIDTH_HALF]]

X_INTERSECTIONS = [[0, 0.75 - LINE_WIDTH_HALF],
                   [0, -0.75 + LINE_WIDTH_HALF],
                   [0, 0],  # center circle + center mark
                   [3, 0], [-3, 0]]  # penalty marks
L_INTERSECTIONS = [[-4.5 + LINE_WIDTH_HALF, -3 + LINE_WIDTH_HALF],
                   [-4.5 + LINE_WIDTH_HALF, 3 - LINE_WIDTH_HALF],  # check
                   [4.5 - LINE_WIDTH_HALF, -3 + LINE_WIDTH_HALF],
                   [4.5 - LINE_WIDTH_HALF,  3 - LINE_WIDTH_HALF],  # field corners
                   [-3.5 - LINE_WIDTH_HALF,  1.5 - LINE_WIDTH_HALF],
                   [-3.5 - LINE_WIDTH_HALF, -1.5 + LINE_WIDTH_HALF],
                   [3.5 + LINE_WIDTH_HALF,  1.5 - LINE_WIDTH_HALF],
                   [3.5 + LINE_WIDTH_HALF, -1.5 + LINE_WIDTH_HALF],  # penalty areas
                   [-2.5 - LINE_WIDTH_HALF, -2.5 + LINE_WIDTH_HALF],
                   [-2.5 - LINE_WIDTH_HALF, 2.5 - LINE_WIDTH_HALF],  # goal areas
                   [2.5 + LINE_WIDTH_HALF, -2.5 + LINE_WIDTH_HALF],
                   [2.5 + LINE_WIDTH_HALF, 2.5 - LINE_WIDTH_HALF],
                   ]


class CameraController:
    def __init__(self):
        self.time = 0
        self.supervisor = Supervisor()

        self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)

        self.motors = []
        self.sensors = []
        self.timestep = int(self.supervisor.getBasicTimeStep())

        # resolve the node for corresponding name
        self.robot_names = ["RED1", "RED2", "RED3", "BLUE1", "BLUE2", "BLUE3", "BLUE4", "FakeCamera"]
        self.robot_nodes = {}
        self.translation_fields = {}
        self.rotation_fields = {}

        # check if None
        for name in self.robot_names:
            node = self.supervisor.getFromDef(name)
            if node is not None:
                self.robot_nodes[name] = node
                self.translation_fields[name] = node.getField("translation")
                self.rotation_fields[name] = node.getField("rotation")

        self.robot_node = self.supervisor.getSelf()
        self.camera = self.supervisor.getDevice("camera")
        self.camera.enable(self.timestep)
        self.depth = self.supervisor.getDevice("depth")
        self.depth.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
        self.camera.enableRecognitionSegmentation()
        self.last_img_saved = 0.0

        self.world_info = self.supervisor.getFromDef("world_info")
        self.ball = self.supervisor.getFromDef("ball")
        self.cam_looks_at_ball = True
        self.fov = []
        self.camera_poses = []
        self.robot_poses = []
        self.ball_positions = []
        self.annotations = []
        self.img_save_dir = "img/images"
        self.seg_save_dir = "img/segmentations"
        self.depth_save_dir = "img/depth"
        self.filenames = []
        self.intersections = []
        self.ball_in_image = True
        self.uniform_camera_position = True
        self.i = 0
        self.poses = {}
        wolfgang_poses = None
        with open("wolfgang_poses.yaml", "r") as f:
            wolfgang_poses = yaml.load(f, Loader=yaml.Loader)

        for r in self.robot_names[:3]:
            self.poses[r] = wolfgang_poses


    def random_placement(self, size="kid"):
        # gleichverteilung über x und y, z: robot description
        if size == "kid":
            z_mean = 0.64
            z_std_dev = 0.12
            z_range = (0.45, 0.95)
            x_range = (-5.0, 5.0)
            y_range = (-3.5, 3.5)
            fov_range = (60, 180)
            fov_mean = 89.3
            fov_std_dev = 28.1
        elif size == "adult":
            z_mean = 1.37571428571429
            z_std_dev = 0.139386410487434
            z_range = (1.10, 1.80)
            x_range = (-7.5, 7.5)
            y_range = (-5.0, 5.0)
            fov_range = (70, 130)
            fov_mean = 93.3333333333333
            fov_std_dev = 22.5092573548455
        else:
            print("size not correct, ether kid or adult")
            return

        z_choice = None
        while z_choice is None:
            z_prelim = np.random.normal(loc=z_mean, scale=z_std_dev)
            if z_range[0] < z_prelim < z_range[1]:
                z_choice = z_prelim

        ball_pos = Point(random.random() * 9.5 - 4.75, random.random() * 6 - 3, 0.08)
        self.ball_positions.append(ball_pos)
        if self.ball_in_image:
            self.set_ball(orientation="rand", position=ball_pos)
        else:
            self.set_ball(orientation="rand", position=Point(0,0,100))

        fov = None
        while fov is None:
            fov_prelim = np.random.normal(loc=fov_mean, scale=fov_std_dev)
            if fov_range[0] < fov_prelim < fov_range[1]:
                fov = fov_prelim
                fov = math.radians(fov)
        self.robot_node.getField("cameraFOV").setSFFloat(fov)
        self.fov.append(fov)

        robot_height_red = 0.41
        # todo clip too far, resample instead
        goalie_y_red = None
        while goalie_y_red is None:
            goalie_y_red_prelim = np.random.normal(loc=0, scale=0.5)
            if -1.4 < goalie_y_red_prelim < 1.4:
                goalie_y_red = goalie_y_red_prelim
        goalie_pos_red = [x_range[0] + 0.5, goalie_y_red, robot_height_red]
        goalie_rpy_red = [0.0, 0, np.random.normal(loc=0.0, scale=0.3)]
        self.reset_robot_pose_rpy(goalie_pos_red, goalie_rpy_red, name="RED1")

        robot_height_blue = 0.327
        goalie_y_blue = None
        while goalie_y_blue is None:
            goalie_y_blue_prelim = np.random.normal(loc=0, scale=0.5)
            if -1.4 < goalie_y_blue_prelim < 1.4:
                goalie_y_blue = goalie_y_blue_prelim
        goalie_pos_blue = [x_range[1] - 0.5, goalie_y_blue, robot_height_blue]
        goalie_rpy_blue = [0.0, DARWIN_PITCH, math.pi + np.random.normal(loc=0.0, scale=0.3)]
        self.reset_robot_pose_rpy(goalie_pos_blue, goalie_rpy_blue, name="BLUE1")

        num_strikers_red = 2  # random.randint(0, 2)
        camera_is_striker = True  # bool(random.randint(0, 1))
        num_strikers_blue = 3  # random.randint(0, 3)
        num_defenders_red = 2 - num_strikers_red
        num_defenders_blue = 3 - num_strikers_blue
        positions = {"RED1": [goalie_pos_red, goalie_rpy_red], "BLUE1": [goalie_pos_blue, goalie_rpy_blue]}
        for i in range(num_strikers_red):
            name = "RED" + str(i + 2)
            r = self.place_striker(name, ball_pos, robot_height_red, [0.0, 0.03, 0.0], positions)
            positions[name] = r
        for i in range(num_strikers_blue):
            name = "BLUE" + str(i + 2)
            r = self.place_striker(name, ball_pos, robot_height_blue, [0.0, DARWIN_PITCH, 0.0],
                                   positions)
            positions[name] = r

        for i in range(num_defenders_red):
            self.place_defender("RED" + str(num_strikers_red + i + 2), "RED", robot_height_red, [0.0, 0.0, 0.0],
                                positions)
        for i in range(num_defenders_blue):
            self.place_defender("BLUE" + str(num_strikers_blue + i + 2), "BLUE", robot_height_blue,
                                [0.0, DARWIN_PITCH, 0.0], positions)
        self.robot_poses.append(positions)
        self.place_camera(ball_pos, z_choice, positions, x_range, y_range, fov)

    def place_striker(self, name, ball_pos, height, orientation, other_positions):
        robot_pos = Point(ball_pos.x, ball_pos.y, ball_pos.z)
        # TODO collision check with other robots (lt < 0.2 m away or something like that)
        while True:
            preliminary_pos_x = robot_pos.x + np.random.normal(0, 2)
            preliminary_pos_y = robot_pos.x + np.random.normal(0, 2)
            pos_collides = self.position_collides(preliminary_pos_x, preliminary_pos_y, other_positions, name)
            if -5 < preliminary_pos_x < 5 and -3.5 < preliminary_pos_y < 3.5 and not pos_collides:
                robot_pos.x = preliminary_pos_x
                robot_pos.y = preliminary_pos_y
                break
        robot_pos.z = height
        if self.ball_in_image:
            robot_to_ball = [ball_pos.x - robot_pos.x, ball_pos.y - robot_pos.y]
            orientation[2] = math.atan2(robot_to_ball[1], robot_to_ball[0])
            orientation[2] += np.random.normal(0, np.pi/2)
        else:
            orientation[2] = random.random() * math.pi * 2 - math.pi

        if name in self.poses.keys():
            for joint_name, initial_rot in self.poses[name]["initial_rot"].items():
                rot_axis = self.poses[name]["axes"][joint_name]
                rotation = math.radians(self.poses[name]["poses"][0][joint_name])
                initial_rot_mat = transforms3d.axangles.axangle2mat(axis=initial_rot[:3], angle=initial_rot[3])
                joint_rot_mat = transforms3d.axangles.axangle2mat(axis=rot_axis, angle=rotation)
                combined_rot_mat = np.matmul(joint_rot_mat, initial_rot_mat)
                axis, angle = transforms3d.axangles.mat2axangle(combined_rot_mat)
                print(joint_name)
                self.robot_nodes[name].getField(joint_name + "Rot").setSFRotation([*axis, angle])

        self.reset_robot_pose_rpy([robot_pos.x, robot_pos.y, robot_pos.z], orientation, name)
        return [[robot_pos.x, robot_pos.y, robot_pos.z], orientation]

    def position_collides(self, x, y, others, name):
        for k,o in others.items():
            if math.sqrt((x-o[0][0]) ** 2 + (y - o[0][1]) ** 2) < 0.2:
                return True
        return False


    def place_defender(self, name, side, height, orientation, other_positions):
        print(f"placing {name} as a defender on side {side}")

    def place_camera(self, ball_pos, height, other_positions, x_range, y_range, fov):
        if self.cam_looks_at_ball:
            if self.uniform_camera_position:
                cam_position = [random.random() * (x_range[1] - x_range[0]) + x_range[0],
                                random.random() * (y_range[1] - y_range[0]) + y_range[0],
                                height]
            else:
                cam_pos_x = None
                while cam_pos_x is None:
                    cam_pos_x_prelim = ball_pos.x + np.random.normal(loc=0, scale=1.5)
                    if x_range[0] < cam_pos_x_prelim < x_range[1]:
                        cam_pos_x = cam_pos_x_prelim
                cam_pos_y = None
                while cam_pos_y is None:
                    cam_pos_y_prelim = ball_pos.x + np.random.normal(loc=0, scale=1.5)
                    if y_range[0] < cam_pos_y_prelim < y_range[1]:
                        cam_pos_y = cam_pos_y_prelim
                cam_position = [cam_pos_x, cam_pos_y, height]

            # this should be the x vector of the camera coordinate system
            cam_to_ball = [ball_pos.x - cam_position[0], ball_pos.y - cam_position[1], ball_pos.z - cam_position[2]]

            yaw = math.atan2(cam_to_ball[0], -cam_to_ball[1]) - math.pi / 2  # dont ask why, it works
            pitch = math.asin(math.fabs(cam_to_ball[2]) / np.linalg.norm(cam_to_ball))
            new_rpy = [0, pitch, yaw]

            width = 1920
            height = 1080
            ball_radius = 0.13 / 2
            dist_to_ball = np.linalg.norm(cam_to_ball)
            ball_angle_in_img = 2 * math.atan(ball_radius / dist_to_ball)
            fov_h = fov
            fov_v = self.h_fov_to_v_fov(fov_h, height, width)
            added_yaw = random.random() * (fov_h + ball_angle_in_img) - (fov_h + ball_angle_in_img) / 2
            added_pitch = random.random() * (fov_v + ball_angle_in_img) - (fov_v + ball_angle_in_img) / 2

            new_mat = np.matmul(transforms3d.euler.euler2mat(0, pitch, yaw),
                                transforms3d.euler.euler2mat(0, added_pitch, added_yaw), )
            new_new_rpy = transforms3d.euler.mat2euler(new_mat)

            camera_pose = Pose()
            camera_pose.position.x = cam_position[0]
            camera_pose.position.y = cam_position[1]
            camera_pose.position.z = cam_position[2]
            quat = transforms3d.euler.euler2quat(*new_new_rpy)
            camera_pose.orientation.w = quat[0]
            camera_pose.orientation.x = quat[1]
            camera_pose.orientation.y = quat[2]
            camera_pose.orientation.z = quat[3]
            self.camera_poses.append(camera_pose)
            self.reset_robot_pose_rpy(cam_position, new_new_rpy, "FakeCamera")
        else:
            print("not implemented")

    def generate_n_images(self, n, folder="img", ball_in_image=True, uniform_camera=True):
        self.ball_in_image = ball_in_image
        self.uniform_camera_position = uniform_camera
        self.step_sim()
        self.fov = []
        self.camera_poses = []
        self.robot_poses = []
        self.ball_positions = []
        self.annotations = []
        self.filenames = []
        self.intersections = []
        self.annotations = []
        self.img_save_dir = os.path.join(folder, "images")
        self.seg_save_dir = os.path.join(folder, "segmentations")
        self.depth_save_dir = os.path.join(folder, "depth")

        if not os.path.exists(self.img_save_dir):
            os.makedirs(self.img_save_dir)
        if not os.path.exists(self.seg_save_dir):
            os.makedirs(self.seg_save_dir)
        if not os.path.exists(self.depth_save_dir):
            os.makedirs(self.depth_save_dir)

        for i in range(n):
            print(f"stepping... {i + 1}/{n}")
            self.step()

        images = self.generate_annotations()
        f = open(self.img_save_dir + "/annotations.yaml", "w")
        yaml.dump({"images": images}, f)
        f.close()

    def generate_annotations(self):
        robot_equivalent = ["RED1", "RED2", "RED3", "BLUE1", "BLUE2", "BLUE3", "BLUE4"]
        gp_equivalent = ["left_goalpost_home", "right_goalpost_home",
                         "left_goalpost_enemy", "right_goalpost_enemy", ]
        hb_equivalent = ["top_bar_enemy", "top_bar_home"]
        gp_poses = [[-4.475, -1.3, 0], [-4.475, 1.3, 0], [4.475, -1.3, 0], [4.475, 1.3, 0]]
        hb_poses = [[0, -4.5, 1.25], [0, -4.5, 1.25]]
        images = {}
        for i in range(len(self.filenames)):
            current_annotation = self.annotations[i]
            new_annotations = []
            found_robot = False
            found_goalpost = False
            found_ball = False
            found_horizontal_goalpost = False
            for a in current_annotation:
                if a["type"] in robot_equivalent and a["in_image"]:
                    current_pose = self.robot_poses[i][a["type"]]
                    quat = transforms3d.euler.euler2quat(*current_pose[1])
                    pose_dict = {"position": {"x": float(current_pose[0][0]), "y": float(current_pose[0][1]), "z": 0},
                                 "orientation": {"x": float(quat[1]), "y": float(quat[2]), "z": float(quat[3]),
                                                 "w": float(quat[0])}}
                    [xmin, ymin] = np.min(a["vector"], axis=0)
                    [xmax, ymax] = np.max(a["vector"], axis=0)
                    vector = [[int(xmin), int(ymin)], [int(xmax), int(ymax)]]
                    new_annotations.append({"type": "robot", "in_image": True, "pose": pose_dict, "vector": vector,
                                            "robot_number" : int(a["type"][-1])})
                    found_robot = True

                elif a["type"] == "ball" and a["in_image"]:
                    pose_dict = {
                        "position": {"x": float(self.ball_positions[i].x), "y": float(self.ball_positions[i].y),
                                     "z": float(self.ball_positions[i].z)}}
                    found_ball = True
                    [xmin, ymin] = np.min(a["vector"], axis=0)
                    [xmax, ymax] = np.max(a["vector"], axis=0)
                    vector = [[int(xmin), int(ymin)], [int(xmax), int(ymax)]]
                    new_annotations.append({"type": "ball", "in_image": True, "pose": pose_dict, "vector": vector})

                elif a["type"] in gp_equivalent and a["in_image"]:
                    current_pose = gp_poses[gp_equivalent.index(a["type"])]
                    pose_dict = {"position": {"x": current_pose[0], "y": current_pose[1], "z": current_pose[2]},
                                 "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}
                    vector = a["vector"]
                    new_annotations.append({"type": "goalpost", "in_image": True, "pose": pose_dict, "vector": vector})
                    found_goalpost = True
                elif a["type"] in hb_equivalent and a["in_image"]:
                    current_pose = hb_poses[hb_equivalent.index(a["type"])]
                    pose_dict = {"position": {"x": current_pose[0], "y": current_pose[1], "z": current_pose[2]},
                                 "orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}
                    vector = a["vector"]
                    new_annotations.append({"type": "top_bar", "in_image": True, "pose": pose_dict, "vector": vector})
                    found_horizontal_goalpost = True
            if not found_robot:
                new_annotations.append({"type": "robot", "in_image": False})
            if not found_goalpost:
                new_annotations.append({"type": "goalpost", "in_image": False})
            if not found_ball:
                new_annotations.append({"type": "ball", "in_image": False})
            if not found_horizontal_goalpost:
                new_annotations.append({"type": "horizontal_post", "in_image": False})
            t_in_image = False
            x_in_image = False
            l_in_image = False
            current_intersections = self.intersections[i]
            for single_crossing in current_intersections["T"]:
                new_annotations.append({"type":"T-Intersection", "in_image": bool(single_crossing[2]),
                                        "vector": [[int(single_crossing[0][0]), int(single_crossing[0][1])]],
                                        "pose": {"position": {"x": float(single_crossing[1][0]),
                                             "y": float(single_crossing[1][1]),
                                             "z": float(0)}}})
                t_in_image = True
            for single_crossing in current_intersections["X"]:
                new_annotations.append({"type":"X-Intersection", "in_image": bool(single_crossing[2]),
                                        "vector": [[int(single_crossing[0][0]), int(single_crossing[0][1])]],
                                        "pose": {"position": {"x": float(single_crossing[1][0]),
                                             "y": float(single_crossing[1][1]),
                                             "z": float(0)}}})
                x_in_image = True
            for single_crossing in current_intersections["L"]:
                new_annotations.append({"type":"L-Intersection", "in_image": bool(single_crossing[2]),
                                        "vector": [[int(single_crossing[0][0]), int(single_crossing[0][1])]],
                                        "pose": {"position": {"x": float(single_crossing[1][0]),
                                             "y": float(single_crossing[1][1]),
                                             "z": float(0)}}})
                l_in_image = True

            if not t_in_image:
                new_annotations.append({"type": "T-Intersection", "in_image": False})
            if not x_in_image:
                new_annotations.append({"type": "X-Intersection", "in_image": False})
            if not l_in_image:
                new_annotations.append({"type": "L-Intersection", "in_image": False})

            camera_pose_dict = {"position": {"x": float(self.camera_poses[i].position.x),
                                             "y": float(self.camera_poses[i].position.y),
                                             "z": float(self.camera_poses[i].position.z)},
                                "orientation": {"x": float(self.camera_poses[i].orientation.x),
                                                "y": float(self.camera_poses[i].orientation.y),
                                                "z": float(self.camera_poses[i].orientation.z),
                                                "w": float(self.camera_poses[i].orientation.w)}}
            metadata_dict = {"fov": float(self.fov[i]), "camera_pose": camera_pose_dict, "tags": ["simulation"],
                             "location": "Webots",}

            images[self.filenames[i]] = {"annotations": new_annotations, "metadata": metadata_dict,
                                         "width": 1920, "height": 1080}
        return images

    def mat_from_fov_and_resolution(self, fov, res):
        return 0.5 * res * (math.cos((fov / 2)) / math.sin((fov / 2)))

    def h_fov_to_v_fov(self, h_fov, height, width):
        return 2 * math.atan(math.tan(h_fov * 0.5) * (height / width))

    def step_sim(self):
        self.time += self.timestep / 1000
        self.supervisor.step(self.timestep)

    def step(self):
        self.random_placement()
        self.step_sim()
        recog, seg_img = self.save_recognition()
        self.annotations.append(recog)
        self.intersections.append(self.check_intersections_in_image(seg_img))

    def reset_robot_pose(self, pos, quat, name="amy"):
        self.set_robot_pose_quat(pos, quat, name)
        if name in self.robot_nodes:
            self.robot_nodes[name].resetPhysics()

    def reset_robot_pose_rpy(self, pos, rpy, name="amy"):
        self.set_robot_pose_rpy(pos, rpy, name)
        if name in self.robot_nodes:
            self.robot_nodes[name].resetPhysics()

    def robot_pose_callback(self, req=None):
        if req.orientation.x == 0 and req.orientation.y == 0 and req.orientation.z == 0 and req.orientation.w == 0:
            self.reset_robot_pose_rpy([req.position.x, req.position.y, req.position.z], [0, 0, 0], req.robot_name)
        else:
            self.set_robot_pose_quat([req.position.x, req.position.y, req.position.z],
                                     [req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w],
                                     req.robot_name)

    def reset_ball(self, req=None):
        self.ball.getField("translation").setSFVec3f([0, 0, 0.0772])
        self.ball.getField("rotation").setSFRotation([0, 0, 1, 0])
        self.ball.resetPhysics()

    def set_ball(self, req=None, orientation="rand", position=None):
        if req is None:
            self.ball.getField("translation").setSFVec3f([position.x, position.y, position.z])
        else:
            self.ball.getField("translation").setSFVec3f([req.p.x, req.p.y, 0.08])
        if orientation == "rand":
            u = random.random()
            v = random.random()
            w = random.random()
            quat = (math.sqrt(1 - u) * math.sin(2 * math.pi * v), math.sqrt(1 - u) * math.sin(2 * math.pi * v),
                    math.sqrt(1 - u) * math.sin(2 * math.pi * w), math.sqrt(1 - u) * math.sin(2 * math.pi * w))
            axis, angle = transforms3d.quaternions.quat2axangle(quat)
            self.ball.getField("rotation").setSFRotation(list(np.append(axis, angle)))
        self.ball.resetPhysics()

    def set_robot_axis_angle(self, axis, angle, name="amy"):
        if name in self.rotation_fields:
            self.rotation_fields[name].setSFRotation(list(np.append(axis, angle)))

    def set_robot_rpy(self, rpy, name="amy"):
        axis, angle = transforms3d.euler.euler2axangle(rpy[0], rpy[1], rpy[2], axes='sxyz')
        self.set_robot_axis_angle(axis, angle, name)

    def set_robot_quat(self, quat, name="amy"):
        axis, angle = transforms3d.quaternions.quat2axangle([quat[3], quat[0], quat[1], quat[2]])
        self.set_robot_axis_angle(axis, angle, name)

    def set_robot_position(self, pos, name="amy"):
        if name in self.translation_fields:
            self.translation_fields[name].setSFVec3f(list(pos))

    def set_robot_pose_rpy(self, pos, rpy, name="amy"):
        self.set_robot_position(pos, name)
        self.set_robot_rpy(rpy, name)

    def set_robot_pose_quat(self, pos, quat, name="amy"):
        self.set_robot_position(pos, name)
        self.set_robot_quat(quat, name)

    def save_recognition(self):
        img_stamp = f"{int(self.time * 1000):08d}"
        img_name = f"img_fake_cam_{self.i:06d}"
        self.i += 1

        self.filenames.append((img_name + ".PNG"))
        seg_img_raw = self.camera.getRecognitionSegmentationImageArray()
        seg_img = np.array(seg_img_raw, dtype=np.uint8)
        # We need to swap axes so it's 1920x1080 instead of 1080x1920
        seg_img = np.swapaxes(seg_img, 0, 1)
        annotations = self.generatePolygonsFromSegmentation(seg_img)


        self.camera.saveImage(filename=os.path.join(self.img_save_dir, img_name + ".PNG"), quality=100)
        self.camera.saveRecognitionSegmentationImage(filename=os.path.join(self.seg_save_dir, img_name + "_seg.PNG"),
                                                     quality=100)
        self.depth.saveImage(filename=os.path.join(self.depth_save_dir, img_name + "_depth.PNG"), quality=100)
        depth_array = self.depth.getRangeImageArray()
        np.save(os.path.join(self.depth_save_dir, img_name + "_depth_raw"), depth_array, allow_pickle=False)
        return annotations, seg_img

    def generatePolygonsFromSegmentation(self, img):
        # this method returns a list with the following items:
        # if not in image: (name, False)
        # otherwise: (name, ((1,2), (2,3), (3,4), (4,5))
        # if multiple objects of the same type are present, then there will be multiple tuples starting with the same name
        debug = False

        # find the colors by saving segmentation image and look at the values in gimp
        # we don't automatically generate a line for the field, we only offer that in the segmentation image
        colors = {"left_goalpost_home": (128, 25, 51), "top_bar_home": (128, 51, 51),
                  "right_goalpost_home": (128, 51, 25),
                  "left_goalpost_enemy": (0, 25, 51), "top_bar_enemy": (0, 51, 51), "right_goalpost_enemy": (0, 51, 25),
                  "BLUE1": (25, 25, 255), "BLUE2": (51, 25, 255), "BLUE3": (25, 51, 255), "BLUE4": (51, 51, 255),
                  "RED1": (255, 25, 25), "RED2": (255, 51, 25), "RED3": (255, 25, 51), "RED4": (255, 51, 51),
                  "ball": (128, 128, 128)}


        # cv2.imwrite("/tmp/foo.png", img)
        output = []
        """cv2.imshow("lol", cv2.resize(img, (1920//2, 1080//2)))
        key = 0
        while key not in [83, 100]:
            key = cv2.waitKey(0)"""

        for key, value in colors.items():
            # we make a mask of every place where in the image the value is exactly as defined
            # thus we basically have a greyscale image with only our object in white
            # calculate *255 to make visible for debug images
            mask = ((img == value).all(axis=2)).astype(np.uint8)
            chull = np.array(convex_hull_image(mask)).astype(np.uint8) * 255

            # Retr External seems to solve the problem of way too many Wolfgangs being a contour we had with RETR_TREE
            # contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours, hierarchy = cv2.findContours(chull, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            """print(f"found {len(contours)} contours with mask name {key}")
            cv2.imshow("lol", cv2.resize(chull, (1920//2, 1080//2)))
            key = 0
            while key not in [83, 100]:
                key = cv2.waitKey(0)"""

            if len(contours) == 0:
                output.append({"type": key, "in_image": False})

            for cnt in contours:
                if key in ['left_goalpost_home', 'top_bar_home', 'right_goalpost_home',
                           'left_goalpost_enemy', 'top_bar_enemy', 'right_goalpost_enemy']:
                    rect = cv2.minAreaRect(cnt)
                else:
                    bound = cv2.boundingRect(cnt)
                    # we format it so it is in the same format as cv2.minAreaRect
                    rect = ((bound[0] + bound[2]/2, bound[1] + bound[3]/2), (bound[2], -bound[3]), 0.0)

                box = cv2.boxPoints(rect)

                # catch coordinates below zero
                for i, boxa in enumerate(box):
                    for j, boxb in enumerate(boxa):
                        box[i][j] = max(0, boxb)

                vector = {"vector": [[int(box[0][0]), int(box[0][1])], [int(box[1][0]), int(box[1][1])],
                                     [int(box[2][0]), int(box[2][1])], [int(box[3][0]), int(box[3][1])]],
                          "type": key,
                          "in_image": True}
                output.append(vector)

                if debug:
                    print(vector)
                    print(box)
                    print([box.astype(int)])
                    print(type([box.astype(int)][0]))

                    debug = cv2.drawContours(cv2.UMat(img), [box.astype(int)], -1, (255, 255, 255), 10)
                    cv2.imshow(key, debug)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
        return output

    def check_intersections_in_image(self, seg_img):
        width = 1920
        height = 1080
        f_y = self.mat_from_fov_and_resolution(
            self.h_fov_to_v_fov(self.camera.getFov(), height, width),
            height)
        f_x = self.mat_from_fov_and_resolution(self.fov[-1], width)
        K = [[f_x, 0, width / 2],
             [0, f_y, height / 2],
             [0, 0, 1]]
        origin_to_camera_pos = [self.camera_poses[-1].position.x,
                                self.camera_poses[-1].position.y,
                                self.camera_poses[-1].position.z]
        origin_to_camera_quat = [self.camera_poses[-1].orientation.w, self.camera_poses[-1].orientation.x,
                                 self.camera_poses[-1].orientation.y, self.camera_poses[-1].orientation.z]
        origin_to_camera_mat = transforms3d.quaternions.quat2mat(origin_to_camera_quat)
        origin_to_camera_transform = transforms3d.affines.compose(origin_to_camera_pos,
                                                                  origin_to_camera_mat,
                                                                  [1, 1, 1])
        camera_to_origin_transform = np.linalg.inv(origin_to_camera_transform)


        def expand_point(p):
            p_new = np.ndarray((4,1))
            p_new[0][0] = p[0]
            p_new[1][0] = p[1]
            p_new[2][0] = 0
            p_new[3][0] = 1
            return p_new

        def in_image(p):
            if p[2] < 0:  # behind camera
                return None
            p_new = p[:3]
            p_pixel = np.matmul(K, p_new)
            p_pixel = p_pixel * (1 / p_pixel[2])
            if 0 < p_pixel[0] < 1920 and 0 < p_pixel[1] < 1080:
                return p_pixel[:2]
            else:
                return None

        def get_pixel(p):
            point = expand_point(p)
            p_transformed = np.matmul(camera_to_origin_transform, point)
            dist = np.linalg.norm(p_transformed[:3])
            p_axis_corrected = np.ndarray((3,1)) # to camera coordinate system
            p_axis_corrected[0][0] = -p_transformed[1][0]
            p_axis_corrected[1][0] = -p_transformed[2][0]
            p_axis_corrected[2][0] = p_transformed[0][0]
            pixel = in_image(p_axis_corrected)
            return pixel, dist

        t_intersecs = []
        l_intersecs = []
        x_intersecs = []
        for t in T_INTERSECTIONS:
            pixel, dist = get_pixel(t)
            if pixel is not None:
                t_intersecs.append((pixel, t, self.intersection_visible(pixel, seg_img, dist)))
        for t in L_INTERSECTIONS:
            pixel, dist = get_pixel(t)
            if pixel is not None:
                l_intersecs.append((pixel, t, self.intersection_visible(pixel, seg_img, dist)))
        for t in X_INTERSECTIONS:
            pixel, dist = get_pixel(t)
            if pixel is not None:
                x_intersecs.append((pixel, t,  self.intersection_visible(pixel, seg_img, dist)))

        return {"T": t_intersecs, "L": l_intersecs, "X": x_intersecs}

    def intersection_visible(self, pixel, image, dist):
        line_color = (204, 204, 204)
        range = 10
        y_min = max(int(pixel[1])-range, 0)
        y_max = min(int(pixel[1])+range, 1080-1)
        x_min = max(int(pixel[0])-range, 0)
        x_max = min(int(pixel[0])+range, 1920-1)

        area = image[y_min:y_max, x_min:x_max]
        true_values = ((area == line_color).all(axis=2))

        max_dist = math.sqrt(10**2 + 8**2)
        min_dist = 0.45
        at_max = 0.02 * (range *2) ** 2
        at_min = 0.3 * (range *2) ** 2
        thresh = min(math.atan(1/dist) ** 2 / math.pi/2 * (range *2) ** 2 * 5, 150)
        res = np.sum(true_values)
        return res > thresh
