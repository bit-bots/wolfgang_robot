from controller import Robot, Node, Supervisor, Field

import os
import rospy

import random
import math
import time

from rosgraph_msgs.msg import Clock
from std_srvs.srv import Empty
from bitbots_msgs.srv import SetBall, SetRobotPose
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose

import transforms3d
import numpy as np
import cv2
import yaml

G = 9.81
DARWIN_PITCH = 0.225


class CameraController:
    def __init__(self, ros_active=False, mode='normal'):
        """
        The SupervisorController, a Webots controller that can control the world.
        Set the environment variable WEBOTS_ROBOT_NAME to "supervisor_robot" if used with 1_bot.wbt or 4_bots.wbt.

        :param ros_active: Whether to publish ROS messages
        :param mode: Webots mode, one of 'normal', 'paused', or 'fast'
        :param do_ros_init: Whether rospy.init_node should be called
        :param base_ns: The namespace of this node, can normally be left empty
        """
        # requires WEBOTS_ROBOT_NAME to be set to "supervisor_robot"
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
        self.img_save_dir = "img"
        self.filenames = []
        if not os.path.exists(self.img_save_dir):
            os.makedirs(self.img_save_dir)

    def random_placement(self, size="kid", n=1000):
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
        tilt_range = (0, 90)
        pan_range = (-180, 180)  # TODO To be evaluated
        roll_range = (-5, 5)  # TODO To be evaulated
        z_choice = None
        while z_choice is None:
            z_prelim = np.random.normal(loc=z_mean, scale=z_std_dev)
            if z_range[0] < z_prelim < z_range[1]:
                z_choice = z_prelim

        ball_pos = Point(random.random() * 9.5 - 4.75, random.random() * 6 - 3,
                         0.08)  # TODO field size by param ---furhtermore, ball height is ignored
        self.ball_positions.append(ball_pos)
        self.set_ball(orientation="rand", position=ball_pos)

        fov = None
        while fov is None:
            fov_prelim = np.random.normal(loc=fov_mean, scale=fov_std_dev)
            if fov_range[0] < fov_prelim < fov_range[1]:
                fov = fov_prelim
                fov = math.radians(fov)
        self.robot_node.getField("cameraFOV").setSFFloat(fov)
        self.fov.append(fov)

        robot_height_red = 0.41
        goalie_pos_red = [x_range[0] + 0.5, np.clip(np.random.normal(loc=0.0, scale=0.5), -2.25, 2.25),
                          robot_height_red]
        goalie_rpy_red = [0.0, 0, np.random.normal(loc=0.0, scale=0.3)]
        self.reset_robot_pose_rpy(goalie_pos_red, goalie_rpy_red, name="RED1")

        robot_height_blue = 0.327
        goalie_pos_blue = [x_range[1] - 0.5, np.clip(np.random.normal(loc=0.0, scale=0.5), -2.25, 2.25),
                           robot_height_blue]
        goalie_rpy_blue = [0.0, DARWIN_PITCH, math.pi + np.random.normal(loc=0.0, scale=0.3)]
        self.reset_robot_pose_rpy(goalie_pos_blue, goalie_rpy_blue, name="BLUE1")

        num_strikers_red = 2  # random.randint(0, 2)
        camera_is_striker = True  # bool(random.randint(0, 1))
        num_strikers_blue = 3  # random.randint(0, 3)
        num_defenders_red = 2 - num_strikers_red
        num_defenders_blue = 3 - num_strikers_blue
        positions = {"RED1":[goalie_pos_red, goalie_rpy_red], "BLUE1": [goalie_pos_blue, goalie_rpy_blue]}
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
        print(f"placing {name} as a striker")
        robot_pos = Point(ball_pos.x, ball_pos.y, ball_pos.z)
        # TODO collision check with other robots (lt < 0.2 m away or something like that)
        while True:
            preliminary_pos_x = robot_pos.x + np.random.normal(0, 2)
            preliminary_pos_y = robot_pos.x + np.random.normal(0, 2)
            pos_collides = False  # self.position_collides(preliminary_pos_x, preliminary_pos_y, other_positions)
            if -5 < preliminary_pos_x < 5 and -3.5 < preliminary_pos_y < 3.5 and not pos_collides:
                robot_pos.x = preliminary_pos_x
                robot_pos.y = preliminary_pos_y
                break
        robot_pos.z = height
        orientation[2] += np.random.normal(0, np.pi / 8)
        self.reset_robot_pose_rpy([robot_pos.x, robot_pos.y, robot_pos.z], orientation, name)
        return [[robot_pos.x, robot_pos.y, robot_pos.z], orientation]

    def place_defender(self, name, side, height, orientation, other_positions):
        print(f"placing {name} as a defender on side {side}")

    def place_camera(self, ball_pos, height, other_positions, x_range, y_range, fov):
        if self.cam_looks_at_ball:
            cam_position = [random.random() * (x_range[1] - x_range[0]) + x_range[0],
                            random.random() * (y_range[1] - y_range[0]) + y_range[0],
                            height]

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
            print(ball_angle_in_img)
            added_yaw = random.random() * (fov_h + ball_angle_in_img) - (fov_h + ball_angle_in_img) / 2
            added_pitch = random.random() * (fov_v + ball_angle_in_img) - (fov_v + ball_angle_in_img) / 2

            print(f" added_yaw = {added_yaw}")
            print(f" added_pitch = {added_pitch}")

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

    def generate_n_images(self, n, folder="img"):
        self.img_save_dir = folder
        if not os.path.exists(self.img_save_dir):
            os.makedirs(self.img_save_dir)
        self.annotations = []
        for i in range(n):
            print("stepping...")
            self.step()

        images = {}
        for i in range(n):
            current_annotation = self.annotations[i]
            for a in current_annotation:
                pose_dict = {}
                if a["type"] == "robot" and a["in_image"]:
                    current_pose = self.robot_poses[i][a["name"]]
                    print(current_pose)
                    quat = transforms3d.euler.euler2quat(*current_pose[1])
                    pose_dict = {"position": {"x": float(current_pose[0][0]), "y": float(current_pose[0][1]), "z": 0},
                                 "orientation": {"x": float(quat[1]), "y":  float(quat[2]), "z":  float(quat[3]), "w": float(quat[0])}}
                    del(a["name"])
                elif a["type"] == "ball" and a["in_image"]:
                    pose_dict = {"position": {"x": float(self.ball_positions[i].x), "y": float(self.ball_positions[i].y), "z": float(self.ball_positions[i].z)}}
                elif a["type"] == "right_goalpost" and a["in_image"]:
                    print("todo l_goalpost")
                elif a["type"] == "left_goalpost" and a["in_image"]:
                    print("todo r_goalpost")
                elif a["type"] == "horizontal_goalpost" and a["in_image"]:
                    print("todo horiz")
                a["pose"] = pose_dict
            camera_pose_dict = {"position": {"x": float(self.camera_poses[i].position.x),
                                             "y": float(self.camera_poses[i].position.y),
                                             "z": float(self.camera_poses[i].position.z)},
                                 "orientation": {"x": float(self.camera_poses[i].orientation.x),
                                                 "y": float(self.camera_poses[i].orientation.y),
                                                 "z": float(self.camera_poses[i].orientation.z),
                                                 "w": float(self.camera_poses[i].orientation.w)}}
            metadata_dict = {"fov": float(self.fov[i]), "camera_pose": camera_pose_dict, "tags": ["simulation"], "location": "Webots"}

            images[self.filenames[i]] = {"annotations": current_annotation, "metadata": metadata_dict }
        f = open(folder + "/annotations.yaml", "w")
        yaml.dump({images)


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
        self.annotations.append(self.save_recognition())

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
            self.ball.getField("translation").setSFVec3f([position.x, position.y, 0.08])
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
        annotations = []
        img_stamp = f"{self.time:.2f}".replace(".", "_")
        img_name = f"img_fake_cam_{img_stamp}"
        recognized_objects = self.camera.getRecognitionObjects()
        # variables for saving not in image later
        found_ball = False
        found_robot = False
        found_post = False
        for e in range(self.camera.getRecognitionNumberOfObjects()):
            model = recognized_objects[e].get_model()
            position = recognized_objects[e].get_position_on_image()
            size = recognized_objects[e].get_size_on_image()
            if model == b"soccer ball":
                found_ball = True
                anno = {"type": "ball", "in_image": True,
                        "vector": [[position[0] - 0.5 * size[0], position[1] - 0.5 * size[1]],
                                   [position[0] + 0.5 * size[0], position[1] + 0.5 * size[1]]]}
                annotations.append(anno)
            if model == b"RED1" or model == b"RED2" or model == b"RED3" or model == b"RED4" \
                    or model == b"BLUE1" or model == b"BLUE2" or model == b"BLUE3" or model == b"BLUE4":
                found_robot = True
                anno = {"type": "robot", "in_image": True,
                        "vector": [[position[0] - 0.5 * size[0], position[1] - 0.5 * size[1]],
                                   [position[0] + 0.5 * size[0], position[1] + 0.5 * size[1]]],
                        "name" : model.decode("utf-8")}
                annotations.append(anno)
            print(model)
        if not found_ball:
            annotations.append({"type": "ball", "in_image": False})
        if not found_robot:
            annotations.append({"type": "robot", "in_image": False})

        self.camera.saveImage(filename=os.path.join(self.img_save_dir, img_name + ".PNG"), quality=100)
        self.filenames.append(os.path.join(self.img_save_dir, img_name + ".PNG"))
        seg_img = self.camera.getRecognitionSegmentationImageArray()
        annos = self.generatePolygonsFromSegmentation(seg_img)
        for e in annos:
            annotations.append(e)
        # with open(os.path.join(self.img_save_dir, "annotations.txt"), "a") as f:
        #    f.write(yaml.dump(annotations))
        self.camera.saveRecognitionSegmentationImage(filename=os.path.join(self.img_save_dir, img_name + "_seg.PNG"),
                                                     quality=100)
        return annotations

    def generatePolygonsFromSegmentation(self, img):
        # this method returns a list with the following items:
        # if not in image: (name, False)
        # otherwise: (name, ((1,2), (2,3), (3,4), (4,5))
        # if multiple objects of the same type are present, then there will be multiple tuples starting with the same name
        # Things to know: If e.g. a robot occludes the top bar, then there will be an individual top bar detection
        #       on the right and left of the robot
        #   If two of the same object are too close too each other, this method will treat them as a single object
        debug = False

        # find the colors by saving segmentation image and look at the values in gimp
        # we don't automatically generate a line for the field, we only offer that in the segmentation image
        colors = {"left_goalpost": (255, 255, 0), "top_bar": (0, 255, 255), "right_goalpost": (255, 0, 255)}
        img = np.array(img, dtype=np.uint8)
        # We need to swap axes so it's 1920x1080 instead of 1080x1920
        img = np.swapaxes(img, 0, 1)

        # cv2.imwrite("/tmp/foo.png", img)
        output = []
        for key, value in colors.items():
            # we make a mask of every place where in the image the value is exactly as defined
            # thus we basically have a greyscale image with only our object in white
            # calculate *255 to make visible for debug images
            mask = ((img == value).all(axis=2)).astype(np.uint8)

            # Retr External seems to solve the problem of way too many Wolfgangs being a contour we had with RETR_TREE
            # contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) == 0:
                output.append({"type": key, "in_image": False})

            for cnt in contours:
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                # skip too small boxes. It is unlikely this is of a relevant size as an object
                if math.sqrt((box[0][0] - box[2][0]) ** 2 + (box[0][1] - box[2][1]) ** 2) < 50:
                    continue

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
                    debug = cv2.drawContours(cv2.UMat(img), [box.astype(int)], -1, (255, 255, 255), 10)
                    cv2.imshow(key, debug)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
        return output
