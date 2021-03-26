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

import transforms3d
import numpy as np
import cv2

G = 9.81


class CameraController:
    def __init__(self, ros_active=False, mode='normal', do_ros_init=True, base_ns=''):
        """
        The SupervisorController, a Webots controller that can control the world.
        Set the environment variable WEBOTS_ROBOT_NAME to "supervisor_robot" if used with 1_bot.wbt or 4_bots.wbt.

        :param ros_active: Whether to publish ROS messages
        :param mode: Webots mode, one of 'normal', 'paused', or 'fast'
        :param do_ros_init: Whether rospy.init_node should be called
        :param base_ns: The namespace of this node, can normally be left empty
        """
        # requires WEBOTS_ROBOT_NAME to be set to "supervisor_robot"
        self.ros_active = ros_active
        self.time = 0
        self.clock_msg = Clock()

        self.supervisor = Supervisor()

        if mode == 'normal':
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
        elif mode == 'paused':
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
        elif mode == 'fast':
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_FAST)
        else:
            self.supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)

        self.motors = []
        self.sensors = []
        self.timestep = int(self.supervisor.getBasicTimeStep())

        # resolve the node for corresponding name
        self.robot_names = ["amy", "rory", "jack", "donna", "FakeCamera"]
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


        # need to handle these topics differently or we will end up having a double //
        if base_ns == "":
            clock_topic = "/clock"
            model_topic = "/model_states"
        else:
            clock_topic = base_ns + "clock"
            model_topic = base_ns + "model_states"
        if do_ros_init:
            rospy.init_node("webots_ros_supervisor", argv=['clock:=' + clock_topic])
        self.clock_publisher = rospy.Publisher(clock_topic, Clock, queue_size=1)
        self.reset_service = rospy.Service(base_ns + "reset", Empty, self.reset)
        self.initial_poses_service = rospy.Service(base_ns + "initial_pose", Empty, self.set_initial_poses)
        self.set_robot_position_service = rospy.Service(base_ns + "set_robot_pose", SetRobotPose,
                                                        self.robot_pose_callback)
        self.pub_cam = rospy.Publisher(base_ns + "recog_img", Image, queue_size=1)
        self.reset_ball_service = rospy.Service(base_ns + "reset_ball", Empty, self.reset_ball)
        self.reset_ball_service = rospy.Service(base_ns + "set_ball", SetBall, self.set_ball)

        self.robot_node = self.supervisor.getSelf()
        self.camera = self.supervisor.getDevice("camera")
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
        self.camera.enableRecognitionSegmentation()
        self.last_img_saved = 0.0
        self.img_save_dir = "/tmp/webots/images" + \
                            time.strftime("%Y-%m-%d-%H-%M-%S") + \
                            "fake_cam"
        if not os.path.exists(self.img_save_dir):
            os.makedirs(self.img_save_dir)

        self.world_info = self.supervisor.getFromDef("world_info")
        self.ball = self.supervisor.getFromDef("ball")

    def random_capture(self, size="kid", n=1000):
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
        elif size=="adult":
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
        pan_range = (-30, 30)  # TODO To be evaluated
        roll_range = (-5, 5)  # TODO To be evaulated
        z_choice = None
        while z_choice is None:
            z_prelim = np.random.normal(loc=z_mean, scale=z_std_dev)
            if z_range[0] < z_prelim < z_range[1]:
                z_choice = z_prelim

        new_position = [random.random()*(x_range[1]-x_range[0])+x_range[0],
                        random.random()*(y_range[1]-y_range[0])+y_range[0],
                        z_choice]
        # todo this has to be changed so it is not changing the 
        new_rpy = [random.random()*(roll_range[1]-roll_range[0])+roll_range[0],
                   random.random()*(tilt_range[1]-tilt_range[0])+tilt_range[0],
                   random.random()*(pan_range[1]-pan_range[0])+pan_range[0]]

        new_rpy = [n * (math.pi/180.0) for n in new_rpy]
        print(new_rpy)
        new_fov = None
        while new_fov is None:
            fov_prelim = np.random.normal(loc=fov_mean, scale=fov_std_dev)
            if fov_range[0] < fov_prelim < fov_range[1]:
                new_fov = fov_prelim
        #self.set_robot_pose_rpy(new_position, new_rpy, name="FakeCamera")
        self.save_recognition()


    def step_sim(self):
        self.time += self.timestep / 1000
        self.supervisor.step(self.timestep)

    def step(self):
        self.step_sim()
        self.random_capture()
        if self.ros_active:
            self.publish_clock()


    def publish_clock(self):
        self.clock_msg.clock = rospy.Time.from_seconds(self.time)
        self.clock_publisher.publish(self.clock_msg)

    def set_gravity(self, active):
        if active:
            self.world_info.getField("gravity").setSFFloat(9.81)
        else:
            self.world_info.getField("gravity").setSFFloat(0)

    def reset_robot_pose(self, pos, quat, name="amy"):
        self.set_robot_pose_quat(pos, quat, name)
        if name in self.robot_nodes:
            self.robot_nodes[name].resetPhysics()

    def reset_robot_pose_rpy(self, pos, rpy, name="amy"):
        self.set_robot_pose_rpy(pos, rpy, name)
        if name in self.robot_nodes:
            self.robot_nodes[name].resetPhysics()

    def reset(self, req=None):
        self.supervisor.simulationReset()
        self.supervisor.simulationResetPhysics()

    def set_initial_poses(self, req=None):
        self.reset_robot_pose_rpy([-1, 3, 0.42], [0, 0.24, -1.57], name="amy")
        self.reset_robot_pose_rpy([-1, -3, 0.42], [0, 0.24, 1.57], name="rory")
        self.reset_robot_pose_rpy([-3, 3, 0.42], [0, 0.24, -1.57], name="jack")
        self.reset_robot_pose_rpy([-3, -3, 0.42], [0, 0.24, 1.57], name="donna")
        self.reset_robot_pose_rpy([0, 6, 0.42], [0, 0.24, -1.57], name="melody")

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
            self.ball.getField("translation").setSFVec3f([position.x, position.y, 0.0772])
        else:
            self.ball.getField("translation").setSFVec3f([req.p.x, req.p.y, 0.0772])
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
        annotation = ""
        img_stamp = f"{self.time:.2f}".replace(".", "_")
        img_name = f"img_fake_cam_{img_stamp}"
        recognized_objects = self.camera.getRecognitionObjects()
        # variables for saving not in image later
        found_ball = False
        found_wolfgang = False
        found_post = False
        for e in range(self.camera.getRecognitionNumberOfObjects()):
            model = recognized_objects[e].get_model()
            position = recognized_objects[e].get_position_on_image()
            size = recognized_objects[e].get_size_on_image()
            if model == b"soccer ball":
                found_ball = True
                vector = f"""{{"x1": {position[0] - 0.5*size[0]}, "y1": {position[1] - 0.5*size[1]}, "x2": {position[0] + 0.5*size[0]}, "y2": {position[1] + 0.5*size[1]}}}"""
                annotation += f"{img_name}|"
                annotation += "ball|"
                annotation += vector
                annotation += "\n"
            if model == b"wolfgang":
                found_wolfgang = True
                vector = f"""{{"x1": {position[0] - 0.5*size[0]}, "y1": {position[1] - 0.5*size[1]}, "x2": {position[0] + 0.5*size[0]}, "y2": {position[1] + 0.5*size[1]}}}"""
                annotation += f"{img_name}|"
                annotation += "robot|"
                annotation += vector
                annotation += "\n"
            print('model == b"left_post"')
            print(model == b"left_post")
            if model == b"left_post":
                print("found a post")
                found_post = True
                vector = f"""{{"x1": {position[0] - 0.5*size[0]}, "y1": {position[1] - 0.5*size[1]}, "x2": {position[0] + 0.5*size[0]}, "y2": {position[1] + 0.5*size[1]}}}"""
                annotation += f"{img_name}|"
                annotation += "goalpost|"
                annotation += vector
                annotation += "\n"
            print(model)
        if not found_ball:
            annotation +=  f"{img_name}|ball|not in image\n"
        if not found_wolfgang:
            annotation += f"{img_name}|robot|not in image\n"
        if not found_post:
            annotation += f"{img_name}|goalpost|not in image\n"
        with open(os.path.join(self.img_save_dir, "annotations.txt"), "a") as f:
            f.write(annotation)
        self.camera.saveImage(filename=os.path.join(self.img_save_dir, img_name + ".PNG"), quality=100)
        seg_img = self.camera.getRecognitionSegmentationImageArray()
        self.generatePolygonsFromSegmentation(seg_img)
        self.camera.saveRecognitionSegmentationImage(filename=os.path.join(self.img_save_dir, img_name + "_seg.PNG"), quality=100)

    def generatePolygonsFromSegmentation(self, img):
        # this method returns a list with the following items:
        # if not in image: (name, False)
        # otherwise: (name, ((1,2), (2,3), (3,4), (4,5))
        # if multiple objects of the same type are present, then there will be multiple tuples starting with the same name

        # find the colors by saving segmentation image and look at the values in gimp
        # TODO goalnets are also field colored
        # we don't automatically generate a line for the field, we only offer that in the segmentation image
        colors = {"left_goalpost" : (255, 0, 255), "top_bar": (255, 255, 0), "right_goalpost": (0, 255, 255),
                  "ball": (223, 193, 29), "wolfgang": (51, 51, 51)}
        img = np.array(img, dtype=np.uint8)
        # We need to swap axes so it's 1920x1080 instead of 1080x1920
        img = np.swapaxes(img, 0, 1)

        # cv2.imwrite("/tmp/foo.png", img)
        output = []
        # TODO there are way too many wolfgangs in the detection
        # TODO check if the found values make sense
        for key, value in colors.items():
            # we make a mask of every place where in the image the value is exactly as defined
            # thus we basically have a greyscale image with only our object in white
            # calculate *255 to make visible for debug images
            mask = ((img == value).all(axis=2)).astype(np.uint8)

            contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) == 0:
                output.append((key, False))
            for cnt in contours:
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                vector = (key, f"""(({box[0][0]},{box[0][1]}), ({box[1][0]}, {box[1][1]}),""" \
                         f"""({box[2][0]}, {box[2][1]}), ({box[3][0]}, {box[3][1]}))""")
                output.append(vector)
                print(vector)
        return output
