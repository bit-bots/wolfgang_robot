import os
import math
import time

from controller import Robot, Node, Field

import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState, Imu, Image, CameraInfo

from bitbots_msgs.msg import JointCommand, FootPressure

CAMERA_DIVIDER = 8  # every nth timestep an image is published, this is n


class RobotController:
    def __init__(self, ros_active=False, robot='wolfgang', do_ros_init=True, robot_node=None, base_ns='',
                 recognize=False, camera_active=True, foot_sensors_active=True):
        """
        The RobotController, a Webots controller that controls a single robot.
        The environment variable WEBOTS_ROBOT_NAME should be set to "amy", "rory", "jack" or "donna" if used with
        4_bots.wbt or to "amy" if used with 1_bot.wbt.

        :param ros_active: Whether ROS messages should be published
        :param robot: The name of the robot to use, currently one of wolfgang, darwin, nao, op3
        :param do_ros_init: Whether to call rospy.init_node (only used when ros_active is True)
        :param external_controller: Whether an external controller is used, necessary for RobotSupervisorController
        :param base_ns: The namespace of this node, can normally be left empty
        """
        self.ros_active = ros_active
        self.recognize = recognize
        self.camera_active = camera_active
        self.foot_sensors_active = foot_sensors_active
        if robot_node is None:
            self.robot_node = Robot()
        else:
            self.robot_node = robot_node
        self.walkready = [0] * 20
        self.time = 0

        self.motors = []
        self.sensors = []
        # for direct access
        self.motors_dict = {}
        self.sensors_dict = {}
        self.timestep = int(self.robot_node.getBasicTimeStep())

        self.switch_coordinate_system = True
        self.is_wolfgang = False
        self.pressure_sensors = None
        if robot == 'wolfgang':
            self.is_wolfgang = True
            self.proto_motor_names = ["RShoulderPitch [shoulder]", "LShoulderPitch [shoulder]", "RShoulderRoll",
                                      "LShoulderRoll", "RElbow", "LElbow", "RHipYaw", "LHipYaw", "RHipRoll [hip]",
                                      "LHipRoll [hip]", "RHipPitch", "LHipPitch", "RKnee", "LKnee", "RAnklePitch",
                                      "LAnklePitch", "RAnkleRoll", "LAnkleRoll", "HeadPan", "HeadTilt"]
            self.external_motor_names = ["RShoulderPitch", "LShoulderPitch", "RShoulderRoll", "LShoulderRoll", "RElbow",
                                         "LElbow", "RHipYaw", "LHipYaw", "RHipRoll", "LHipRoll", "RHipPitch",
                                         "LHipPitch", "RKnee", "LKnee", "RAnklePitch", "LAnklePitch", "RAnkleRoll",
                                         "LAnkleRoll", "HeadPan", "HeadTilt"]
            self.initial_joint_positions = {"LAnklePitch": -30, "LAnkleRoll": 0, "LHipPitch": 30, "LHipRoll": 0,
                                            "LHipYaw": 0, "LKnee": 60, "RAnklePitch": 30, "RAnkleRoll": 0,
                                            "RHipPitch": -30, "RHipRoll": 0, "RHipYaw": 0, "RKnee": -60,
                                            "LShoulderPitch": 75, "LShoulderRoll": 0, "LElbow": 36,
                                            "RShoulderPitch": -75, "RShoulderRoll": 0, "RElbow": -36, "HeadPan": 0,
                                            "HeadTilt": 0}
            self.sensor_suffix = "_sensor"
            accel_name = "imu accelerometer"
            gyro_name = "imu gyro"
            camera_name = "camera"
            self.pressure_sensor_names = []
            if self.foot_sensors_active:
                self.pressure_sensor_names = ["llb", "llf", "lrf", "lrb", "rlb", "rlf", "rrf", "rrb"]
            self.pressure_sensors = []
            for name in self.pressure_sensor_names:
                sensor = self.robot_node.getDevice(name)
                sensor.enable(self.timestep)
                self.pressure_sensors.append(sensor)

        elif robot == 'darwin':
            self.proto_motor_names = ["ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR", "ArmLowerL",
                                      "PelvYR", "PelvYL", "PelvR", "PelvL", "LegUpperR", "LegUpperL", "LegLowerR",
                                      "LegLowerL", "AnkleR", "AnkleL", "FootR", "FootL", "Neck", "Head"]
            self.external_motor_names = ["RShoulderPitch", "LShoulderPitch", "RShoulderRoll", "LShoulderRoll", "RElbow",
                                         "LElbow", "RHipYaw", "LHipYaw", "RHipRoll", "LHipRoll", "RHipPitch",
                                         "LHipPitch", "RKnee", "LKnee", "RAnklePitch", "LAnklePitch", "RAnkleRoll",
                                         "LAnkleRoll", "HeadPan", "HeadTilt"]
            self.sensor_suffix = "S"
            accel_name = "Accelerometer"
            gyro_name = "Gyro"
            camera_name = "Camera"
        elif robot == 'nao':
            self.proto_motor_names = ["RShoulderPitch", "LShoulderPitch", "RShoulderRoll", "LShoulderRoll", "RElbowYaw",
                                      "LElbowYaw", "RHipYawPitch", "LHipYawPitch", "RHipRoll", "LHipRoll", "RHipPitch",
                                      "LHipPitch",
                                      "RKneePitch", "LKneePitch", "RAnklePitch", "LAnklePitch", "RAnkleRoll",
                                      "LAnkleRoll",
                                      "HeadYaw",
                                      "HeadPitch"]
            self.external_motor_names = self.proto_motor_names
            self.sensor_suffix = "S"
            accel_name = "accelerometer"
            gyro_name = "gyro"
            camera_name = "CameraTop"
            self.switch_coordinate_system = False
        elif robot == 'op3':
            self.proto_motor_names = ["ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR", "ArmLowerL",
                                      "PelvYR", "PelvYL", "PelvR", "PelvL", "LegUpperR", "LegUpperL", "LegLowerR",
                                      "LegLowerL", "AnkleR", "AnkleL", "FootR", "FootL", "Neck", "Head"]
            self.external_motor_names = ["r_sho_pitch", "l_sho_pitch", "r_sho_roll", "l_sho_roll",
                                         "r_el", "l_el", "r_hip_yaw", "l_hip_yaw", "r_hip_roll", "l_hip_roll",
                                         "r_hip_pitch", "l_hip_pitch", "r_knee", "l_knee", "r_ank_pitch",
                                         "l_ank_pitch", "r_ank_roll", "l_ank_roll", "head_pan", "head_tilt"]
            self.sensor_suffix = "S"
            accel_name = "Accelerometer"
            gyro_name = "Gyro"
            camera_name = "Camera"
            self.switch_coordinate_system = False

        self.motor_names_to_external_names = {}
        self.external_motor_names_to_motor_names = {}
        for i in range(len(self.proto_motor_names)):
            self.motor_names_to_external_names[self.proto_motor_names[i]] = self.external_motor_names[i]
            self.external_motor_names_to_motor_names[self.external_motor_names[i]] = self.proto_motor_names[i]

        self.current_positions = {}
        self.joint_limits = {}
        for motor_name in self.proto_motor_names:
            motor = self.robot_node.getDevice(motor_name)
            motor.enableTorqueFeedback(self.timestep)
            self.motors.append(motor)
            self.motors_dict[self.motor_names_to_external_names[motor_name]] = motor
            sensor = self.robot_node.getDevice(motor_name + self.sensor_suffix)
            sensor.enable(self.timestep)
            self.sensors.append(sensor)
            self.sensors_dict[self.motor_names_to_external_names[motor_name]] = sensor
            self.current_positions[self.motor_names_to_external_names[motor_name]] = sensor.getValue()
            # min, max and middle position (precomputed since it will be used at each step)
            self.joint_limits[self.motor_names_to_external_names[motor_name]] = (
                motor.getMinPosition(), motor.getMaxPosition(),
                0.5 * (motor.getMinPosition() + motor.getMaxPosition()))

        self.accel = self.robot_node.getDevice(accel_name)
        self.accel.enable(self.timestep)
        self.gyro = self.robot_node.getDevice(gyro_name)
        self.gyro.enable(self.timestep)
        if self.is_wolfgang:
            self.accel_head = self.robot_node.getDevice("imu_head accelerometer")
            self.accel_head.enable(self.timestep)
            self.gyro_head = self.robot_node.getDevice("imu_head gyro")
            self.gyro_head.enable(self.timestep)
        self.camera = self.robot_node.getDevice(camera_name)
        self.camera_counter = 0
        if self.camera_active:
            self.camera.enable(self.timestep * CAMERA_DIVIDER)
        if self.recognize:
            self.camera.recognitionEnable(self.timestep)
            self.last_img_saved = 0.0
            self.img_save_dir = "/tmp/webots/images" + \
                                time.strftime("%Y-%m-%d-%H-%M-%S") + \
                                os.getenv('WEBOTS_ROBOT_NAME')
            if not os.path.exists(self.img_save_dir):
                os.makedirs(self.img_save_dir)

        self.imu_frame = rospy.get_param("~imu_frame", "imu_frame")
        if self.ros_active:
            if base_ns == "":
                clock_topic = "/clock"
            else:
                clock_topic = base_ns + "clock"
            if do_ros_init:
                rospy.init_node("webots_ros_interface", argv=['clock:=' + clock_topic])
            self.l_sole_frame = rospy.get_param("~l_sole_frame", "l_sole")
            self.r_sole_frame = rospy.get_param("~r_sole_frame", "r_sole")
            self.camera_optical_frame = rospy.get_param("~camera_optical_frame", "camera_optical_frame")
            self.head_imu_frame = rospy.get_param("~head_imu_frame", "imu_frame_2")
            self.pub_js = rospy.Publisher(base_ns + "joint_states", JointState, queue_size=1)
            self.pub_imu = rospy.Publisher(base_ns + "imu/data_raw", Imu, queue_size=1)

            self.pub_imu_head = rospy.Publisher(base_ns + "imu_head/data", Imu, queue_size=1)
            self.pub_cam = rospy.Publisher(base_ns + "camera/image_proc", Image, queue_size=1)
            self.pub_cam_info = rospy.Publisher(base_ns + "camera/camera_info", CameraInfo, queue_size=1)

            self.pub_pres_left = rospy.Publisher(base_ns + "foot_pressure_left/raw", FootPressure, queue_size=1)
            self.pub_pres_right = rospy.Publisher(base_ns + "foot_pressure_right/raw", FootPressure, queue_size=1)
            self.cop_l_pub_ = rospy.Publisher(base_ns + "cop_l", PointStamped, queue_size=1)
            self.cop_r_pub_ = rospy.Publisher(base_ns + "cop_r", PointStamped, queue_size=1)
            rospy.Subscriber(base_ns + "DynamixelController/command", JointCommand, self.command_cb)

        if robot == "op3":
            # start pose
            command = JointCommand()
            command.joint_names = ["r_sho_roll", "l_sho_roll"]
            command.positions = [-math.tau / 8, math.tau / 8]
            self.command_cb(command)

        # needed to run this one time to initialize current position, otherwise velocity will be nan
        self.get_joint_values(self.external_motor_names)

    def mat_from_fov_and_resolution(self, fov, res):
        return 0.5 * res * (math.cos((fov / 2)) / math.sin((fov / 2)))

    def h_fov_to_v_fov(self, h_fov, height, width):
        return 2 * math.atan(math.tan(h_fov * 0.5) * (height / width))

    def step_sim(self):
        self.time += self.timestep / 1000
        self.robot_node.step(self.timestep)

    def step(self):
        self.step_sim()
        if self.ros_active:
            self.publish_ros()

    def publish_ros(self):
        self.publish_imu()
        self.publish_joint_states()
        if self.camera_active and self.camera_counter == 0:
            self.publish_camera()
            self.publish_camera_info()
        self.publish_pressure()
        if self.recognize:
            self.save_recognition()
        self.camera_counter = (self.camera_counter + 1) % CAMERA_DIVIDER

    def convert_joint_radiant_to_scaled(self, joint_name, pos):
        # helper method to convert to scaled position between [-1,1] for this joint using min max scaling
        lower_limit, upper_limit, mid_position = self.joint_limits[joint_name]
        return 2 * (pos - mid_position) / (upper_limit - lower_limit)

    def convert_joint_scaled_to_radiant(self, joint_name, position):
        # helper method to convert to scaled position for this joint using min max scaling
        lower_limit, upper_limit, mid_position = self.joint_limits[joint_name]
        return position * (upper_limit - lower_limit) / 2 + mid_position

    def set_joint_goal_position(self, joint_name, goal_position, goal_velocity=-1, scaled=False, relative=False):
        motor = self.motors_dict[joint_name]
        if scaled:
            goal_position = self.convert_joint_radiant_to_scaled(joint_name, goal_position)
        if relative:
            goal_position = goal_position + self.get_joint_values([joint_name])[0]
        motor.setPosition(goal_position)
        if goal_velocity == -1:
            motor.setVelocity(motor.getMaxVelocity())
        else:
            motor.setVelocity(goal_velocity)

    def set_joint_goals_position(self, joint_names, goal_positions, goal_velocities=[]):
        for i in range(len(joint_names)):
            try:
                if len(goal_velocities) != 0:
                    self.set_joint_goal_position(joint_names[i], goal_positions[i], goal_velocities[i])
                else:
                    self.set_joint_goal_position(joint_names[i], goal_positions[i])
            except ValueError:
                print(f"invalid motor specified ({joint_names[i]})")

    def command_cb(self, command: JointCommand):
        if len(command.positions) != 0:
            # position control
            # todo maybe needs to match external motor names to interal ones fist?
            self.set_joint_goals_position(command.joint_names, command.positions, command.velocities)
        else:
            # torque control
            for i, name in enumerate(command.joint_names):
                try:
                    self.motors_dict[name].setTorque(command.accelerations[i])
                except ValueError:
                    print(f"invalid motor specified ({name})")

    def set_head_tilt(self, pos):
        self.motors[-1].setPosition(pos)

    def set_arms_zero(self):
        positions = [-0.8399999308200574, 0.7200000596634105, -0.3299999109923385, 0.35999992683575216,
                     0.5099999812500172, -0.5199999789619728]
        for i in range(0, 6):
            self.motors[i].setPosition(positions[i])

    def get_joint_values(self, used_joint_names, scaled=False):
        joint_positions = []
        joint_velocities = []
        joint_torques = []
        for joint_name in used_joint_names:
            value = self.sensors_dict[joint_name].getValue()
            if scaled:
                value = self.convert_joint_radiant_to_scaled(joint_name, value)
            joint_positions.append(value)
            joint_velocities.append(self.current_positions[joint_name] - value)
            joint_torques.append(self.motors_dict[joint_name].getTorqueFeedback())
            self.current_positions[joint_name] = value
        return joint_positions, joint_velocities, joint_torques

    def get_joint_state_msg(self):
        js = JointState()
        js.name = []
        js.header.stamp = rospy.Time.from_seconds(self.time)
        js.position = []
        js.effort = []
        for joint_name in self.external_motor_names:
            js.name.append(joint_name)
            value = self.sensors_dict[joint_name].getValue()
            js.position.append(value)
            js.velocity.append(self.current_positions[joint_name] - value)
            js.effort.append(self.motors_dict[joint_name].getTorqueFeedback())
            self.current_positions[joint_name] = value
        return js

    def publish_joint_states(self):
        self.pub_js.publish(self.get_joint_state_msg())

    def get_imu_msg(self, head=False):
        msg = Imu()
        msg.header.stamp = rospy.Time.from_seconds(self.time)
        if head:
            msg.header.frame_id = self.head_imu_frame
        else:
            msg.header.frame_id = self.imu_frame

        # change order because webots has different axis
        if head:
            accel_vels = self.accel_head.getValues()
            msg.linear_acceleration.x = accel_vels[2]
            msg.linear_acceleration.y = -accel_vels[0]
            msg.linear_acceleration.z = -accel_vels[1]
        else:
            accel_vels = self.accel.getValues()
            msg.linear_acceleration.x = accel_vels[0]
            msg.linear_acceleration.y = accel_vels[1]
            msg.linear_acceleration.z = accel_vels[2]

        # make sure that acceleration is not completely zero or we will get error in filter.
        # Happens if robot is moved manually in the simulation
        if msg.linear_acceleration.x == 0 and msg.linear_acceleration.y == 0 and msg.linear_acceleration.z == 0:
            msg.linear_acceleration.z = 0.001

        if head:
            gyro_vels = self.gyro_head.getValues()
            msg.angular_velocity.x = gyro_vels[2]
            msg.angular_velocity.y = -gyro_vels[0]
            msg.angular_velocity.z = -gyro_vels[1]
        else:
            gyro_vels = self.gyro.getValues()
            msg.angular_velocity.x = gyro_vels[0]
            msg.angular_velocity.y = gyro_vels[1]
            msg.angular_velocity.z = gyro_vels[2]
        return msg

    def publish_imu(self):
        self.pub_imu.publish(self.get_imu_msg(head=False))
        if self.is_wolfgang:
            self.pub_imu_head.publish(self.get_imu_msg(head=True))

    def publish_camera(self):
        img_msg = Image()
        img_msg.header.stamp = rospy.Time.from_seconds(self.time)
        img_msg.header.frame_id = self.camera_optical_frame
        img_msg.height = self.camera.getHeight()
        img_msg.width = self.camera.getWidth()
        img_msg.encoding = "bgra8"
        img_msg.step = 4 * self.camera.getWidth()
        img = self.camera.getImage()
        img_msg.data = img
        self.pub_cam.publish(img_msg)

    def publish_camera_info(self):
        self.cam_info = CameraInfo()
        self.cam_info.header.stamp = rospy.Time.from_seconds(self.time)
        self.cam_info.header.frame_id = self.camera_optical_frame
        self.cam_info.height = self.camera.getHeight()
        self.cam_info.width = self.camera.getWidth()
        f_y = self.mat_from_fov_and_resolution(
            self.h_fov_to_v_fov(self.camera.getFov(), self.cam_info.height, self.cam_info.width),
            self.cam_info.height)
        f_x = self.mat_from_fov_and_resolution(self.camera.getFov(), self.cam_info.width)
        self.cam_info.K = [f_x, 0, self.cam_info.width / 2,
                            0, f_y, self.cam_info.height / 2,
                            0, 0, 1]
        self.cam_info.P = [f_x, 0, self.cam_info.width / 2, 0,
                            0, f_y, self.cam_info.height / 2, 0,
                            0, 0, 1, 0]
        self.pub_cam_info.publish(self.cam_info)

    def save_recognition(self):
        if self.time - self.last_img_saved < 1.0:
            return
        self.last_img_saved = self.time
        annotation = ""
        img_stamp = f"{self.time:.2f}".replace(".", "_")
        img_name = f"img_{os.getenv('WEBOTS_ROBOT_NAME')}_{img_stamp}.PNG"
        recognized_objects = self.camera.getRecognitionObjects()
        # variables for saving not in image later
        found_ball = False
        found_wolfgang = False
        for e in range(self.camera.getRecognitionNumberOfObjects()):
            model = recognized_objects[e].get_model()
            position = recognized_objects[e].get_position_on_image()
            size = recognized_objects[e].get_size_on_image()
            if model == b"soccer ball":
                found_ball = True
                vector = f"""{{"x1": {position[0] - 0.5 * size[0]}, "y1": {position[1] - 0.5 * size[1]}, "x2": {position[0] + 0.5 * size[0]}, "y2": {position[1] + 0.5 * size[1]}}}"""
                annotation += f"{img_name}|"
                annotation += "ball|"
                annotation += vector
                annotation += "\n"
            if model == b"wolfgang":
                found_wolfgang = True
                vector = f"""{{"x1": {position[0] - 0.5 * size[0]}, "y1": {position[1] - 0.5 * size[1]}, "x2": {position[0] + 0.5 * size[0]}, "y2": {position[1] + 0.5 * size[1]}}}"""
                annotation += f"{img_name}|"
                annotation += "robot|"
                annotation += vector
                annotation += "\n"
        if not found_ball:
            annotation += f"{img_name}|ball|not in image\n"
        if not found_wolfgang:
            annotation += f"{img_name}|robot|not in image\n"
        with open(os.path.join(self.img_save_dir, "annotations.txt"), "a") as f:
            f.write(annotation)
        self.camera.saveImage(filename=os.path.join(self.img_save_dir, img_name), quality=100)

    def get_image(self):
        return self.camera.getImage()

    def get_pressure_message(self):

        current_time = rospy.Time.from_sec(self.time)
        if not self.foot_sensors_active:
            cop_r = PointStamped()
            cop_r.header.frame_id = self.r_sole_frame
            cop_r.header.stamp = current_time
            cop_l = PointStamped()
            cop_l.header.frame_id = self.l_sole_frame
            cop_l.header.stamp = current_time
            return FootPressure(), FootPressure(), cop_l, cop_r

        left_pressure = FootPressure()
        left_pressure.header.stamp = current_time
        left_pressure.left_back = self.pressure_sensors[0].getValue()
        left_pressure.left_front = self.pressure_sensors[1].getValue()
        left_pressure.right_front = self.pressure_sensors[2].getValue()
        left_pressure.right_back = self.pressure_sensors[3].getValue()

        right_pressure = FootPressure()
        right_pressure.header.stamp = current_time
        right_pressure.left_back = self.pressure_sensors[4].getValue()
        right_pressure.left_front = self.pressure_sensors[5].getValue()
        right_pressure.right_front = self.pressure_sensors[6].getValue()
        right_pressure.right_back = self.pressure_sensors[7].getValue()

        # compute center of pressures of the feet
        pos_x = 0.085
        pos_y = 0.045
        # we can take a very small threshold, since simulation gives more accurate values than reality
        threshold = 1

        cop_l = PointStamped()
        cop_l.header.frame_id = self.l_sole_frame
        cop_l.header.stamp = current_time
        sum = left_pressure.left_back + left_pressure.left_front + left_pressure.right_front + left_pressure.right_back
        if sum > threshold:
            cop_l.point.x = (left_pressure.left_front + left_pressure.right_front -
                             left_pressure.left_back - left_pressure.right_back) * pos_x / sum
            cop_l.point.x = max(min(cop_l.point.x, pos_x), -pos_x)
            cop_l.point.y = (left_pressure.left_front + left_pressure.left_back -
                             left_pressure.right_front - left_pressure.right_back) * pos_y / sum
            cop_l.point.y = max(min(cop_l.point.x, pos_y), -pos_y)
        else:
            cop_l.point.x = 0
            cop_l.point.y = 0

        cop_r = PointStamped()
        cop_r.header.frame_id = self.r_sole_frame
        cop_r.header.stamp = current_time
        sum = right_pressure.right_back + right_pressure.right_front + right_pressure.right_front + right_pressure.right_back
        if sum > threshold:
            cop_r.point.x = (right_pressure.left_front + right_pressure.right_front -
                             right_pressure.left_back - right_pressure.right_back) * pos_x / sum
            cop_r.point.x = max(min(cop_r.point.x, pos_x), -pos_x)
            cop_r.point.y = (right_pressure.left_front + right_pressure.left_back -
                             right_pressure.right_front - right_pressure.right_back) * pos_y / sum
            cop_r.point.y = max(min(cop_r.point.x, pos_y), -pos_y)
        else:
            cop_r.point.x = 0
            cop_r.point.y = 0

        return left_pressure, right_pressure, cop_l, cop_r

    def publish_pressure(self):
        left, right, cop_l, cop_r = self.get_pressure_message()
        self.pub_pres_left.publish(left)
        self.pub_pres_right.publish(right)
        self.cop_l_pub_.publish(cop_l)
        self.cop_r_pub_.publish(cop_r)
