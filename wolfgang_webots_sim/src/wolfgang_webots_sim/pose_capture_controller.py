from controller import Robot
import rospy
import message_filters
from sensor_msgs.msg import JointState
from bitbots_msgs.msg import ModelStatesStamped
import yaml

class Capture:
    def __init__(self):
        rospy.init_node("pose_capture")
        self.current_js = None
        self.current_pose = None
        ms_sub = message_filters.Subscriber("model_states", ModelStatesStamped)
        js_sub = message_filters.Subscriber("joint_states", JointState)
        ts = message_filters.TimeSynchronizer([ms_sub, js_sub], 10)
        ts.registerCallback(self.cb)
        #s = Robot()
        #timestep = int(s.getBasicTimeStep())
        pose_dict = {}
        pose_dict["poses"] = []
        print("press enter to capture pose, press s to step once, d to step 10 times, f to step 100 q to quit: ")
        while (1):
            i = input("...: ")
            if i == "":
                single_pose = {}
                for i,name in enumerate(self.current_js.name):
                    single_pose[str(name)] = float(self.current_js.position[i])
                single_pose["pose"] = {"position": {"x": float(self.current_pose.position.x),
                                                    "y": float(self.current_pose.position.y),
                                                    "z": float(self.current_pose.position.z),},
                                       "orientation": {"w": float(self.current_pose.orientation.w),
                                                       "x": float(self.current_pose.orientation.x),
                                                       "y": float(self.current_pose.orientation.y),
                                                       "z": float(self.current_pose.orientation.z),}
                                       }
                pose_dict["poses"].append(single_pose)
                print("capture")
            elif i == "q":
                print(pose_dict)
                ms_sub.sub.unregister()
                js_sub.sub.unregister()
                rospy.signal_shutdown("finished")
                with open("capture.yaml", "w") as f:
                    yaml.dump(pose_dict, f)
                #del s
                break

    def cb(self, ms, js):
        for i,n in enumerate(ms.ms.name):
            if n == "amy":
                self.current_pose = ms.ms.pose[i]
                break
        self.current_js = js


c = Capture()


