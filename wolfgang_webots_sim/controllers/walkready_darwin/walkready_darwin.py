#!/usr/bin/env python3
from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

goals = {'Neck': -2.775320771747065e-12,
 'Head': 0.17355552825075066,
 'PelvYL': -6.254673996786142e-08,
 'PelvL': -0.006143576375813058,
 'LegUpperL': 0.6312505601832611,
 'LegLowerL': -0.9292138418821763,
 'AnkleL': -0.5237383600397395,
 'FootL': -0.013822861199158423,
 'PelvYR': 3.835882111252742e-08,
 'PelvR': 0.006143580251058486,
 'LegUpperR': -0.6312504822202137,
 'LegLowerR': 0.9292138681185262,
 'AnkleR': 0.523738402881564,
 'FootR': 0.013822858158549035,
 'ShoulderL': 0.7218681113423607,
 'ArmUpperL': 0.3071779763446628,
 'ArmLowerL': -0.5160589339523639,
 'ShoulderR': -0.8447392839017046,
 'ArmUpperR': -0.3117856402307673,
 'ArmLowerR': 0.5114512666756715}

motors = {}
for k in goals.keys():
    motors[k] = robot.getDevice(k)

while (robot.step(timestep) != -1):
  for k,v in motors.items():
      v.setPosition(goals[k])

