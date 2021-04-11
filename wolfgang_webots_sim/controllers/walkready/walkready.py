#!/usr/bin/env python3
from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

goals = {
 "HeadPan" : 0.0,
 "HeadTilt" : 0.0,
 "LHipYaw" : -0.000191931040049739,
 "LHipRoll" : 5.844524923482412,
 "LHipPitch" : 27.51014255896886,
 "LKnee" : 66.40800034993379,
 "LAnklePitch" : -23.898215720924245,
 "LAnkleRoll" : 5.843814459156438,
 "LShoulderPitch" : 0.07011122288517968,
 "LShoulderRoll" : 0.0,
 "LElbow" : 60.0001403060998,
 "RHipYaw" : 0.00024901724922846396,
 "RHipRoll" : -5.563228448991872,
 "RHipPitch" : -27.427812667332002,
 "RKnee" : -66.43047543386545,
 "RAnklePitch" : 26.09208387655842,
 "RAnkleRoll" : -6.851202125376632,
 "RShoulderPitch" : -0.07130823071763022,
 "RShoulderRoll" : 0.0,
 "RElbow" : -60.0001403060998
}
motors = {}
for k in goals.keys():
    motors[k] = robot.getDevice(k)
    goals[k] = goals[k]/180.0 * 3.14159

while (robot.step(timestep) != -1):
  for k,v in motors.items():
      v.setPosition(goals[k])

