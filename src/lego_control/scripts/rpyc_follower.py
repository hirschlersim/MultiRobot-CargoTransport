#!/usr/bin/env python3

import rpyc
import rospy
from geometry_msgs.msg import Vector3


def callback(data):
    rospy.loginfo(f"control signal: x={data.x}, y={data.y}, z={data.z}")

    motor1.run_forever(speed_sp=data.x * 1000)   # left
    motor2.run_forever(speed_sp=data.y * 1000)   # right


def listener():
    rospy.init_node('listener', anonymous=True)

    # control signal from main.cpp of robot_01 (/home/labuser/catkin_ws/src/exper_dmpc/src/follower_vehicle/src)
    rospy.Subscriber("cs_follower", Vector3, callback)
    rospy.spin()

    motor1.run_forever(speed_sp=0)
    motor2.run_forever(speed_sp=0)


# Create a RPyC connection to the remote ev3dev device.
# Use the hostname or IP address of the ev3dev device.
# If this fails, verify your IP connectivty via ``ping X.X.X.X``
conn = rpyc.classic.connect('192.168.0.107')

ev3dev2_motor1 = conn.modules['ev3dev2.motor']

motor1 = ev3dev2_motor1.LargeMotor(ev3dev2_motor1.OUTPUT_A)    # left motor of robot_01
motor2 = ev3dev2_motor1.LargeMotor(ev3dev2_motor1.OUTPUT_C)    # right motor of robot_01


if __name__ == '__main__':
    listener()

