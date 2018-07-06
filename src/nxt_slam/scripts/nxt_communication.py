#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import tf

import nxt.locator
import time
import nxt.bluesock

from nxt.sensor import UltrasonicSensor, PORT_2
from nxt.motor import Motor, PORT_A, PORT_B, PORT_C
from odometry import Robot, Pose, ComputePoseByTacho

br = tf.TransformBroadcaster()
pub = rospy.Publisher('scan', LaserScan, queue_size=10)


def ConnectToNxt():
    brick = nxt.bluesock.BlueSock('00:16:53:04:17:F1').connect()

    motor_right = Motor(brick, PORT_A)
    motor_left = Motor(brick, PORT_C)
    ultrasonic = UltrasonicSensor(brick, PORT_2)

    return motor_right, motor_left, ultrasonic


def GetOdometry(motor_right, motor_left):
    motor_state_a = motor_right.get_output_state()
    motor_state_c = motor_left.get_output_state()
    tacho_motor_right = motor_state_a[7]
    tacho_motor_left = motor_state_c[7]
    rospy.loginfo('Motor A State: ' + str(motor_state_a))
    rospy.loginfo('Motor C State: ' + str(motor_state_c))
    rospy.loginfo('Motor A Tacho: ' + str(tacho_motor_right))
    rospy.loginfo('Motor C Tacho: ' + str(tacho_motor_left))
    return tacho_motor_right, tacho_motor_left


def GetUltrasonic(ultrasonic):
    measurement = ultrasonic.get_sample()
    rospy.loginfo('Ultrasonic: ' + str(measurement))
    return measurement


def SendOdoTransform(pose):
    rospy.loginfo("Sending Pose: [x:" + str(pose.x) + ",y:" + str(pose.y) + ",theta:" + str(pose.theta) +"]")
    br.sendTransform((pose.x, pose.y, 0),
                 tf.transformations.quaternion_from_euler(0, 0, pose.theta),
                 rospy.Time.now(),
                 'base_link',
                 'odom')


def SendStatictransform():
    br.sendTransform((0.08, 0.0, 0.073),
                 tf.transformations.quaternion_from_euler(0, 0, 0),
                 rospy.Time.now(),
                 'base_laser',
                 'base_link')


def SendScan(sequence, distance_meter):
    scan = LaserScan()

    distance = distance_meter / 100.0

    scan.header.seq = sequence
    scan.header.stamp = rospy.get_rostime()
    scan.header.frame_id = 'base_laser'
    scan.angle_min = -0.01
    scan.angle_max = 0.01
    scan.angle_increment = 0.02
    scan.range_max = 2.54
    scan.ranges = [distance, distance]
    scan.intensities = [0.5, 0.5]

    rospy.loginfo(scan)
    pub.publish(scan)


def SpinAround(motor_right, motor_left):
    motor_left.run(20)
    motor_right.run(10)


def main():
    rospy.init_node('nxt_communication', anonymous=True)
    rate = rospy.Rate(10)

    motor_right, motor_left, ultrasonic = ConnectToNxt()

    tacho_motor_right, tacho_motor_left = GetOdometry(motor_right, motor_left)
    robot = Robot(tacho_motor_right, tacho_motor_left)

    SpinAround(motor_right, motor_left)

    sequence = 1
    while not rospy.is_shutdown():
        tacho_motor_right, tacho_motor_left = GetOdometry(motor_right, motor_left)
        measurement = GetUltrasonic(ultrasonic)

        tacho_right_diff = robot.tacho_right - tacho_motor_right
        tacho_left_diff = robot.tacho_left - tacho_motor_left

        robot.tacho_right = tacho_motor_right
        robot.tacho_left = tacho_motor_left
        robot.pose = ComputePoseByTacho(robot.pose, tacho_right_diff, tacho_left_diff)

        SendStatictransform()
        SendOdoTransform(robot.pose)

        if (measurement < 255.0):
            SendScan(sequence, measurement)

        sequence += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
