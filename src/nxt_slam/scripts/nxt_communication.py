#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import tf

import nxt.locator
import time
import nxt.bluesock

from nxt.sensor import Ultrasonic, PORT_2, PORT_1, PORT_3
from nxt.motor import Motor, PORT_A, PORT_B, PORT_C
from odometry import Robot, Pose, ComputePoseByTacho

br = tf.TransformBroadcaster()
pub = rospy.Publisher('scan', LaserScan, queue_size=10)

kAngleMax = 0.5
kNumberOfScans = 20
kSpeed = 20


def ConnectToNxt():
    brick = nxt.bluesock.BlueSock('00:16:53:04:17:F1').connect()

    motor_right = Motor(brick, PORT_A)
    motor_left = Motor(brick, PORT_C)
    ultrasonic_front = Ultrasonic(brick, PORT_2)
    ultrasonic_left = Ultrasonic(brick, PORT_1)
    ultrasonic_right = Ultrasonic(brick, PORT_3)

    return motor_right, motor_left, ultrasonic_front, ultrasonic_left, ultrasonic_right


def GetOdometry(motor_right, motor_left):
    # Old versions seemed to be outdated
    # motor_state_a = motor_right.get_output_state()
    # motor_state_c = motor_left.get_output_state()
    # motor_state_a = motor_right.get_tacho_and_state()[0]
    # motor_state_c = motor_left.get_tacho_and_state()[0]

    # Old version assumed output state
    # tacho_motor_right = motor_state_a[7]
    # tacho_motor_left = motor_state_c[7]
    tacho_motor_right = motor_right.get_tacho().rotation_count
    tacho_motor_left = motor_left.get_tacho().rotation_count
    # rospy.loginfo('Motor A State: ' + str(motor_state_a))
    # rospy.loginfo('Motor C State: ' + str(motor_state_c))
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
    # br.sendTransform((0.08, 0.0, 0.073),
    br.sendTransform((0.0, 0.0, 0.0),
                 tf.transformations.quaternion_from_euler(0, 0, 0),
                 rospy.Time.now(),
                 'base_laser_front',
                 'base_link')

    br.sendTransform((0.0, 0.0, 0.0),
                 tf.transformations.quaternion_from_euler(0, 0, 3.1415 / 2),
                 rospy.Time.now(),
                 'base_laser_left',
                 'base_link')

    br.sendTransform((0.0, 0.0, 0.0),
                 tf.transformations.quaternion_from_euler(0, 0, -3.1415 / 2),
                 rospy.Time.now(),
                 'base_laser_right',
                 'base_link')


def PrepareScan(sequence, frame_id):
    scan = LaserScan()
    scan.header.seq = sequence
    scan.header.stamp = rospy.get_rostime()
    scan.header.frame_id = frame_id
    scan.angle_min = -kAngleMax
    scan.angle_max = kAngleMax
    scan.angle_increment = (2 * kAngleMax) / kNumberOfScans
    scan.range_max = 2.54
    return scan


def SendScan(sequence, frame_id, distance_meter):
    scan = PrepareScan(sequence, frame_id)

    distance = distance_meter / 100.0

    ranges = range(-1 * (kNumberOfScans / 2), (kNumberOfScans / 2) + 1)
    scan.ranges = [distance for i in ranges if i != 0]
    scan.intensities = [abs(1/float(i)) for i in ranges if i != 0]

    rospy.loginfo(scan)
    pub.publish(scan)


def SpinAround(motor_right, motor_left, speed_right, speed_left):
    motor_right.run(speed_right)
    motor_left.run(speed_left)


def UpdateMotorTacho(robot, tacho_motor_right, tacho_motor_left):
    tacho_right_diff = robot.tacho_right - tacho_motor_right
    tacho_left_diff = robot.tacho_left - tacho_motor_left

    robot.tacho_right = tacho_motor_right
    robot.tacho_left = tacho_motor_left
    return robot, tacho_right_diff, tacho_left_diff


def main():
    rospy.init_node('nxt_communication', anonymous=True)
    rate = rospy.Rate(10)

    motor_right, motor_left, ultrasonic_front, ultrasonic_left, ultrasonic_right = ConnectToNxt()

    tacho_motor_right, tacho_motor_left = GetOdometry(motor_right, motor_left)
    robot = Robot(tacho_motor_right, tacho_motor_left)

    sequence = 1
    while not rospy.is_shutdown():
        tacho_motor_right, tacho_motor_left = GetOdometry(motor_right, motor_left)
        measurement_front = GetUltrasonic(ultrasonic_front)
        measurement_left = GetUltrasonic(ultrasonic_left)
        measurement_right = GetUltrasonic(ultrasonic_right)

        robot, tacho_right_diff, tacho_left_diff = UpdateMotorTacho(robot, tacho_motor_right, tacho_motor_left)
        robot.pose = ComputePoseByTacho(robot.pose, tacho_right_diff, tacho_left_diff)

        SendStatictransform()
        SendOdoTransform(robot.pose)

        if (measurement_front < 255):
            SendScan(sequence, 'base_laser_front', measurement_front)
        if (measurement_left < 255):
            SendScan(sequence, 'base_laser_left', measurement_left)
        if (measurement_right < 255):
            SendScan(sequence, 'base_laser_right', measurement_right)

        speed_right = kSpeed
        speed_left = kSpeed
        if (measurement_front < 50):
            speed_right = -kSpeed

        # SpinAround(motor_right, motor_left, speed_right, speed_left)

        sequence += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
