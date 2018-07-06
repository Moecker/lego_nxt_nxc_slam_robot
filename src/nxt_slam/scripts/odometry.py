#!/usr/bin/env python

import math

kTicksPerTurn = 360.0
kDiameter = 0.055
kRadius = kDiameter / 2.0
kPi = 3.14159265359
kAxisDistance = 0.113


class Robot:
    def __init__(self, tacho_left=0.0, tacho_right=0.0):
        self.pose = Pose()
        self.tacho_right = tacho_right
        self.tacho_left = tacho_left
        self.theta = 0.0


class Pose:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta


def ComputeDistance(tacho):
    return (tacho / kTicksPerTurn) * (2 * kPi * kRadius)


def ComputePoseByTacho(pose, tacho_right, tacho_left):
    distance_right = ComputeDistance(tacho_right)
    distance_left = ComputeDistance(tacho_left)
    distance_center = (distance_left + distance_right) / 2

    heading = (distance_right - distance_left) / kAxisDistance

    print("distance_right: " + str(distance_right))
    print("distance_left: " + str(distance_left))

    print("distance_center: " + str(distance_center))
    print("heading: " + str(heading))

    x = pose.x - (distance_center * math.cos(heading))
    y = pose.y + (distance_center * math.sin(heading))
    theta = pose.theta + heading

    return Pose(x, y, theta)


def PrintPose(pose):
    print("x: " + str(pose.x))
    print("y: " + str(pose.y))
    print("theta: " + str(pose.theta))


def almost_equal(x,y,threshold=0.001):
  return abs(x-y) < threshold


def test_straight():
    pose = Pose()
    pose = ComputePoseByTacho(pose, 360.0, 360.0)
    PrintPose(pose)
    assert(pose.x == 0.0)
    assert(almost_equal(pose.y, 0.094))
    assert(pose.x == 0.0)


def test_only_left():
    pose = Pose()
    pose = ComputePoseByTacho(pose, 360.0, 0.0)
    PrintPose(pose)


if __name__ == '__main__':
    test_straight()
    test_only_left()
