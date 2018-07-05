#!/usr/bin/env python

import math

kTicksPerTurn = 360.0
kRadius = 0.015
kPi = 3.14159265359
kAxisDistance = 0.75


class Pose:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta


def ComputeDistance(tacho):
    return (tacho / kTicksPerTurn) * (2 * kPi * kRadius)


def ComputePoseByTacho(pose, tacho_left, tacho_right):
    distance_left = ComputeDistance(tacho_left)
    distance_right = ComputeDistance(tacho_right)
    distance_center = (distance_left + distance_right) / 2

    heading = (distance_right - distance_left) / kAxisDistance

    print("distance_left: " + str(distance_left))
    print("distance_right: " + str(distance_right))
    print("distance_center: " + str(distance_center))
    print("heading: " + str(heading))

    x = pose.x - (distance_center * math.sin(heading))
    y = pose.y + (distance_center * math.cos(heading))
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
