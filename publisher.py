#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import tf

def talker():
    pub = rospy.Publisher('scan', LaserScan, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    br = tf.TransformBroadcaster()

    sequence = 1
    while not rospy.is_shutdown():
        br.sendTransform((0.1, 0.1, 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     'base_laser',
                     'base_link')

        br.sendTransform((sequence * 0.01, sequence * 0.01, 0),
                     tf.transformations.quaternion_from_euler(0, 0, sequence * 0.01),
                     rospy.Time.now(),
                     'base_link',
                     'odom')

        scan = LaserScan()
        scan.header.seq = sequence
        scan.header.stamp = rospy.get_rostime()
        scan.header.frame_id = 'base_laser'
        scan.angle_min = -0.1
        scan.angle_max = 0.1
        scan.angle_increment = 0.1
        scan.range_max = 5.0
        scan.ranges = [4.0, 4.0, 4.0]
        scan.intensities = [0.1, 0.1, 0.1]

        rospy.loginfo(scan)
        pub.publish(scan)
        sequence += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
