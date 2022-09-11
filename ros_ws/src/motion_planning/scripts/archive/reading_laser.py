#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan


def callback_laser(msg):
    # 120 degrees into 3 regions
    # receive a value of range between 0 and 10.
    d = len(msg.ranges) // 3
    regions = {
        'right':  min(min(msg.ranges[0:d]), 10),
        'front':  min(min(msg.ranges[d:d*2]), 10),
        'left':   min(min(msg.ranges[d*2:d*3]), 10),
    }

    rospy.loginfo(len(msg.ranges))
    rospy.loginfo("-------------")
    rospy.loginfo(regions)


def main():
    rospy.init_node('reading_laser')
    sub = rospy.Subscriber("/laser_scan", LaserScan, callback_laser)
    rospy.spin()


if __name__ == '__main__':
    main()
