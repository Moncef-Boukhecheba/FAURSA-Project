#! /usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Point


def main():
    rospy.init_node("update_goal")

    # Get rosrun arguments
    args = rospy.myargv(argv=sys.argv)

    # If not enough arguments, throw error
    if len(args) < 2:
        print("ERROR : NOT ENOUGH ARGUMENTS")
        sys.exit(1)

    # Create goal from arguments
    goal = Point(x=int(args[1]), y=int(args[2]))

    # Publish goal
    pub = rospy.Publisher("/update_goal", Point, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.loginfo(f"Publishing Goal : {goal.x} {goal.y}")
        pub.publish(goal)
        rate.sleep()


if __name__ == "__main__":
    main()
