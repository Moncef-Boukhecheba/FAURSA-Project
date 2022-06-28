#! /usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

map_info = None
offsets = None


def callback_map(msg):
    global offsets
    global map_info

    # Set map info
    if map_info == None or map_info != msg.info:
        map_info = msg.info

        # Set map offsets
        if offsets == None:
            offset_x = msg.info.width // 2
            offset_y = msg.info.height // 2
            offsets = (offset_x, offset_y)

    # Create Numpy array to store occupancy grid
    data = np.array(msg.data)
    data = data.reshape(msg.info.height, msg.info.width)

    rospy.loginfo(f"Offset x : {offsets[0]}")
    rospy.loginfo(f"size : {len(data)}")
    rospy.loginfo(f"----- Example : {coordinates_to_cell(data, 0, 0)}")


def coordinates_to_cell(data, x, y):
    global map_info
    x = round(x / map_info.resolution)
    y = round(y / map_info.resolution)
    rospy.loginfo(f"x : {x} y : {y}")
    return data[offsets[1] + y][offsets[0] + x]


def main():
    rospy.init_node("reading_map")
    sub = rospy.Subscriber("/map", OccupancyGrid, callback_map)
    rospy.spin()


if __name__ == "__main__":
    main()
