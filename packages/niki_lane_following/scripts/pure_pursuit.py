#!/usr/bin/env python
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped, SegmentList, Segment, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading
import math
import time

class pp_lane_controller(object):
    def __init__(self):

        # Start rospy for this node
        rospy.init_node("lane_controller_node", anonymous=False)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("/chloe/lane_filter_node/seglist_filtered", SegmentList, self.processSegments, queue_size=1)

        # Publication
        self.pub_car_cmd = rospy.Publisher("/chloe/joy_mapper_node/car_cmd", Twist2DStamped, queue_size=1)

        # Stop on shutdown
        rospy.on_shutdown(self.custom_shutdown())

    def custom_shutdown(self):
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0 
        self.pub_car_cmd.publish(car_control_msg)

    def processSegments(self, input_segment_list):
        all_segments = input_segment_list.segments # this is a list of type Segment

        num_white = 0
        num_yellow = 0
        total_white_x = 0
        total_white_y = 0
        total_yellow_x = 0
        total_yellow_y = 0
        num_points = 2. * len(all_segments)

        for segment in all_segments:

            point0 = segment.points[0]
            point1 = segment.points[1]
            col = segment.color

            if col == 0:
                num_white += 1
                total_white_x += point0.x + point1.x
                total_white_y += point0.y + point1.y
            elif col == 1:
                num_yellow += 1
                total_yellow_x += point0.x + point1.x
                total_yellow_y += point0.y + point1.y

        # Tell the car to do based on what we see
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0

        # First, corner cases for when see no lines or only one line
        if num_white + num_yellow == 0:
            car_control_msg.v = 0.0
            car_control_msg.omega = -1
        elif num_white == 0: # and num_yellow != 0
            # Want to turn right (improve later)
            car_control_msg.v = 0.06
            car_control_msg.omega = -1
        elif num_yellow == 0: # and num_white != 0
            car_control_msg.v = 0.06
            car_control_msg.omega = 1
        else: # see both colours
            ave_white_x = total_white_x / num_white
            ave_white_y = total_white_y / num_white
            ave_yellow_x = total_yellow_x / num_yellow
            ave_yellow_y = total_yellow_y / num_yellow

            full_ave_x = (ave_white_x + ave_yellow_x) / 2.
            full_ave_y = (ave_white_y + ave_yellow_y) / 2.

            alpha = np.arctan2(full_ave_y, full_ave_x)
            omega = 3 * np.sin(alpha)

            car_control_msg.v = 0.1
            car_control_msg.omega = omega

        # Send the command to the car
        self.pub_car_cmd.publish(car_control_msg)
    
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = pp_lane_controller()
        node.spin()
    except rospy.ROSInterruptException:
        pass
