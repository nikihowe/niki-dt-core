#!/usr/bin/env python
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped, SegmentList, Segment, LanePose, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading
import math
import time

class pp_lane_controller(object):
    def __init__(self):

        location = "default" # change this to run in sim / on robot
        location = "chloe"

        # Start rospy for this node
        rospy.init_node("lane_controller_node", anonymous=False)

        # Subscriptions
        self.sub_lane_reading = rospy.Subscriber("/{}/lane_filter_node/seglist_filtered".format(location), SegmentList, self.process_segments, queue_size=1)

        # Publication
        self.pub_car_cmd = rospy.Publisher("/{}/joy_mapper_node/car_cmd".format(location), Twist2DStamped, queue_size=1)

        # Stop on shutdown
        rospy.on_shutdown(self.custom_shutdown)

    def custom_shutdown(self):
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0 
        self.pub_car_cmd.publish(car_control_msg)

    def process_segments(self, input_segment_list):
        all_segments = input_segment_list.segments # this is a list of type Segment

        num_white = 0
        num_yellow = 0
        num_white_far = 0
        num_yellow_far = 0
        '''
        total_white_x = 0
        total_white_y = 0
        total_yellow_x = 0
        total_yellow_y = 0
        total_white_far_x = 0
        total_white_far_y = 0
        total_yellow_far_x = 0
        total_yellow_far_y = 0
        '''

        total_white = np.zeros(2)
        total_white_far = np.zeros(2)
        total_yellow = np.zeros(2)
        total_yellow_far = np.zeros(2)

        far = 0.2

        for segment in all_segments:

            point0 = segment.points[0]
            point1 = segment.points[1]
            col = segment.color

            # We only care about the average point of the segment
            ave_point = np.array([point0.x + point1.x, point0.y + point1.y]) / 2.
            # ^ Q: is this misguided? Will this unfairly remove points?

            if col == 0:
                num_white += 1
                total_white += ave_point

                if ave_point.dot(ave_point) > far:
                    num_white_far += 1
                    total_white_far += ave_point

            elif col == 1:
                num_yellow += 1
                total_yellow += ave_point

                if ave_point.dot(ave_point) > far:
                    num_white_far += 1
                    total_yellow_far += ave_point

        #ave_white_far = total_white_far * 1. / num_white_far
        #ave_yellow_far = total_yellow_far * 1. / num_yellow_far

        # Tell the car to do based on what we see
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0

        if num_white + num_yellow == 0:
            # Want to turn right (improve later)
            car_control_msg.v = 0.15
            car_control_msg.omega = -2

        elif num_white == 0: # and num_yellow != 0
            if num_yellow_far > 0:
                ave_yellow = total_yellow_far * 1. / num_yellow_far
                ave_yellow[1] -= 0.15 # subtract offset
            else:
                ave_yellow = total_yellow * 1. / num_yellow
                ave_yellow[1] -= 0.25 # subtract offset

            alpha = np.arctan2(ave_yellow[1], ave_yellow[0])
            omega = 6 * np.sin(alpha)

            car_control_msg.v = 0.2
            car_control_msg.omega = omega

        elif num_yellow == 0: # and num_white != 0
            if num_white_far > 0:
                ave_white = total_white_far * 1. / num_white_far
                ave_white += 0.15 # add offset
            else:
                ave_white = total_white * 1. / num_white
                ave_white += 0.25 # add offset

            alpha = np.arctan2(ave_white[1], ave_white[0])
            omega = 6 * np.sin(alpha)

            car_control_msg.v = 0.2
            car_control_msg.omega = omega

        else: # see both colours
            if num_white_far > 0 and num_yellow_far > 0:
                ave_white = total_white_far * 1. / num_white_far
                ave_yellow = total_yellow_far * 1. / num_yellow_far
            else:
                ave_white = total_white * 1. / num_white
                ave_yellow = total_yellow * 1. / num_yellow

            overall_ave = (ave_white + ave_yellow) / 2.

            alpha = np.arctan2(overall_ave[1], overall_ave[0])
            omega = 3 * np.sin(alpha)

            car_control_msg.v = 0.3
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
