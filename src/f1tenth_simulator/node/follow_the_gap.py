#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import numpy as np

class FollowTheGap:
    def __init__(self):
        rospy.init_node("follow_the_gap")
        self.lidar_fov = 0.69

        # Get topic names
        drive_topic = rospy.get_param("~gap_drive_topic", "/drive")
        odom_topic = rospy.get_param("~odom_topic", "/odom")
        scan_topic = rospy.get_param("~scan_topic", "/scan")

        # Make a publisher for drive messages
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

        # Start a subscriber to listen to odom messages
        #self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)

        # Start a subscriber to listen to laser scan messages
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan, self.lidar_callback)

    #def odom_callback(self, msg):
        #pass
    def preprocess_lidar(self, ranges, window_size=15, max_value=10, avoid = 50):
        """ Preprocess the LiDAR scan array.
        Args:
            ranges (List[float]): A list of ranges from the LiDAR scan.
            window_size (int): The size of the window for calculating the mean.
            max_value (float): The maximum value to reject.
        Returns:
            List[float]: The preprocessed LiDAR scan array.
        """
        ranges = list(ranges)
        # Check if the range list is empty
        if not ranges:
            return []
        minima = min(ranges)
        preprocessed_ranges = []
        # Iterate over the ranges
        for i in range(len(ranges)):
            if ranges[i] == minima:
                #create an avoidance of anything near minimum distance
                for j in range(avoid):
                    ranges[min(max(0,i-(avoid/2)+j),len(ranges)-1)] = 0.0

            # Calculate the start and end indices of the window
            start = max(0, i - window_size // 2)
            end = min(len(ranges), i + window_size // 2 + 1)

            # Extract the values within the window
            window = ranges[start:end]

            # Calculate the mean of the values within the window disregarding large values
            mean_value = np.mean(window[ranges<max_value]) #! disregrads any number greater than max_value
            preprocessed_ranges.append(mean_value)
        return preprocessed_ranges
    
    def find_max_gap(self, free_space_ranges, max = 2):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        max_gap_start = 0
        max_gap_end = 0
        max_gap_length = 0
        

        gap_start = None
        gap_end = None
        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] > max/2:  #! Make this adaptive at some point 1/2 the max length perhaps
                if gap_start is None:
                    gap_start = i
                else:
                    gap_end = i
            else:
                if gap_start is not None and gap_end is not None:
                    gap_length = gap_end - gap_start
                    if gap_length > max_gap_length:
                        max_gap_length = gap_length
                        max_gap_start = gap_start
                        max_gap_end = gap_end
                    gap_start = None
                    gap_end = None
        if gap_start is not None and gap_end is not None:
            gap_length = gap_end - gap_start
            if gap_length > max_gap_length:
                max_gap_length = gap_length
                max_gap_start = gap_start
                max_gap_end = gap_end

        return max_gap_start, max_gap_end

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges 
        #added to handle 270 degree scan

        cut_points = int((data.angle_max - self.lidar_fov)/data.angle_increment)
        ranges=ranges[cut_points:-cut_points]

        
        #proc_ranges = self.preprocess_lidar(ranges)
        gap_start, gap_end = self.find_max_gap(ranges)
        best_point = (gap_start+gap_end)/2
        speed = min(2 + max(ranges)/6, 3.25)
        steering_angle = (best_point+cut_points)*data.angle_increment + data.angle_min    #(data.angle_min + data.angle_increment * best_point)*(1.5/speed)
        
        self.pub_drive(speed, steering_angle)

    def pub_drive(self, speed, steering_angle):
        #publish drive messages
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

if __name__ == "__main__":
    try:
        FollowTheGap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
