#!/usr/bin/env python
import time
import os
import sys
import ast

import rospy
import rospkg
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from tfpose_ros.msg import Persons, Person, BodyPartElm
from tf_pose.common import CocoPart, CocoColors
from utils import msg_to_humans, project_2d_to_3d, coordinate_to_marker
import numpy as np


class ProjectPoseTo3d:

    def __init__(self):
        """Constructor
    
        """
        # Define indiviual subscribers
        self.pose_topic  = rospy.get_param("~pose_topic", "/pose_estimator/pose")
        self.depth_topic = rospy.get_param("~depth_topic", "/depth_to_rgb/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/rgb/camera_info")
        self.sub_persons = message_filters.Subscriber(self.pose_topic, Persons)
        self.sub_depth   = message_filters.Subscriber(self.depth_topic, Image)
        self.sub_camera_info = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.callback_camera_info, queue_size=1)

        # A synchronizer for the pose and depth topics
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer([self.sub_persons, self.sub_depth], 10, 1/30*0.5)
        self.time_synchronizer.registerCallback(self.callback_persons_and_depth)

        # Define publishers
        self.pub_marker = rospy.Publisher("~marker", Marker, queue_size=10)

        # CV Bridge
        self.bridge = CvBridge()

        # Camera info
        self.camera_info = None


    def callback_persons_and_depth(self, persons, depth):
        """Joint callback for person poses and a depth image

        Parameters
        ----------
        persons : Persons
            Persons message
        depth : Image
            Depth image message
        """
        if self.camera_info is None:
            return 

        # List of human poses
        humans = msg_to_humans(persons)

        # If the list is empty, return
        if not humans:
            return

        # Depth
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr('Converting Image Error. ' + str(e))
            return

        img_w, img_h = persons.image_w, persons.image_h
        # Sort the list by the distance (depth) 
        # Choose the closest human
        min_mean_depth = 1000000
        closest_human = None
        for human in humans:
            mean_depth = self.mean_depth_of_body_parts(human, depth_image, img_w, img_h)
            if mean_depth < min_mean_depth:
                min_mean_depth = mean_depth
                closest_human = human
        
        # If the right hand is not detected, return
        if CocoPart.RWrist.value not in humans[0].body_parts.keys():
            rospy.logwarn("Right hand is not detected")
            return

        # Get the coordinates of the right hand in the image coordinate
        right_wrist = closest_human.body_parts[CocoPart.RWrist.value]
        x, y = right_wrist.x, right_wrist.y
        u, v = int(x*img_w), int(y*img_h)

        # Calculate depth
        depth_val = self.get_median_depth(depth_image, u, v, img_w, img_h)

        # Project the 2D joint coordinate to the 3D space
        x_3d, y_3d, z_3d = project_2d_to_3d(u, v, depth_val, self.camera_info)

        color_right_hand = CocoColors[CocoPart.RWrist.value]
        marker = coordinate_to_marker(x_3d, y_3d, z_3d,
            r=color_right_hand[2], g=color_right_hand[1], b=color_right_hand[0],
            header=depth.header)

        self.pub_marker.publish(marker)


    def callback_camera_info(self, camera_info):
        """Callback for camera info used in the projection of 2D coordinates to the 3D space

        Save the subscribed camera_info in the class variable 'self.camera_info'

        Parameters
        ----------
        camera_info : sensor_msgs.CameraInfo
            Camera info message

        """
        self.camera_info = camera_info


    def mean_depth_of_body_parts(self, human, depth, img_w, img_h):
        """Calculate the mean depth of the detected body parts of a human

        Parameters
        ----------
        human : Human
            A human object
        depth : numpy.ndarray
            Depth image
        img_w : int
            Width of the image
        img_h : int
            Height of the image

        Returns
        -------
        mean_depth : float
            A mean depth value among the body parts
        """
        depth_sum = 0
        for body_part in human.body_parts.values():
            u, v = int(body_part.x*img_w), int(body_part.y*img_h)
            depth_sum = self.get_median_depth(depth, u, v, img_w, img_h)

        return depth_sum / len(human.body_parts)


    def get_median_depth(self, depth, u, v, img_w, img_h, window_size=5):
        """Get median depth of the neighbor pixels of the pixel (u, v)

        Parameters
        ----------
        depth : numpy.ndarray
            Depth image
        u : int
            x coordinate in the image frame
        y : int
            y coordinate in the image frame
        img_w : int
            Width of the image
        img_h : int
            Height of the image
        window_size : int
            Size of the window of the neighboring pixels. Default: 5

        Returns
        -------
        depth_val : float
            Median depth within the neighbors
        """
        # Calculate depth
        h_min, h_max = max(0, v-window_size), min(v+window_size, img_h)
        w_min, w_max = max(0, u-window_size), min(u+window_size, img_w)
        d_neighbor = depth[h_min:h_max, w_min:w_max]
        depth_val = np.median(d_neighbor)

        return depth_val


if __name__ == '__main__':
    rospy.loginfo('initialization+')
    rospy.init_node('project_to_3d')

    project_to_3d = ProjectPoseTo3d()

    rospy.spin()

