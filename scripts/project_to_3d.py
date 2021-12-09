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
from sensor_msgs.msg import Image
from tfpose_ros.msg import Persons, Person, BodyPartElm
from tf_pose.common import CocoPart
from utils import msg_to_humans
import numpy as np


class ProjectPoseTo3d:

    def __init__(self):
        """
    
        """
        # Define indiviual subscribers
        self.pose_topic  = "/pose_estimator/pose"
        self.depth_topic = "/depth_to_rgb/image_raw"
        self.sub_persons = message_filters.Subscriber(self.pose_topic, Persons)
        self.sub_depth   = message_filters.Subscriber(self.depth_topic, Image)

        # Define a synchronizer
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer([self.sub_persons, self.sub_depth], 10, 1/30*0.5)
        self.time_synchronizer.registerCallback(self.callback_persons_and_depth)

        # CV Bridge
        self.bridge = CvBridge()


    def callback_persons_and_depth(self, persons, depth):
        """
    
        """
        # List of human poses
        humans = msg_to_humans(persons)

        # Depth
        try:
            cv_image = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr('Converting Image Error. ' + str(e))
            return
        
        # Id=3 -> Right elbow
        # Id=4 -> Right hand
        if CocoPart.RWrist.value not in humans[0].body_parts.keys():
            return

        right_wrist = humans[0].body_parts[CocoPart.RWrist.value]
        x = right_wrist.x
        y = right_wrist.y
        img_w = persons.image_w
        img_h = persons.image_h

        # Calculate depth
        h_min = max(0, int(y*img_h)-5)
        h_max = min(int(y*img_h)+5, img_h)
        w_min = max(0, int(x*img_w)-5)
        w_max = min(int(x*img_w)+5, img_w)
        d_neighbor = cv_image[h_min:h_max, w_min:w_max]
        depth = np.median(d_neighbor)

if __name__ == '__main__':
    rospy.loginfo('initialization+')
    rospy.init_node('project_to_3d', anonymous=True, log_level=rospy.INFO)

    project_to_3d = ProjectPoseTo3d()

    rospy.spin()

