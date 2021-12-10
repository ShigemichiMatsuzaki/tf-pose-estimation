#!/usr/bin/env python
import time
import os
import sys
import ast

from threading import Lock
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from tfpose_ros.msg import Persons, Person, BodyPartElm
from tf_pose.estimator import Human, BodyPart
from tf_pose.networks import model_wh, get_graph_path

def humans_to_msg(humans):
    """
    Convert human information from OpenPose to ROS message

    Parameters
    ----------
    humans : 
        List of human body parts information

    Returns
    -------
    persons : Persons
        ROS message of body parts of humans

    """
    persons = Persons()

    for human in humans:
        person = Person()

        for k in human.body_parts:
            body_part = human.body_parts[k]

            body_part_msg = BodyPartElm()
            body_part_msg.part_id = body_part.part_idx
            body_part_msg.x = body_part.x
            body_part_msg.y = body_part.y
            body_part_msg.confidence = body_part.score
            person.body_part.append(body_part_msg)
        persons.persons.append(person)

    return persons


def msg_to_humans(persons_msg):
    """
    Convert Persons ROS message to human information

    Parameters
    ----------
    msg : 
        Message

    Returns
    -------
    humans : List
        List of a datum of a human which consists of body parts

    """
    humans = []

    for i, person in enumerate(persons_msg.persons):
        human = Human([])
        for body_part in person.body_part:
            human.body_parts[body_part.part_id] = \
                BodyPart(
                    '',
                    body_part.part_id,
                    body_part.x, body_part.y, body_part.confidence) 

        humans.append(human)

    return humans


def project_2d_to_3d(u, v, depth, camera_info):
    """Project a 2D coordinate to the 3D space using the depth value and the camera parameters
    
    Parameters
    ----------
    u : float
        x value in camera coordinate
    v : float
        y value in camera coordinate
    depth : float
        Depth value of the pixel (x, y)
    camera_info : sensor_msgs.CameraInfo
        Camera info message

    Results
    -------
    x_3d : float
        x coordinate of the 3D point
    y_3d : float
        y coordinate of the 3D point
    z_3d : float
        z coordinate of the 3D point

    """
    # Get camera parameters from the camera_info message
    fx, fy, cx, cy = camera_info_to_parameters(camera_info)

    x_3d = (u - cx) * depth / fx
    y_3d = (v - cy) * depth / fy
    z_3d = depth

    return x_3d, y_3d, z_3d


def camera_info_to_parameters(camera_info):
    """

    Parameters
    ----------
    camera_info : sensor_msgs.CameraInfo

    Returns
    -------
    fx : float
        Focal length in x axis
    fy : float
        Focal length in y axis
    cx : float
        Image center in x axis
    cy : float
        Image center in y axis

    """
    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]

    return fx, fy, cx, cy


def coordinate_to_marker(x, y, z, r=1.0, g=0.0, b=0.0, a=1.0, header=None):
    """

    Parameters
    ----------
    x : float
        x coordinate of the point
    y : float
        y coordinate of the point
    z : float
        z coordinate of the point
    r : float
        Color red
    g : float
        Color green
    b : float
        Color blue
    a : float
        Alpha
    header : 
        Header

    Returns
    -------
    marker : visualization_msgs.Marker
        Marker message

    """
    marker = Marker()

    # Set header
    if header is None:
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
    else:
        marker.header = header

    marker.ns = "right_hand_3d"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = a
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b

    return marker
