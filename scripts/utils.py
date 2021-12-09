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

