#!/usr/bin/env python
import rospy
import actionlib

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Empty
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from cv_bridge import CvBridge

import numpy as np
import cv2
import threading
import math
from scipy.ndimage.measurements import label
from view_tuner import ViewTuner

class TestViewTuner(object):
    def __init__(self, tune_img_topic, img_topic, head_state_topic):
        """
        Subscribes to a tune_img_topic (type: Empty) and an img_topic (type: CompressedImage).
        When it gets a message on the tune_img_topic, it initializes the
        ViewTuner. After that, for every image recieved by the img_topic, it
        tunes the image. Once tuning is done, it deinitializes the view tuner.
        """
        self.has_loaded = False

        # Initialize the view_tuner
        self.view_tuner = ViewTuner(head_state_topic)
        self.view_tuner_lock = threading.Lock()

        self.bridge = CvBridge()

        # Initialize the tune_img_topic
        self.tune_img_sub = rospy.Subscriber(
            tune_img_topic, Empty, self.tune_img_callback, queue_size=1)

        # Initialize the image topics
        self.img_pub_connected_components = rospy.Publisher('connected_components', Image, queue_size=1)
        self.img_pub_annotated = rospy.Publisher('img_annotated', Image, queue_size=1)
        self.img_sub = rospy.Subscriber(
            img_topic, CompressedImage, self.img_callback, queue_size=1)
        self.most_recent_img_msg_lock = threading.Lock()
        self.most_recent_img_msg = None

        self.has_loaded = True

    def tune_img_callback(self, tune_img_msg):
        rospy.loginfo("Got tune_img_msg!")
        if not self.has_loaded: return
        with self.view_tuner_lock:
            is_view_tuner_initialized = self.view_tuner.is_initialized()
        if not is_view_tuner_initialized:
            with self.most_recent_img_msg_lock:
                img_msg = self.most_recent_img_msg
            rospy.loginfo("about to call self.view_tuner.initialize_tuner %s" % img_msg)
            if img_msg is None: return
            with self.view_tuner_lock:
                connected_components_cv2 = self.view_tuner.initialize_tuner(img_msg)

            connected_components_msg = self.bridge.cv2_to_imgmsg(connected_components_cv2, encoding="passthrough")
            connected_components_msg.step = int(connected_components_msg.step)
            self.img_pub_connected_components.publish(connected_components_msg)

    def img_callback(self, img_msg):
        rospy.loginfo("Got img_msg!")
        if not self.has_loaded: return
        with self.most_recent_img_msg_lock:
            self.most_recent_img_msg = img_msg
        with self.view_tuner_lock:
            is_view_tuner_initialized = self.view_tuner.is_initialized()
        if is_view_tuner_initialized:
            rospy.loginfo("about to call view_tuner tune_image")
            with self.view_tuner_lock:
                is_done, img_annotated = self.view_tuner.tune_image(img_msg, return_annotated_img=True)
            
            if img_annotated is not None:
                img_msg_annotated = self.bridge.cv2_to_imgmsg(img_annotated, encoding="passthrough")
                img_msg_annotated.step = int(img_msg_annotated.step)
                self.img_pub_annotated.publish(img_msg_annotated)

            if is_done:
                self.view_tuner.deinitialize_tuner()

if __name__ == '__main__':
    rospy.init_node('test_view_tuner')

    tune_img_topic = rospy.get_param('~tune_img_topic', '/tune_image')
    img_topic = rospy.get_param('~img_topic', '/upward_looking_camera/compressed')
    head_state_topic = rospy.get_param('~head_state_topic', '/head_controller/state')
    view_tuner = TestViewTuner(tune_img_topic, img_topic, head_state_topic)

    rospy.spin()
