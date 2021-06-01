#!/usr/bin/env python
# ROS Libraries
import rospy
from sensor_msgs.msg import CompressedImage
from kuri_cmm_demo.srv import ObjectDetection, ObjectDetectionResponse
# Python Default Libraries
import base64
import cv2 as cv
import numpy as np
import os
import requests
import threading
import time
import traceback
# Custom Libraries
from sent_messages_database import SentMessagesDatabase
from subsampling_policy import SubsamplingPolicy
from to_send_policy import ToSendPolicy

class CMMDemo(object):
    """
    The central executive node of our demo. This node subscribes to images from
    the robot, detects objects in the image, vectorizes it, determines whether
    to send it (and to whom), and sends it to the Slackbot. It has a separate
    thread that continually queries the Slackbot for new responses and updates
    the beliefs accordingly.
    """
    def __init__(self, img_topic, object_detection_srv, slackbot_url,
        send_messages_database_filepath):
        """
        Initialize an instance of the CMMDemo class
        """
        self.has_loaded = False

        # Parameters relevant to subsampling
        self.subsampling_policy = SubsamplingPolicy(
            rule='n_per_sec', rule_config={'n':1},
        )
        self.img_sub = rospy.Subscriber(
            img_topic, CompressedImage, self.img_callback, queue_size=1)

        # Parameters relevant to object detection
        self.object_detection_srv_name = object_detection_srv
        self.object_detection_srv_lock = threading.Lock()
        self.object_detection_srv = rospy.ServiceProxy(object_detection_srv, ObjectDetection)

        # Parameters relevant to storing images and message IDs
        self.send_messages_database_filepath = send_messages_database_filepath
        self.sent_messages_database = SentMessagesDatabase.load(self.send_messages_database_filepath)
        self.database_save_interval = 5
        self.database_updates_since_last_save = 0

        # Parameters relevant to determine whether to send the image
        self.to_send_policy = ToSendPolicy(self.sent_messages_database)
        self.to_send_policy_lock = threading.Lock()

        # Parameters relevant to communicating with the Slackbot
        self.slackbot_url = slackbot_url
        self.slackbot_responses_thread = threading.Thread(
            target=self.get_slackbot_responses,
        )
        self.slackbot_responses_thread.start()

        self.has_loaded = True

    def database_updated(self):
        """
        Called everytime the database is updated. Saves the database every
        self.database_save_interval updates
        """
        self.database_updates_since_last_save += 1
        if self.database_updates_since_last_save % self.database_save_interval == 0:
            self.sent_messages_database.save(self.send_messages_database_filepath)
            rospy.logdebug("Saved sent_messages_database!")

    @staticmethod
    def is_similar(most_recent_sent_img_msg, most_recent_sent_img_vector,
        img_msg, img_vector, objects_threshold=3, image_threshold=0.75):
        """
        Returns True if img is sufficiently similar to most_recent_img to not
        sent it, False otherwise. Images are deemed to be similar if there
        are at least num_new_objects_threshold objects that each had confidence
        >= object_confidence_threshold in one image but not the other.

        TODO: There is stochasticity in AWS Rekognition, and given two images
        that are visually the same, it may detect different objects. Therefore,
        an improvement to this would be also looking at the histogram distribution
        (either grayscale or separately for RGB) and deeming an image "similar"
        if either its histogram is too similar or its detected objects are too
        similar. The reason we don't do only-histogram is because of changing
        lighting, but the hybrid approach should work well.
        """
        # Pad the most_recent_sent_img_vector
        num_new_objects = img_vector.shape[0] - most_recent_sent_img_vector.shape[0]
        most_recent_sent_img_vector = np.pad(most_recent_sent_img_vector, (0, num_new_objects), 'constant', constant_values=0)

        # Get the change in probability for each object
        different_objects = np.abs(img_vector - most_recent_sent_img_vector)

        # Get the total chnge in probability
        change_in_prob = np.sum(different_objects)
        object_is_similar = change_in_prob <= objects_threshold

        # Get the similarity of the histograms
        most_recent_sent_img = cv.imdecode(np.fromstring(most_recent_sent_img_msg.data, np.uint8), cv.IMREAD_COLOR)
        img = cv.imdecode(np.fromstring(img_msg.data, np.uint8), cv.IMREAD_COLOR)
        avg_histogram_similarity = 0.0
        channels = [0,1,2]
        for channel in channels:
            most_recent_sent_img_hist = cv.calcHist([most_recent_sent_img],[channel],None,[256],[0,256])
            img_hist = cv.calcHist([img],[channel],None,[256],[0,256])
            avg_histogram_similarity += cv.compareHist(most_recent_sent_img_hist, img_hist, cv.HISTCMP_CORREL)/len(channels)
        image_is_similar = avg_histogram_similarity >= image_threshold

        rospy.logdebug("is_similar object_similarity %f <= %f?, image_similarity %f >= %f?" % (change_in_prob, objects_threshold, avg_histogram_similarity, image_threshold))
        return image_is_similar or object_is_similar


    def subsampled_image(self, img_msg):
        """
        Called when an image is subsampled. First, this function calls the
        object_detection service to return detected objects. Then, it determines
        whether to send the image. If it decides to send it, it calls the
        Slackbot's send_image endpoint
        """
	rospy.logdebug("Got subsampled image!")
        # Get the detected objects in the image
        rospy.wait_for_service(self.object_detection_srv_name)
        with self.object_detection_srv_lock:
            object_detection_response = self.object_detection_srv(img_msg)
        if not object_detection_response.success:
            rospy.logwarn("Object detection failed, not sending image")
            return
        detected_objects_msg = object_detection_response.detected_objects

        # Determine whether to send the image
        with self.to_send_policy_lock:
            to_send, img_vector = self.to_send_policy.to_send_policy(detected_objects_msg)

        if np.any(to_send):
            # Remove any users who got a sufficiently similar image recently
            users_to_send_to_initial = np.where(to_send)[0].tolist()
            users_to_send_to_final = []
            for user in users_to_send_to_initial:
                most_recent_sent_img_msg, most_recent_sent_img_vector = self.sent_messages_database.get_most_recent_image(user)
                if most_recent_sent_img_msg is None or not CMMDemo.is_similar(most_recent_sent_img_msg, most_recent_sent_img_vector, img_msg, img_vector):
                    users_to_send_to_final.append(user)
            # Skip the image if there are no users to send it to
            if len(users_to_send_to_final) == 0:
                return
            # Get a new message ID
            local_msg_id = self.sent_messages_database.get_new_local_msg_id()
            # Add this image to the database
            rospy.loginfo("Send image! To users %s" % users_to_send_to_final)
            rospy.logdebug("most_recent_sent_img_vector %s" % most_recent_sent_img_vector)
            rospy.logdebug("                 img_vector %s" % img_vector)
            self.sent_messages_database.add_message(
                local_msg_id, img_msg, img_vector, users_to_send_to_final)
            self.database_updated()
            # Send the image
            content = bytearray(img_msg.data)
            send_image_data = {
                'image' : base64.b64encode(content).decode('ascii'),
                'users' : users_to_send_to_final,
                # 'callback_url' : callback_url,
            }
            try:
            	res = requests.post(os.path.join(self.slackbot_url, 'send_image'), json=send_image_data)
                slackbot_msg_id = res.json()["message_id"]
                self.sent_messages_database.add_slackbot_message_id(local_msg_id, slackbot_msg_id)
                self.database_updated()
            except Exception as e:
                rospy.logwarn("Error communicating with Slackbot /send_image at URL %s." % self.slackbot_url)
                rospy.logwarn("Response text %s." % res.text)
                rospy.logwarn(traceback.format_exc())
                rospy.logwarn("Error %s." % e)

    def img_callback(self, img_msg):
        """
        If this image gets subsampled, run self.subsampled_image(img_msg) (in a
        separate thread to avoid dropping messages on the img stream). Else,
        return.
        """
        if not self.has_loaded: return
	rospy.logdebug("Got image!")
        if self.subsampling_policy.subsample(img_msg): # This image was selected
            thread = threading.Thread(
                target=self.subsampled_image,
                args=(img_msg,)
            )
            thread.start()
        else: # This image was not selected
            pass

    def get_slackbot_responses(self, refresh_secs=10.0):#30.0):#
        """
        Once every refresh_secs seconds, get the message_ids that haven't yet
        been responded to, request the Slackbot for responses, and update the
        sent_messages_database accordingly.
        """
        r = rospy.Rate(1.0/refresh_secs)
        while not rospy.is_shutdown():
            if not self.has_loaded: r.sleep()
            # Get the message_ids that haven't yet been reacted to
            message_ids_without_responses = self.sent_messages_database.get_slackbot_msg_ids_without_responses()
            try:
            	# Request responses for those message_ids
            	res = requests.post(
                	os.path.join(self.slackbot_url, 'get_responses'),
                	json={'message_ids_and_user_ids':message_ids_without_responses},
            	)
                message_id_to_user_reactions = res.json()["message_id_to_user_reactions"]

                updated_users = set()
                if len(message_id_to_user_reactions) > 0:
                    # Insert reactions into the database
                    for message_id in message_id_to_user_reactions:
                        for user, reaction in message_id_to_user_reactions[message_id]:
                            rospy.loginfo("Got reaction %d from user %s for message_id %s" % (reaction, user, message_id))
                            self.sent_messages_database.add_user_reaction(message_id, user, reaction)
                            updated_users.add(user)
                    self.database_updated()
                    for user in updated_users:
                        with self.to_send_policy_lock:
                            self.to_send_policy.got_reaction(user)
            except Exception as e:
                rospy.logwarn("Error communicating with Slackbot /get_responses at URL %s." % self.slackbot_url)
                rospy.logwarn("Response text %s." % res.text)
		rospy.logwarn(traceback.format_exc())
                rospy.logwarn("Error %s." % e)
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("cmm_demo")

    img_topic = rospy.get_param('~img_topic', '/upward_looking_camera/compressed')
    object_detection_srv = rospy.get_param('~object_detection_srv', 'object_detection')
    slackbot_url = rospy.get_param('~slackbot_url', 'http://ec2-34-222-57-139.us-west-2.compute.amazonaws.com:8194')
    send_messages_database_filepath = rospy.get_param('~send_messages_database_filepath', "/workspace/src/kuri_cmm_demo/kuri_cmm_demo/cfg/sent_messages_database.pkl")

    cmm_demo = CMMDemo(img_topic, object_detection_srv, slackbot_url, send_messages_database_filepath)

    rospy.spin()
