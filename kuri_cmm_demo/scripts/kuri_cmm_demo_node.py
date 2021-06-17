#!/usr/bin/env python
# ROS Libraries
import actionlib
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from kuri_cmm_demo.srv import ObjectDetection, ObjectDetectionResponse
from local_coverage_navigation.msg import NavigateAction, NavigateGoal
import rospy
from sensor_msgs.msg import CompressedImage, Image
# Python Default Libraries
import base64
import cv2 as cv
from enum import Enum
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
from view_tuner import ViewTuner

class CMMDemoState(Enum):
    """
    During NORMAL, the base moves according to local_coverage_navigation, and
    images are sent to the object detector, vectorized, and whether or not
    humans will like the image is computed. If at least one human will like the
    image, the state is changed to INITIALIZE_TUNER.

    During INITIALIZE_TUNER, navigation is stopped, and the image is used to
    initialize the tuner. Then, the state is changed to TUNE_IMAGE.

    During TUNE_IMAGE, navigation remains stopped, the images are sent to the
    view_tuner until the view_tuner indicates that it is done tuning the image
    composition. Then, the state is changed to TAKE_PICTURE.

    During TAKE_PICTURE, the robot waits some amount of time to ensure the
    picture is not blurry and then takes the picture. Then, the state is
    changed to NORMAL.
    """
    NORMAL = 1
    INITIALIZE_TUNER = 2
    TUNE_IMAGE = 3
    TAKE_PICTURE = 4

class CMMDemo(object):
    """
    The central executive node of our demo. This node subscribes to images from
    the robot, detects objects in the image, vectorizes it, determines whether
    to send it (and to whom), and sends it to the Slackbot. It has a separate
    thread that continually queries the Slackbot for new responses and updates
    the beliefs accordingly.
    """
    def __init__(self, img_topic, head_state_topic, object_detection_srv, slackbot_url,
        send_messages_database_filepath, visualize_view_tuner=False):
        """
        Initialize an instance of the CMMDemo class
        """
        self.has_loaded = False

        # Parameters relevant to subsampling
        self.subsampling_policy = SubsamplingPolicy(
            rule='n_per_sec', rule_config={'n':1},
        )
        # TODO: make this subscribe to madmux, controlled by a param
        self.img_sub = rospy.Subscriber(
            img_topic, CompressedImage, self.img_callback, queue_size=1)

        # Initialize the state.
        self.state_lock = threading.Lock()
        self.state_changed = True
        self.state = CMMDemoState.NORMAL
        self.view_tuner = ViewTuner(head_state_topic)
        self.visualize_view_tuner = visualize_view_tuner
        if visualize_view_tuner:
            self.img_pub_connected_components = rospy.Publisher('connected_components', Image, queue_size=1)
            self.img_pub_annotated = rospy.Publisher('img_annotated', Image, queue_size=1)
            self.bridge = CvBridge()

        # Parameters relevant to local navigation
        self.local_coverage_navigator_action = actionlib.SimpleActionClient('/navigate', NavigateAction)

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
            target=self.get_slackbot_updates,
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
    def is_similar(most_recent_stored_img_msgs, most_recent_stored_img_vectors,
        img_msg, img_vector, objects_threshold=8, image_threshold=0.75):
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
        for i in range(len(most_recent_stored_img_msgs)):
            most_recent_stored_img_msg = most_recent_stored_img_msgs[i]
            most_recent_stored_img_vector = most_recent_stored_img_vectors[i]

            # Pad the most_recent_stored_img_vector
            num_new_objects = img_vector.shape[0] - most_recent_stored_img_vector.shape[0]
            most_recent_stored_img_vector = np.pad(most_recent_stored_img_vector, (0, num_new_objects), 'constant', constant_values=0)

            # Get the change in probability for each object
            different_objects = np.abs(img_vector - most_recent_stored_img_vector)

            # Get the total chnge in probability
            change_in_prob = np.sum(different_objects)
            object_is_similar = change_in_prob <= objects_threshold

            # Get the similarity of the histograms
            most_recent_sent_img = cv.imdecode(np.fromstring(most_recent_stored_img_msg.data, np.uint8), cv.IMREAD_COLOR)
            img = cv.imdecode(np.fromstring(img_msg.data, np.uint8), cv.IMREAD_COLOR)
            avg_histogram_similarity = 0.0
            channels = [0,1,2]
            for channel in channels:
                most_recent_stored_img_hist = cv.calcHist([most_recent_sent_img],[channel],None,[256],[0,256])
                img_hist = cv.calcHist([img],[channel],None,[256],[0,256])
                avg_histogram_similarity += cv.compareHist(most_recent_stored_img_hist, img_hist, cv.HISTCMP_CORREL)/len(channels)
            image_is_similar = avg_histogram_similarity >= image_threshold

            rospy.loginfo("is_similar object_similarity %f <= %f?, image_similarity %f >= %f?" % (change_in_prob, objects_threshold, avg_histogram_similarity, image_threshold))
            if image_is_similar or object_is_similar:
                return True

        return False

    def img_msg_to_img_vector(self, img_msg):
        """
        Takes in an img_msg, calls the object_detection service to get the
        detected objects, and computes the img_vector from the
        detected_objects_msg, and returns that.
        """
        # Get the detected objects in the image
        rospy.wait_for_service(self.object_detection_srv_name)
        with self.object_detection_srv_lock:
            object_detection_response = self.object_detection_srv(img_msg)
        if not object_detection_response.success:
            rospy.logwarn("Object detection failed, not sending image")
            return None
        detected_objects_msg = object_detection_response.detected_objects

        # Determine whether to send the image
        with self.to_send_policy_lock:
            # Get the image vector
            img_vector = self.to_send_policy.vectorize(detected_objects_msg)
            return img_vector

    def send_images(self, user, n_images=5, n_objects=5):
        """
        Takes in an img_msg and sends it to the Slackbot
        """
        # Get the images the robot thinks the user is likely to like, and
        # compute the likleihood that the user will like them
        img_msgs, img_vectors, local_img_ids = self.sent_messages_database.get_stored_images_for_user(user)
        if len(img_msgs) > 0:
            probabilities = []
            for img_vector in img_vectors:
                num_new_objects = self.sent_messages_database.get_num_objects() - img_vector.shape[0]
                img_vector = np.pad(img_vector, (0, num_new_objects), 'constant', constant_values=0)
                probabilities.append(self.to_send_policy.get_probability(user, img_vector))

            # Determine the images to send.
            # TODO: make this not just take the 5 max, but also account for the
            # difference amongst the 5 images selected
            top_img_indices = np.argsort(probabilities)[-1:-n_images-1:-1]
            selected_images = []
            selected_images_debug_description = []
            selected_local_image_ids = []
            for i in top_img_indices:
                img_msg = img_msgs[i]
                local_img_id = local_img_ids[i]

                content = bytearray(img_msg.data)
                selected_images.append(base64.b64encode(content).decode('ascii'))
                selected_local_image_ids.append(local_img_id)

                probability = probabilities[i]
                debug_description = "Kuri thinks your likelihood of liking this image is %.02f. " % probability
                img_vector = img_vectors[i]
                top_objects = np.argsort(img_vector)[-1:-n_objects-1:-1]
                objects = self.sent_messages_database.get_objects()
                debug_description += "Kuri thinks the image has the following top-%d objects: " % n_objects
                for j in top_objects:
                    object_name = objects[j]
                    object_probability = img_vector[j]
                    debug_description += "%s (%.02f), " % (object_name, object_probability)
                debug_description = debug_description[:-2]
                selected_images_debug_description.append(debug_description)

            # Send the image
            send_image_data = {
                'images' : selected_images,
                'user' : user,
                'image_descriptions' : selected_images_debug_description,
                # 'callback_url' : callback_url,
            }
            try:
                res = requests.post(os.path.join(self.slackbot_url, 'send_images'), json=send_image_data)
                slackbot_img_ids = res.json()["image_ids"]
                self.sent_messages_database.add_slackbot_image_id(selected_local_image_ids, slackbot_img_ids, user)
                self.database_updated()
            except Exception as e:
                rospy.logwarn("Error communicating with Slackbot /send_images at URL %s." % self.slackbot_url)
                rospy.logwarn("Response text %s." % res.text)
                rospy.logwarn(traceback.format_exc())
                rospy.logwarn("Error %s." % e)
        else:
            rospy.logwarn("Tried to send images from user %s, but had no images stored" % user)

    def store_image(self, img_msg):
        """
        Stores an image in the database as a potential image that the users in
        self.users_to_send_to_final will like. When send_images is called for
        a user, the top n images from that set are sent.
        """
        img_vector = self.img_msg_to_img_vector(img_msg)
        # Get a new message ID
        local_img_id = self.sent_messages_database.get_new_local_img_id()

        rospy.loginfo("Store image! For users %s" % self.users_to_send_to_final)
        self.sent_messages_database.add_image(
            local_img_id, img_msg, img_vector, self.users_to_send_to_final)
        self.database_updated()

    def subsampled_image(self, img_msg):
        """
        Called when an image is subsampled. First, this function calls the
        object_detection service to return detected objects. Then, it determines
        whether to send the image. If it decides to send it, it calls the
        Slackbot's send_image endpoint

        TODO: if the state changes from NORMAL, return.
        """
        rospy.loginfo("Got subsampled image!")
        img_vector = self.img_msg_to_img_vector(img_msg)

        # Determine whether to send the image
        with self.to_send_policy_lock:
            to_send = self.to_send_policy.to_send_policy(img_vector)
        rospy.loginfo("to_send %s" % to_send)

        with self.state_lock:
            rospy.loginfo("with state_lock")
            if self.state == CMMDemoState.NORMAL and np.any(to_send):
                # Remove any users who got a sufficiently similar image recently
                users_to_send_to_initial = np.where(to_send)[0].tolist()
                self.users_to_send_to_final = []
                for user in users_to_send_to_initial:
                    most_recent_stored_img_msgs, most_recent_stored_img_vectors = self.sent_messages_database.get_most_recent_stored_and_sent_images(user)
                    if not CMMDemo.is_similar(most_recent_stored_img_msgs, most_recent_stored_img_vectors, img_msg, img_vector):
                        self.users_to_send_to_final.append(user)
                rospy.loginfo("most_recent_stored_img_vectors %s" % most_recent_stored_img_vectors)
                rospy.loginfo("                 img_vector %s" % img_vector)
                # Skip the image if there are no users to send it to
                if len(self.users_to_send_to_final) == 0:
                    return
                self.state = CMMDemoState.INITIALIZE_TUNER
                self.local_coverage_navigator_action.cancel_all_goals()
                rospy.loginfo("State: NORMAL ==> INITIALIZE_TUNER")


    def img_callback(self, img_msg):
        """
        If this image gets subsampled, run self.subsampled_image(img_msg) (in a
        separate thread to avoid dropping messages on the img stream). Else,
        return.

        TODO: See how Madmux does queuing, and if it doesn't then implement a
        separate thread that loops at a fixed ros Rate and gets the most recent
        madmux images
        """
        if not self.has_loaded: return
        rospy.logdebug("Got image!")
        with self.state_lock:
            state_at_start_of_loop = self.state
            if self.state == CMMDemoState.NORMAL:
                if self.state_changed or self.local_coverage_navigator_action.get_state() == GoalStatus.ABORTED:
                    rospy.loginfo("Waiting for local_coverage_navigator_action server")
                    self.local_coverage_navigator_action.wait_for_server()
                    rospy.loginfo("Sending goal to local_coverage_navigator_action")
                    self.local_coverage_navigator_action.send_goal(NavigateGoal())
                if self.subsampling_policy.subsample(img_msg): # This image was selected
                    thread = threading.Thread(
                        target=self.subsampled_image,
                        args=(img_msg,)
                    )
                    thread.start()
                else: # This image was not selected
                    pass
            elif self.state == CMMDemoState.INITIALIZE_TUNER:
                connected_components_cv2 = self.view_tuner.initialize_tuner(img_msg)
                if self.visualize_view_tuner:
                    connected_components_msg = self.bridge.cv2_to_imgmsg(connected_components_cv2, encoding="passthrough")
                    connected_components_msg.step = int(connected_components_msg.step)
                    self.img_pub_connected_components.publish(connected_components_msg)
                self.state = CMMDemoState.TUNE_IMAGE
                rospy.loginfo("State: INITIALIZE_TUNER ==> TUNE_IMAGE")
            elif self.state == CMMDemoState.TUNE_IMAGE:
                if self.visualize_view_tuner:
                    is_done, img_annotated = self.view_tuner.tune_image(img_msg, return_annotated_img=True)

                    if img_annotated is not None:
                        img_msg_annotated = self.bridge.cv2_to_imgmsg(img_annotated, encoding="passthrough")
                        img_msg_annotated.step = int(img_msg_annotated.step)
                        self.img_pub_annotated.publish(img_msg_annotated)
                else:
                    is_done = self.view_tuner.tune_image(img_msg)

                if is_done:
                    # Sleep to allow the image to not be blurry
                    rospy.sleep(2.0)
                    self.state = CMMDemoState.TAKE_PICTURE
                    rospy.loginfo("State: TUNE_IMAGE ==> TAKE_PICTURE")
            elif self.state == CMMDemoState.TAKE_PICTURE:
                self.store_image(img_msg)
                self.view_tuner.deinitialize_tuner()
                self.state = CMMDemoState.NORMAL
                rospy.loginfo("State: TAKE_PICTURE ==> NORMAL")
            state_at_end_of_loop = self.state
            self.state_changed = (state_at_start_of_loop != state_at_end_of_loop)


    def get_slackbot_updates(self, refresh_secs=10.0):#30.0):#
        """
        Once every refresh_secs seconds, get the image_ids that haven't yet
        been responded to, request the Slackbot for responses, and update the
        sent_messages_database accordingly.
        """
        r = rospy.Rate(1.0/refresh_secs)
        while not rospy.is_shutdown():
            if not self.has_loaded: r.sleep()
            # Get the image_ids that haven't yet been reacted to
            image_ids_without_responses = self.sent_messages_database.get_slackbot_img_ids_without_responses()
            try:
            	# Request responses for those image_ids
            	res = requests.post(
            		os.path.join(self.slackbot_url, 'get_updates'),
            		json={'image_ids_and_users':image_ids_without_responses},
            	)
                res_json = res.json()
                rospy.loginfo("Got updates from Slackbot %s" % res_json)
            	image_id_to_user_reactions = res_json["image_id_to_user_reactions"]

            	updated_users = set()
            	if len(image_id_to_user_reactions) > 0:
            	    # Insert reactions into the database
            	    for image_id in image_id_to_user_reactions:
            	        for user, reaction in image_id_to_user_reactions[image_id]:
            	            rospy.loginfo("Got reaction %d from user %s for image_id %s" % (reaction, user, image_id))
            	            self.sent_messages_database.add_user_reaction(image_id, user, reaction)
            	            updated_users.add(user)
            	    self.database_updated()
            	    for user in updated_users:
            	        with self.to_send_policy_lock:
            	            self.to_send_policy.got_reaction(user)

                time_to_send = res_json["time_to_send"]
                for user in time_to_send:
                    if time_to_send[user] <= refresh_secs:
                        # Send the top n images that user is most likely to like
                        self.send_images(int(user))
            except Exception as e:
            	rospy.logwarn("Error communicating with Slackbot /get_updates at URL %s." % self.slackbot_url)
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
    head_state_topic = rospy.get_param('~head_state_topic', '/head_controller/state')

    cmm_demo = CMMDemo(img_topic, head_state_topic, object_detection_srv, slackbot_url, send_messages_database_filepath, visualize_view_tuner=True)

    rospy.spin()
