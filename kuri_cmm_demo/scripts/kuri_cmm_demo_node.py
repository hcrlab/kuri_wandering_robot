#!/usr/bin/env python
# ROS Libraries
import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from cv_bridge import CvBridge
from kuri_cmm_demo.srv import ObjectDetection, ObjectDetectionResponse
from kuri_cmm_demo.msg import Power
from local_coverage_navigation.msg import NavigateAction, NavigateGoal
import rospy
from sensor_msgs.msg import CompressedImage, Image
from trajectory_msgs.msg import JointTrajectoryPoint
# Python Default Libraries
import base64
import csv
import cv2
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
    CHARGING = 5

class CMMDemo(object):
    """
    The central executive node of our demo. This node subscribes to images from
    the robot, detects objects in the image, vectorizes it, determines whether
    to send it (and to whom), and sends it to the Slackbot. It has a separate
    thread that continually queries the Slackbot for new responses and updates
    the beliefs accordingly.
    """
    def __init__(self, img_topic, head_state_topic, object_detection_srv, slackbot_url,
        sent_messages_database_filepath, visualize_view_tuner=False,
        max_time_between_stored_images=10*60, human_prior_filepath=None,
        objects_filepath=None):
        """
        Initialize an instance of the CMMDemo class
        """
        self.has_loaded = False
        self.slackbot_url = slackbot_url

        # Load the user data from the Slackbot
        try:
            res = requests.get(os.path.join(slackbot_url, 'get_num_users'))
            self.n_users = int(res.json()["num_users"])
            user_to_learning_condition = res.json()["user_to_learning_condition"]
            self.user_to_learning_condition = {}
            for key in user_to_learning_condition:
                self.user_to_learning_condition[int(key)] = int(user_to_learning_condition[key])
            rospy.loginfo("%d users, %s conditions" % (self.n_users, self.user_to_learning_condition))
        except Exception as e:
            rospy.logerr("Error communicating with Slackbot /get_num_users at URL %s." % self.slackbot_url)
            if "res" in locals():
                rospy.logerr("Response text %s." % res.text)
            rospy.logerr(traceback.format_exc())
            rospy.logerr("Error %s." % e)
            rospy.logerr("Quitting rospy.")
            rospy.signal_shutdown("Cannot communicate with the slackbot at %s" % self.slackbot_url)
            return

        # Parameters relevant to subsampling
        self.subsampling_policy = SubsamplingPolicy(
            rule='n_per_sec', rule_config={'n':1},
        )

        # # Parameters relevant to the camera
        # self.use_madmux = use_madmux
        # if self.use_madmux:
        #     import madmux
        #     self.bridge = CvBridge()
        #     self.madmux_sub = madmux.Stream("/var/run/madmux/ch3.sock")
        #     self.madmux_sub.register_cb(self.img_callback)
        # else:
        self.img_sub = rospy.Subscriber(
            img_topic, CompressedImage, self.img_callback, queue_size=1)
        self.latest_image = None
        self.latest_image_lock = threading.Lock()
        self.has_new_image = False

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
        self.local_coverage_navigator_action = actionlib.SimpleActionClient('/local_coverage_navigator/navigate', NavigateAction)

        # Parameters relevant to opening eyes
        self.eyelid_controller_action = actionlib.SimpleActionClient('/eyelids_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.eye_closed_position = 0.41
        self.eye_open_position = 0.0

        # Each variable below contains the filename and the header row
        self.csv_dir = "/mayfield/data/kuri_cmm_demo/"
        self.stored_images_csv = ("stored_images.csv", ["Time", "Robot Image ID", "Robot User ID"])
        self.sent_images_csv = ("sent_images.csv", ["Time", "Robot Image ID", "Robot User ID", "Predicted Probability", "Slackbot Image ID"])
        for csv_filename, csv_header in [self.stored_images_csv, self.sent_images_csv]:
            csv_filepath = os.path.join(self.csv_dir, csv_filename)
            if not os.path.exists(csv_filepath):
                with open(csv_filepath, "w") as f:
                    csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    csv_writer.writerow(csv_header)

        # Parameters relevant to the battery
        self.battery_sub = rospy.Subscriber(
            "/mobile_base/power", Power, self.power_callback, queue_size=1)
        self.previous_battery_lock = threading.Lock()
        self.previous_battery = None
        self.previous_dock_present = None
        self.battery_notification_thresholds = [40, 30, 20, 15, 10, 5, 4, 3, 2, 1]
        self.to_charge_threshold = 50 # if the battery is less than this and you are docked, charge
        self.charging_done_threshold = 90 # if the batter is greater than this and you are charging, switch back to NORMAL

        # Parameters relevant to object detection
        self.object_detection_srv_name = object_detection_srv
        self.object_detection_srv_lock = threading.Lock()
        self.object_detection_srv = rospy.ServiceProxy(object_detection_srv, ObjectDetection)

        # Parameters relevant to storing images and message IDs
        self.objects_filepath = objects_filepath
        self.sent_messages_database_filepath = sent_messages_database_filepath
        self.sent_messages_database = SentMessagesDatabase.load(
            self.sent_messages_database_filepath, n_users=self.n_users,
            user_to_learning_condition=self.user_to_learning_condition,
            objects_filepath=self.objects_filepath)
        self.database_save_interval = 1
        self.database_updates_since_last_save = 0

        # Parameters relevant to determine whether to send the image
        self.human_prior_filepath = human_prior_filepath
        self.max_time_between_stored_images = max_time_between_stored_images
        self.to_send_policy = ToSendPolicy(self.sent_messages_database,
            n_users=self.n_users,
            user_to_learning_condition=self.user_to_learning_condition,
            human_prior_filepath=self.human_prior_filepath)
        self.to_send_policy_lock = threading.Lock()

        # Parameters relevant to communicating with the Slackbot
        self.slackbot_responses_thread = threading.Thread(
            target=self.get_slackbot_updates,
        )
        self.slackbot_responses_thread.start()

        # Parameters relevant to the state machine
        self.state_machine_thread = threading.Thread(
            target=self.state_machine_control_loop,
        )
        self.state_machine_thread.start()

        self.has_loaded = True

    def database_updated(self):
        """
        Called everytime the database is updated. Saves the database every
        self.database_save_interval updates
        """
        self.database_updates_since_last_save += 1
        if self.database_updates_since_last_save % self.database_save_interval == 0:
            self.sent_messages_database.save(self.sent_messages_database_filepath)
            rospy.logdebug("Saved sent_messages_database!")

    @staticmethod
    def is_similar(most_recent_stored_img_cv2s, most_recent_stored_img_vectors,
        img_msg, img_vector, objects_threshold=4.0, image_threshold=0.75, return_similarity=False):
        """
        Returns True if img is sufficiently similar to most_recent_img to not
        sent it, False otherwise. Images are deemed to be similar if there
        are at least objects_threshold objects that each had confidence
        >= object_confidence_threshold in one image but not the other.

        TODO: There is stochasticity in AWS Rekognition, and given two images
        that are visually the same, it may detect different objects. Therefore,
        an improvement to this would be also looking at the histogram distribution
        (either grayscale or separately for RGB) and deeming an image "similar"
        if either its histogram is too similar or its detected objects are too
        similar. The reason we don't do only-histogram is because of changing
        lighting, but the hybrid approach should work well.
        """
        is_similar = False
        object_similarities = []
        image_similarities = []
        for i in range(len(most_recent_stored_img_cv2s)):
            most_recent_stored_img = most_recent_stored_img_cv2s[i]
            most_recent_stored_img_vector = most_recent_stored_img_vectors[i]

            # Pad the most_recent_stored_img_vector
            num_new_objects = img_vector.shape[0] - most_recent_stored_img_vector.shape[0]
            most_recent_stored_img_vector = np.pad(most_recent_stored_img_vector, (0, num_new_objects), 'constant', constant_values=0)

            # Get the change in probability for each object
            different_objects = np.abs(img_vector - most_recent_stored_img_vector)

            # Get the total chnge in probability
            change_in_prob = np.sum(different_objects)
            object_is_similar = change_in_prob <= objects_threshold
            object_similarities.append(change_in_prob)

            # Get the similarity of the histograms
            img = cv2.imdecode(np.fromstring(img_msg.data, np.uint8), cv2.IMREAD_COLOR)
            avg_histogram_similarity = 0.0
            channels = [0,1,2]
            for channel in channels:
                most_recent_stored_img_hist = cv2.calcHist([most_recent_stored_img],[channel],None,[256],[0,256])
                img_hist = cv2.calcHist([img],[channel],None,[256],[0,256])
                avg_histogram_similarity += cv2.compareHist(most_recent_stored_img_hist, img_hist, cv2.HISTCMP_CORREL)/len(channels)
            image_is_similar = avg_histogram_similarity >= image_threshold
            image_similarities.append(avg_histogram_similarity)

            rospy.logdebug("is_similar object_similarity %f <= %f?, image_similarity %f >= %f?" % (change_in_prob, objects_threshold, avg_histogram_similarity, image_threshold))
            if image_is_similar or object_is_similar:
                is_similar = True
                if not return_similarity: return True

        if return_similarity:
            return False, object_similarities, image_similarities
        return False

    def img_msg_to_img_vector(self, img_msg, n_tries=3):
        """
        Takes in an img_msg, calls the object_detection service to get the
        detected objects, and computes the img_vector from the
        detected_objects_msg, and returns that.
        """
        # Get the detected objects in the image
        success = False
        for try_i in range(n_tries):
            try:
                rospy.wait_for_service(self.object_detection_srv_name, timeout=5)
                with self.object_detection_srv_lock:
                    object_detection_response = self.object_detection_srv(img_msg)
                if object_detection_response.success:
                    success = True
                    break
            except Exception as e:
                rospy.logwarn("Failed to contact object_detection_srv, try %d/%d" % (try_i+1, n_tries))
                if "object_detection_response" in locals():
                    rospy.logwarn("object_detection_response %s." % object_detection_response)
                rospy.logwarn(traceback.format_exc())
                rospy.logwarn("Error %s." % e)

        if not success:
            rospy.logwarn("Failed to detect objects. Returning from img_msg_to_img_vector")
            return None, None

        detected_objects_msg = object_detection_response.detected_objects

        # Determine whether to send the image
        # with self.to_send_policy_lock:

        # Get the image vector
        img_vector = self.to_send_policy.vectorize(detected_objects_msg)
        return img_vector, detected_objects_msg

    def send_images(self, user, n_images=5, n_objects=5, debug=False):
        """
        Sends the top 5 stored images for the user to the Slackbot
        """
        # Get the images the robot thinks the user is likely to like, and
        # compute the likleihood that the user will like them
        img_vectors, local_img_ids = self.sent_messages_database.get_stored_images_for_user(user, return_img_cv2=False, return_img_vector=True, return_detected_objects_dict=False)
        if len(img_vectors) > 0:
            probabilities = []
            for img_vector in img_vectors:
                with self.to_send_policy_lock:
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
            selected_image_probabilities = []
            top_object_names = []
            for i in top_img_indices:
                local_img_id = local_img_ids[i]
                img_cv2 = self.sent_messages_database.get_image(local_img_id, return_img_cv2=True, return_img_vector=False, return_detected_objects_dict=False)[0]

                content = bytearray(np.array(cv2.imencode('.jpg', img_cv2)[1]).tostring())
                selected_images.append(base64.b64encode(content).decode('ascii'))
                selected_local_image_ids.append(local_img_id)
                probability = probabilities[i]
                selected_image_probabilities.append(probability)

                img_vector = img_vectors[i]
                top_objects = np.argsort(img_vector)[-1:-n_objects-1:-1]
                objects = self.sent_messages_database.get_objects()
                top_object_names.append([])
                for j in top_objects:
                    object_name = objects[j]
                    top_object_names[-1].append(object_name)

                if debug:
                    debug_description = "Kuri thinks your likelihood of liking this image is %.02f. " % probability
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
                'objects' : top_object_names,
                # 'callback_url' : callback_url,
            }
            try:
                res = requests.post(os.path.join(self.slackbot_url, 'send_images'), json=send_image_data)
                slackbot_img_ids = res.json()["image_ids"]
                rospy.loginfo("Got image_ids %s" % slackbot_img_ids)
                self.sent_messages_database.add_slackbot_image_id(selected_local_image_ids, slackbot_img_ids, user)
                self.database_updated()

                # Save it in the CSV
                csv_filepath = os.path.join(self.csv_dir, self.sent_images_csv[0])
                with open(csv_filepath, "a") as f:
                    csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    for i in range(len(slackbot_img_ids)):
                        try:
                            slackbot_img_id = slackbot_img_ids[i]
                            local_img_id = selected_local_image_ids[i]
                            predicted_probability = selected_image_probabilities[i]
                            csv_writer.writerow([time.time(), local_img_id, user, predicted_probability, slackbot_img_id])
                        except Exception as e:
                            rospy.loginfo("Error writing to sent_images_csv %s" % e)

            except Exception as e:
                rospy.logwarn("Error communicating with Slackbot /send_images at URL %s." % self.slackbot_url)
                if "res" in locals():
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
        with self.to_send_policy_lock:
            img_vector, detected_objects_msg = self.img_msg_to_img_vector(img_msg)
        if img_vector is None:
            rospy.logwarn("Failed to get img_vector. Returning from store_image")
            return

        rospy.loginfo("Store image! For users %s" % self.users_to_send_to_final)
        img_cv2 = cv2.imdecode(np.fromstring(img_msg.data, np.uint8), cv2.IMREAD_COLOR)
        local_img_id = self.sent_messages_database.get_new_local_img_id(img_cv2)
        self.sent_messages_database.add_image(
            local_img_id, img_cv2, img_vector, detected_objects_msg, self.users_to_send_to_final)
        self.database_updated()

        # Save it in the CSV
        csv_filepath = os.path.join(self.csv_dir, self.stored_images_csv[0])
        with open(csv_filepath, "a") as f:
            csv_writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            for user in self.users_to_send_to_final:
                csv_writer.writerow([time.time(), local_img_id, user])

    def subsampled_image(self, img_msg):
        """
        Called when an image is subsampled. First, this function calls the
        object_detection service to return detected objects. Then, it determines
        whether to send the image. If it decides to send it, it calls the
        Slackbot's send_image endpoint

        TODO: if the state changes from NORMAL, return.
        """
        rospy.logdebug("Got subsampled image!")
        with self.to_send_policy_lock:
            img_vector, detected_objects_msg = self.img_msg_to_img_vector(img_msg)
            if img_vector is None:
                rospy.logwarn("Failed to get img_vector. Returning from subsampled_image")
                return
            # Determine whether to send the image
            to_send = self.to_send_policy.to_send_policy(img_vector)
            rospy.loginfo("to_send_policy output %s" % to_send)

        # If we haven't sent an image to the user in more than self.max_time_between_stored_images
        # seconds, send it.
        users_who_havent_had_an_image_recently = []
        curr_time = time.time()
        for user in range(self.n_users):
            time_last_sent = self.sent_messages_database.get_time_of_last_stored_image(user)
            rospy.logdebug("user %d time_last_sent %f" % (user, time_last_sent))
            if curr_time - time_last_sent >= self.max_time_between_stored_images:
                to_send[user] = True
                users_who_havent_had_an_image_recently.append(user)
        rospy.loginfo("users added due to not having an image sent recently %s" % (users_who_havent_had_an_image_recently))

        rospy.logdebug("with state_lock")
        if self.state == CMMDemoState.NORMAL and np.any(to_send):
            # Remove any users who got a sufficiently similar image recently
            users_to_send_to_initial = np.where(to_send)[0].tolist()
            users_to_not_send_to = []
            self.users_to_send_to_final = []
            rospy.logdebug("img_vector %s" % (img_vector))
            for user in users_to_send_to_initial:
                most_recent_stored_img_cv2s, most_recent_stored_img_vectors = self.sent_messages_database.get_most_recent_stored_and_sent_images(user)
                rospy.logdebug("user %d most_recent_stored_img_cv2s %s, most_recent_stored_img_vectors %s" % (user, most_recent_stored_img_cv2s, most_recent_stored_img_vectors))
                is_similar = CMMDemo.is_similar(most_recent_stored_img_cv2s, most_recent_stored_img_vectors, img_msg, img_vector, return_similarity=False)
                # is_similar, object_similarities, image_similarities = CMMDemo.is_similar(most_recent_stored_img_cv2s, most_recent_stored_img_vectors, img_msg, img_vector, return_similarity=True)
                # rospy.loginfo("user %d object_similarities %s, image_similarities %s" % (user, object_similarities, image_similarities))
                if user in users_who_havent_had_an_image_recently or not is_similar:
                    self.users_to_send_to_final.append(user)
                else:
                    users_to_not_send_to.append(user)
            rospy.loginfo("Not storing it for users %s due to similarity" % users_to_not_send_to)
            rospy.logdebug("most_recent_stored_img_vectors %s" % most_recent_stored_img_vectors)
            rospy.logdebug("                 img_vector %s" % img_vector)
            # Skip the image if there are no users to send it to
            if len(self.users_to_send_to_final) == 0:
                return
            self.state = CMMDemoState.INITIALIZE_TUNER
            self.local_coverage_navigator_action.cancel_all_goals()
            rospy.loginfo("State: NORMAL ==> INITIALIZE_TUNER")


    def open_eyes(self, duration_secs=0.2):
        """
        Open the robot's eyes
        """
        duration = rospy.Duration.from_sec(duration_secs)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.joint_names = ["eyelids_joint"]
        point = JointTrajectoryPoint()
        point.positions = [self.eye_open_position]
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start = duration
        goal.trajectory.points = [point]
        # rospy.loginfo("move_head goal %s" % goal)

        # Send the goal
        self.eyelid_controller_action.wait_for_server()
        self.eyelid_controller_action.send_goal(goal)
        self.eyelid_controller_action.wait_for_result(duration)

    def close_eyes(self, duration_secs=0.2):
        """
        Open the robot's eyes
        """
        duration = rospy.Duration.from_sec(duration_secs)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.joint_names = ["eyelids_joint"]
        point = JointTrajectoryPoint()
        point.positions = [self.eye_closed_position]
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start = duration
        goal.trajectory.points = [point]
        # rospy.loginfo("move_head goal %s" % goal)

        # Send the goal
        self.eyelid_controller_action.wait_for_server()
        self.eyelid_controller_action.send_goal(goal)
        self.eyelid_controller_action.wait_for_result(duration)

    def img_callback(self, data):
        """
        If this image gets subsampled, run self.subsampled_image(img_msg) (in a
        separate thread to avoid dropping messages on the img stream). Else,
        return.

        TODO: See how Madmux does queuing, and if it doesn't then implement a
        separate thread that loops at a fixed ros Rate and gets the most recent
        madmux images
        """
        if not self.has_loaded: return

        # if self.use_madmux:
        #     # Read the bytes as a jpeg image
        #     data = np.fromstring(data, np.uint8)
        #     decoded = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_COLOR)
        #     img_msg = self.bridge.cv2_to_compressed_imgmsg(decoded)
        # else:
        img_msg = data

        with self.latest_image_lock:
            self.latest_image = img_msg
            self.has_new_image = True

    def state_machine_control_loop(self, rate_hz=2):
        """
        rate is in Hz
        """
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            rate.sleep()
            with self.state_lock:
                state_at_start_of_loop = self.state
                if self.state == CMMDemoState.NORMAL:
                    goal_state = self.local_coverage_navigator_action.get_state()
                    if self.state_changed or goal_state == GoalStatus.ABORTED or goal_state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Waiting for local_coverage_navigator_action server")
                        self.local_coverage_navigator_action.wait_for_server()
                        rospy.loginfo("Sending goal to local_coverage_navigator_action")
                        self.local_coverage_navigator_action.send_goal(NavigateGoal(effort=-1))
                        self.view_tuner.move_head() # Center the view_tuner head
                        self.open_eyes()
                    with self.previous_battery_lock:
                        if self.previous_battery is not None and self.previous_battery < self.to_charge_threshold and self.previous_dock_present:
                            self.close_eyes()
                            self.state = CMMDemoState.CHARGING
                            self.local_coverage_navigator_action.cancel_all_goals()
                            rospy.loginfo("State: NORMAL ==> CHARGING")
                        else:
                            with self.latest_image_lock:
                                if self.has_new_image:
                                    img_msg = self.latest_image
                                    self.has_new_image = False
                                else:
                                    continue
                            if self.subsampling_policy.subsample(img_msg): # This image was selected
                                self.subsampled_image(img_msg)
                                # thread = threading.Thread(
                                #     target=self.subsampled_image,
                                #     args=(img_msg,)
                                # )
                                # thread.start()
                            else: # This image was not selected
                                pass
                elif self.state == CMMDemoState.INITIALIZE_TUNER:
                    with self.latest_image_lock:
                        if self.has_new_image:
                            img_msg = self.latest_image
                            self.has_new_image = False
                        else:
                            continue
                    connected_components_cv2 = self.view_tuner.initialize_tuner(img_msg)
                    if self.visualize_view_tuner:
                        connected_components_msg = self.bridge.cv2_to_imgmsg(connected_components_cv2, encoding="passthrough")
                        connected_components_msg.step = int(connected_components_msg.step)
                        self.img_pub_connected_components.publish(connected_components_msg)
                    self.state = CMMDemoState.TUNE_IMAGE
                    rospy.loginfo("State: INITIALIZE_TUNER ==> TUNE_IMAGE")
                elif self.state == CMMDemoState.TUNE_IMAGE:
                    with self.latest_image_lock:
                        if self.has_new_image:
                            img_msg = self.latest_image
                            self.has_new_image = False
                        else:
                            continue
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
                    with self.latest_image_lock:
                        if self.has_new_image:
                            img_msg = self.latest_image
                            self.has_new_image = False
                        else:
                            continue
                    self.store_image(img_msg)
                    self.view_tuner.deinitialize_tuner()
                    self.state = CMMDemoState.NORMAL
                    rospy.loginfo("State: TAKE_PICTURE ==> NORMAL")
                elif self.state == CMMDemoState.CHARGING:
                    with self.previous_battery_lock:
                        if self.previous_battery is None or not self.previous_dock_present or  self.previous_battery >= self.charging_done_threshold:
                            self.state = CMMDemoState.NORMAL
                            rospy.loginfo("State: CHARGING ==> NORMAL")
                state_at_end_of_loop = self.state
                self.state_changed = (state_at_start_of_loop != state_at_end_of_loop)

    def power_callback(self, msg):
        """
        Callback function for Kuri's power update
        """
        if not self.has_loaded: return
        with self.state_lock:
            with self.previous_battery_lock:
                self.previous_dock_present = msg.dock_present
                if self.state == CMMDemoState.CHARGING:
                    self.previous_battery = msg.battery.pct
                else:
                    update_previous_battery = True
                    if msg.battery.pct <= self.battery_notification_thresholds[0]:
                        for i in range(len(self.battery_notification_thresholds)):
                            if (self.previous_battery is None or self.previous_battery > self.battery_notification_thresholds[i]) and msg.battery.pct <= self.battery_notification_thresholds[i]:
                                try:
                                    # Request responses for those image_ids
                                    rospy.loginfo("Sent battery request for pct %s" % msg.battery.pct)
                                    res = requests.post(
                                        os.path.join(self.slackbot_url, 'low_battery_alert'),
                                        json={'battery_pct':msg.battery.pct},
                                    )
                                    res_json = res.json()
                                    if not res_json['success']:
                                        update_previous_battery = False
                                except Exception as e:
                                    rospy.logwarn("Error communicating with Slackbot /low_battery_alert at URL %s." % self.slackbot_url)
                                    if "res" in locals():
                                        rospy.logwarn("Response text %s." % res.text)
                                    rospy.logwarn(traceback.format_exc())
                                    rospy.logwarn("Error %s." % e)
                                    update_previous_battery = False
                                break
                    if update_previous_battery and (self.previous_battery is None or msg.battery.pct < self.previous_battery):
                        self.previous_battery = msg.battery.pct

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
            	if "res" in locals():
                	rospy.logwarn("Response text %s." % res.text)
            	rospy.logwarn(traceback.format_exc())
            	rospy.logwarn("Error %s." % e)
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("cmm_demo")

    img_topic = rospy.get_param('~img_topic', '/upward_looking_camera/compressed')
    object_detection_srv = rospy.get_param('~object_detection_srv', 'object_detection')
    slackbot_url = rospy.get_param('~slackbot_url', 'http://ec2-52-33-153-87.us-west-2.compute.amazonaws.com:8194')
    sent_messages_database_filepath = rospy.get_param('~sent_messages_database_filepath', "/workspace/src/kuri_cmm_demo/kuri_cmm_demo/cfg/sent_messages_database.pkl")
    human_prior_filepath = rospy.get_param('~human_prior_filepath', "/workspace/src/kuri_cmm_demo/kuri_cmm_demo/cfg/prior.npz")
    objects_filepath = rospy.get_param('~objects_filepath', "/workspace/src/kuri_cmm_demo/kuri_cmm_demo/cfg/objects.json")
    head_state_topic = rospy.get_param('~head_state_topic', '/head_controller/state')

    cmm_demo = CMMDemo(img_topic, head_state_topic, object_detection_srv,
        slackbot_url, sent_messages_database_filepath, visualize_view_tuner=False,
        human_prior_filepath=human_prior_filepath, objects_filepath=objects_filepath)

    rospy.spin()
