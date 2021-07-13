#!/usr/bin/env python
import rospy
import actionlib

from sensor_msgs.msg import CompressedImage, Image
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
# from cv_bridge import CvBridge

import numpy as np
import cv2
import threading
import math
from scipy.ndimage.measurements import label
import time

class ViewTuner(object):
    """
    Uses an approach from https://www.researchgate.net/profile/Aaron-Steinfeld/publication/220939264_An_assisted_photography_method_for_street_scenes/links/02e7e51e569d3720f6000000/An-assisted-photography-method-for-street-scenes.pdf
    to adjust the robot head's pan/tilt to center the region of max saliency. When
    the view_tuner is first initialized with an image, it initializes optical flow
    and computes the center of the region of greatest saliency. For every subsequent
    image, it uses optical flow to deteming the new center of the region of max saliency,
    and moves the head to center that point. When it is deinitialized, it centers the
    head.
    """
    def __init__(self, head_state_topic, secs_of_stationary_before_returning=10,
        eps=0.05):
        """
        Initializes an instance of the ViewTuner class
        """
        # Initialize the image processing components
        self.saliency = cv2.saliency.StaticSaliencySpectralResidual_create()

        # Initialize the values that keep track of the desired center of saliency
        self.saliency_p0 = None
        self.saliency_img_msg = None
        self.lk_params = dict(winSize  = (15,15),
                         maxLevel = 2,
                         criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # If the head joint hasn't moved more than self.eps away from the joint
        # position, assume the motor is stalled or the head is stuck or something
        # and return done from tuning
        self.eps = eps
        self.secs_of_stationary_before_returning = secs_of_stationary_before_returning
        self.latest_pan_center = None
        self.latest_tilt_center = None
        self.latest_head_state_time = None

        # Initialize the head state topics
        self.current_pan = None
        self.current_tile = None
        self.head_state_lock = threading.Lock()
        self.head_state_sub = rospy.Subscriber(
            head_state_topic, JointTrajectoryControllerState, self.head_state_callback, queue_size=1)
        self.head_controller_action = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # self.head_controller_action.wait_for_server()
        self.head_tilt_max = 0.3
        self.head_tilt_min = -0.8
        self.head_tilt_speed = 0.2
        self.head_pan_max = 0.75
        self.head_pan_min = -0.75
        self.head_pan_speed = 0.2

    def initialize_tuner(self, img_msg):
        """
        Takes in an img_msg, computes the saliency map of the image, and
        computes the center of the most salient region of the image.

        Inputs:
        - img_msg is the ROS CompressedImage message

        Return:
        - connected_components_relative_entropy, a grayscale image that shows
          the connected components and their relative entropy.
        """
        # Convert to a cv2 image
        img_data = np.fromstring(img_msg.data, np.uint8)
        img_cv2 = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
        w = img_cv2.shape[1]
        h = img_cv2.shape[0]

        # Initialize connected_components_relative_entropy
        connected_components_relative_entropy = np.zeros((h,w), dtype=np.uint8)

        # Get the center of saliency (cx, cy) in the image
        (success, saliency_map) = self.saliency.computeSaliency(img_cv2)
        total_saliency = np.sum(saliency_map)
        if not success:
            rospy.logwarn("Failed to get saliency_map")
            return connected_components_relative_entropy

        # Turn it into a binary map
        binary_saliency_map = saliency_map >= total_saliency/w/h
        connected_components, n_components = label(binary_saliency_map, np.ones((3,3), dtype=np.int32))
        # For each connected components, compute its relative entropy
        max_relative_entropy, max_relative_entropy_comp_i = None, None
        for comp_i in range(1,n_components+1):
            # If saliency was uniformly distributed, what proportion of it
            # would fall into this connected component?
            region_prob = float(np.count_nonzero(connected_components == comp_i))/w/h
            # What proportion of the saliency actually falls into this connected
            # component?
            region_density = np.sum(saliency_map[np.where(connected_components == comp_i)])/total_saliency
            if region_density <= region_prob:
                relative_entropy = 0.0
            else:
                relative_entropy = (region_density*math.log(region_density/region_prob) +
                    (1-region_density)*math.log((1-region_density)/(1-region_prob)))

            connected_components_relative_entropy = np.where(connected_components == comp_i, int(round(relative_entropy*255)), connected_components_relative_entropy)

            if max_relative_entropy is None or relative_entropy > max_relative_entropy:
                max_relative_entropy = relative_entropy
                max_relative_entropy_comp_i = comp_i

        saliency_of_region_of_interest = np.where(connected_components == max_relative_entropy_comp_i, saliency_map, 0)
        weights = saliency_of_region_of_interest/np.sum(saliency_of_region_of_interest)
        cy = np.dot(np.arange(h), np.sum(weights, axis=1))
        cx = np.dot(np.arange(w), np.sum(weights, axis=0))
        self.saliency_p0 = np.array([[[cx, cy]]], dtype=np.float32)
        self.saliency_prev_img_cv2_gray = cv2.cvtColor(img_cv2, cv2.COLOR_BGR2GRAY)

        return connected_components_relative_entropy

    def tune_image(self, img_msg, threshold=0.1, duration_secs=0.2,
        return_annotated_img=False):
        """
        Takes in an img_msg, uses optical flow to determine the new location of
        the target point, and adjusts the head to center the point.

        Inputs:
        - img_msg is the ROS CompressedImage message

        Return:
        - a boolean indicating whether the point is centered or not.
        """
        img_annotated = None
        if self.saliency_p0 is None:
            rospy.logerror("You must call initialize_tuner before tune_image.")
            if return_annotated_img:
                return False, img_annotated
            return False

        # Convert to a cv2 image
        img_data = np.fromstring(img_msg.data, np.uint8)
        img_cv2_gray = cv2.imdecode(img_data, cv2.IMREAD_GRAYSCALE)
        w = img_cv2_gray.shape[1]
        h = img_cv2_gray.shape[0]

        # Determine the new location of the point
        p1, status, error = cv2.calcOpticalFlowPyrLK(self.saliency_prev_img_cv2_gray, img_cv2_gray, self.saliency_p0, None, **self.lk_params)
        # rospy.loginfo("before optical flow %s, after %s, status %s" % (self.saliency_p0, p1, status))
        if status[0] == 1:
            self.saliency_p0 = p1
            self.saliency_prev_img_cv2_gray = img_cv2_gray
            # Annotate the image
            if return_annotated_img:
                img_cv2 = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
                img_annotated = cv2.circle(img_cv2, (self.saliency_p0[0,0,0], self.saliency_p0[0,0,1]), 5, color=(0, 0, 255), thickness=-1)
                img_annotated = cv2.rectangle(img_annotated, (int(round((0.5-threshold)*w)), int(round((0.5-threshold)*h))),  (int(round((0.5+threshold)*w)), int(round((0.5+threshold)*h))), color=(255, 0, 0), thickness=2)
        else:
            if return_annotated_img:
                return True, img_annotated
            return True
        cx, cy = self.saliency_p0[0,0,:]

        # Compute the distance to the center
        dx_to_center = float(cx)/w-0.5
        dy_to_center = float(cy)/h-0.5
        if abs(dx_to_center) <= threshold and abs(dy_to_center) <= threshold:
            rospy.loginfo("Done tuning head position")
            self.head_controller_action.cancel_all_goals()
            if return_annotated_img:
                return True, img_annotated
            return True

        # Get the current head state
        with self.head_state_lock:
            current_pan = self.current_pan
            current_tilt = self.current_tilt
            latest_head_state_time = self.latest_head_state_time

        # If the head is not moving, return
        if (time.time() - latest_head_state_time) >= self.secs_of_stationary_before_returning:
            if return_annotated_img:
                return True, img_annotated
            return True

        # If the head is already at the most extreme positions in the direction
        # of the goal, return
        if (((current_pan >= self.head_pan_max-self.eps and dx_to_center<0) or
             (current_pan <= self.head_pan_min+self.eps and dx_to_center>=0)) and
            ((current_tilt >= self.head_tilt_max-self.eps and dy_to_center>0) or
             (current_tilt <= self.head_tilt_min+self.eps and dy_to_center<0))):
            # We are already at the most extreme head orientation, and can't move closer to the center of saliency.
            if return_annotated_img:
                return True, img_annotated
            return True

        # Compute the target endpoint and move there
        pan_endpoint = current_pan - dx_to_center
        tilt_endpoint = current_tilt + dy_to_center
        self.move_head(pan_endpoint, tilt_endpoint, duration_secs,
            current_pan=current_pan, current_tilt=current_tilt)

        if return_annotated_img:
            return False, img_annotated
        return False

    def move_head(self, pan_endpoint=0.0, tilt_endpoint=-0.3,
        duration_secs=None, n_waypoints=10, wait_for_result=True,
        current_pan=None, current_tilt=None):
        """
        Moves the head's pan and tile to pan_endpoint and tilt_endpoint,
        respectively. If duration_secs is None, it takes however long it takes
        to get there moving at <= self.head_pan_speed and self.head_tilt_speed.
        Else, it moves as close as it can get to the endpoint in duration_secs.
        n_waypoints is the number of waypoints in the trajectory. If
        wait_for_result is True, it blocks for up to duration_secs.

        If current_pan or current_tilt are None, it gets the most up-to-date
        values, else it uses the provided values.

        This function clips pan_endpoint and tilt_endpoint to stay within the
        necessary range.

        The default parameters will re-center the head.
        """
        # Get the current pan and tilt
        if current_pan is None:
            with self.head_state_lock:
                current_pan = self.current_pan
                current_tilt = self.current_tilt

        # Compute the actual endpoint and duration_secs
        pan_endpoint = max(min(pan_endpoint, self.head_pan_max), self.head_pan_min)
        tilt_endpoint = max(min(tilt_endpoint, self.head_tilt_max), self.head_tilt_min)
        time_to_move_to_endpoint = max(
            abs(pan_endpoint-current_pan)/self.head_pan_speed,
            abs(tilt_endpoint-current_tilt)/self.head_tilt_speed)
        if duration_secs is None: # compute the time required to reach the goal
            duration_secs = time_to_move_to_endpoint
        elif time_to_move_to_endpoint > duration_secs: # compute the actual endpoint given duration_secs
            pan_sign = -1.0 if pan_endpoint < current_pan else 1.0
            tilt_sign = -1.0 if tilt_endpoint < current_tilt else 1.0
            pan_endpoint = current_pan + pan_sign*duration_secs*self.head_pan_speed
            tilt_endpoint = current_tilt + tilt_sign*duration_secs*self.head_tilt_speed
        duration = rospy.Duration.from_sec(duration_secs)

        # Create the goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        goal.trajectory.points = []
        pan_interval = (pan_endpoint-current_pan)/(n_waypoints-1)
        tilt_interval = (tilt_endpoint-current_tilt)/(n_waypoints-1)
        time_interval = duration/n_waypoints
        for i in range(n_waypoints):
            point = JointTrajectoryPoint()
            point.positions = [current_pan + i*pan_interval, current_tilt + i*tilt_interval]
            point.velocities = []
            point.accelerations = []
            point.effort = []
            point.time_from_start = (i+1)*time_interval
            goal.trajectory.points.append(point)
        # rospy.loginfo("move_head goal %s" % goal)

        # Send the goal
        self.head_controller_action.wait_for_server()
        self.head_controller_action.send_goal(goal)
        self.head_controller_action.wait_for_result(duration)


    def deinitialize_tuner(self, center_head=True):
        """
        Deinitialize tuner.
        """
        self.saliency_p0 = None
        self.saliency_img_msg = None

        if center_head:
            rospy.loginfo("Centering Head")
            self.move_head()

    def is_initialized(self):
        """
        Returns a boolean indicating whether the tuner is initialized or not.
        """
        return self.saliency_p0 is not None

    def head_state_callback(self, head_state_msg):
        current_pan = head_state_msg.actual.positions[0]
        current_tilt = head_state_msg.actual.positions[1]
        # current_pan = head_state_msg.desired.positions[0]
        # current_tilt = head_state_msg.desired.positions[1]

        with self.head_state_lock:
            self.current_pan = current_pan
            self.current_tilt = current_tilt
            if self.latest_head_state_time is None:
                self.latest_pan_center = current_pan
                self.latest_tilt_center = current_tilt
                self.latest_head_state_time = time.time()
            elif abs(self.latest_pan_center - current_pan) > self.eps or abs(self.latest_tilt_center - current_tilt) > self.eps:
                self.latest_pan_center = current_pan
                self.latest_tilt_center = current_tilt
                self.latest_head_state_time = time.time()
