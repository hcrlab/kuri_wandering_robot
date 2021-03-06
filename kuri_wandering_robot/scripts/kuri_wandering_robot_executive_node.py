#!/usr/bin/env python
# ROS Libraries
import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import JointTrajectoryControllerState, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from kuri_wandering_robot.msg import Power
from wandering_behavior.msg import WanderAction, WanderGoal
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectoryPoint
# Python Default Libraries
import base64
import csv
from enum import Enum
import os
import requests
import threading
import time
import traceback
# Custom Libraries
from sent_messages_database import SentMessagesDatabase

class KuriWanderingRobotState(Enum):
    """
    During NORMAL, the base moves according to wandering_behavior.
    During CHARGING, the robot's eyes are closed and it is charging. The robot
    transitions from NORMAL to CHARGING if its battery is below a threshold and
    it is on the charger. It transitions from CHARGING to NORMAL if it's battery
    is above a threshold or it is off the charger.
    """
    NORMAL = 1
    CHARGING = 2

class KuriWanderingRobot(object):
    """
    The central executive node. This node runs a control loop that manages the
    robot's state: turning on and monitoring progress of the wandering module
    in NORMAL, turning off wandering in CHARGING, and switching back to NORMAL
    when the robot is sufficiently charged.

    This node also runs anomaly detection to detect low battery; when it detects
    low battery, it sends a low battery request to the Slackbot, which then
    sends it to the helpers. This node can be extended with additional anomaly
    detection and help requests, as needed. This node also subscribes to a dummy
    `where_am_i_help` topic, which sends helpers the sample `where_am_i` help
    message. Note that that is only in place to illsutrate the sample
    `where_am_i` help message, and actually using that would require developing
    a custom anomaly detection system to trigger the robot asking for that type
    of help.

    Finally, this node has a separate thread that continually queries the
    Slackbot for responses to its help requests.
    """
    def __init__(self):
        """
        Initialize an instance of the KuriWanderingRobot class
        """
        self.has_loaded = False

        # Get the Slackbot URL
        self.slackbot_url = rospy.get_param('~slackbot_url')

        # Initialize the state.
        self.state_lock = threading.Lock()
        self.state_changed = True
        self.state = KuriWanderingRobotState.NORMAL

        # Initialize the wandering module
        self.wandering_module_action = actionlib.SimpleActionClient('/wandering_behavior/navigate', WanderAction)

        # Initialize the eye controller
        self.eyelid_controller_action = actionlib.SimpleActionClient('/eyelids_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.eye_closed_position = 0.41
        self.eye_open_position = 0.0

        # Initialize the camera
        self.img_sub = rospy.Subscriber(
            '/upward_looking_camera/compressed', CompressedImage, self.image_callback, queue_size=1)
        self.latest_image = None
        self.latest_image_lock = threading.Lock()

        # Initialize low battery anomaly detector
        self.battery_sub = rospy.Subscriber(
            "/mobile_base/power", Power, self.power_callback, queue_size=1)
        self.previous_battery_lock = threading.Lock()
        self.previous_battery = None
        self.previous_dock_present = None
        self.battery_notification_thresholds = rospy.get_param('~battery_notification_thresholds', [40, 20, 10, 5, 4, 3, 2, 1])
        # if the battery is less than this and Kuri is docked, charge
        self.to_charge_threshold = rospy.get_param('~to_charge_threshold', 50)
        # if the batter is greater than this and Kuri is charging, switch back to NORMAL
        self.charging_done_threshold = rospy.get_param('~charging_done_threshold', 90)
        # Whether the low battery message should include Kuri's current camera image
        self.low_battery_message_include_image = rospy.get_param('~low_battery_message_include_image', True)

        # Initialize the dummy `where_am_i` anomaly detector
        self.where_am_i_help_sub = rospy.Subscriber(
            "/where_am_i_help", Empty, self.where_am_i_help_callback, queue_size=1)

        # Initialize storing images and message IDs
        self.sent_messages_database_filepath = rospy.get_param('~send_messages_database_filepath')
        self.sent_messages_database = SentMessagesDatabase.load(
            self.sent_messages_database_filepath)
        self.database_save_interval = 1
        self.database_updates_since_last_save = 0

        # Initialize the head controller
        self.head_state_sub = rospy.Subscriber(
            "/head_controller/state", JointTrajectoryControllerState, self.head_state_callback, queue_size=1)
        self.head_controller_action = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.head_tilt_speed = 0.2 # head tilt is in [-0.8, 0.3]
        self.head_pan_speed = 0.2 # head pan is in [-0.75, 0.75]

        # Initialize the Slackbot updates thread
        self.slackbot_responses_thread = threading.Thread(
            target=self.get_slackbot_updates,
        )
        self.slackbot_responses_thread.start()

        # Initialize the state machine
        self.state_machine_thread = threading.Thread(
            target=self.state_machine_control_loop,
        )
        self.state_machine_thread.start()

        self.has_centered_head = False
        self.has_loaded = True

    def database_updated(self, num_updates=1):
        """
        Called everytime the database is updated. Saves the database every
        self.database_save_interval updates
        """
        self.database_updates_since_last_save += num_updates
        if self.database_updates_since_last_save % self.database_save_interval == 0:
            self.sent_messages_database.save(self.sent_messages_database_filepath)
            rospy.logdebug("Saved sent_messages_database!")

    def open_eyes(self, duration_secs=0.2):
        """
        Open the robot's eyes
        """
        rospy.logdebug("Open Eyes")
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

        # Send the goal
        self.eyelid_controller_action.wait_for_server()
        self.eyelid_controller_action.send_goal(goal)
        self.eyelid_controller_action.wait_for_result(duration)

    def close_eyes(self, duration_secs=0.2):
        """
        Close the robot's eyes
        """
        rospy.logdebug("Close Eyes")
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

        # Send the goal
        self.eyelid_controller_action.wait_for_server()
        self.eyelid_controller_action.send_goal(goal)
        self.eyelid_controller_action.wait_for_result(duration)

    def head_state_callback(self, head_state_msg):
        """
        Get the head's current position
        """
        if not self.has_loaded:
            return

        if not self.has_centered_head:
            self.center_head(head_state_msg.actual.positions[0], head_state_msg.actual.positions[1])

    def center_head(self, current_pan, current_tilt):
        """
        Center Kuri's head. This involves moving from the current pan and tilt
        to the centered values of (0.0, -0.3)
        """
        pan_endpoint = 0.0
        tilt_endpoint = -0.3
        n_waypoints = 10

        # Compute the actual endpoint and duration_secs
        duration_secs = max(
            abs(pan_endpoint-current_pan)/self.head_pan_speed,
            abs(tilt_endpoint-current_tilt)/self.head_tilt_speed)
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

        # Send the goal
        self.head_controller_action.wait_for_server()
        self.head_controller_action.send_goal(goal)
        self.head_controller_action.wait_for_result(duration)

        self.has_centered_head = True


    def state_machine_control_loop(self, rate_hz=10):
        """
        The control loop for the state machine. All of the state machine logic
        is handled in this function and the functions it calls.

        During NORMAL, the base moves according to wandering_behavior.
        During CHARGING, the robot's eyes are closed and it is charging. The
        robot transitions from NORMAL to CHARGING if its battery is below a
        threshold and it is on the charger. It transitions from CHARGING to
        NORMAL if it's battery is above a threshold or it is off the charger.
        """
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            rate.sleep()
            with self.state_lock:
                state_at_start_of_loop = self.state
                if (self.state == KuriWanderingRobotState.NORMAL):
                    goal_state = self.wandering_module_action.get_state()
                    if (self.state_changed or goal_state == GoalStatus.ABORTED or goal_state == GoalStatus.SUCCEEDED):
                        rospy.logdebug("Waiting for wandering_module_action server")
                        self.wandering_module_action.wait_for_server()
                        rospy.logdebug("Sending goal to wandering_module_action")
                        # Effort -1 means "don't stop unless preempted"
                        self.wandering_module_action.send_goal(WanderGoal(effort=-1))
                        self.open_eyes()
                    with self.previous_battery_lock:
                        if (self.previous_battery is not None and self.previous_battery < self.to_charge_threshold and self.previous_dock_present):
                            self.close_eyes()
                            self.state = KuriWanderingRobotState.CHARGING
                            self.wandering_module_action.cancel_all_goals()
                            rospy.loginfo("State: NORMAL ==> CHARGING")
                elif self.state == KuriWanderingRobotState.CHARGING:
                    with self.previous_battery_lock:
                        if (self.previous_battery is None or not self.previous_dock_present or self.previous_battery >= self.charging_done_threshold):
                            self.state = KuriWanderingRobotState.NORMAL
                            rospy.loginfo("State: CHARGING ==> NORMAL")
                state_at_end_of_loop = self.state
                self.state_changed = (state_at_start_of_loop != state_at_end_of_loop)

    def image_callback(self, img_msg):
        """
        Store the latest image.
        """
        if not self.has_loaded: return

        with self.latest_image_lock:
            self.latest_image = img_msg

    def power_callback(self, msg):
        """
        Callback function for Kuri's power update. It Kuri's battery has crossed
        a battery_notification_threshold, notify the Slackbot.
        """
        if not self.has_loaded: return
        with self.state_lock:
            with self.previous_battery_lock:
                self.previous_dock_present = msg.dock_present
                if self.state == KuriWanderingRobotState.CHARGING:
                    self.previous_battery = msg.battery.pct
                else:
                    update_previous_battery = True
                    if msg.battery.pct <= self.battery_notification_thresholds[0]:
                        # Send the low-battery helper notifications when the battery
                        # crosses the thresholds defined in self.battery_notification_thresholds
                        for i in range(len(self.battery_notification_thresholds)):
                            if (self.previous_battery is None or (self.previous_battery > self.battery_notification_thresholds[i]) and msg.battery.pct <= self.battery_notification_thresholds[i]):
                                try:
                                    # Send a low_battery_alert
                                    dict_to_send = {'battery_pct':msg.battery.pct}
                                    if self.low_battery_message_include_image:
                                        with self.latest_image_lock:
                                            if self.latest_image is not None:
                                                image_contents = base64.b64encode(bytearray(self.latest_image.data)).decode('ascii')
                                                dict_to_send['image'] = image_contents
                                    rospy.loginfo("Sending battery request for pct %s" % msg.battery.pct)
                                    res = requests.post(
                                        os.path.join(self.slackbot_url, 'low_battery'),
                                        json=dict_to_send,
                                    )
                                    res_json = res.json()
                                    if not res_json['success']:
                                        update_previous_battery = False
                                except Exception as e:
                                    rospy.logwarn("Error communicating with Slackbot /low_battery at URL %s." % self.slackbot_url)
                                    if "res" in locals():
                                        rospy.logwarn("Response text %s." % res.text)
                                    rospy.logwarn(traceback.format_exc())
                                    rospy.logwarn("Error %s." % e)
                                    update_previous_battery = False
                                break
                    if (update_previous_battery and (self.previous_battery is None or msg.battery.pct < self.previous_battery)):
                        self.previous_battery = msg.battery.pct

    def where_am_i_help_callback(self, msg):
        """
        A dummy callback that triggers sending a where_am_i help message to the
        Slackbot. This is merely intended to showcase some of the Slackbot's
        capabilities. Users who want a robot that autonomously asks the human to
        tell it where it is should implement their own anomaly detection system
        for triggering this help request.
        """
        with self.latest_image_lock:
            if self.latest_image is None:
                rospy.loginfo("Attempted to send where_am_i help request but have no image.")
                return
        try:
            # Send a low_battery_alert
            rospy.loginfo("Sending where_am_i help request")
            with self.latest_image_lock:
                image_contents = base64.b64encode(bytearray(self.latest_image.data)).decode('ascii')
            res = requests.post(
                os.path.join(self.slackbot_url, 'where_am_i'),
                json={'image':image_contents, 'options':['Lounge', "Office#252", "200 Corridoor", "Atrium"]},
            )
            res_json = res.json()
            message_id = res_json['message_id']
            self.sent_messages_database.add_respondable_message(message_id)
            self.database_updated()
        except Exception as e:
            rospy.logwarn("Error communicating with Slackbot /where_am_i at URL %s." % self.slackbot_url)
            if "res" in locals():
                rospy.logwarn("Response text %s." % res.text)
            rospy.logwarn(traceback.format_exc())
            rospy.logwarn("Error %s." % e)

    def get_slackbot_updates(self, refresh_secs=5.0):
        """
        Once every refresh_secs seconds, request updates (e.g., human responses)
        from the Slackbot. Note that you can optionally request updates for
        partular message_ids (e.g., those that have not received responses yet)
        """
        r = rospy.Rate(1.0/refresh_secs)
        while not rospy.is_shutdown():
            if not self.has_loaded: r.sleep()
            try:
                message_ids_and_action_ts = self.sent_messages_database.get_message_ids_and_latest_action_ts()

            	# Request responses for those message_ids
                res = requests.post(
                    os.path.join(self.slackbot_url, 'get_updates'),
                    json={'message_ids_and_action_ts':message_ids_and_action_ts},
                )
                res_json = res.json()
                rospy.logdebug("Got updates from Slackbot %s" % res_json)
                message_id_to_responses = res_json["message_id_to_responses"]

                if len(message_id_to_responses) > 0:
                    num_updates = 0
                    # Insert reactions into the database
                    for message_id in message_id_to_responses:
                        for action_ts, response in message_id_to_responses[message_id]:
                            rospy.loginfo("Got reaction %s from at ts %s" % (response, action_ts))
                            self.sent_messages_database.add_user_response(message_id, action_ts, response)
                            num_updates += 1
                    self.database_updated(num_updates)

            except Exception as e:
            	rospy.logwarn("Error communicating with Slackbot /get_updates at URL %s." % self.slackbot_url)
            	if "res" in locals():
                	rospy.logwarn("Response text %s." % res.text)
            	rospy.logwarn(traceback.format_exc())
            	rospy.logwarn("Error %s." % e)
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("kuri_wandering_robot")

    kuri_wandering_robot = KuriWanderingRobot()

    rospy.spin()
