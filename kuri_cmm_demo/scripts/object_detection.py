#!/usr/bin/env python
import rospy
import boto3
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from kuri_cmm_demo.srv import ObjectDetection, ObjectDetectionResponse, GetObjectsAtTime, GetObjectsAtTimeResponse
from kuri_cmm_demo.msg import DetectedObjects, Object, ObjectInstance
import pprint
import time
import threading

class DetectObjectsDB(object):
    """
    This ROS node provides two services:
    1) `object_detection` takes in a ROS CompressedImage messages and returns
        the detected objects from AWS Rekognition for that image.
    2) `get_objects_at_time` takes in a timestamp and returns the  detected
        objects from AWS Rekognition for the temporally closest image to that
        timestamp.
    """
    def __init__(self, aws_profile_name, aws_region_name):
        """
        Initialize an instance of the DetectObjectsDB class
        """
        self.bridge = CvBridge()

        # Configure the data structure to cache data in
        # TODO: This should be stored locally in a file -- not all in memory
        self.img_timestamps = [] # in order of receiving the service call
        self.img_timestamp_to_detected_objects = {}

        # Configure the AWS Rekognition client
        self.session = boto3.session.Session(
            profile_name=aws_profile_name, region_name=aws_region_name)
        self.client = self.session.client('rekognition')

        # Configure the Services
        self.object_detection_service = rospy.Service(
            'object_detection', ObjectDetection, self.handle_object_detection,
        )
        self.get_objects_at_time_service = rospy.Service(
            'get_objects_at_time', GetObjectsAtTime, self.handle_get_objects_at_time,
        )

    def detect(self, img_msg, max_labels=100, num_tries=3):
        """
        Given an image message, query AWS Rekognition to get its objects, and
        return the response.
        """
        # Convert the image message to bytes
        if  type(img_msg) == CompressedImage:
            img_bytes = bytearray(img_msg.data)
            # img_data = np.fromstring(img_msg.data, np.uint8)
            # img_cv2 = cv2.imdecode(img_data, cv2.CV_LOAD_IMAGE_COLOR)
        else:
            img_cv2 = self.bridge.imgmsg_to_cv2(img_msg)
            img_bytes = cv2.imencode('.jpg', img_cv2)[1].tobytes()

        # Send the image to AWS Rekognition, try up to num_tries times
        success = False
        for try_i in range(num_tries):
            response = self.client.detect_labels(Image={'Bytes':img_bytes},
                MaxLabels=max_labels)
            if "ResponseMetadata" in response:
                if "HTTPStatusCode" in response["ResponseMetadata"]:
                    if response["ResponseMetadata"]["HTTPStatusCode"] == 200:
                        success = True
                        break

        # Return it
        if success:
            return response
        else:
            return None

    @staticmethod
    def aws_response_to_msg(img_msg, response):
        """
        Convert the response from AWS Rekognition to a DetectedObjects msg
        """
        detected_objects_msg = DetectedObjects()
        detected_objects_msg.header = img_msg.header
        detected_objects_msg.header.stamp = rospy.Time.now()
        detected_objects_msg.objects = []

        for label in response["Labels"]:
            object = Object()
            object.object_name = label["Name"]
            object.instances = []
            object.parents = [parent["Name"] for parent in label["Parents"]]
            object.confidence = label["Confidence"]

            for instance in label["Instances"]:
                objectInstance = ObjectInstance()
                objectInstance.bbox_width = instance["BoundingBox"]["Width"]
                objectInstance.bbox_height = instance["BoundingBox"]["Height"]
                objectInstance.bbox_left = instance["BoundingBox"]["Left"]
                objectInstance.bbox_top = instance["BoundingBox"]["Top"]
                objectInstance.confidence = instance["Confidence"]

                object.instances.append(objectInstance)

            detected_objects_msg.objects.append(object)

        return detected_objects_msg

    def handle_object_detection(self, req):
        """
        Invoked when the object_detection` service is called.
        """
        # Extract the request information
        img_msg = req.compressed_image

        # Detect the objects
        response = self.detect(img_msg)

        # Send the response
        if response is None:
            detected_objects_msg = DetectedObjects(header=img_msg.header, objects=[])
            detected_objects_msg.header.stamp = rospy.Time.now()
            return ObjectDetectionResponse(detected_objects_msg, False)

        # Convert the response to a ROS msg
        detected_objects_msg = DetectObjectsDB.aws_response_to_msg(img_msg, response)

        # Store the timestamp and response
        img_timestamp = img_msg.header.stamp
        self.img_timestamps.append(img_timestamp)
        self.img_timestamp_to_detected_objects[img_timestamp] = detected_objects_msg

        return ObjectDetectionResponse(detected_objects_msg, True)

    def handle_get_objects_at_time(self, req):
        """
        Invoked when the `get_objects_at_time` service is called.
        """
        # Extract the request information
        req_timestamp = req.stamp

        # If there are no images so far, this service call fails
        if len(self.img_timestamps) == 0:
            detected_objects_msg = DetectedObjects(header=img_msg.header, objects=[])
            detected_objects_msg.header.stamp = rospy.Time.now()
            return GetObjectsAtTimeResponse(detected_objects_msg, False)

        # Find the img_msg with the closest timestamp and return its detected
        # objects
        self.img_timestamps.sort() # Sort it in case msgs are received out of order
        prev_img_timestamp = None
        for next_img_timestamp in self.img_timestamps:
            if next_img_timestamp > req_timestamp:
                break
            prev_img_timestamp = next_img_timestamp
        if prev_img_timestamp is None: # req_timestamp is before the first img
            nearest_timestamp = next_img_timestamp
        elif prev_img_timestamp == next_img_timestamp: # req_timestamp is after the last img
            nearest_timestamp = next_img_timestamp
        else: # req_timestamp is between prev_img_timestamp and next_img_timestamp
            time_from_prev = req_timestamp - prev_img_timestamp
            time_from_next = next_img_timestamp - req_timestamp
            if time_from_prev < time_from_next:
                nearest_timestamp = prev_img_timestamp
            else:
                nearest_timestamp = next_img_timestamp

        # Return the detected objects for that image
        detected_objects_msg = self.img_timestamp_to_detected_objects[nearest_timestamp]
        return GetObjectsAtTimeResponse(detected_objects_msg, True)


if __name__ == "__main__":
    rospy.init_node("object_detection")

    aws_profile_name = rospy.get_param('~aws_profile_name', 'default')
    aws_region_name = rospy.get_param('~aws_region_name', 'us-west-2')
    detect_objects_db = DetectObjectsDB(aws_profile_name, aws_region_name)

    rospy.spin()
