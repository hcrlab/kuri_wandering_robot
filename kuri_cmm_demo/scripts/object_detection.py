#!/usr/bin/env python
import rospy
import boto3
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from kuri_cmm_demo.srv import ObjectDetection, ObjectDetectionResponse
from kuri_cmm_demo.msg import DetectedObjects, Object, ObjectInstance
import pprint
import time
import threading

class DetectObjectsDB(object):
    """
    A service that takes in a ROS CompressedImage messages and returns the
    detected objects from AWS Rekognition for either that image or a
    sufficiently similar image.
    """
    def __init__(self, aws_profile_name, aws_region_name):
        self.bridge = CvBridge()

        # Configure the AWS Rekognition client
        self.session = boto3.session.Session(
            profile_name=aws_profile_name, region_name=aws_region_name)
        self.client = self.session.client('rekognition')

        # Configure the Service
        self.service = rospy.Service(
            'object_detection', ObjectDetection, self.handle_object_detection,
        )

    def detect(self, img_msg, max_labels=100, num_tries=3):
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
        Invoked when the service is called.
        """
        recv_time = time.time()

        # Extract the request information
        img_msg = req.compressed_image
        force_exact_image = req.force_exact_image

        # Detect the objects
        # For now, use the exact image regardless of the force_exact_image flag
        response = self.detect(img_msg)

        # Send the response
        if response is None:
            return ObjectDetectionResponse(
                DetectedObjects(
                    header=compressed_image.header, objects=[]),
                False,
                time.time()-recv_time,
            )
        detected_objects_msg = DetectObjectsDB.aws_response_to_msg(
            img_msg, response)
        return ObjectDetectionResponse(
            DetectObjectsDB.aws_response_to_msg(
                img_msg, response),
            True,
            time.time()-recv_time,
        )


if __name__ == "__main__":
    rospy.init_node("object_detection")

    aws_profile_name = rospy.get_param('~aws_profile_name', 'default')
    aws_region_name = rospy.get_param('~aws_region_name', 'us-west-2')
    detect_objects_db = DetectObjectsDB(aws_profile_name, aws_region_name)

    rospy.spin()
