#!/usr/bin/env python
import rospy
import boto3
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from kuri_cmm_demo.msg import DetectedObjects, Object, ObjectInstance
import pprint

class DetectObjects(object):
    """
    A class that takes in ROS (Compressed)Image messages and outputs the
    detected objects from AWS Rekognition. This could either be used as a
    function within a larger ROS node, or as a standalone ROS node that
    subscribes to an image topic and publishes to a detected objects topic.
    """
    def __init__(self, subscribe_to=None, img_type=CompressedImage,
        publish_to=None):
        self.bridge = CvBridge()

        self.session = boto3.session.Session(
            profile_name='hcrlab_AmalAWS', region_name='us-west-2')
        self.client = self.session.client('rekognition')

        if subscribe_to is not None:
            self.sub = rospy.Subscriber(
                subscribe_to, img_type, self.sub_callback, queue_size=1)

        if publish_to is not None:
            self.pub = rospy.Publisher(
                publish_to, DetectedObjects, queue_size=1)
        else:
            self.pub = None

    def detect(self, img_msg, max_labels=100, num_tries=3):
        # Convert the image message to bytes
        print(type(img_msg))
        if  type(img_msg) == CompressedImage:
            img_bytes = bytearray(img_msg.data)
            # img_data = np.fromstring(img_msg.data, np.uint8)
            # img_cv2 = cv2.imdecode(img_data, cv2.CV_LOAD_IMAGE_COLOR)
        else:
            img_cv2 = self.imgmsg_to_cv2(img_msg)
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
            if self.pub is not None:
                detected_objects_msg = DetectObjects.aws_response_to_msg(
                    img_msg, response)
                self.pub.publish(detected_objects_msg)

            return response
        else:
            return None

    @staticmethod
    def aws_response_to_msg(img_msg, response):
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

    def sub_callback(self, img_msg):
        self.detect(img_msg)

if __name__ == "__main__":
    rospy.init_node("object_detection")

    sub_topic = rospy.get_param('sub_topic', '/upward_looking_camera/compressed/subsampled')
    pub_topic = rospy.get_param('pub_topic', sub_topic + '/detected_objects')
    detect_objects = DetectObjects(subscribe_to=sub_topic, publish_to=pub_topic)

    rospy.spin()
