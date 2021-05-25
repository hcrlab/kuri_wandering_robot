#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import CompressedImage

class SubsamplingPolicy(object):
    """
    A that subsamples messages from one topic. It can either be used as a ROS
    node, or its subsampling_policy function can be called as part of a larger
    node. Default behavior is to sample 1 image per second.
    """
    def __init__(self, subscribe_to=None, msg_type=CompressedImage,
        publish_to=None, rule='n_per_sec', rule_config={}):
        """
        Initialize the SubsamplingPolicy class, which subscribes to
        subscribe_to, subsamples the messages according to rule, and publishes
        them to publish_to. Below we define the possible rules and relevant
        configs.

        - 'n_per_sec': Subsample n messages per second.
           default config: {'n' : 1}
        - '1_per_n': Subsample 1 per every n messages.
           default config: {'n' : 100}
        """
        if subscribe_to is not None:
            self.sub = rospy.Subscriber(
                subscribe_to, msg_type, self.sub_callback, queue_size=1)

        if publish_to is not None:
            self.pub = rospy.Publisher(
                publish_to, msg_type, queue_size=1)
        else:
            self.pub = None

        self.rule = rule
        self.rule_config = rule_config
        if self.rule == 'n_per_sec':
            n = self.rule_config.get('n', 1)
            self.interval = 1.0/n # secs to wait before sending
            self.last_sent_msg_time = 0.0
        else: # '1_per_n'
            self.interval = self.rule_config.get('n', 100)
            self.num_msg_since_sent = self.interval

    def subsampling_policy(self, msg):
        if self.rule == 'n_per_sec':
            recv_time =time.time()
            if recv_time - self.last_sent_msg_time >= self.interval:
                if self.pub is not None:
                    self.pub.publish(msg)

                self.last_sent_msg_time = recv_time
                return True
        else: # '1_per_n'
            self.num_msg_since_sent += 1

            if self.num_msg_since_sent >= self.interval:
                if self.pub is not None:
                    self.pub.publish(msg)

                self.num_msg_since_sent = 0
                return True
        return False

    def sub_callback(self, msg):
        self.subsampling_policy(msg)


if __name__ == "__main__":
    rospy.init_node("subsampling_policy")

    sub_topic = rospy.get_param('sub_topic', '/upward_looking_camera/compressed')
    pub_topic = rospy.get_param('pub_topic', sub_topic+'/subsampled')
    detect_objects = SubsamplingPolicy(subscribe_to=sub_topic,
        publish_to=pub_topic)

    rospy.spin()
