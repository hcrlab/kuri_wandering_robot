#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import CompressedImage

class SubsamplingPolicy(object):
    """
    A class that subsamples messages from one topic. Default behavior is to
    sample 1 image per second.
    """
    def __init__(self, rule='n_per_sec', rule_config={}):
        """
        Initialize the SubsamplingPolicy class, which subsamples according to
        one of these possible rules.

        - 'n_per_sec': Subsample n messages per second.
           default config: {'n' : 1}
        - '1_per_n': Subsample 1 per every n messages.
           default config: {'n' : 100}
        """
        self.rule = rule
        self.rule_config = rule_config
        if self.rule == 'n_per_sec':
            n = self.rule_config.get('n', 1)
            self.interval = 1.0/n # secs to wait before sending
            self.last_sent_msg_time = 0.0
        else: # '1_per_n'
            self.interval = self.rule_config.get('n', 100)
            self.num_msg_since_sent = self.interval


    def subsample(self, msg):
        if self.rule == 'n_per_sec':
            recv_time = time.time()
            if recv_time - self.last_sent_msg_time >= self.interval:
                self.last_sent_msg_time = recv_time
                return True
        else: # '1_per_n'
            self.num_msg_since_sent += 1
            if self.num_msg_since_sent >= self.interval:
                self.num_msg_since_sent = 0
                return True
        return False
