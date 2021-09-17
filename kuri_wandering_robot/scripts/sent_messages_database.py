import hashlib
import json
import os
import pickle
import random
import rospy
import threading
import time

class SentMessagesDatabase(object):
    """
    A local database that stores the message_ids and responses for messages that
    can be responded to.

    """
    def __init__(self):
        """
        Initialize the database
        """
        self.message_id_to_action_ts_response = {}

        # Self-lock to ensure this class is thread-safe.
        # TODO: This can be improved with a read-write lock.
        self.lock = threading.Lock()

    def add_respondable_message(self, message_id):
        """
        Adds a respondable message_id to the database.
        """
        with self.lock:
            if message_id in self.message_id_to_action_ts_response:
                rospy.loginfo("Got repeated message_id %s, ignoring." % message_id)
            else:
                self.message_id_to_action_ts_response[message_id] = []

    def get_message_ids_and_latest_action_ts(self):
        """
        Returns a dictionary of message_ids and the latest action_ts received
        for that ID.
        """
        with self.lock:
            retval = {}
            for message_id in self.message_id_to_action_ts_response:
                retval[message_id] = 0.0
                if len(self.message_id_to_action_ts_response[message_id]) > 0:
                    retval[message_id] = self.message_id_to_action_ts_response[message_id][-1][0]
            return retval

    def add_user_response(self, message_id, action_ts, response):
        """
        Adds the user response and action_id for message_id
        """
        with self.lock:
            if message_id not in self.message_id_to_action_ts_response:
                rospy.logwarn("Got message_id %s which has not yet been added to the sent_messages_database. Ignoring." % message_id)
            else:
                self.message_id_to_action_ts_response[message_id].append((action_id, response))

    def get_responses(self, message_id):
        """
        Returns the responses to this message_id
        """
        with self.lock:
            retval = []
            if message_id not in self.message_id_to_action_ts_response:
                rospy.logwarn("Requested responses for message_id %s which has not yet been added to the sent_messages_database. Ignoring." % message_id)
            else:
                for action_ts, response in self.message_id_to_action_ts_response[message_id]:
                    retval.append(response)
            return retval

    def get_pickleable_shallow_copy(self):
        """
        Returns an instance of SentMessagesDatabase with the exact same data
        structures and data as self, but without any non-pickleable attributes
        (e.g., self.lock)
        """
        retval = SentMessagesDatabase()
        retval.__dict__.update(self.__dict__)
        retval.lock = None

        return retval

    def save(self, pkl_filepath):
        """
        Pickles self and saves it at pkl_filepath
        """
        with self.lock:
            rospy.loginfo("Saving the sent_messages_database")
            with open(pkl_filepath, "wb") as f:
                pickle.dump(self.get_pickleable_shallow_copy(), f)

    @staticmethod
    def load(pkl_filepath):
        """
        Attempts to load the SentMessagesDatabase pickle stored at pkl_filepath.
        If this fails, initializes a new SentMessagesDatabase and returns it.
        """
        try:
            with open(pkl_filepath, "rb") as f:
                sent_messages_database = pickle.load(f)
                sent_messages_database.lock = threading.Lock()
                return sent_messages_database
        except Exception as e:
            rospy.logwarn("Could not load pickled SentMessagesDatabase, initializing new one %s" % e)
            return SentMessagesDatabase()
