import logging
import pickle
import random
import string
import time

class SentMessagesDatabase(object):
    """
    Keeps track messages that were sent to users, the images that were uploaded
    to Slack, and the users' responses to messages.
    """
    def __init__(self):
        """
        Initializes a SentMessagesDatabase object.

        :returns: an instance of the SentMessagesDatabase class
        """

        self.image_id_to_url = {}

        self.num_messages_sent = 0
        self.message_id_to_user_id_ts = {}
        self.user_id_ts_to_responses = {}

    def get_message_id(self):
        """
        Returns a unique message_id.

        :returns: a unique message ID (int)
        """
        message_id = self.num_messages_sent
        self.num_messages_sent += 1
        return str(message_id)

    def add_sent_message(self, user_id, ts, message_id, can_users_respond=False):
        """
        Adds the metadata for a sent message to the database.

        :param user_id: (str) the user ID
        :param ts: (str) the timestamp of the sent message on Slack
        :param message_id: (int) the unique message ID
        :param can_users_respond: (bool) whether or not this is a message that
                                  users can respond to.
        :returns: None
        """
        if message_id not in self.message_id_to_user_id_ts:
            self.message_id_to_user_id_ts[message_id] = []
        self.message_id_to_user_id_ts[message_id].append((user_id, ts))

        if can_users_respond:
            self.user_id_ts_to_responses[(user_id, ts)] = []

    def get_image_url(self, image_id):
        """
        Returns the image_url if it exists in self.image_id_to_url, else None.

        :param image_id: (str) the image ID
        :returns: the image URL (str) if it exists in the database, else None
        """
        if image_id in self.image_id_to_url:
            return self.image_id_to_url[image_id]
        return None

    def add_image_url(self, image_id, image_url):
        """
        If image_url is not None, adds the image_id and corresponding URL to
        self.image_id_to_url.

        :param image_id: (str) the image ID
        :param image_url: (str) the image URL
        :returns: None
        """
        if image_url is not None:
            self.image_id_to_url[image_id] = image_url

    def add_response(self, user_id, ts, response, action_ts):
        """
        Adds a response from a user to a message sent at a particular timestamp.

        :param user_id: (str) the user ID
        :param ts: (str) the timestamp of the sent message on Slack
        :param response: (str) the user's response
        :param action_ts: (str) the timestamp of the response
        :returns: None
        """
        if (user_id, ts) not in self.user_id_ts_to_responses:
            logging.info("Message with (user_id, ts) (%s, %f) is not expected to receive responses. Skipping." % (user_id, ts))
            return
        self.user_id_ts_to_responses[(user_id, ts)].append((action_ts, response))

    def get_responses(self, message_ids_and_action_ts=None):
        """
        For the specified message IDs, return the user responses if any.

        :param message_ids_and_action_ts: ((dict: message_id -> action_ts) or
            None) a dict of the requested message IDs and the action_ts after
            which to send updates. If None, return responses for all message IDs.
        :returns: dictionary mapping message IDs to a chronological list of
                  responses (where a response is a tuple of (action_id, response)).
        """
        if message_ids_and_action_ts is None:
            message_ids_and_action_ts = {}
            for message_id in self.message_id_to_user_id_ts.keys():
                message_ids_and_action_ts[message_id] = 0.0

        message_id_to_responses = {}

        for message_id in message_ids_and_action_ts:
            if message_id not in self.message_id_to_user_id_ts: continue
            for (user_id, ts) in self.message_id_to_user_id_ts[message_id]:
                if (user_id, ts) not in self.user_id_ts_to_responses: continue
                for (action_ts, response) in self.user_id_ts_to_responses[(user_id, ts)]:
                    if float(action_ts) > float(message_ids_and_action_ts[message_id]):
                        if message_id not in message_id_to_responses:
                            message_id_to_responses[message_id] = []
                        message_id_to_responses[message_id].append((action_ts, response))

        return message_id_to_responses

    def save(self, pkl_filepath):
        """
        Pickles self and saves it at pkl_filepath

        :param pkl_filepath: (str) the filepath to store the pickled database
        :returns: None
        """
        with open(pkl_filepath, "wb") as f:
            pickle.dump(self, f)

    @staticmethod
    def load(pkl_filepath):
        """
        Attempts to load the SentMessagesDatabase pickle stored at pkl_filepath.
        If this fails, initializes a new SentMessagesDatabase and returns it.

        :param pkl_filepath: (str) the filepath to load the pickled database
        :returns: a SentMessagesDatabase instance
        """
        try:
            with open(pkl_filepath, "rb") as f:
                return pickle.load(f)
        except Exception as e:
            logging.info("Could not load pickled SentMessagesDatabase, initializing new one %s" % e)
            return SentMessagesDatabase()
