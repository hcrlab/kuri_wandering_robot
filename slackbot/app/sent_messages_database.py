import pickle
import random
import logging

def get_random_alphanumeric_string(stringLen=10):
    lettersAndDigits = string.ascii_letters + string.digits
    return ''.join((random.choice(lettersAndDigits) for i in range(stringLen)))

class SentMessagesDatabase(object):
    """
    Keeps track of message_id, the corresponding (user_id, timestamp)s and the
    users' reactions. Generates new message_id values
    """
    def __init__(self, message_id_method="numeric"):
        """
        Initializes a SentMessagesDatabase object.

        message_id_method can take on the following values:
        - "numeric" means that message_ids will start at 0 and increment for
          each new message. message_id will be a string
        - "alphanumeric_10" means that message_id will be a unique 10-character
          random alphanumeric string
        """
        self.message_id_method = message_id_method
        if self.message_id_method not in ["numeric", "alphanumeric_10"]:
            logging.info("Unknown message_id_method %s, using \"numeric\"" % message_id_method)
            self.message_id_method = "numeric"

        self.message_ids = set()
        self.message_id_to_user_id_ts = {}
        self.user_id_ts_to_message_id = {}
        self.user_id_ts_to_reactions = {}

    def get_new_message_id(self):
        """
        Generates a new unique message_id, and adds it to self.message_ids.
        Returns the message_id.
        """
        if self.message_id_method == "numeric":
            # The message_id is the number of mesages sent before this one
            message_id = str(len(self.message_ids))
        else:
            # The message_id is a random unique alphanumeric string of len 10
            message_id = None
            while message_id is None or message_id in self.message_ids:
                message_id = get_random_alphanumeric_string(10)

        self.message_ids.add(message_id)

        return message_id

    def add_sent_message(self, message_id, user_id, ts):
        """
        Adds a message that was sent to a user at a particular timestamp to
        self.message_id_to_user_id_ts and self.user_id_ts_to_message_id, and
        initializes self.user_id_ts_to_reactions
        """
        if message_id not in self.message_id_to_user_id_ts:
            self.message_id_to_user_id_ts[message_id] = set()
        self.message_id_to_user_id_ts[message_id].add((user_id, ts))
        self.user_id_ts_to_message_id[(user_id, ts)] = message_id
        self.user_id_ts_to_reactions[(user_id, ts)] = [0, 0]

    def add_reaction(self, user_id, ts, reaction):
        """
        Adds a reaction from a user to a message sent at a particular timestamp
        to self.user_id_ts_to_reactions
        """
        if (user_id, ts) not in self.user_id_ts_to_reactions:
            logging.info("add_reaction for (user_id, ts) (%s, %f) not in self.user_id_ts_to_reactions %s" % (user_id, ts, self.user_id_ts_to_reactions))
            return
        if reaction < 0 or reaction > 1:
            logging.info("Got unknown reaction %s for (user_id, ts) (%s, %f)" % (reaction, user_id, ts))
            return
        self.user_id_ts_to_reactions[(user_id, ts)][reaction] += 1

    def get_reactions(self, message_ids):
        """
        For a message that was sent, return a list of (user_id, reaction) for
        the users who have reacted
        """
        message_id_to_user_id_reaction = {}
        for message_id in message_ids:
            if message_id in self.message_id_to_user_id_ts:
                message_id_to_user_id_reaction[message_id] = []
                for (user_id, ts) in self.message_id_to_user_id_ts[message_id]:
                    neg_reactions, pos_reactions = self.user_id_ts_to_reactions[(user_id, ts)]
                    if neg_reactions != 0 or pos_reactions != 0:
                        message_id_to_user_id_reaction[message_id].append((user_id, 1 if pos_reactions >= neg_reactions else 0))
        return message_id_to_user_id_reaction

    def get_message_id(self, user_id, ts):
        """
        Returns the message_id for (user_id, ts)
        """
        return self.user_id_ts_to_message_id[(user_id, ts)]

    def save(self, pkl_filepath):
        """
        Pickles self and saves it at pkl_filepath
        """
        with open(pkl_filepath, "wb") as f:
            return pickle.dump(self, f)

    @staticmethod
    def load(pkl_filepath):
        """
        Attempts to load the SentMessagesDatabase pickle stored at pkl_filepath.
        If this fails, initializes a new SentMessagesDatabase and returns it.
        """
        try:
            with open(pkl_filepath, "rb") as f:
                return pickle.load(f)
        except Exception as e:
            logging.info("Could not load pickled SentMessagesDatabase, initializing new one %s" % e)
            return SentMessagesDatabase()
