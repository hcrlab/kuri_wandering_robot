import logging
import pickle
import random
import time

def get_random_alphanumeric_string(stringLen=10):
    lettersAndDigits = string.ascii_letters + string.digits
    return ''.join((random.choice(lettersAndDigits) for i in range(stringLen)))

class SentMessagesDatabase(object):
    """
    Keeps track of image_id, the corresponding (user_id, timestamp)s and the
    users' reactions.
    """
    def __init__(self):#, message_id_method="numeric"):
        """
        Initializes a SentMessagesDatabase object.
        """
        # """
        # message_id_method can take on the following values:
        # - "numeric" means that message_ids will start at 0 and increment for
        #   each new message. message_id will be a string
        # - "alphanumeric_10" means that message_id will be a unique 10-character
        #   random alphanumeric string
        # """
        # self.message_id_method = message_id_method
        # if self.message_id_method not in ["numeric", "alphanumeric_10"]:
        #     logging.info("Unknown message_id_method %s, using \"numeric\"" % message_id_method)
        #     self.message_id_method = "numeric"

        self.image_id_to_url = {}

        # self.message_ids = set()
        self.image_id_to_user_id_ts = {}
        self.user_id_ts_to_image_id = {}
        self.user_id_ts_to_reactions = {}
        self.user_id_to_next_image_send_time = {}

        self.user_id_to_remaining_image_urls_to_send = {}

    # def get_new_message_id(self):
    #     """
    #     Generates a new unique message_id, and adds it to self.message_ids.
    #     Returns the message_id.
    #     """
    #     if self.message_id_method == "numeric":
    #         # The message_id is the number of mesages sent before this one
    #         message_id = str(len(self.message_ids))
    #     else:
    #         # The message_id is a random unique alphanumeric string of len 10
    #         message_id = None
    #         while message_id is None or message_id in self.message_ids:
    #             message_id = get_random_alphanumeric_string(10)
    #
    #     self.message_ids.add(message_id)
    #
    #     return message_id

    def get_image_url(self, image_id):
        """
        Returns the image_url if it exists in self.image_id_to_url, else None.
        """
        if image_id in self.image_id_to_url:
            return self.image_id_to_url[image_id]
        return None

    def add_image_urls(self, image_ids, image_urls):
        """
        For each entry in image_urls, if it is not None, adds the image_id and
        corresponding URL to self.image_id_to_url.
        """
        for i in range(len(image_ids)):
            image_id = image_ids[i]
            image_url = image_urls[i]
            if image_url is not None:
                self.image_id_to_url[image_id] = image_url

    def set_remaining_images_to_send(self, user_id, image_ids, image_urls, image_descriptions):
        """
        Sets the remaining images to send to this user. image_urls should be a
        list of strings.
        """
        self.user_id_to_remaining_image_urls_to_send[user_id] = list(zip(image_ids, image_urls, image_descriptions))

    def get_next_image_to_send(self, user_id):
        """
        Gets the next image_url to send to user_id, or returns None if there us
        None
        """
        if user_id not in self.user_id_to_remaining_image_urls_to_send or len(self.user_id_to_remaining_image_urls_to_send[user_id]) == 0:
            return None, None, None
        return self.user_id_to_remaining_image_urls_to_send[user_id].pop(0)

    def add_sent_message(self, image_id, user_id, ts):
        """
        Adds a message that was sent to a user at a particular timestamp to
        self.image_id_to_user_id_ts and self.user_id_ts_to_image_id, and
        initializes self.user_id_ts_to_reactions
        """
        if image_id not in self.image_id_to_user_id_ts:
            self.image_id_to_user_id_ts[image_id] = set()
        self.image_id_to_user_id_ts[image_id].add((user_id, ts))
        self.user_id_ts_to_image_id[(user_id, ts)] = image_id
        self.user_id_ts_to_reactions[(user_id, ts)] = [0, 0]

        # Reset the user's time to send
        self.user_id_to_next_image_send_time.pop(user_id, None)

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

    def get_reactions(self, image_ids_and_user_ids):
        """
        For the specified image_ids and corresponding user_ids, return their
        responses if they have responded
        """
        image_id_to_user_id_reaction = {}
        for image_id in image_ids_and_user_ids:
            if image_id in self.image_id_to_user_id_ts:
                image_id_to_user_id_reaction[image_id] = []
                for (user_id, ts) in self.image_id_to_user_id_ts[image_id]:
                    if user_id in image_ids_and_user_ids[image_id]:
                        neg_reactions, pos_reactions = self.user_id_ts_to_reactions[(user_id, ts)]
                        if neg_reactions != 0 or pos_reactions != 0:
                            image_id_to_user_id_reaction[image_id].append((user_id, 1 if pos_reactions >= neg_reactions else 0))
        return image_id_to_user_id_reaction

    def add_user_send_image_time(self, user_id, image_send_time):
        """
        Adds the next time user_id should get image sent to them to the database.
        """
        self.user_id_to_next_image_send_time[user_id] = image_send_time

    def get_user_time_to_send(self):
        """
        Returns the time user_id should have their next image(s) sent to them.
        """
        return self.user_id_to_next_image_send_time

    def get_image_id(self, user_id, ts):
        """
        Returns the image_id for (user_id, ts)
        """
        return self.user_id_ts_to_image_id[(user_id, ts)]

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
