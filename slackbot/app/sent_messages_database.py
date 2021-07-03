import logging
import pickle
import random
import string
import time

class SentMessagesDatabase(object):
    """
    Keeps track of image_id, the corresponding (user_id, timestamp)s and the
    users' reactions.
    """
    def __init__(self):
        """
        Initializes a SentMessagesDatabase object.
        """

        self.image_id_to_url = {}

        # self.message_ids = set()
        self.image_id_to_user_id_ts = {}
        self.user_id_ts_to_image_id = {}
        self.user_id_ts_to_reactions = {}
        self.user_id_to_next_image_send_time = {}

        # Track Scheduled Messages
        self.user_id_to_scheduled_message_ts = {}

        # Random IDs (Survey Messages)
        self.random_ids = set()
        self.user_id_to_random_ids = {}
        self.user_id_to_survey_urls = {}
        self.survey_url_to_user_id_ts = {}
        self.user_id_ts_to_survey_url = {}

        # Intro Messages
        self.intro_message_user_id_ts = set()
        self.pre_study_message_user_id_ts = set()

    def get_random_id(self, user_id, n_digits=10):
        if user_id not in self.user_id_to_random_ids or len(self.user_id_to_random_ids[user_id]) == 0:
            random_id = None
            while random_id is None or random_id in self.random_ids:
                random_id = random.randint(10**(n_digits-1),10**(n_digits)-1)
            self.random_ids.add(random_id)
            return random_id
        else:
            return self.user_id_to_random_ids[user_id][0]

    def add_random_id(self, user_id, random_id, survey_url):
        if user_id not in self.user_id_to_random_ids:
            self.user_id_to_random_ids[user_id] = []
            self.user_id_to_survey_urls[user_id] = []
        self.user_id_to_random_ids[user_id].append(random_id)
        self.user_id_to_survey_urls[user_id].append(survey_url)

    def add_intro_message(self, user_id, ts):
        self.intro_message_user_id_ts.add((user_id, ts))

    def add_pre_study_message(self, user_id, ts):
        self.pre_study_message_user_id_ts.add((user_id, ts))

    def add_sent_survey(self, user_id, ts, survey_url):
        self.survey_url_to_user_id_ts[survey_url] = (user_id, ts)
        self.user_id_ts_to_survey_url[(user_id, ts)] = survey_url

    def was_scheduled_message_sent(self, user_id, scheduled_ts):
        if user_id not in self.user_id_to_scheduled_message_ts or scheduled_ts not in self.user_id_to_scheduled_message_ts[user_id]:
            return False
        return True

    def scheduled_message_was_sent(self, user_id, scheduled_ts):
        if user_id not in self.user_id_to_scheduled_message_ts:
            self.user_id_to_scheduled_message_ts[user_id] = set()
        self.user_id_to_scheduled_message_ts[user_id].add(scheduled_ts)

    def unsend_scheduled_messages_after(self, time_cutoff):
        for user_id in self.user_id_to_scheduled_message_ts:
            for scheduled_ts in list(self.user_id_to_scheduled_message_ts[user_id]):
                if scheduled_ts >= time_cutoff:
                    # The below if statement is likley redundant
                    if scheduled_ts in self.user_id_to_scheduled_message_ts[user_id]:
                        self.user_id_to_scheduled_message_ts[user_id].remove(scheduled_ts)

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

    def add_user_send_image_time(self, user_id, image_send_time=None):
        """
        Adds the next time user_id should get image sent to them to the database.
        """
        if image_send_time is None:
            # Reset the user's time to send
            self.user_id_to_next_image_send_time.pop(user_id, None)
        else:
            # Set the image_send_time
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
