import rospy
import pickle
import threading

class SentMessagesDatabase(object):
    """
    A local database that stores the images associated with each local message
    ID, as well as the mapping between local message IDs and Slackbot message
    IDs. This database also generates unique local message IDs, and stores the
    objects in our universe.

    To add sent message data, a client should first call get_new_local_msg_id,
    then add_message, then add_slackbot_message_id, then add_user_reaction.

    This class is thread-safe.
    """
    def __init__(self):
        """
        Initialize the database
        """
        self.num_local_msg_ids = 0

        # Mapping between the local and Slackbot message IDs
        self.local_msg_id_to_slackbot_msg_id = {}
        self.slackbot_msg_id_to_local_msg_id = {}

        # Data about the sent messages / images
        self.local_msg_id_to_image = {}
        self.local_msg_id_to_users = {}
        self.user_to_most_recent_local_msg_id = {}
        self.slackbot_msg_id_to_user_to_reaction = {}

        # The objects in our universe
        self.objects = []

        # Self-lock to ensure this class is thread-safe.
        # TODO: This can be improved with a read-write lock.
        self.lock = threading.Lock()

    def get_new_local_msg_id(self):
        """
        Gets a new local message ID
        """
        with self.lock:
            return self.num_local_msg_ids

    def add_message(self, local_msg_id, img_msg, img_vector, users):
        """
        Add a local message that was sent to Slack users
        """
        with self.lock:
            if local_msg_id in self.local_msg_id_to_image:
                rospy.logwarn("Overriding local_msg_id %s" % local_msg_id)
            else:
                self.num_local_msg_ids += 1

            self.local_msg_id_to_image[local_msg_id] = (img_msg, img_vector)
            self.local_msg_id_to_users[local_msg_id] = users

            for user in users:
                self.user_to_most_recent_local_msg_id[user] = local_msg_id

    def add_slackbot_message_id(self, local_msg_id, slackbot_msg_id):
        """
        Add a correspondance between a local msg_id and a slackbot msg_id
        """
        with self.lock:
            self.local_msg_id_to_slackbot_msg_id[local_msg_id] = slackbot_msg_id
            self.slackbot_msg_id_to_local_msg_id[slackbot_msg_id] = local_msg_id

            self.slackbot_msg_id_to_user_to_reaction[slackbot_msg_id] = {}

    def add_user_reaction(self, slackbot_msg_id, user, reaction):
        """
        Adds the user's reaction for message slackbot_msg_id
        """
        with self.lock:
            if slackbot_msg_id not in self.slackbot_msg_id_to_user_to_reaction:
                rospy.logwarn("Please call add_slackbot_message_id before add_user_reaction")
            else:
                self.slackbot_msg_id_to_user_to_reaction[slackbot_msg_id][user] = reaction

    def get_slackbot_msg_ids_without_responses(self):
        """
        Returns the slackbot_msg_ids of the messages that have at least one user
        who hasn't yet responded, along with the users who haven't yet responded.
        Return type is {slackbot_msg_id : [list of users]}

        TODO: In the future, this could also include an optional timestamp, and
        only return messages after that stamp.
        """
        with self.lock:
            retval = {}
            # For each message that has been sent, put it in retval if...
            for local_msg_id in self.local_msg_id_to_users:
                users = self.local_msg_id_to_users[local_msg_id]
                if local_msg_id in self.local_msg_id_to_slackbot_msg_id:
                    slackbot_msg_id = self.local_msg_id_to_slackbot_msg_id[local_msg_id]
                    for user in users:
                        # ...at least one user hasn't reacted yet
                        if user not in self.slackbot_msg_id_to_user_to_reaction[slackbot_msg_id]:
                            if slackbot_msg_id not in retval:
                                retval[slackbot_msg_id] = []
                            retval[slackbot_msg_id].append(user)
            return retval

    def get_most_recent_image(self, user):
        """
        Returns the most recent img_msg and img_vector that was send to user.
        """
        with self.lock:
            # No image has been sent to this user
            if user not in self.user_to_most_recent_local_msg_id:
                return None, None
            return self.local_msg_id_to_image[self.user_to_most_recent_local_msg_id[user]]

    def get_img_vectors_and_reactions(self, user):
        """
        Returns two lists: (1) a list of img_vectors that have been sent to user
        and they have responded to; (2) an equally-long list of user's responses.

        NOTE: this can be made more efficient by just storing a map from users
        to slackbot_msg_ids to reactions in add_user_reaction
        """
        img_vectors = []
        reactions = []
        for slackbot_msg_id in self.slackbot_msg_id_to_user_to_reaction:
            for user_temp in self.slackbot_msg_id_to_user_to_reaction[slackbot_msg_id]:
                if user_temp != user: continue
                local_msg_id = self.slackbot_msg_id_to_local_msg_id[slackbot_msg_id]
                img_vector = self.local_msg_id_to_image[local_msg_id][1]
                reaction = self.slackbot_msg_id_to_user_to_reaction[slackbot_msg_id][user]
                img_vectors.append(img_vector)
                reactions.append(reaction)
        return img_vectors, reactions

    def get_object_i(self, object_name):
        """
        Get the index of object_name. If it doesn't exist, add it to self.objects.

        Returns object_i and a bool indicating whether a new object was added
        or not.
        """
        try:
            object_i = self.objects.index(object_name)
            was_added = False
        except ValueError: # object_name isn't in list -- extend the dimentionality
            object_i = len(self.objects)
            self.objects.append(object_name)
            was_added = True

        return object_i, was_added

    def get_num_objects(self):
        """
        Returns the number of objects in the universe
        """
        return len(self.objects)

    def get_objects(self):
        """
        Returns the objects in the universe
        """
        return self.objects

    def get_pickleable_shallow_copy(self):
        """
        Returns an instance of SentMessagesDatabase with the exact same data
        structures and data as self, but without any non-pickleable attributes
        (e.g., self.lock)
        """
        retval = SentMessagesDatabase()
        retval.num_local_msg_ids = self.num_local_msg_ids

        retval.local_msg_id_to_slackbot_msg_id = self.local_msg_id_to_slackbot_msg_id
        retval.slackbot_msg_id_to_local_msg_id = self.slackbot_msg_id_to_local_msg_id

        retval.local_msg_id_to_image = self.local_msg_id_to_image
        retval.local_msg_id_to_users = self.local_msg_id_to_users
        retval.user_to_most_recent_local_msg_id = self.user_to_most_recent_local_msg_id
        retval.slackbot_msg_id_to_user_to_reaction = self.slackbot_msg_id_to_user_to_reaction

        retval.objects = self.objects

        retval.lock = None

        return retval

    def save(self, pkl_filepath):
        """
        Pickles self and saves it at pkl_filepath

        TODO: This has a race condition where after we set self.lock to None,
        if another function is called, it will no longer be thread-safe. To fix
        this, do not pickle this object, but rather have a function that returns
        a shallow copy of SentMessagesDatabase with all the pickleable attributes
        set and all the non-pickleable attributes set to None. That way, the
        lock on this instance itself has not changed.
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
