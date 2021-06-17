import rospy
import pickle
import threading

class SentMessagesDatabase(object):
    """
    A local database that stores the images associated with each local message
    ID, as well as the mapping between local message IDs and Slackbot message
    IDs. This database also generates unique local message IDs, and stores the
    objects in our universe.

    To add sent message data, a client should first call get_new_local_img_id,
    then add_image, then add_slackbot_image_id, then add_user_reaction.

    This class is thread-safe.

    Note that local_img_ids are not necessarily the same as slackbot_img_ids,
    but might be.
    """
    def __init__(self):
        """
        Initialize the database
        """
        self.num_local_img_ids = 0

        # Mapping between the local and Slackbot message IDs
        self.local_img_id_to_slackbot_img_id = {}
        self.slackbot_img_id_to_local_img_id = {}

        # Data about stored images
        self.local_img_id_to_image = {}
        self.user_to_stored_local_img_ids = {}

        # Data about sent images
        self.slackbot_img_id_to_user_to_reaction = {}
        self.user_to_sent_local_img_ids = {}

        # The objects in our universe
        self.objects = []

        # Self-lock to ensure this class is thread-safe.
        # TODO: This can be improved with a read-write lock.
        self.lock = threading.Lock()

    def get_new_local_img_id(self):
        """
        Gets a new local image ID
        """
        with self.lock:
            return self.num_local_img_ids

    def add_image(self, local_img_id, img_msg, img_vector, users):
        """
        Add a local message that was sent to Slack users
        """
        with self.lock:
            if local_img_id in self.local_img_id_to_image:
                rospy.logwarn("Overriding local_img_id %s" % local_img_id)
            else:
                self.num_local_img_ids += 1

            self.local_img_id_to_image[local_img_id] = (img_msg, img_vector)

            for user in users:
                if user not in self.user_to_stored_local_img_ids:
                    self.user_to_stored_local_img_ids[user] = []
                self.user_to_stored_local_img_ids[user].append(local_img_id)

    def get_stored_images_for_user(self, user):
        """
        Returns the messages and vectors for every stored image for the user.
        """
        img_msgs = []
        img_vectors = []
        local_img_ids = []
        if user in self.user_to_stored_local_img_ids:
            for local_img_id in self.user_to_stored_local_img_ids[user]:
                img_msg, img_vector = self.local_img_id_to_image[local_img_id]
                img_vectors.append(img_vector)
                img_msgs.append(img_msg)
                local_img_ids.append(local_img_id)
        return img_msgs, img_vectors, local_img_ids

    def add_slackbot_image_id(self, local_img_ids, slackbot_img_ids, user):
        """
        Add a correspondance between a local img_id and a slackbot img_id.
        Called when an image is sent to user.
        """
        with self.lock:
            for i in range(len(local_img_ids)):
                # Store the mapping between local and slackbot IDs
                local_img_id = local_img_ids[i]
                slackbot_img_id = slackbot_img_ids[i]
                self.local_img_id_to_slackbot_img_id[local_img_id] = slackbot_img_id
                self.slackbot_img_id_to_local_img_id[slackbot_img_id] = local_img_id

                # initialize self.slackbot_img_id_to_user_to_reaction
                if slackbot_img_id not in self.slackbot_img_id_to_user_to_reaction:
                    self.slackbot_img_id_to_user_to_reaction[slackbot_img_id] = {}

                # Remove the image from the stored list and add it to the sent list
                try:
                    self.user_to_stored_local_img_ids[user].remove(local_img_id)
                except ValueError as e:
                    rospy.logwarn("Sent local_img_id %d for user %d to the slackbot, even though it was not stored for that user" % (local_img_id, user))
                if user not in self.user_to_sent_local_img_ids:
                    self.user_to_sent_local_img_ids[user] = []
                self.user_to_sent_local_img_ids[user].append(local_img_id)

    def add_user_reaction(self, slackbot_img_id, user, reaction):
        """
        Adds the user's reaction for message slackbot_img_id
        """
        with self.lock:
            if slackbot_img_id not in self.slackbot_img_id_to_user_to_reaction:
                rospy.logwarn("Please call add_slackbot_image_id before add_user_reaction")
            else:
                self.slackbot_img_id_to_user_to_reaction[slackbot_img_id][user] = reaction

    def get_slackbot_img_ids_without_responses(self):
        """
        Returns the slackbot_img_ids of the messages that have at least one user
        who hasn't yet responded, along with the users who haven't yet responded.
        Return type is {slackbot_img_id : [list of users]}

        TODO: In the future, this could also include an optional timestamp, and
        only return messages after that stamp.
        """
        with self.lock:
            retval = {}
            # For each message that has been sent, put it in retval if...
            for user in self.user_to_sent_local_img_ids:
                for local_img_id in self.user_to_sent_local_img_ids[user]:
                    slackbot_img_id = self.local_img_id_to_slackbot_img_id[local_img_id]
                    # ...at least one user hasn't reacted yet
                    if user not in self.slackbot_img_id_to_user_to_reaction[slackbot_img_id]:
                        if slackbot_img_id not in retval:
                            retval[slackbot_img_id] = []
                        retval[slackbot_img_id].append(user)
            return retval

    def get_most_recent_stored_and_sent_images(self, user, n_images=5):
        """
        Returns up to n_images most recent img_msg and img_vector that were
        stored for user, and up to another n_images that were sent to the user.
        """
        with self.lock:
            # No image has been sent to this user
            img_msgs = []
            img_vectors = []
            if user in self.user_to_stored_local_img_ids:
                for local_img_id in self.user_to_stored_local_img_ids[user][-n_images:]: # self.user_to_sent_local_img_ids[user][-n_images:]: #
                    img_msg, img_vector = self.local_img_id_to_image[local_img_id]
                    img_msgs.append(img_msg)
                    img_vectors.append(img_vector)
            if user in self.user_to_sent_local_img_ids:
                for local_img_id in self.user_to_sent_local_img_ids[user][-n_images:]: # self.user_to_sent_local_img_ids[user][-n_images:]: #
                    img_msg, img_vector = self.local_img_id_to_image[local_img_id]
                    img_msgs.append(img_msg)
                    img_vectors.append(img_vector)
            return img_msgs, img_vectors

    def get_img_vectors_and_reactions(self, user):
        """
        Returns two lists: (1) a list of img_vectors that have been sent to user
        and they have responded to; (2) an equally-long list of user's responses.

        NOTE: this can be made more efficient by just storing a map from users
        to slackbot_img_ids to reactions in add_user_reaction
        """
        img_vectors = []
        reactions = []
        rospy.logdebug("self.slackbot_img_id_to_user_to_reaction %s" % self.slackbot_img_id_to_user_to_reaction)
        for slackbot_img_id in self.slackbot_img_id_to_user_to_reaction:
            for user_temp in self.slackbot_img_id_to_user_to_reaction[slackbot_img_id]:
                if user_temp != user: continue
                local_img_id = self.slackbot_img_id_to_local_img_id[slackbot_img_id]
                img_vector = self.local_img_id_to_image[local_img_id][1]
                reaction = self.slackbot_img_id_to_user_to_reaction[slackbot_img_id][user]
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
        retval.__dict__.update(self.__dict__)
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
