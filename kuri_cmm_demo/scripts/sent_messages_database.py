import cv2
import hashlib
import json
import numpy as np
import os
import pickle
import random
import rospy
import threading
import time

class ImageLoader(object):
    """
    The ImageLoader class stores copies of images, vectors, and detected_objects
    on the robot's harddrive. It then loads and caches them as necessary.
    """

    def __init__(self, cache_size=300, base_dir="/mayfield/data/kuri_cmm_demo/images"):
        """
        Initialize the class
        """
        self.base_dir = base_dir
        self.cache_size = cache_size

        if not os.path.isdir(self.base_dir):
            os.makedirs(self.base_dir)

        self.cache = {}
        self.img_id_order = []

    @staticmethod
    def detected_objects_msg_to_dict(detected_objects_msg):
        """
        Convert a ROS DetectedObjects msg to a dictionary to save as a JSON
        """
        return {
            "header" : {
                "seq" : detected_objects_msg.header.seq,
                "stamp" : detected_objects_msg.header.stamp.to_sec(),
                "frame_id" : detected_objects_msg.header.frame_id,
            },
            "objects" : [
                {
                    "object_name" : object.object_name,
                    "instances" : [
                        {
                            "bbox_width" : instance.bbox_width,
                            "bbox_height" : instance.bbox_height,
                            "bbox_left" : instance.bbox_left,
                            "bbox_top" : instance.bbox_top,
                            "confidence" : instance.confidence,
                        }
                        for instance in object.instances
                    ],
                    "parents" : object.parents,
                    "confidence" : object.confidence,
                }
                for object in detected_objects_msg.objects
            ]
        }

    def add_to_cache(self, img_id, val):
        """
        Adds an image to the cache
        """
        was_already_in_cache = (img_id in self.cache)
        self.cache[img_id] = val
        if not was_already_in_cache:
            self.img_id_order.append(img_id)
        while len(self.img_id_order) > self.cache_size:
            img_id_to_remove = self.img_id_order.pop(0)
            self.cache.pop(img_id)

    def add_image(self, img_id, img_cv2, img_vector, detected_objects_msg):
        """
        Write the image, then add it to the cache

        TODO: can make the vector representation more efficient by using npz
        instead of json
        """
        img_id = str(img_id)
        # Save the image
        # img_data = np.fromstring(img_msg.data, np.uint8)
        # img_cv2 = cv2.imdecode(img_data, cv2.CV_LOAD_IMAGE_COLOR)
        cv2.imwrite(os.path.join(self.base_dir, str(img_id)+".jpg"), img_cv2)
        # Save the vector
        with open(os.path.join(self.base_dir, str(img_id)+"_vector.json"), 'w') as f:
            json.dump(img_vector.tolist(), f, indent=4)
        # Save the detected objects output
        detected_objects_dict = ImageLoader.detected_objects_msg_to_dict(detected_objects_msg)
        with open(os.path.join(self.base_dir, str(img_id)+"_detected_objects.json"), 'w') as f:
            json.dump(detected_objects_dict, f, indent=4)
        # Add to the cache
        self.add_to_cache(img_id, val=[img_cv2, img_vector, detected_objects_dict])

    def get_images(self, img_ids, return_img_cv2=False, return_img_vector=True, return_detected_objects_dict=False):
        """
        Gets images with the img_ids. Is None for any values that were not requested.
        """
        retval = []
        for img_id in img_ids:
            if img_id in self.cache:
                val = self.cache[img_id]
            else:
                # Load the files from the cache
                val = [None, None, None]

            if return_img_cv2 and val[0] is None:
                try:
                    img_cv2 = cv2.imread(os.path.join(self.base_dir, str(img_id)+".jpg"))
                    val[0] = img_cv2
                except Exception as e:
                    rospy.logwarn("Failed to get img_cv2 for img_id %s" % img_id)
            if return_img_vector and val[1] is None:
                try:
                    with open(os.path.join(self.base_dir, str(img_id)+"_vector.json"), 'r') as f:
                        img_vector = np.array(json.load(f))
                        val[1] = img_vector
                except Exception as e:
                    rospy.logwarn("Failed to get img_vector for img_id %s" % img_id)
            if return_detected_objects_dict and val[2] is None:
                try:
                    with open(os.path.join(self.base_dir, str(img_id)+"_detected_objects.json"), 'r') as f:
                        detected_objects_dict = json.load(f)
                        val[2] = detected_objects_dict
                except Exception as e:
                    rospy.logwarn("Failed to get image for detected_objects %s" % img_id)
            self.add_to_cache(img_id, val)
            retval.append(val)
        return retval


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
    def __init__(self, objects_filepath=None, n_users=1, most_recent_images=5, user_to_learning_condition={0:1}):
        """
        Initialize the database
        """
        self.n_users = n_users
        self.most_recent_images = most_recent_images
        self.user_to_learning_condition = user_to_learning_condition

        # Mapping between the local and Slackbot message IDs
        self.local_img_id_to_slackbot_img_id = {}
        self.slackbot_img_id_to_local_img_id = {}

        # Data about stored images
        self.image_loader = ImageLoader(cache_size=int(2.5*self.most_recent_images*self.n_users))
        self.local_img_ids = set()
        self.user_to_stored_local_img_ids = {}
        self.time_of_last_stored_image_per_user = [time.time() for _ in range(self.n_users)]

        # Data about sent images
        self.slackbot_img_id_to_user_to_reaction = {}
        self.user_to_sent_local_img_ids = {}

        # The objects in our universe
        self.objects_filepath = objects_filepath
        if self.objects_filepath is None:
            self.objects = []
        else:
            with open(objects_filepath, 'r') as f:
                self.objects = json.load(f)

        # Self-lock to ensure this class is thread-safe.
        # TODO: This can be improved with a read-write lock.
        self.lock = threading.Lock()

    def get_new_local_img_id(self, img_cv2):
        """
        Gets a new local image ID
        """
        with self.lock:
            # return len(self.local_img_ids)
            return hashlib.sha1(bytearray(np.array(cv2.imencode('.jpg', img_cv2)[1]).tostring())).hexdigest()

    def add_image(self, local_img_id, img_cv2, img_vector, detected_objects_msg, users):
        """
        Add a local message that was sent to Slack users
        """
        with self.lock:
            if local_img_id in self.local_img_ids:
                rospy.logwarn("Overriding local_img_id %s" % local_img_id)

            self.image_loader.add_image(local_img_id, img_cv2, img_vector, detected_objects_msg)
            self.local_img_ids.add(local_img_id)

            for user in users:
                if user not in self.user_to_stored_local_img_ids:
                    self.user_to_stored_local_img_ids[user] = []
                self.user_to_stored_local_img_ids[user].append(local_img_id)
                self.time_of_last_stored_image_per_user[user] = time.time()

    def get_image(self, local_img_id, return_img_cv2=False, return_img_vector=True, return_detected_objects_dict=False):
        """
        Get a single image
        """
        img_cv2, img_vector, detected_objects_msg = self.image_loader.get_images([local_img_id], return_img_cv2, return_img_vector, return_detected_objects_dict)[0]
        retval = []
        if return_img_cv2:
            retval.append(img_cv2)
        if return_img_vector:
            retval.append(img_vector)
        if return_detected_objects_dict:
            retval.append(detected_objects_msg)
        return tuple(retval)

    def get_time_of_last_stored_image(self, user):
        """
        Gets the time of the last stored image
        """
        with self.lock:
            return self.time_of_last_stored_image_per_user[user]

    def get_stored_images_for_user(self, user, return_img_cv2=False, return_img_vector=True, return_detected_objects_dict=False):
        """
        Returns the messages and vectors for every stored image for the user.
        """
        img_cv2s = []
        img_vectors = []
        detected_objects_dicts = []
        local_img_ids = []
        if user in self.user_to_stored_local_img_ids:
            for local_img_id in self.user_to_stored_local_img_ids[user]:
                img_cv2, img_vector, detected_objects_dict = self.image_loader.get_images([local_img_id], return_img_cv2, return_img_vector, return_detected_objects_dict)[0]
                img_vectors.append(img_vector)
                img_cv2s.append(img_cv2)
                detected_objects_dicts.append(detected_objects_dict)
                local_img_ids.append(local_img_id)

        # Randomly sort the lists, so that tie-breaking is randomized
        final_img_cv2s = []
        final_img_vectors = []
        final_detected_objects_dicts = []
        final_local_img_ids = []
        indices = list(range(len(img_cv2s)))
        random.shuffle(indices)
        for i in indices:
            remove_img = False
            if return_img_cv2 and img_cv2s[i] is None:
                remove_img = True
            if return_img_vector and img_vectors[i] is None:
                remove_img = True
            if return_detected_objects_dict and detected_objects_dicts[i] is None:
                remove_img = True
            if not remove_img:
                final_img_cv2s.append(img_cv2s[i])
                final_img_vectors.append(img_vectors[i])
                final_detected_objects_dicts.append(detected_objects_dicts[i])
                final_local_img_ids.append(local_img_ids[i])

        retval = []
        if return_img_cv2:
            retval.append(final_img_cv2s)
        if return_img_vector:
            retval.append(final_img_vectors)
        if return_detected_objects_dict:
            retval.append(final_detected_objects_dicts)

        retval.append(final_local_img_ids)
        return tuple(retval)

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

    def get_most_recent_stored_and_sent_images(self, user, return_img_cv2=True, return_img_vector=True, return_detected_objects_dict=False):
        """
        Returns up to self.most_recent_images most recent img_msg and img_vector
        that were stored for user, and up to another self.most_recent_images
        that were sent to the user.
        """
        with self.lock:
            # No image has been sent to this user
            img_cv2s = []
            img_vectors = []
            detected_objects_dicts = []
            if user in self.user_to_stored_local_img_ids:
                for local_img_id in self.user_to_stored_local_img_ids[user][-self.most_recent_images:]: # self.user_to_sent_local_img_ids[user][-self.most_recent_images:]: #
                    img_cv2, img_vector, detected_objects_dict = self.image_loader.get_images([local_img_id], return_img_cv2, return_img_vector, return_detected_objects_dict)[0]
                    img_cv2s.append(img_cv2)
                    img_vectors.append(img_vector)
                    detected_objects_dicts.append(detected_objects_dict)
            if user in self.user_to_sent_local_img_ids:
                for local_img_id in self.user_to_sent_local_img_ids[user][-self.most_recent_images:]: # self.user_to_sent_local_img_ids[user][-self.most_recent_images:]: #
                    img_cv2, img_vector, detected_objects_dict = self.image_loader.get_images([local_img_id], return_img_cv2, return_img_vector, return_detected_objects_dict)[0]
                    img_cv2s.append(img_cv2)
                    img_vectors.append(img_vector)
                    detected_objects_dicts.append(detected_objects_dict)

            retval = []
            if return_img_cv2:
                retval.append(img_cv2s)
            if return_img_vector:
                retval.append(img_vectors)
            if return_detected_objects_dict:
                retval.append(detected_objects_dicts)

            return tuple(retval)

    def get_img_vectors_and_reactions(self, user):
        """
        Returns two lists: (1) a list of img_vectors that have been sent to user
        and they have responded to; (2) an equally-long list of user's responses.
        """
        img_vectors = []
        reactions = []
        rospy.logdebug("self.slackbot_img_id_to_user_to_reaction %s" % self.slackbot_img_id_to_user_to_reaction)
        for slackbot_img_id in self.slackbot_img_id_to_user_to_reaction:
            for user_temp in self.slackbot_img_id_to_user_to_reaction[slackbot_img_id]:
                if user_temp != user: continue
                local_img_id = self.slackbot_img_id_to_local_img_id[slackbot_img_id]
                img_vector = self.image_loader.get_images([local_img_id], return_img_vector=True)[0][1]
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
            if self.objects_filepath is not None:
                with open(self.objects_filepath, 'w') as f:
                    json.dump(self.objects, f, indent=4)

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
        retval.image_loader = None # don't save the cache

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
    def load(pkl_filepath, objects_filepath=None, n_users=1, user_to_learning_condition={0:1}):
        """
        Attempts to load the SentMessagesDatabase pickle stored at pkl_filepath.
        If this fails, initializes a new SentMessagesDatabase and returns it.
        """
        try:
            with open(pkl_filepath, "rb") as f:
                sent_messages_database = pickle.load(f)
                # If the n_users changed, we have to initialize the additional users
                num_new_users = n_users - sent_messages_database.n_users
                if num_new_users > 0:
                    sent_messages_database.n_users = n_users
                    for _ in range(num_new_users):
                        print("Adding a new user to sent_messages_database")
                        sent_messages_database.time_of_last_stored_image_per_user.append(time.time())
                print("sent_messages_database.slackbot_img_id_to_local_img_id", sent_messages_database.slackbot_img_id_to_local_img_id)
                sent_messages_database.lock = threading.Lock()
                sent_messages_database.image_loader = ImageLoader(cache_size=int(2.5*sent_messages_database.most_recent_images*sent_messages_database.n_users))
                return sent_messages_database
        except Exception as e:
            rospy.logwarn("Could not load pickled SentMessagesDatabase, initializing new one %s" % e)
            return SentMessagesDatabase(objects_filepath=objects_filepath, n_users=n_users, user_to_learning_condition=user_to_learning_condition)
