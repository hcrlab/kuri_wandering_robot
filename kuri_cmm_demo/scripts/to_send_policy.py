from bayesian_logistic_regression import BayesianLogisticRegression
from kuri_cmm_demo.msg import DetectedObjects
import math
import numpy as np
import os
import random
import rospy
from scipy.special import expit
from scipy.stats import multivariate_normal
import time

class ToSendPolicy(object):
    """
    The ToSendPolicy class takes in an image messae, vectorizes it, and then
    determines whether or not to send it. It does so my maintaining a belief
    over the human preference for each human. It then takes in human
    responses to the sent images, and updates the belief accordingly.
    """
    def __init__(self, sent_messages_database, classes_cache_filepath=None,
        human_prior_filepath=None, n_users=1, default_variance=0.1,
        user_to_learning_condition={0:1}, not_learning_condition_probability=0.005):
        """
        Initialize the an instance of the ToSendPolicy class.
        """
        # Database Parameters
        self.sent_messages_database = sent_messages_database
        self.n_objects = self.sent_messages_database.get_num_objects()

        # Load the human preference priors
        self.human_prior_filepath = human_prior_filepath
        self.n_users = n_users
        self.user_to_learning_condition = user_to_learning_condition
        self.not_learning_condition_probability = not_learning_condition_probability
        self.default_variance = default_variance
        self.load_human_preferences()

        # Parameters for the reward function
        self.human_like_reward = 1
        self.human_dislike_reward = -1
        self.human_send_penalty = 0

    def load_human_preferences(self):
        """
        Loads the human preferences
        """
        self.beliefs = []

        if self.human_prior_filepath is not None and os.path.isfile(self.human_prior_filepath):
            prior = np.load(self.human_prior_filepath)
            prior_mean = prior['prior_mean']
            prior_covariance = prior['prior_covariance']
        else:
            prior_mean = np.zeros(self.n_objects+1)
            prior_covariance = np.zeros((self.n_objects+1, self.n_objects+1))
            np.fill_diagonal(prior_covariance, [self.default_variance for _ in range(self.n_objects+1)])

        for human_i in range(self.n_users):
            if self.user_to_learning_condition[human_i] == 1:
                self.beliefs.append(BayesianLogisticRegression(prior_mean, prior_covariance))
                # Pad the prior mean and covariance based on the number of objects
                if self.n_objects+1 > prior_mean.shape[0]:
                    num_new_objects = self.n_objects+1-prior_mean.shape[0]
                    self.beliefs[-1].add_dimensions(num_new_objects, self.default_variance)
                elif self.n_objects+1 < prior_mean.shape[0]:
                    rospy.logwarn("Error on load_human_preferences, prior size %d is greater than num objects %d" % (prior_mean.shape[0], self.n_objects+1))
            else:
                self.beliefs.append(None)


        # Recompute the posterior from any past reactions that have been received.
        for human_i in range(self.n_users):
            self.got_reaction(human_i)

    def vectorize(self, detected_objects_msg):
        """
        Convert a detected_objects_msg to an image vector
        """
        img_vector = [0.0 for i in range(self.n_objects)]
        num_new_objects = 0
        for object in detected_objects_msg.objects:
            object_name = object.object_name.lower()
            object_i, was_added = self.sent_messages_database.get_object_i(object_name)
            if was_added:
                self.n_objects += 1
                img_vector.append(0.0)
                num_new_objects += 1
            confidence = object.confidence/100.0
            img_vector[object_i] = confidence

        # Pad stored vectors based on the new dimensionality
        if num_new_objects > 0:
            rospy.logdebug("Added %d new objects" % (num_new_objects))
            for human_i in range(self.n_users):
                if self.user_to_learning_condition[human_i] == 1:
                    self.beliefs[human_i].add_dimensions(num_new_objects, self.default_variance)

        return np.array(img_vector)

    @staticmethod
    def image_to_context(img_vector):
        """
        Converts an img_vector to a context vector that is used by the
        belief (BayesianLogisticRegression). It does so by prepending
        a 1 to the img_vector, to add an intercept term.
        """
        return np.insert(img_vector, 0, 1., axis=0)

    @staticmethod
    def multivariate_normal_sample(local_mean, local_covariance, permutation,
        partition_point, reduce_dimensionality=True):
        """
        First applies the permutation to the mean and covariance. Then, applies
        the algorithm at the below link, where partition_point elements are
        included in the first partition.

        https://en.wikipedia.org/wiki/Multivariate_normal_distribution#Conditional_distributions

        In practice, this should be called such that permutation applied to
        context results in all the 0 elements being after all the non-zero
        elements. Further, partition_point should be the number of non-zero
        elements.
        """
        if reduce_dimensionality:
            inv_permutation = np.argsort(permutation)

            mean_permuted = local_mean[permutation]
            covariance_permuted = local_covariance[:,permutation][permutation,:]

            mean_1 = mean_permuted[:partition_point]
            mean_2 = mean_permuted[partition_point:]

            cov_11 = covariance_permuted[:partition_point,:partition_point]
            cov_12 = covariance_permuted[:partition_point,partition_point:]
            cov_21 = covariance_permuted[partition_point:,:partition_point]
            cov_22 = covariance_permuted[partition_point:,partition_point:]

            mean_conditional = mean_1 #+ np.dot(cov_12, np.dot(np.linalg.inv(cov_22), (mean_2 - mean_2)))
            covariance_conditional = cov_11 - np.dot(cov_12, np.dot(np.linalg.inv(cov_22), cov_21))

            sampled_theta_1 = np.random.multivariate_normal(mean=mean_conditional, cov=covariance_conditional)
            sampled_theta_permuted = np.concatenate([sampled_theta_1, mean_2])
            sampled_theta = sampled_theta_permuted[inv_permutation]
        else:
            # sampled_theta = multivariate_normal.rvs(mean=local_mean, cov=local_covariance)
            sampled_theta = np.random.multivariate_normal(mean=local_mean, cov=local_covariance)
        return sampled_theta

    def to_send_policy(self, img_vector):
        """
        Returns a boolean vector of size self.n_users to indicate whether or
        not to send an image to each human
        """
        # Add an intercept term to the image
        context = ToSendPolicy.image_to_context(img_vector)

        # To make the computation more efficient, we will first permute
        # theta so that all the elements with a context of 0 are at the end.
        # We will then simplify the multivariate sampling problem by taking
        # cross-sections of the distribution (at the mean) for every element
        # where the context is 0, and then sampling only for the elements
        # where context is not 0. We will then reconstruct the sampled theta.
        permutation = np.append(np.argwhere(context!=0), np.argwhere(context==0))
        len_nonzero = np.count_nonzero(context)

        # Determine which human(s) to send it to
        to_send = np.zeros((self.n_users,), dtype=np.bool)
        for human_i in range(self.n_users):
            if self.user_to_learning_condition[human_i] == 1:
                # Sample a human preference vector
                mean = self.beliefs[human_i].get_mean()
                cov = self.beliefs[human_i].get_covariance()
                sampled_theta = ToSendPolicy.multivariate_normal_sample(mean, cov, permutation, len_nonzero)
                # Behave optimally with regards to sampled_theta
                human_preference = np.dot(sampled_theta, context)
                probability_of_liking = 1.0 / (1+math.exp(-1*human_preference))
                if probability_of_liking > (self.human_send_penalty - self.human_dislike_reward)/(self.human_like_reward - self.human_dislike_reward):
                    to_send[human_i] = True
            else:
                if random.random() <= self.not_learning_condition_probability:
                    to_send[human_i] = True

        return to_send

    def got_reaction(self, user):
        """
        This function is invoked when user has reacted to a previously-unreacted
        message. It queries sent_messages_database for all the img_vectors and
        observations from the user and then computers a new posterior for that
        user.
        """
        img_vectors, reactions = self.sent_messages_database.get_img_vectors_and_reactions(user)
        n_obs = len(img_vectors)

        observations = np.array(reactions)
        # This type of construction for contexts is necessary to pad vectors with
        # 0 for classes that weren't in the universe when the vectors were created
        contexts = np.zeros((n_obs, self.n_objects+1), dtype=np.float64)
        contexts[:,0] = 1.0 # Add the intercept
        for i in range(len(img_vectors)):
            img_vector = img_vectors[i]
            contexts[i,1:1+len(img_vector)] = img_vector

        # Update the belief
        if self.user_to_learning_condition[user] == 1:
            self.beliefs[user].compute_posterior(contexts, observations)
            rospy.loginfo("Recomputed posterior for user %d" % user)

    def get_probability(self, user, img_vector):
        """
        Return the probability that user will like img_vector.
        """
        # Add an intercept term to the image
        if self.user_to_learning_condition[user] == 1:
            context = ToSendPolicy.image_to_context(img_vector)
            return self.beliefs[user].get_probability(context)
        else:
            return 0.5
