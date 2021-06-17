import math
import numpy as np
from scipy.optimize import minimize
from scipy.special import expit
from scipy.stats import multivariate_normal

class BayesianLogisticRegression(object):
    """
    Implements Bayesian Logistic Regression. Generally follows the formulations
    in the following two resources:
      [1] https://cedar.buffalo.edu/~srihari/CSE574/Chap4/4.5.1-BayesLogistic.pdf
      [2] https://arxiv.org/pdf/1906.08947.pdf
    """
    def __init__(self, prior_mean, prior_covariance,
        include_prior_in_posterior=True, default_variance=0.1):
        """
        Initializes the BayesianLogisticRegression class.

        Inputs:
          - prior_mean is an np array of size (d,)
          - prior_covariance is a PSD np array of size (d,d)
          - contexts is the history of contexts that have been seen. It is
            either None (no history), or an np array of size (n, d).
          - observations is the history of observations that have been seen. It
            is either None (no history), or an np array of size (n,)
          - include_prior_in_posterior specifies whether to use the prior
            distribution when computing the posterior or not. If True, the
            posterior is computed using calculations similar to [1]. If False,
            the posterior is computed using calculations similar to [2]. Note
            that one way of conceptualizing [2] is using a uniform prior.
          - default_variance is the value that will be placed on the diagonal
            of the covariance matrices when we add_dimensions. All other new
            values will be 0.
        """

        # Prior
        self.prior_mean = prior_mean
        self.prior_covariance = prior_covariance
        # Posterior
        self.posterior_mean = prior_mean
        self.posterior_covariance = prior_covariance

        self.include_prior_in_posterior = include_prior_in_posterior
        self.default_variance = default_variance

    def add_dimensions(self, n_new_dims, default_variance):
        """
        Add dimensions to the stored prior and posterior distributions. New
        values for the mean are initialized to 0. New values for the covariance
        are initialized to self.default_variance for diagonal elements and 0
        for non-diagonal elements.

        n_new_dims are the number of new dimensions to add
        """
        old_dims = self.prior_mean.shape[0]

        self.prior_mean = np.pad(self.prior_mean, (0, n_new_dims), 'constant', constant_values=0.0)
        self.prior_covariance = np.pad(self.prior_covariance, [(0, n_new_dims), (0, n_new_dims)], 'constant', constant_values=0.0)
        np.fill_diagonal(self.prior_covariance[old_dims:, old_dims:], default_variance*np.ones(n_new_dims))

        self.posterior_mean = np.pad(self.posterior_mean, (0, n_new_dims), 'constant', constant_values=0.0)
        self.posterior_covariance = np.pad(self.posterior_covariance, [(0, n_new_dims), (0, n_new_dims)], 'constant', constant_values=0.0)
        np.fill_diagonal(self.posterior_covariance[old_dims:, old_dims:], default_variance*np.ones(n_new_dims))

    def prior(self, theta):
        """
        The prior is a multivariate_normal distribuion with parameters
        self.prior_mean and self.prior_covariance.

        NOTE: This function is never called, and is included for clarity on the
        negative log and gradient computations below.
        """
        # return multivariate_normal.pdf(theta, mean=self.prior_mean, cov=self.prior_covariance)
        k = theta.shape[0]
        mean_diff = theta-self.prior_mean
        c0 = (2*math.pi)**(-k/2)*np.linalg.det(self.prior_covariance)**(-0.5)
        c1 = math.e**(-0.5*np.dot(mean_diff.T, np.dot(np.linalg.inv(self.prior_covariance), mean_diff)))
        return c0*c1

    def neg_log_prior(self, theta):
        """
        The negative logarithm of the prior.
        """
        k = theta.shape[0]
        mean_diff = theta-self.prior_mean
        c0 = (k/2)*math.log(2*math.pi) + 0.5*math.log(np.linalg.det(self.prior_covariance))
        c1 = 0.5*np.dot(mean_diff.T, np.dot(np.linalg.inv(self.prior_covariance), mean_diff))
        return c0 + c1

    def gradient_of_neg_log_prior(self, theta):
        """
        The gradient w.r.t. theta of the negative logarith of the prior.
        """
        mean_diff = theta-self.prior_mean
        cov_inv = np.linalg.inv(self.prior_covariance)
        return 0.5*np.dot(cov_inv+cov_inv.T, mean_diff)

    def likelihood(self, theta):
        """
        The likleihood function is a product of logistic functions, for every
        (context, observation) pair that has been observed.

        NOTE: This function is never called, and is included for clarity on the
        negative log and gradient computations below.
        """
        retval = 1.0
        for i in range(len(self.contexts)):
            prob = expit(np.dot(theta, self.contexts[i]))
            if observations[i]:
                retval *= prob
            else:
                retval *= 1.0-prob
        return retval

    def neg_log_likelihood(self, theta):
        """
        The negative logarithm of the likelihood function.
        """
        retval = 0.0
        for i in range(len(self.contexts)):
            observation = 1 if self.observations[i] else 0
            context = self.contexts[i]
            preference = np.dot(context, theta)
            retval += preference + math.log(1+math.e**(-1.0*preference)) - observation*preference
        return retval

    def gradient_of_neg_log_likelihood(self, theta):
        """
        The gradient of the negative logarithm of the likelihood function.
        """
        retval = np.zeros(theta.shape)
        for i in range(len(self.contexts)):
            observation = 1 if self.observations[i] else 0
            context = self.contexts[i]
            retval += (expit(np.dot(context, theta)) - observation)*context
        return retval

    def unnormalized_posterior(self, theta):
        """
        The unnormalized posterior is a product of the prior and likelihood
        functions.

        NOTE: This function is never called, and is included for clarity on the
        negative log and gradient computations below.
        """
        if self.include_prior_in_posterior:
            return self.prior(theta) * self.likelihood(theta)
        else:
            return self.likelihood(theta)

    def neg_log_posterior(self, theta):
        """
        The negative logarithm of the unnormalized posterior.
        """
        if self.include_prior_in_posterior:
            return self.neg_log_prior(theta) + self.neg_log_likelihood(theta)
        else:
            return self.neg_log_likelihood(theta)

    def gradient_of_neg_log_posterior(self, theta):
        """
        The gradient of the negative logarithm of the unnormalized posterior
        function.
        """
        if self.include_prior_in_posterior:
            return self.gradient_of_neg_log_prior(theta) + self.gradient_of_neg_log_likelihood(theta)
        else:
            return self.gradient_of_neg_log_likelihood(theta)

    def compute_posterior(self, contexts, observations):
        """
        Computes the normalized posterior distribution based on the contexts
        and observations.
        """

        self.contexts = contexts
        self.observations = observations

        # Minimize the negative logarith of the posterior function -- this will
        # be the mean for our approximation of the posterior
        optimize_result = minimize(
            self.neg_log_posterior,
            x0=self.posterior_mean, # warm-start the minimization with the old posterior mean
            jac=self.gradient_of_neg_log_posterior,
            method="BFGS",#"L-BFGS-B",#"SLSQP",#
            options={'gtol':1e-60},
        )
        self.posterior_mean = optimize_result.x

        # The posterior covariance, based on the Laplace Approximation
        covariance_sum = np.zeros(self.prior_covariance.shape)
        for context_past in self.contexts:
            prob = expit(np.dot(self.posterior_mean, context_past))
            covariance_sum += prob*(1.0-prob)*np.outer(context_past, context_past.T)
        if self.include_prior_in_posterior:
            self.posterior_covariance = np.linalg.inv(np.linalg.inv(self.prior_covariance) + covariance_sum)
        else:
            self.posterior_covariance = np.linalg.inv(covariance_sum)

    def get_probability(self, context, theta=None):
        """
        Returns the probability of observing a 1 on context. If theta is None,
        use the posterior districution for theta (using calculations in [1]).
        Else, assume theta is the true theta
        """
        if theta is None:
            mu = np.dot(self.posterior_mean.T, context)
            var = np.dot(context.T, np.dot(self.posterior_covariance, context))
            return expit((1+np.pi*var/8)**(-0.5)*mu)
        else:
            human_preference = np.dot(theta, context)
            return 1.0 / (1+math.exp(-1*human_preference))

    def get_mean(self):
        """
        Returns the latest posterior mean calculation
        """
        return self.posterior_mean

    def get_covariance(self):
        """
        Returns the latest posterior covariance calculation
        """
        return self.posterior_covariance
