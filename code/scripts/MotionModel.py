import sys
import numpy as np
import math

class MotionModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 5]
    """

    def __init__(self):

        """
        TODO : Initialize Motion Model parameters here
        """
        self.alpha1 = 1
        self.alpha2 = 1
        self.alpha3 = 1
        self.alpha4 = 1

        # self.alpha1 = 0
        # self.alpha2 = 0
        # self.alpha3 = 0
        # self.alpha4 = 0


    def update(self, u_t0, u_t1, x_t0):
        """
        param[in] u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]
        param[in] u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
        param[in] x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
        param[out] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        """

        delta_rot1 = math.atan2(u_t1[1] - u_t0[1], u_t1[0] - u_t0[0]) - u_t0[2]
        delta_trans = math.sqrt(math.pow(u_t0[0] - u_t1[0], 2) + math.pow(u_t0[1] - u_t1[1], 2))
        delta_rot2 = u_t1[2] - u_t0[2] - delta_rot1

        noise_var_rot1 = self.alpha1 * math.fabs(delta_rot1) + self.alpha2 * delta_trans
        noise_var_trans = self.alpha3 * delta_trans + self.alpha4 * (math.fabs(delta_rot1) + math.fabs(delta_rot2))
        noise_var_rot2 = self.alpha1 * math.fabs(delta_rot2) + self.alpha2 * delta_trans

        delta_rot1_hat = delta_rot1 - self.sample(noise_var_rot1)
        delta_trans_hat = delta_trans - self.sample(noise_var_trans)
        delta_rot2_hat = delta_rot2 - self.sample(noise_var_rot2)

        x_prime = x_t0[0] + delta_trans_hat * math.cos(x_t0[2] + delta_rot1_hat)
        y_prime = x_t0[1] + delta_trans_hat * math.sin(x_t0[2] + delta_rot1_hat)
        theta_prime = x_t0[2] + delta_rot1_hat + delta_rot2_hat

        x_t1 = [x_prime, y_prime, theta_prime]
        return x_t1

    def sample(self, noise_var):
        """
        Sample a noise value from a given distribution parameter.
        The value is drawn from a normal distribution with zero mean.
        """

        return np.sum(noise_var * (np.random.rand(1, 12) - 0.5)) / 2

if __name__=="__main__":
    pass
