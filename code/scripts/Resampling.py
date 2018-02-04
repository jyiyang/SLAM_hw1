import numpy as np
import pdb
import math

from MapReader import MapReader

class Resampling:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 4.3]
    """

    def __init__(self):
        """
        TODO : Initialize resampling process parameters here
        """

    def multinomial_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """
        num_particles = np.shape(X_bar)[0]
        X_bar_resampled = np.zeros([num_particles, 4])
        for i in xrange(num_particles):
            x_bar_index = np.random.choice(num_particles, p=X_bar[:, 3])
            X_bar_resampled[i, :] = X_bar[x_bar_index, :]

        return X_bar_resampled

    def low_variance_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """
        num_particles = np.shape(X_bar)[0]
        X_bar_resampled = np.zeros([num_particles, 4])

        r = (1.0/num_particles) * np.random.rand()
        c = X_bar[0, 3]

        i = 1
        for m in xrange(1, num_particles):
            U = r + (m - 1) * (1.0/num_particles)
            while U > c:
                i += 1
                c += X_bar[i, 3]
            X_bar_resampled[m, :] = X_bar[i, :]

        return X_bar_resampled

def init_particles_freespace(num_particles, occupancy_map):

    # initialize [x, y, theta] positions in world_frame for all particles
    # (in free space areas of the map)
    w0_val = 1.0 / num_particles
    X_bar_init = np.zeros([500, 4])

    i = 0
    while i < num_particles:
        y0_val = np.random.uniform(0, 7000)
        x0_val = np.random.uniform(3000, 7000)
        theta0_val = np.random.uniform(-3.1415, 3.1415)

        occupied = occupancy_map[math.floor(y0_val / 10.0), math.floor(x0_val / 10.0)]
        # print type(occupied)
        if math.fabs(occupied) < 1e-3:
            X_bar_init[i, :] = np.array([x0_val, y0_val, theta0_val, w0_val])
            i += 1

    return X_bar_init

def testSampler():
    num_particles = 500
    src_path_map = '../data/map/wean.dat'
    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()

    X_bar = init_particles_freespace(num_particles, occupancy_map)
    resampler = Resampling()
    # X_bar_resampled = resampler.multinomial_sampler(X_bar)
    X_bar_resampled = resampler.low_variance_sampler(X_bar)
    print X_bar_resampled

if __name__ == "__main__":
    testSampler()
