import numpy as np
import pdb
import math
from matplotlib import pyplot as plt
from matplotlib import figure as fig
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
        self.flag = 1

    def multinomial_sampler(self, X_bar):

        """
        param[in] X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
        param[out] X_bar_resampled : [num_particles x 4] sized array containing [x, y, theta, wt] values for resampled set of particles
        """
        self.normalize(X_bar)
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
        if math.fabs(np.sum(X_bar[:,3])-1) > 0.1:
            X_bar = self.normalize(X_bar)
        num_particles = np.shape(X_bar)[0]
        X_bar_resampled = np.zeros([num_particles, 4])

        r = (1.0/num_particles) * np.random.rand()
        c = X_bar[0, 3]

        i = 0
        for m in xrange(1,num_particles+1):
            U = r + (m - 1) * (1.0/num_particles)
            while U > c:
                i += 1
                c += X_bar[i, 3]
            X_bar_resampled[m-1, :] = X_bar[i, :]

        return X_bar_resampled

    def normalize(self, X_bar):
        normalized_factor = np.sum(X_bar[:, 3])
        num_particles = np.shape(X_bar)[0]
        tmp = X_bar[:,3]/normalized_factor
        if self.flag==1:

            fig,axes = plt.subplots(nrows=2, ncols=1)
            ax0,ax1 = axes.flatten()
            ax0.hist(X_bar[:,3])
            ax1.hist(tmp)
            plt.show()

        for i in xrange(num_particles):
            X_bar[i, 3] = X_bar[i, 3] / normalized_factor
        return X_bar

        # num_particles = np.shape(X_bar)[0]
        # norm_min = np.amin(X_bar[:,3])
        # norm_max = np.amax(X_bar[:,3])
        # #print norm_max,norm_min
        # for i in xrange(num_particles):
        #     X_bar[i,3] = (X_bar[i,3]-norm_min)/(norm_max-norm_min)

        # return X_bar

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
    # testSampler()
    pass
