import numpy as np
import multiprocessing as mp
import sys
import pdb
import math
import pickle

from MapReader import MapReader
from MotionModel import MotionModel
from SensorModel import SensorModel
from Resampling import Resampling

from matplotlib import pyplot as plt
from matplotlib import figure as fig
import time

def visualize_map(occupancy_map):
    fig = plt.figure()
    # plt.switch_backend('TkAgg')
    mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    plt.ion(); plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);


def visualize_timestep(X_bar, tstep):
    x_locs = X_bar[:,0]/10.0
    y_locs = X_bar[:,1]/10.0
    scat = plt.scatter(x_locs, y_locs, c='r', marker='o')
    plt.pause(0.00001)
    scat.remove()

def init_particles_random(num_particles, occupancy_map):

    # initialize [x, y, theta] positions in world_frame for all particles
    # (randomly across the map)
    y0_vals = np.random.uniform( 0, 7000, (num_particles, 1) )
    x0_vals = np.random.uniform( 3000, 7000, (num_particles, 1) )
    theta0_vals = np.random.uniform( -3.14, 3.14, (num_particles, 1) )

    # initialize weights for all particles
    w0_vals = np.ones( (num_particles,1), dtype=np.float64)
    w0_vals = w0_vals / num_particles

    X_bar_init = np.hstack((x0_vals,y0_vals,theta0_vals,w0_vals))

    return X_bar_init

def init_particles_freespace(num_particles, occupancy_map):

    # initialize [x, y, theta] positions in world_frame for all particles
    # (in free space areas of the map)
    w0_val = 1.0 / num_particles
    X_bar_init = np.zeros([num_particles, 4])

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

def visualize_odometry(odom):

    fig = plt.figure()
    plt.plot(odom[:, 0], odom[:, 1])
    plt.show()

def visualize_ray(x,y):
    # if len(x)!=0:
    #     scat = plt.plot(x,y, c='b')
    #     plt.pause(0.00001)
    #     #del(scat)
    #     scat.remove(scat[0])
    #     del scat
    if len(x)!=0:

        scat = plt.scatter(x, y, c='b', marker='o')
        plt.pause(0.00001)
        scat.remove()


def main(mode):

    """
    Description of variables used
    u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]
    u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
    x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
    x_t1 : particle state belief [x, y, theta] at time t [world_frame]
    X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
    z_t : array of 180 range measurements for each laser scan
    """

    """
    Initialize Parameters
    """
    x_l = []
    y_l = []
    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata1.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    logfile = open(src_path_log, 'r')

    motion_model = MotionModel()
    sensor_model = SensorModel(occupancy_map)
    if mode == 0:
        sensor_model.readTable()
    elif mode == 1:
        pass
    elif mode == 2:
        sensor_model.computeTable()

    resampler = Resampling()

    num_particles = 8000
    # print occupancy_map
    X_bar = init_particles_freespace(num_particles, occupancy_map)

    vis_flag = 1
    odom = np.zeros((2218, 2))
    """
    Monte Carlo Localization Algorithm : Main Loop
    """
    if vis_flag:
        visualize_map(occupancy_map)

    first_time_idx = True

    numSteps = 1

    for time_idx, line in enumerate(logfile):
        # Read a single 'line' from the log file (can be either odometry or laser measurement)
        meas_type = line[0] # L : laser scan measurement, O : odometry measurement
        meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ') # convert measurement values from string to double

        odometry_robot = meas_vals[0:3] # odometry reading [x, y, theta] in odometry frame
        # print odometry_robot
        time_stamp = meas_vals[-1]

        # if ((time_stamp <= 0.0) | (meas_type == "O")): # ignore pure odometry measurements for now (faster debugging)
            # continue

        if (meas_type == "L"):
             odometry_laser = meas_vals[3:6] # [x, y, theta] coordinates of laser in odometry frame
             ranges = meas_vals[6:-1] # 180 range measurement values from single laser scan

        print "Processing time step " + str(time_idx) + " at time " + str(time_stamp) + "s"

        if (first_time_idx):
            u_t0 = odometry_robot
            first_time_idx = False
            odom[0, :] = u_t0[0:2]
            continue
        X_bar_new = np.zeros( (num_particles,4), dtype=np.float64)
        u_t1 = odometry_robot
        odom[numSteps, :] = u_t0[0:2]
        numSteps += 1
        # print u_t1
        for m in xrange(0, num_particles):

            """
            MOTION MODEL
            """
            x_t0 = X_bar[m, 0:3]
            x_t1 = motion_model.update(u_t0, u_t1, x_t0)
            """
            SENSOR MODEL
            """
            if (meas_type == "L"):
                z_t = ranges
                w_t,x_l,y_l = sensor_model.beam_range_finder_model(z_t, x_t1)
                #w_t = sensor_model.beam_range_finder_model(z_t, x_t1)
                #w_t = 1/num_particles
                X_bar_new[m,:] = np.hstack((x_t1, w_t))
            else:
                X_bar_new[m,:] = np.hstack((x_t1, X_bar[m,3]))

        X_bar = X_bar_new
        u_t0 = u_t1

        """
        RESAMPLING
        """
        X_bar = resampler.low_variance_sampler(X_bar)

        if vis_flag:
            visualize_timestep(X_bar, time_idx)
            #visualize_ray(x_l,y_l)


    #visualize_odometry(odom)


def test():

    """
    Description of variables used
    u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]
    u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
    x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
    x_t1 : particle state belief [x, y, theta] at time t [world_frame]
    X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
    z_t : array of 180 range measurements for each laser scan
    """

    """
    Initialize Parameters
    """
    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata1.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    logfile = open(src_path_log, 'r')

    motion_model = MotionModel()
    sensor_model = SensorModel(occupancy_map)
    resampler = Resampling()

    num_particles = 1
    # print occupancy_map
    #X_bar = init_particles_freespace(num_particles, occupancy_map)
    X_bar = np.zeros([1,4])
    X_bar[0,:] = np.array([6000,1700,0,1])
    print X_bar
    vis_flag = 1
    ray_flag = 0
    odom = np.zeros((2218, 2))
    """
    Monte Carlo Localization Algorithm : Main Loop
    """
    if vis_flag:
        visualize_map(occupancy_map)

    first_time_idx = True

    numSteps = 1

    for time_idx, line in enumerate(logfile):

        # Read a single 'line' from the log file (can be either odometry or laser measurement)
        meas_type = line[0] # L : laser scan measurement, O : odometry measurement
        meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ') # convert measurement values from string to double

        odometry_robot = meas_vals[0:3] # odometry reading [x, y, theta] in odometry frame
        # print odometry_robot
        time_stamp = meas_vals[-1]

        # if ((time_stamp <= 0.0) | (meas_type == "O")): # ignore pure odometry measurements for now (faster debugging)
            # continue

        if (meas_type == "L"):
             odometry_laser = meas_vals[3:6] # [x, y, theta] coordinates of laser in odometry frame
             ranges = meas_vals[6:-1] # 180 range measurement values from single laser scan

        print "Processing time step " + str(time_idx) + " at time " + str(time_stamp) + "s"

        if (first_time_idx):
            u_t0 = odometry_robot
            first_time_idx = False
            odom[0, :] = u_t0[0:2]
            continue

        X_bar_new = np.zeros( (num_particles,4), dtype=np.float64)
        u_t1 = odometry_robot
        odom[numSteps, :] = u_t0[0:2]
        numSteps += 1
        for m in xrange(0, num_particles):

            """
            MOTION MODEL
            """
            x_t0 = X_bar[m, 0:3]
            x_t1 = motion_model.update(u_t0, u_t1, x_t0)
            """
            SENSOR MODEL
            """
            if (meas_type == "L"):
                ray_flag = 1
                z_t = ranges
                w_t,x_l,y_l = sensor_model.beam_range_finder_model(z_t, x_t1)

                # w_t = 1/num_particles
                X_bar_new[m,:] = np.hstack((x_t1, w_t))
            else:
                X_bar_new[m,:] = np.hstack((x_t1, X_bar[m,3]))

        X_bar = X_bar_new
        u_t0 = u_t1

        """
        RESAMPLING
        """
        #X_bar = resampler.low_variance_sampler(X_bar)

        if vis_flag:
            visualize_timestep(X_bar, time_idx)
        if ray_flag:
            #visualize_ray(x_l,y_l)
            pass


def motion_sensor_model(u_t0, u_t1, X_bar, m, meas_type, ranges,  motion_model, sensor_model, output):
    x_t0 = X_bar[m, 0:3]
    x_t1 = motion_model.update(u_t0, u_t1, x_t0)
    if (meas_type == "L"):
        z_t = ranges
        w_t,x_l,y_l = sensor_model.beam_range_finder_model(z_t, x_t1)
                # w_t = 1/num_particles
        result = np.hstack((x_t1, w_t))
    else:
        result = np.hstack((x_t1, X_bar[m,3]))
    output.put((m,result))


def parallel_main():
    """
    Description of variables used
    u_t0 : particle state odometry reading [x, y, theta] at time (t-1) [odometry_frame]
    u_t1 : particle state odometry reading [x, y, theta] at time t [odometry_frame]
    x_t0 : particle state belief [x, y, theta] at time (t-1) [world_frame]
    x_t1 : particle state belief [x, y, theta] at time t [world_frame]
    X_bar : [num_particles x 4] sized array containing [x, y, theta, wt] values for all particles
    z_t : array of 180 range measurements for each laser scan
    """

    """
    Initialize Parameters
    """
    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata1.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    logfile = open(src_path_log, 'r')

    motion_model = MotionModel()
    sensor_model = SensorModel(occupancy_map)
    resampler = Resampling()

    num_particles = 500
    # print occupancy_map
    X_bar = init_particles_freespace(num_particles, occupancy_map)

    vis_flag = 1
    odom = np.zeros((2218, 2))
    """
    Monte Carlo Localization Algorithm : Main Loop
    """
    if vis_flag:
        visualize_map(occupancy_map)

    first_time_idx = True

    numSteps = 1

    for time_idx, line in enumerate(logfile):
        # Read a single 'line' from the log file (can be either odometry or laser measurement)
        meas_type = line[0] # L : laser scan measurement, O : odometry measurement
        meas_vals = np.fromstring(line[2:], dtype=np.float64, sep=' ') # convert measurement values from string to double

        odometry_robot = meas_vals[0:3] # odometry reading [x, y, theta] in odometry frame
        # print odometry_robot
        time_stamp = meas_vals[-1]

        # if ((time_stamp <= 0.0) | (meas_type == "O")): # ignore pure odometry measurements for now (faster debugging)
            # continue

        if (meas_type == "L"):
             odometry_laser = meas_vals[3:6] # [x, y, theta] coordinates of laser in odometry frame
             ranges = meas_vals[6:-1] # 180 range measurement values from single laser scan

        print "Processing time step " + str(time_idx) + " at time " + str(time_stamp) + "s"

        if (first_time_idx):
            u_t0 = odometry_robot
            first_time_idx = False
            odom[0, :] = u_t0[0:2]
            continue

        X_bar_new = np.zeros( (num_particles,4), dtype=np.float64)
        u_t1 = odometry_robot
        odom[numSteps, :] = u_t0[0:2]
        numSteps += 1
        output = mp.Queue()
        processes = [mp.Process(target=motion_sensor_model, args=(u_t0, u_t1, X_bar, m, meas_type, ranges, motion_model, sensor_model, output)) for m in range(0,num_particles)]

        for p in processes:
            p.start()

        for p in processes:
            p.join()

        results = [output.get() for p in processes]
        results = [r[1] for r in results]
        for m in range(0,num_particles):
            X_bar_new[m,:] = results[m]
        X_bar = X_bar_new
        u_t0 = u_t1

        """
        RESAMPLING
        """
        X_bar = resampler.low_variance_sampler(X_bar)

        if vis_flag:
            visualize_timestep(X_bar, time_idx)



    visualize_odometry(odom)

if __name__=="__main__":
    # To run the program with different modes:
    # 0: read a map
    # 1: do real time ray casting
    # 2: to compute and store a map
    mode = int(sys.argv[1])
    main(mode)
    #test()
    # parallel_main()
