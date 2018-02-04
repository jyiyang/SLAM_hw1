import numpy as np
import math
import time
from matplotlib import pyplot as plt
from matplotlib import figure as fig

from scipy.stats import norm
import pdb

from MapReader import MapReader

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):

        """
        TODO : Initialize Sensor Model parameters here
        """
        self._sigma_hit = 0.1
        self._lambda_short = 0.1
        self._z_max = 8183;
        self._weight = np.array([0.1,0.2,0.3,0.4])
        self._map = occupancy_map
        # _weight is for weighted average of posterior
    
    def ray_casting(self, x, n):
        """
        n: ray number from RIGHT to LEFT
        """
        x_occu = math.floor(x[0]/10.0)
        y_occu = math.floor(x[1]/10.0)
        theta = x[2]
        phi = n*math.pi/180
        T_r_l = np.matrix([[math.cos(phi),-math.sin(phi),25],[math.sin(phi),math.cos(phi),0],[0,0,1]])
        T_w_r = np.matrix([[math.cos(theta),-math.sin(theta),x[0]],[math.sin(theta),math.cos(theta),x[1]],[0,0,1]])
        T_w_l = T_r_l*T_w_r
        v = np.array([T_w_l.item(0),T_w_l.item(3)])
        p0 = np.array([T_w_l.item(2),T_w_l.item(5)])
        t = 0
        counter = 1
        # p = p0 + t*v
        testx = []
        testy = []
        prob = []
        while counter < 4000:
            t = t + 1
            counter = counter + 1
            p = p0 + t*v
            print p
            px_occu = math.floor(p[0]/10.0)
            py_occu = math.floor(p[1]/10.0)
            occu_val = self._map[py_occu,px_occu]
            testx.append(px_occu)
            testy.append(py_occu)
            if occu_val > 0.1:
                dist = np.array([10*(px_occu-1)+5-x[0],10*(py_occu-1)+5-x[1]])
                return np.linalg.norm(dist), testx, testy   









    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """
        
        """
        TODO : Add your code here
        """
        
        q = 1;
        #for i in range(1,181)



        return q    
 
if __name__=='__main__':
    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata1.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map() 
    logfile = open(src_path_log, 'r')

    sensor_model = SensorModel(occupancy_map)
    x = np.array([5000,1000,math.pi/2])
    n = 1
    test,testx,testy = sensor_model.ray_casting(x,n)
    print test
    print testx
    print testy
    fig = plt.figure()
    # plt.switch_backend('TkAgg')
    mng = plt.get_current_fig_manager(); mng.resize(*mng.window.maxsize())
    #plt.ion();  plt.axis([0, 800, 0, 800]); 
    #plt.draw()
    plt.plot(testx,testy);
    #plt.imshow(sensor_model._map, cmap='Greys');
    plt.show()
    #plt.pause(100)