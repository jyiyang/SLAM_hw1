import numpy as np
import math
import time
from matplotlib import pyplot as plt
from matplotlib import figure as fig

from scipy.stats import norm
import pdb

from MapReader import MapReader

radi = 0.01745329
scale = 10

class SensorModel:

    """
    References: Thrun, Sebastian, Wolfram Burgard, and Dieter Fox. Probabilistic robotics. MIT press, 2005.
    [Chapter 6.3]
    """

    def __init__(self, occupancy_map):

        self._sigma_hit = 50
        self._lambda_short = 0.1
        self._z_max = 8183;
        self._weight = [0.6,0.2,0.1,0.1]
        self._map = occupancy_map

        size = np.shape(occupancy_map)
        # print size
        self._table = [[[-1 for i in xrange(180)] for j in xrange(size[0])] for k in xrange(size[1])]
        # print len(self._table)
        # print len(self._table[0])
        # print len(self._table[0][0])
        # _weight is for weighted average of posterior

    def computeTable(self):
        # for i in xrange(self.)
        # self.ray_casting
        for x in xrange(500, 501):
            for y in xrange(100, 101):
                if math.fabs(self._map[y, x]) < 0.001:
                    for theta in xrange(len(self._table[x][y])):
                        self._table[x][y][theta] = self.ray_casting_table([x*10, y*10], 2*theta*radi)
                        print "Map value for location at x = ", scale*x, " y = ", scale*y, "theta = ", 2*theta*radi
                        print self._table[x][y][theta]
            print "Currently in outer loop: ", x

    def visualize_ray(self,x,y):
        mng = plt.get_current_fig_manager();
        plt.plot(x,y)
        plt.show()

    def ray_casting_table(self, x, theta):
        """
        n: ray number from RIGHT to LEFT
        """
        # x_occu = math.floor((x[0]-5)/10.0)
        # y_occu = math.floor((x[1]-5)/10.0)
        # theta = x[2]
        # phi = (n-90)*radi
        # R_r_l = np.matrix([[math.cos(phi),-math.sin(phi)],[math.sin(phi),math.cos(phi)]])
        # R_w_r = np.matrix([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
        # R_w_l = R_r_l*R_w_r
        # v = np.array([R_w_l.item(0),R_w_l.item(2)])
        # p0 = R_w_r*np.array([[25],[0]]) + np.array([[x[0]],[x[1]]])
        # p0 = np.transpose(p0)
        p0 = np.array([x])
        print p0
        v = np.array([[math.cos(theta)], [math.sin(theta)]])
        print v
        t = 0
        counter = 1
        p = p0 + t*v

        while counter < 4000:
            t = t + 3
            counter = counter + 1
            p = p0 + t*v
            px_occu = math.floor((p[0,0]-5)/10.0)
            py_occu = math.floor((p[0,1]-5)/10.0)
            # print px_occu
            # print py_occu
            if py_occu < 800 and px_occu < 800:
                occu_val = self._map[py_occu,px_occu]
            else:
                return self._z_max

            if occu_val > 0.1:
                dist = np.array([10*(px_occu-1)+5-x[0],10*(py_occu-1)+5-x[1]])
                # testx.append(px_occu)
                # testy.append(py_occu)
                # print counter
                # print px_occu, py_occu
                return np.linalg.norm(dist)#,testx,testy

        return -1

    def ray_casting(self, x, n):
        """
        n: ray number from RIGHT to LEFT
        """
        x_occu = math.floor((x[0]-5)/10.0)
        y_occu = math.floor((x[1]-5)/10.0)
        theta = x[2]
        phi = (n-90)*radi
        # print phi
        R_r_l = np.matrix([[math.cos(phi),-math.sin(phi)],[math.sin(phi),math.cos(phi)]])
        R_w_r = np.matrix([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
        R_w_l = R_r_l*R_w_r
        v = np.array([R_w_l.item(0),R_w_l.item(2)])
        p0 = R_w_r*np.array([[25],[0]]) + np.array([[x[0]],[x[1]]])
        p0 = np.transpose(p0)
        t = 0
        counter = 1
        testx = [math.floor((p0[0,0]-5)/10.0)]
        testy = [math.floor((p0[0,1]-5)/10.0)]
        while counter < 4000:
            t = t + 5
            counter = counter + 1
            p = p0 + t*v
            #print p
            px_occu = math.floor((p[0,0]-5)/10.0)
            py_occu = math.floor((p[0,1]-5)/10.0)
            # print px_occu
            # print py_occu
            if py_occu < 800 and px_occu < 800:
                occu_val = self._map[py_occu,px_occu]
            else:
                testx.append(px_occu)
                testy.append(py_occu)
                return self._z_max,testx,testy

            if occu_val > 0.1:
                dist = np.array([10*(px_occu-1)+5-x[0],10*(py_occu-1)+5-x[1]])
                testx.append(px_occu)
                testy.append(py_occu)
                return np.linalg.norm(dist),testx,testy

        return -1,[],[]

    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """
        q = 1;
        #q = 0;
        for i in xrange(1,181,10):
            z_t1 = z_t1_arr[i-1]
            z_k_opt = self.ray_casting(x_t1,i)
            # print z_k_opt
            if z_k_opt == -1:
                continue
            # 1. Hit model
            if z_t1 >= 0 and z_t1 <= self._z_max:
                p_hit = math.exp(-0.5*((z_t1-z_k_opt)**2)/(self._sigma_hit**2))/math.sqrt(2*math.pi*(self._sigma_hit**2))
                # print p_hit
            else:
                # print "Hit error"
                p_hit = 0

            # 2. Unexpected objects
            if z_t1 >= 0 and z_t1 <= z_k_opt:
                p_short = self._lambda_short*math.exp(-self._lambda_short*z_t1)/(1-math.exp(-self._lambda_short*z_k_opt))
                # print p_short
            else:
                # print "Short error"
                p_short = 0

            # 3. Failures
            if z_t1 == self._z_max:
                p_max = 1
            else:
                # print "Max error"
                p_max = 0

            #4. Random Measurements
            if z_t1 >= 0 and z_t1 <= self._z_max:
                p_rand = 1/float(self._z_max)
                # print p_rand
            else:
                # print "Random error"
                p_rand = 0;


            p_total = self._weight[0]*p_hit + self._weight[1]*p_short + self._weight[2]*p_max + self._weight[3]*p_rand

            q = q*p_total

        return q

if __name__=='__main__':
    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata1.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    # logfile = open(src_path_log, 'r')
    # fig = plt.figure()
    sensor_model = SensorModel(occupancy_map)
    sensor_model.computeTable()

    # fig = plt.figure()
    # mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    # plt.ion(); plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);
    # plt.pause(100)
    x = np.array([5000,1000,0])
    x_l = [];
    y_l = [];
    for i in range(1,181):
        print "Ray num: ", i
        test,testx,testy = sensor_model.ray_casting_table(x,i)
        print "d: ", test
        x_l.extend(testx)
        y_l.extend(testy)
        print test


    print sensor_model._map.shape
    fig = plt.figure()
    #plt.switch_backend('TkAgg')
    mng = plt.get_current_fig_manager(); mng.resize(*mng.window.maxsize())
    plt.ion();  plt.axis([0, 800, 0, 800]);
    plt.plot(testx,testy)
    plt.show()

    plt.imshow(sensor_model._map, cmap='Greys');
    plt.pause(100)
    pass
