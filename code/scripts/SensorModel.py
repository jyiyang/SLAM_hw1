import numpy as np
import math
import time
import pickle
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

        self._sigma_hit = 60
        self._lambda_short = 10
        self._z_max = 8183;
        self._weight = [0.8,0.6,0.1,0.1]
        self._map = occupancy_map

        size = np.shape(occupancy_map)
        # print size
        #self._table = [[[0 for i in xrange(180)] for j in xrange(size[0])] for k in xrange(size[1])]
        # print len(self._table)
        # print len(self._table[0])
        # print len(self._table[0][0])
        # _weight is for weighted average of posterior

    def computeTable(self):
        # for i in xrange(self.)
        # self.ray_casting
        for x in xrange(300, 700):
            for y in xrange(0, 700):
                if math.fabs(self._map[y, x]) < 0.001:
                    for theta in xrange(len(self._table[x][y])):
                        self._table[x][y][theta] = self.ray_casting_table(x*10, y*10, 2*theta*radi)
                        # print "Map value for location at x = ", scale*x, " y = ", scale*y, "theta = ", 2*theta*radi
                        # print self._table[x][y][theta]
            print "Currently in outer loop: ", x
        with open('ray_cast_table.dat', 'wb') as fp:
            pickle.dump(self._table, fp)

    def readTable(self):
        print "Now reading the table from file..."
        start = time.time()
        with open("ray_cast_table.dat", "rb") as fp:
            self._table = pickle.load(fp)
        end = time.time()
        print "Finished reading ray casting table in ", end - start, " seconds."

    def visualize_ray(self,x,y):
        mng = plt.get_current_fig_manager();
        plt.plot(x,y)
        plt.show()

    def ray_casting_table(self, x, y, theta):
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
        p0 = np.array([x, y])
        # print "p0: ", p0
        v = np.array([math.cos(theta), math.sin(theta)])

        t = 0
        counter = 1
        # p = p0 + t*v
        # print t*v

        while counter < 4000:
            t = t + 5
            counter = counter + 1
            p = p0 + t*v
            # print "New Position: ", p
            # px_occu = math.ceil((p[0,0]-5.0)/10.0)
            # py_occu = math.ceil((p[0,1]-5.0)/10.0)
            # print p
            # print p[0, 0]
            # print p[0, 1]
            px_occu = math.ceil(p[0] / 10.0)
            py_occu = math.ceil(p[1] / 10.0)
            # print px_occu
            # print py_occu
            if py_occu < 800 and px_occu < 800 and py_occu >= 0 and px_occu >= 0:
                occu_val = self._map[py_occu, px_occu]
            else:
                # print "==============gg============"
                # testx.append(px_occu)
                # testy.append(py_occu)
                return self._z_max#,testx,testy

            if occu_val > 0.1:
                dist = np.array([10*(px_occu-1)+5-x,10*(py_occu-1)+5-y])
                # print dist
                # testx.append(px_occu)
                # testy.append(py_occu)
                # print counter
                # print px_occu, py_occu
                return np.linalg.norm(dist)#,testx,testy
        return -1#,[],[]

    def ray_casting(self, x, n):
        """
        n: ray number from RIGHT to LEFT
        """
        theta = x[2]
        phi = (n-90)*math.pi/180
        R_r_l = np.matrix([[math.cos(phi),-math.sin(phi)],[math.sin(phi),math.cos(phi)]])
        R_w_r = np.matrix([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
        R_w_l = R_r_l*R_w_r
        v = np.array([R_w_l.item(0),R_w_l.item(2)])
        p0 = R_w_r*np.array([[25],[0]]) + np.array([[x[0]],[x[1]]])
        p0 = np.transpose(p0)
        t = 0
        counter = 1
        # p = p0 + t*v
        # testx = [math.ceil(p0[0,0]/10.0)]
        # testy = [math.ceil(p0[0,1]/10.0)]
        testx = []
        testy = []
        while counter < 1200:
            t = t + 5
            counter = counter + 1
            p = p0 + t*v
            #print p
            px_occu = math.ceil(p[0,0]/10.0)
            py_occu = math.ceil(p[0,1]/10.0)
            # print px_occu
            # print py_occu
            if py_occu < 800 and px_occu < 800 and py_occu > 0 and px_occu > 0:
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

    def norm_angle(self, angle):
    	"""normalize an angle to be within 0 to 2*pi"""

    	while (angle >= 2*math.pi):
    		angle -= 2*math.pi
    	while (angle < 0):
    		angle += 2*math.pi

    	return angle

    def get_range_from_table(self, x, n):
        theta = x[2]
        phi = (n-90)*math.pi/180
        # R_r_l = np.matrix([[math.cos(phi),-math.sin(phi)],[math.sin(phi),math.cos(phi)]])
        R_w_r = np.matrix([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
        # R_w_l = R_r_l*R_w_r
        # v = np.array([R_w_l.item(0),R_w_l.item(2)])
        R_r_w = np.transpose(R_w_r)
        #p0 = R_r_w*np.array([[25],[0]]) + np.array([[x[0]],[x[1]]])
        
        #p0 = np.transpose(p0)

        p0 = np.array([[x[0]],[x[1]]])
        p0 = np.transpose(p0)

        angle = self.norm_angle(theta + phi)
        angle_ind = int(math.ceil((angle) * 180.0 / math.pi / 2.0)) - 1
        x_ind = int(math.ceil(p0[0,0]/10.0))
        y_ind = int(math.ceil(p0[0,1]/10.0))

        # print x_ind, y_ind, angle_ind

        dist = self._table[y_ind][x_ind][angle_ind]

        return dist

    def beam_range_finder_model(self, z_t1_arr, x_t1):
        """
        param[in] z_t1_arr : laser range readings [array of 180 values] at time t
        param[in] x_t1 : particle state belief [x, y, theta] at time t [world_frame]
        param[out] prob_zt1 : likelihood of a range scan zt1 at time t
        """
        q = 0;
        x_l = [];
        y_l = [];

        for i in xrange(1,181,5):
            z_t1 = z_t1_arr[i-1]
            z_k_opt,x_ray,y_ray = self.ray_casting(x_t1,i)
            #z_k_opt = self.get_range_from_table(x_t1,i-1)

            # print "data: ",z_t1
            # print "measure: ",z_k_opt
            if z_k_opt == -1:
                continue
            x_l.extend(x_ray)
            y_l.extend(y_ray)
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
            # print p_total
            q = q + math.log(p_total)
            #q = q*p_total
        return q,x_l,y_l

if __name__=='__main__':
    src_path_map = '../data/map/wean.dat'
    src_path_log = '../data/log/robotdata1.log'

    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    # logfile = open(src_path_log, 'r')
    # fig = plt.figure()
    sensor_model = SensorModel(occupancy_map)

    computeTable = 1
    if computeTable:
        sensor_model.computeTable()
        with open('ray_cast_table.dat', 'wb') as fp:
            pickle.dump(sensor_model._table, fp)
    else:
        sensor_model.readTable()



    # fig = plt.figure()
    # mng = plt.get_current_fig_manager();  # mng.resize(*mng.window.maxsize())
    # plt.ion(); plt.imshow(occupancy_map, cmap='Greys'); plt.axis([0, 800, 0, 800]);
    # plt.pause(100)
    # x = np.array([5000,1000,0])
    # x_l = [];
    # y_l = [];
    # for i in range(1,181):
    #     print "Ray num: ", i
    #     test,testx,testy = sensor_model.ray_casting(x,i)
    #     print "d: ", test
    #     x_l.extend(testx)
    #     y_l.extend(testy)
    #     print test


    # print sensor_model._map.shape
    # fig = plt.figure()
    # #plt.switch_backend('TkAgg')
    # mng = plt.get_current_fig_manager(); mng.resize(*mng.window.maxsize())
    # plt.ion();  plt.axis([0, 800, 0, 800]);
    # plt.plot(testx,testy)
    # plt.show()
    #
    # plt.imshow(sensor_model._map, cmap='Greys');
    # plt.pause(100)
    pass
