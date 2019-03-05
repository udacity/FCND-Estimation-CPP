#!/usr/bin/env python

import numpy as np

gps_x_data = np.loadtxt('config/log/Graph1.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]
acc_x_data = np.loadtxt('config/log/Graph2.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]

gps_x_std = np.std(gps_x_data)
acc_x_std = np.std(acc_x_data)

print "gps_x_std: ", gps_x_std
print "acc_x_std: ", acc_x_std

#gps x std:  0.7234761537348884
#acc x std:  0.5096556221430085
