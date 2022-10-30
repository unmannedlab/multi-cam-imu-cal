#!/usr/bin/env python3

import numpy as np
from typing import List

from multi_imu.imu import IMU


class CF:
    def __init__(   self, 
                    sensors: List[IMU], 
                    time : np.array,
                    acc : np.array,
                    omg : np.array,
                    var : np.array, 
                    id  : int):

        self.t = np.unique(time)
        self.acc     = np.zeros([len(self.t),3])
        self.omg     = np.zeros([len(self.t),3])
        self.cov_acc = np.zeros([len(self.t),3])
        self.cov_omg = np.zeros([len(self.t),3])
        self.id = id

        for i in range(len(self.t)):
            idx, = np.where(time == self.t[i])
            cnt = len(idx)

            self.cov_acc[i,:] = np.ones([1,3]) / np.sum(np.ones([cnt,3]) / np.square(var[idx,0:3]),0)
            self.cov_omg[i,:] = np.ones([1,3]) / np.sum(np.ones([cnt,3]) / np.square(var[idx,3:6]),0)

            self.acc[i,:] = np.sum(acc[idx,:] / np.square(var[idx,0:3]), 0) * self.cov_acc[i,:]
            self.omg[i,:] = np.sum(omg[idx,:] / np.square(var[idx,3:6]), 0) * self.cov_omg[i,:]

    def get_state_history(self):
            return self.t, self.acc, self.omg, self.cov_acc, self.cov_omg


    def get_id(self):
        return self.id