#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation

from multi_imu.truth import TruthModel


class IMU:
    def __init__(self, id: int, truth_model: TruthModel):
        self.truth_model = truth_model
        self.acc_sigma  = np.ones([1, 3])
        self.omg_sigma  = np.ones([1, 3])
        self.pos_offset = np.zeros([1, 3])
        self.ang_offset = np.zeros([1, 3])
        self.acc_bias   = np.zeros([1, 3])
        self.omg_bias   = np.zeros([1, 3])

        self.pos_offset_hist = [(0, np.zeros([1, 3]))]
        self.ang_offset_hist = [(0, np.zeros([1, 3]))]
        self.acc_bias_hist   = [(0, np.zeros([1, 3]))]
        self.omg_bias_hist   = [(0, np.zeros([1, 3]))]

        self.id = id

    def get_id(self):
        return self.id

    def set_hz(self, hz):
        self.hz = hz
        self.period = 1 / hz
    
    def get_hz(self):
        return self.hz

    def get_period(self):
        return self.period

    def set_period(self, period):
        self.period = period
        self.hz = 1 / period

    def get_truth_model(self):
        return self.truth_model

    def set_acc_sigma(self, x_acc_sigma, y_acc_sigma, z_acc_sigma):
        self.acc_sigma = np.array([[x_acc_sigma, y_acc_sigma, z_acc_sigma]])

    def set_omg_sigma(self, x_omg_sigma, y_omg_sigma, z_omg_sigma):
        self.omg_sigma = np.array([[x_omg_sigma, y_omg_sigma, z_omg_sigma]])

    def set_pos_offset(self, x_pos_offset, y_pos_offset, z_pos_offset, time=None):
        self.pos_offset = np.array([[x_pos_offset, y_pos_offset, z_pos_offset]])
        if (time is not None):
            self.pos_offset_hist.append((time, self.pos_offset))

    def set_ang_offset(self, x_ang_offset, y_ang_offset, z_ang_offset, time=None):
        self.ang_offset = np.array([[x_ang_offset, y_ang_offset, z_ang_offset]])
        if (time is not None):
            self.ang_offset_hist.append((time, self.ang_offset))

    def set_acc_bias(self, x_acc_offset, y_acc_offset, z_acc_offset, time=None):
        self.acc_bias = np.array([[x_acc_offset, y_acc_offset, z_acc_offset]])
        if (time is not None):
            self.acc_bias_hist.append((time, self.acc_bias))

    def set_omg_bias(self, x_omg_offset, y_omg_offset, z_omg_offset, time=None):
        self.omg_bias = np.array([[x_omg_offset, y_omg_offset, z_omg_offset]])
        if (time is not None):
            self.omg_bias_hist.append((time, self.omg_bias))

    def set_acc_noise_density(self, x_a_n_d, y_a_n_d, z_a_n_d):
        self.acc_noise_density = np.array([[x_a_n_d, y_a_n_d, z_a_n_d]])

    def set_omg_noise_density(self, x_o_n_d, y_o_n_d, z_o_n_d):
        self.omg_noise_density = np.array([[x_o_n_d, y_o_n_d, z_o_n_d]])

    def get_acc_sigma(self):
        return self.acc_sigma

    def get_omg_sigma(self):
        return self.omg_sigma

    def get_pos_offset(self):
        return self.pos_offset

    def get_ang_offset(self):
        return self.ang_offset

    def get_acc_bias(self):
        return self.acc_bias

    def get_omg_bias(self):
        return self.omg_bias

    def get_pos_offset_hist(self):
        self.pos_offset_hist.sort(key=lambda x: x[0])
        return self.pos_offset_hist

    def get_ang_offset_hist(self):
        self.ang_offset_hist.sort(key=lambda x: x[0])
        return self.ang_offset_hist

    def get_acc_bias_hist(self):
        self.acc_bias_hist.sort(key=lambda x: x[0])
        return self.acc_bias_hist

    def get_omg_bias_hist(self):
        self.omg_bias_hist.sort(key=lambda x: x[0])
        return self.omg_bias_hist

    def get_acc_noise_density(self):
        return self.acc_noise_density

    def get_omg_noise_density(self):
        return self.omg_noise_density

    def measure_acc(self, time):
        # Body truth
        body_acc = self.truth_model.get_acc(time)
        body_omg = self.truth_model.get_omg(time)
        body_omg_dot = self.truth_model.get_omg_dot(time)

        # Transform to acceleration location
        imu_acc = (
            body_acc
            + np.cross(body_omg_dot, self.pos_offset)
            + np.cross(body_omg, np.cross(body_omg, self.pos_offset))
        )

        # Rotate IMU in place
        rot = Rotation.from_euler("xyz", self.ang_offset)
        imu_acc_rot = rot.apply(imu_acc)

        # Add bias
        imu_acc_biased = imu_acc_rot + self.acc_bias

        # Add noise
        imu_acc_meas = np.random.normal(imu_acc_biased, self.acc_sigma)

        # Return
        return imu_acc_meas

    def measure_omg(self, time):
        # Body truth
        body_omg = self.truth_model.get_omg(time)

        # Rotate in place
        rot = Rotation.from_euler("xyz", self.ang_offset)
        imu_omg_rot = rot.apply(body_omg)

        # Add bias
        imu_omg_biased = imu_omg_rot + self.omg_bias

        # Add Noise
        imu_omg_meas = np.random.normal(imu_omg_biased, self.omg_sigma)

        return imu_omg_meas

    def get_measurements(self, time_max, time_start=0):
        num_steps = int((time_max-time_start) * self.hz + 1)
        measurement_times = np.linspace(time_start, time_max, num_steps)
        acc = self.measure_acc(measurement_times)
        omg = self.measure_omg(measurement_times)
        ids = np.ones(num_steps) * self.id
        r = np.repeat(
            np.concatenate((self.acc_sigma * 100, self.acc_sigma * 100), axis=1),
            num_steps,
            axis=0,
        )

        return ids, measurement_times, acc, omg, r
