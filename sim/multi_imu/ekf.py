#!/usr/bin/env python3

import numpy as np
from typing import List
from scipy.spatial.transform import Rotation
from scipy.linalg import inv
from scipy import stats
from collections import defaultdict

from multi_imu.imu import IMU


def cross_product_matrix(input):
    output = np.zeros([3, 3])
    output[0, 1] = -input[2]
    output[0, 2] = input[1]
    output[1, 2] = -input[0]
    output[1, 0] = input[2]
    output[2, 0] = -input[1]
    output[2, 1] = input[0]
    return output


class EKF:
    def __init__(self, sensors: List[IMU], id=0):
        """
        Body State Definition
            [0:3] - Body Acceleration
            [3:6] - Body Angular Velocity
            [6:9] - Body Angular Acceleration

        IMU State Definition
            n = (id * 12) - 3
            [n + 0 : n +  3] - IMU Positional Offset
            [n + 3 : n +  6] - IMU Angular Offset
            [n + 6 : n +  9] - IMU Gyroscope Bias
            [n + 9 : n + 12] - IMU Accelerometer Bias
        """
        self.sensors = sensors
        self.state_size = 12 * (len(self.sensors)) - 3
        self.state = np.zeros([self.state_size])
        self.id = id
        self.cov = np.eye(self.state_size)
        self.current_time = 0
        self.history_index = 0
        self.dT = 0
        self.dof1 = 1000
        self.dof2 = 400
        self.settled = defaultdict(bool)
        self.re_init = defaultdict(bool)
        self.residual_count = defaultdict(float)

    def init_state(self, state):
        self.state = state

    def get_id(self):
        return self.id

    def get_state_size(self):
        return self.state_size

    def get_sensor_count(self):
        return len(self.sensors)

    def get_sensors(self):
        return self.sensors

    def get_current_time(self):
        return self.current_time

    def get_state_transition_jacobian(self, dT) -> np.array:
        F = np.eye(self.state_size)
        F[3:6, 6:9] = np.eye(3) * dT
        return F

    def get_input_matrix(self) -> np.array:
        G = np.zeros([self.state_size, self.state_size])
        G[0:3, 0:3] = np.eye(3)
        G[6:9, 6:9] = np.eye(3)
        return G

    def get_process_noise_matrix(self) -> np.array:
        Q = np.zeros([self.state_size, self.state_size])
        Q[0:3, 0:3] = np.eye(3) / 1e3
        Q[6:9, 6:9] = np.eye(3) / 1e4
        return Q

    def predict(self, time):
        dT = time - self.current_time
        if dT == 0:
            return

        F = self.get_state_transition_jacobian(dT)
        G = self.get_input_matrix()
        Q = self.get_process_noise_matrix()
        self.state = np.matmul(F, self.state)
        self.cov = np.matmul(F, np.matmul(self.cov, F.transpose())) + np.matmul(
            F, np.matmul(G, np.matmul(Q, np.matmul(G.transpose(), F.transpose())))
        )
        self.current_time = time

    def get_predicted_measurement(self, id: int) -> np.array:
        body_acc = self.state[0:3]
        body_omg = self.state[3:6]
        body_omg_dot = self.state[6:9]

        if id == 0:
            predicted_measurement = np.concatenate([body_acc, body_omg])
            return predicted_measurement
        else:
            n = int((12 * id) - 3)
            pos_offset = self.state[n + 0 : n + 3]
            ang_offset = self.state[n + 3 : n + 6]
            acc_bias = self.state[n + 6 : n + 9]
            omg_bias = self.state[n + 9 : n + 12]

            # Transform acceleration to IMU location
            imu_acc = (
                body_acc
                + np.cross(body_omg_dot, pos_offset)
                + np.cross(body_omg, np.cross(body_omg, pos_offset))
            )

            # Rotate measurements in place
            rot = Rotation.from_rotvec(ang_offset)
            imu_acc_rot = rot.apply(imu_acc) + acc_bias
            imu_omg_rot = rot.apply(body_omg) + omg_bias

            predicted_measurement = np.concatenate([imu_acc_rot, imu_omg_rot])
            return predicted_measurement

    def get_observation_jacobian(self, id) -> np.array:
        H = np.zeros([6, self.state_size])

        if id == 0:
            H[0:3, 0:3] = np.eye(3)
            H[3:6, 3:6] = np.eye(3)
            return H
        else:
            n = int((12 * id) - 3)
            pos_offset = self.state[n + 0 : n + 3]
            ang_offset = self.state[n + 3 : n + 6]
            ang_off_rot = Rotation.from_euler("xyz", ang_offset)
            body_acc = self.state[0:3]
            body_omg = self.state[3:6]
            body_omg_dot = self.state[6:9]

            # Body acceleration
            H[0:3, 0:3] = ang_off_rot.as_matrix()

            # Body angular velocity
            temp = np.zeros([3, 3])
            temp[0, 0] = pos_offset[1] * body_omg[1] + 1 * pos_offset[2] * body_omg[2]
            temp[0, 1] = pos_offset[1] * body_omg[0] - 2 * pos_offset[0] * body_omg[1]
            temp[0, 2] = pos_offset[2] * body_omg[0] - 2 * pos_offset[0] * body_omg[2]
            temp[1, 0] = pos_offset[0] * body_omg[1] - 2 * pos_offset[1] * body_omg[0]
            temp[1, 1] = pos_offset[0] * body_omg[0] + 1 * pos_offset[2] * body_omg[2]
            temp[1, 2] = pos_offset[2] * body_omg[1] - 2 * pos_offset[1] * body_omg[2]
            temp[2, 0] = pos_offset[0] * body_omg[2] - 2 * pos_offset[2] * body_omg[0]
            temp[2, 1] = pos_offset[1] * body_omg[2] - 2 * pos_offset[2] * body_omg[1]
            temp[2, 2] = pos_offset[0] * body_omg[0] + 1 * pos_offset[1] * body_omg[1]
            H[0:3, 3:6] = np.matmul(ang_off_rot.as_matrix(), temp)

            # Body Angular Acceleration
            H[0:3, 6:9] = np.matmul(
                ang_off_rot.as_matrix(), cross_product_matrix(pos_offset)
            )

            # IMU Positional Offset
            temp = np.zeros([3, 3])
            temp[0, 0] = -body_omg[1] ** 2 - body_omg[2] ** 2
            temp[0, 1] = body_omg[0] * body_omg[1]
            temp[0, 2] = body_omg[0] * body_omg[2]
            temp[1, 0] = body_omg[0] * body_omg[1]
            temp[1, 1] = -body_omg[0] ** 2 - body_omg[2] ** 2
            temp[1, 2] = body_omg[1] * body_omg[2]
            temp[2, 0] = body_omg[0] * body_omg[2]
            temp[2, 1] = body_omg[1] * body_omg[2]
            temp[2, 2] = -body_omg[0] ** 2 - body_omg[1] ** 2
            H[0:3, n + 0 : n + 3] = np.matmul(
                ang_off_rot.as_matrix(), cross_product_matrix(body_omg_dot) + temp
            )

            # IMU Angular Offset
            imu_acc = (
                body_acc
                + np.cross(body_omg_dot, pos_offset)
                + np.cross(body_omg, np.cross(body_omg, pos_offset))
            )
            H[0:3, n + 3 : n + 6] = np.matmul(
                -ang_off_rot.as_matrix(), cross_product_matrix(imu_acc)
            )

            # IMU Accelerometer Bias
            H[0:3, n + 6 : n + 9] = np.eye(3)

            # IMU Body Angular Velocity
            H[3:6, 3:6] = ang_off_rot.as_matrix()

            # IMU Angular Offset
            H[3:6, n + 3 : n + 6] = np.matmul(
                -ang_off_rot.as_matrix(), cross_product_matrix(body_omg)
            )

            # IMU Gyroscope Bias
            H[3:6, n + 9 : n + 12] = np.eye(3)

            return H

    def update(self, id, time, acc, omg, r):
        self.predict(time)

        z = np.concatenate((acc, omg))
        z_pred = self.get_predicted_measurement(id)
        H = self.get_observation_jacobian(id)
        R = np.diag(r)

        residual = z - z_pred
        residual_covariance = np.matmul(np.matmul(H, self.cov), H.transpose()) + R
        K = np.matmul(
            self.cov,
            np.matmul(H.transpose(), inv(residual_covariance)),
        )
        self.state += np.matmul(K, residual)
        self.cov = np.matmul((np.eye(self.state_size) - np.matmul(K, H)), self.cov)
        self._store_state(id, residual)

    def get_body_state(self):
        time = self.current_time
        acc = self.state[0:3]
        omg = self.state[3:6]
        omg_dot = self.state[6:9]
        return time, acc, omg, omg_dot

    def init_state_history(self, time_length):
        self.time_hist        = np.zeros([time_length])
        self.state_hist       = np.zeros([self.state_size, time_length])
        self.cov_hist         = np.zeros([self.state_size, time_length])
        self.residual_hist    = defaultdict(list)
        self.shift_test_hist  = defaultdict(list)
        self.settle_test_hist = defaultdict(list)
        self.imu_cov_hist     = defaultdict(list)
        self.settled_hist     = defaultdict(list)
        self.re_init_hist     = defaultdict(list)

    def _store_state(self, imu_id, residual):
        self.time_hist[self.history_index] = self.current_time
        self.state_hist[:, self.history_index] = self.state
        self.cov_hist[:, self.history_index] = np.diagonal(self.cov)

        n = int((12 * imu_id) - 3)
        imu_cov = np.diagonal(self.cov)[n:n+12]
        if ((imu_id != 0) and self.residual_count[imu_id] > (self.dof1 + self.dof2)):
            sample_mean = np.mean([x[1] for x in self.residual_hist[imu_id][-self.dof2:-1]], axis=0)
            shift_mean  = np.mean([x[1] for x in self.residual_hist[imu_id][-(self.dof1 + self.dof2):-self.dof2]], axis=0)
            shift_std   = np.std( [x[1] for x in self.residual_hist[imu_id][-(self.dof1 + self.dof2):-self.dof2]], axis=0)
            self.shift_test_hist[imu_id].append((self.current_time,  (sample_mean - shift_mean) / shift_std))
            self.settle_test_hist[imu_id].append((self.current_time, (self.imu_cov_hist[imu_id][0][1] - imu_cov) / self.imu_cov_hist[imu_id][0][1]))

            # Set bits
            self.settled[imu_id] = (self.settled[imu_id] or all(self.settle_test_hist[imu_id][-1][1] > 0.98))
            self.re_init[imu_id] = any(self.shift_test_hist[imu_id][-1][1] > 2)

        # Increment
        self.residual_count[imu_id] += 1

        self.residual_hist[imu_id].append((self.current_time, residual))
        self.imu_cov_hist[imu_id].append((self.current_time, imu_cov))
        self.settled_hist[imu_id].append((self.current_time, self.settled[imu_id]))
        self.re_init_hist[imu_id].append((self.current_time, self.re_init[imu_id]))
        self.history_index += 1
        if (self.re_init[imu_id]):
            self.reinitialize_covariance(imu_id)


    def reinitialize_covariance(self, imu_id):
        n = int((12 * imu_id) - 3)
        # scaling_matrix = np.eye(self.state_size)
        # scaling_matrix[n:n+12,n:n+12] = np.eye(12)
        # self.cov = np.matmul(self.cov, scaling_matrix)
        # self.cov[0:9,0:9] = self.cov[0:9,0:9] + np.eye(9) * 1
        # self.cov[n:n+12,n:n+12] = self.cov[n:n+12,n:n+12] + np.eye(12) * 1
        self.cov[n:n+12,n:n+12] = np.eye(12) * 10
        # self.cov = np.eye(self.state_size)
        self.re_init[imu_id] = False
        self.settled[imu_id] = False
        self.residual_count[imu_id] = 0


    def get_state_history(self):
        return self.time_hist, self.state_hist


    def get_covariance_history(self):
        return self.time_hist, self.cov_hist


    def get_residual_history(self):
        return self.residual_hist


    def get_test_history(self):
        return self.shift_test_hist, self.settle_test_hist


    def get_flag_history(self):
        return self.settled_hist, self.re_init_hist