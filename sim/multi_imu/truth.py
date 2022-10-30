#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation


class TruthModel:
    def __init__(self, x_rate, y_rate, z_rate, alpha_rate, beta_rate, gamma_rate):
        self.x_rate = x_rate
        self.y_rate = y_rate
        self.z_rate = z_rate
        self.a_rate = alpha_rate
        self.b_rate = beta_rate
        self.g_rate = gamma_rate

    def get_pos(self, time) -> np.array:
        p_x = np.sin(self.x_rate * time)
        p_y = np.sin(self.y_rate * time)
        p_z = np.sin(self.z_rate * time)
        return np.array([p_x, p_y, p_z]).transpose()

    def get_vel(self, time) -> np.array:
        v_x = self.x_rate * np.cos(self.x_rate * time)
        v_y = self.y_rate * np.cos(self.y_rate * time)
        v_z = self.z_rate * np.cos(self.z_rate * time)
        rot = Rotation.from_euler(
            "xyz",
            np.array(
                [
                    np.sin(self.a_rate * time),
                    np.sin(self.b_rate * time),
                    np.sin(self.g_rate * time),
                ]
            ).transpose(),
        )
        return rot.apply(np.array([v_x, v_y, v_z]).transpose())

    def get_acc(self, time) -> np.array:
        a_x = -self.x_rate ** 2 * np.sin(self.x_rate * time)
        a_y = -self.y_rate ** 2 * np.sin(self.y_rate * time)
        a_z = -self.z_rate ** 2 * np.sin(self.z_rate * time)
        rot = Rotation.from_euler(
            "xyz",
            np.array(
                [
                    np.sin(self.a_rate * time),
                    np.sin(self.b_rate * time),
                    np.sin(self.g_rate * time),
                ]
            ).transpose(),
        )
        return rot.apply(np.array([a_x, a_y, a_z]).transpose())

    def get_eul(self, time) -> np.array:
        a = np.sin(self.a_rate * time)
        b = np.sin(self.b_rate * time)
        g = np.sin(self.g_rate * time)
        return np.array([a, b, g]).transpose()

    def get_omg(self, time) -> np.array:
        w_x = self.a_rate * np.cos(self.a_rate * time)
        w_y = self.b_rate * np.cos(self.b_rate * time)
        w_z = self.g_rate * np.cos(self.g_rate * time)
        return np.array([w_x, w_y, w_z]).transpose()

    def get_omg_dot(self, time) -> np.array:
        a_x = -self.a_rate ** 2 * np.sin(self.a_rate * time)
        a_y = -self.b_rate ** 2 * np.sin(self.b_rate * time)
        a_z = -self.g_rate ** 2 * np.sin(self.g_rate * time)
        return np.array([a_x, a_y, a_z]).transpose()
