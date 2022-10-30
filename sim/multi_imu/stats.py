#!/usr/bin/env python3

import os
import datetime
import numpy as np
from typing import List

from multi_imu.ekf import EKF
from multi_imu.cf  import CF
from multi_imu.truth import TruthModel


def vec_norm_rms(mat, vec):
    del_x = mat[0,:] - vec[0]
    del_y = mat[1,:] - vec[1]
    del_z = mat[2,:] - vec[2]
    err = np.sqrt(np.square(del_x) + np.square(del_y) + np.square(del_z))
    rms = np.sum(np.square(err)) / len(err)
    return rms


def mat_norm_rms(mat1, mat2):
    del_x = mat1[0,:] - mat2[0,:]
    del_y = mat1[1,:] - mat2[1,:]
    del_z = mat1[2,:] - mat2[2,:]
    err = np.sqrt(np.square(del_x) + np.square(del_y) + np.square(del_z))
    rms = np.sum(np.square(err)) / len(err)
    return rms


def evaluate_ekfs(ekfs: List[EKF], cfs: List[CF], name=None):

    file_name = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S.txt")
    file_path = os.path.abspath(os.path.join('output','stats', file_name))
    with open(file_path, "w") as f:
        if (name is not None):
            f.write("Simulation: {}\n".format(name))
        f.write("EKF Count: {:d}\n".format(len(ekfs)))
        f.write("IMU Count: {:d}\n".format(ekfs[0].get_sensor_count()))
        f.write("Max Time: {:f}\n".format(ekfs[0].get_current_time()))
        f.write("\n")

        # Summarize IMU Data
        f.write("EKF_ID,IMU_ID,")
        f.write("acc_sig_x, acc_sig_y, acc_sig_z, ")
        f.write("omg_sig_x, omg_sig_y, omg_sig_z, ")
        f.write("pos_off_x, pos_off_y, pos_off_z, ")
        f.write("ang_off_x, ang_off_y, ang_off_z\n")
        for ekf in ekfs:
            for imu in ekf.get_sensors():
                f.write("{:d},".format(ekf.get_id()))
                f.write("{:d},".format(imu.get_id()))
                acc_sig = imu.get_acc_sigma()[0]
                omg_sig = imu.get_omg_sigma()[0]
                pos_off = imu.get_pos_offset()[0]
                ang_off = imu.get_pos_offset()[0]
                f.write("{},{},{},".format(acc_sig[0], acc_sig[1], acc_sig[2]))
                f.write("{},{},{},".format(omg_sig[0], omg_sig[1], omg_sig[2]))
                f.write("{},{},{},".format(pos_off[0], pos_off[1], pos_off[2]))
                f.write("{},{},{}\n".format(ang_off[0], ang_off[1], ang_off[2]))
        f.write("\n")

        # Summarize EKF errors
        f.write("EKF_ID,acc_rms,omg_rms,acc_bias_rms,omg_bias_rms")
        for ekf in ekfs:
            f.write("\n{:d}".format(ekf.get_id()))
            [time, ekf_state] = ekf.get_state_history()
            truth_model = ekf.sensors[0].truth_model
            acc_body = truth_model.get_acc(time)
            omg_body = truth_model.get_omg(time)

            # Body Acceleration
            acc_rms = mat_norm_rms(ekf_state[0:3, -101:-1], np.transpose(acc_body[-101:-1, 0:3]))
            f.write(",{:1.6E}".format(acc_rms))

            # Body Angular Velocity
            omg_rms = mat_norm_rms(ekf_state[3:6, -101:-1], np.transpose(omg_body[-101:-1, 0:3]))
            f.write(",{:1.6E}".format(omg_rms))

            for imu in ekf.get_sensors()[1:]:
                id = imu.get_id()
                n = (12 * id) - 3

                # Position Offset
                pos_offset = imu.get_pos_offset()[0]
                imu_pos_rms = vec_norm_rms(ekf_state[n+0:n+3, -101:-1], pos_offset)
                f.write(",{:1.6E}".format(imu_pos_rms))

                # Angular Offset
                ang_offset = imu.get_ang_offset()[0]
                imu_ang_rms = vec_norm_rms(ekf_state[n+3:n+6, -101:-1], ang_offset)
                f.write(",{:1.6E}".format(imu_ang_rms))

                # Accelerometer Bias
                acc_bias = imu.get_acc_bias()[0]
                imu_acc_bias_rms = vec_norm_rms(ekf_state[n+6:n+9, -101:-1], acc_bias)
                f.write(",{:1.6E}".format(imu_acc_bias_rms))

                # Gyroscope Bias
                omg_bias = imu.get_omg_bias()[0]
                imu_omg_bias_rms = vec_norm_rms(ekf_state[n+9:n+12, -101:-1], omg_bias)
                f.write(",{:1.6E}".format(imu_omg_bias_rms))

        # Summarize CF errors
        f.write("\n\nCF_ID,acc_rms,omg_rms\n")
        for cf in cfs:
            f.write("{:d}".format(cf.get_id()))
            time, acc, omg, cov_acc, cov_omg = cf.get_state_history()

            acc_body = truth_model.get_acc(time)
            omg_body = truth_model.get_omg(time)

            # Body Acceleration
            acc_rms = mat_norm_rms(np.transpose(acc[-101:-1, 0:3]), np.transpose(acc_body[-101:-1, 0:3]))
            f.write(",{:1.6E}".format(acc_rms))

            # Body Angular Velocity
            omg_rms = mat_norm_rms(np.transpose(omg[-101:-1, 0:3]), np.transpose(omg_body[-101:-1, 0:3]))
            f.write(",{:1.6E}\n".format(omg_rms))