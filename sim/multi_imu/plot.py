#!/usr/bin/env python3

import os
import numpy as np
import matplotlib.pyplot as plt
from typing import List

from multi_imu.ekf import EKF
from multi_imu.constants import M_TO_MM


def save_figure_to_dir(fig, name):
    figPath = os.path.join("output", "plots")
    if not os.path.exists(figPath):
        os.makedirs(figPath)
    fig.savefig(os.path.join(
        figPath, "{}.png".format(name).replace(" ", "_")))


def set_axes_equal_3d(ax):
    """
    Make axes of 3D plot have equal scale
    Mimics function of axis('equal') for 3D plots

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim()
    y_limits = ax.get_ylim()
    z_limits = ax.get_zlim()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim([z_middle - plot_radius, z_middle + plot_radius])


def plot_measurements(ids, time, acc, omg):
    fig1, axs1 = plt.subplots(3, 1)
    fig2, axs2 = plt.subplots(3, 1)

    for id in np.unique(ids):
        indices = np.where(ids == id)
        temp_label = "IMU {:d}".format(int(id))
        axs1[0].plot(time[indices], omg[indices,
                     0].transpose(), label=temp_label)
        axs1[1].plot(time[indices], omg[indices, 1].transpose())
        axs1[2].plot(time[indices], omg[indices, 2].transpose())
        axs2[0].plot(time[indices], acc[indices,
                     0].transpose(), label=temp_label)
        axs2[1].plot(time[indices], acc[indices, 1].transpose())
        axs2[2].plot(time[indices], acc[indices, 2].transpose())

    axs1[0].legend()
    axs1[0].grid(True)
    axs1[1].grid(True)
    axs1[2].grid(True)
    temp_title = "Angular Rate Measurements"
    fig1.suptitle(temp_title)
    fig1.canvas.set_window_title(temp_title)
    fig1.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig1, temp_title)

    axs2[0].legend()
    axs2[0].grid(True)
    axs2[1].grid(True)
    axs2[2].grid(True)
    temp_title = "Acceleration Measurements"
    fig2.suptitle(temp_title)
    fig2.canvas.set_window_title(temp_title)
    fig2.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig2, temp_title)
    plt.show()


def plot_ekf_state_history(ekf: EKF):

    [time, state_hist] = ekf.get_state_history()

    truth_model = ekf.get_sensors()[0].get_truth_model()

    # pos_body = truth_model.get_pos(time)
    acc_body = truth_model.get_acc(time)
    # eul_body = truth_model.get_eul(time)
    omg_body = truth_model.get_omg(time)
    omg_dot_body = truth_model.get_omg_dot(time)

    # Body Acceleration
    fig1, axs1 = plt.subplots(3, 1)
    axs1[0].plot(time, acc_body[:, 0], label="Truth")
    axs1[0].plot(time, state_hist[0, :], label="EKF")

    axs1[1].plot(time, acc_body[:, 1])
    axs1[1].plot(time, state_hist[1, :])

    axs1[2].plot(time, acc_body[:, 2])
    axs1[2].plot(time, state_hist[2, :])

    axs1[0].legend()
    axs1[0].grid(True)
    axs1[1].grid(True)
    axs1[2].grid(True)
    axs1[0].set_ylabel("X [m/s/s]")
    axs1[1].set_ylabel("Y [m/s/s]")
    axs1[2].set_ylabel("Z [m/s/s]")
    axs1[2].set_xlabel("Time [s]")
    temp_title = "Body Acceleration"
    fig1.suptitle(temp_title)
    fig1.canvas.set_window_title(temp_title)
    fig1.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig1, temp_title)

    # Body Angular Velocity
    fig2, axs2 = plt.subplots(3, 1)
    axs2[0].plot(time, omg_body[:, 0], label="Truth")
    axs2[0].plot(time, state_hist[3, :], label="EKF")

    axs2[1].plot(time, omg_body[:, 1])
    axs2[1].plot(time, state_hist[4, :])

    axs2[2].plot(time, omg_body[:, 2])
    axs2[2].plot(time, state_hist[5, :])

    axs2[0].legend()
    axs2[0].grid(True)
    axs2[1].grid(True)
    axs2[2].grid(True)
    axs2[0].set_ylabel("X [rad/s]")
    axs2[1].set_ylabel("Y [rad/s]")
    axs2[2].set_ylabel("Z [rad/s]")
    axs2[2].set_xlabel("Time [s]")
    temp_title = "Body Angular Rate"
    fig2.suptitle(temp_title)
    fig2.canvas.set_window_title(temp_title)
    fig2.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig2, temp_title)

    # Body Angular Acceleration
    fig3, axs3 = plt.subplots(3, 1)
    axs3[0].plot(time, omg_dot_body[:, 0], label="Truth")
    axs3[0].plot(time, state_hist[6, :], label="EKF")

    axs3[1].plot(time, omg_dot_body[:, 1])
    axs3[1].plot(time, state_hist[7, :])

    axs3[2].plot(time, omg_dot_body[:, 2])
    axs3[2].plot(time, state_hist[8, :])

    axs3[0].legend()
    axs3[0].grid(True)
    axs3[1].grid(True)
    axs3[2].grid(True)
    axs3[0].set_ylabel("X [rad/s/s]")
    axs3[1].set_ylabel("Y [rad/s/s]")
    axs3[2].set_ylabel("Z [rad/s/s]")
    axs3[2].set_xlabel("Time [s]")
    temp_title = "Body Angular Acceleration"
    fig3.suptitle(temp_title)
    fig3.canvas.set_window_title(temp_title)
    fig3.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig3, temp_title)

    if ekf.get_sensor_count() > 1:

        fig4, axs4 = plt.subplots(3, 1)
        fig5, axs5 = plt.subplots(3, 1)
        fig6, axs6 = plt.subplots(3, 1)
        fig7, axs7 = plt.subplots(3, 1)

        imus = ekf.get_sensors()

        for i in range(1, ekf.get_sensor_count()):
            imu = imus[i]
            n = (i * 12) - 3
            imu_label = "IMU {:d}".format(i)

            pos_offset = imu.get_pos_offset()
            ang_offset = imu.get_ang_offset()

            # IMU positional offset
            axs4[0].plot(time, state_hist[n + 0, :] -
                         pos_offset[0, 0], label=imu_label)
            axs4[1].plot(time, state_hist[n + 1, :] - pos_offset[0, 1])
            axs4[2].plot(time, state_hist[n + 2, :] - pos_offset[0, 2])

            # IMU angular offset
            axs5[0].plot(time, state_hist[n + 3, :] -
                         ang_offset[0, 0], label=imu_label)
            axs5[1].plot(time, state_hist[n + 4, :] - ang_offset[0, 1])
            axs5[2].plot(time, state_hist[n + 5, :] - ang_offset[0, 2])

            # IMU accelerometer bias
            axs6[0].plot(time, state_hist[n + 6, :] -
                         ang_offset[0, 0], label=imu_label)
            axs6[1].plot(time, state_hist[n + 7, :] - ang_offset[0, 1])
            axs6[2].plot(time, state_hist[n + 8, :] - ang_offset[0, 2])

            # IMU gyroscope bias
            axs7[0].plot(time, state_hist[n + 9, :] -
                         ang_offset[0, 0], label=imu_label)
            axs7[1].plot(time, state_hist[n + 10, :] - ang_offset[0, 1])
            axs7[2].plot(time, state_hist[n + 11, :] - ang_offset[0, 2])

        axs4[0].legend()
        axs4[0].grid(True)
        axs4[1].grid(True)
        axs4[2].grid(True)
        axs4[0].set_ylabel("X [m]")
        axs4[1].set_ylabel("Y [m]")
        axs4[2].set_ylabel("Z [m]")
        axs4[2].set_xlabel("Time [s]")
        temp_title = "IMU Positional Offset"
        fig4.suptitle(temp_title)
        fig4.canvas.set_window_title(temp_title)
        fig4.tight_layout(rect=[0, 0.03, 1, 0.95])
        save_figure_to_dir(fig4, "mc_pos_off_err")

        axs5[0].legend()
        axs5[0].grid(True)
        axs5[1].grid(True)
        axs5[2].grid(True)
        axs5[0].set_ylabel("X [rad]")
        axs5[1].set_ylabel("Y [rad]")
        axs5[2].set_ylabel("Z [rad]")
        axs5[2].set_xlabel("Time [s]")
        temp_title = "IMU Angular Offset"
        fig5.suptitle(temp_title)
        fig5.canvas.set_window_title(temp_title)
        fig5.tight_layout(rect=[0, 0.03, 1, 0.95])
        save_figure_to_dir(fig5, "mc_ang_off_err")

        axs6[0].legend()
        axs6[0].grid(True)
        axs6[1].grid(True)
        axs6[2].grid(True)
        axs6[0].set_ylabel("X [m/s/s]")
        axs6[1].set_ylabel("Y [m/s/s]")
        axs6[2].set_ylabel("Z [m/s/s]")
        axs6[2].set_xlabel("Time [s]")
        temp_title = "IMU Accelerometer Bias"
        fig6.suptitle(temp_title)
        fig6.canvas.set_window_title(temp_title)
        fig6.tight_layout(rect=[0, 0.03, 1, 0.95])
        save_figure_to_dir(fig6, "mc_acc_bias_err")

        axs7[0].legend()
        axs7[0].grid(True)
        axs7[1].grid(True)
        axs7[2].grid(True)
        axs7[0].set_ylabel("X [rad/s]")
        axs7[1].set_ylabel("Y [rad/s]")
        axs7[2].set_ylabel("Z [rad/s]")
        axs7[2].set_xlabel("Time [s]")
        temp_title = "IMU Gyroscope Bias"
        fig7.suptitle(temp_title)
        fig7.canvas.set_window_title(temp_title)
        fig7.tight_layout(rect=[0, 0.03, 1, 0.95])
        save_figure_to_dir(fig7, "mc_omg_bas_err")

    plt.show()


def plot_ekf_cov_history(ekf: EKF):

    [time, cov_hist] = ekf.get_covariance_history()

    fig1, axs1 = plt.subplots(1, 1)
    axs1.plot(time, cov_hist[0, :], color="tab:blue", label="Acceleration")
    axs1.plot(time, cov_hist[1, :], color="tab:blue")
    axs1.plot(time, cov_hist[2, :], color="tab:blue")
    axs1.plot(time, cov_hist[3, :], color="tab:orange", label="Angular Rate")
    axs1.plot(time, cov_hist[4, :], color="tab:orange")
    axs1.plot(time, cov_hist[5, :], color="tab:orange")
    axs1.plot(time, cov_hist[6, :], color="tab:green",
              label="Angular Acceleration")
    axs1.plot(time, cov_hist[7, :], color="tab:green")
    axs1.plot(time, cov_hist[8, :], color="tab:green")
    axs1.legend()
    axs1.grid(True)
    axs1.set_ylabel("Covariance Value")
    axs1.set_xlabel("Time [s]")
    temp_title = "IMU Covariance"
    fig1.suptitle(temp_title)
    fig1.canvas.set_window_title(temp_title)
    fig1.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig1, temp_title)

    if ekf.get_sensor_count() > 1:
        fig2, axs2 = plt.subplots(3, 1)
        fig3, axs3 = plt.subplots(3, 1)
        fig4, axs4 = plt.subplots(3, 1)
        fig5, axs5 = plt.subplots(3, 1)

        for i in range(1, ekf.get_sensor_count()):
            n = (i * 12) - 3
            imu_label = "IMU {:d}".format(i)

            # IMU positional offset
            axs2[0].plot(time, cov_hist[n + 0, :], label=imu_label)
            axs2[1].plot(time, cov_hist[n + 1, :])
            axs2[2].plot(time, cov_hist[n + 2, :])

            # IMU angular offset
            axs3[0].plot(time, cov_hist[n + 3, :], label=imu_label)
            axs3[1].plot(time, cov_hist[n + 4, :])
            axs3[2].plot(time, cov_hist[n + 5, :])

            # IMU accelerometer bias
            axs4[0].plot(time, cov_hist[n + 6, :], label=imu_label)
            axs4[1].plot(time, cov_hist[n + 7, :])
            axs4[2].plot(time, cov_hist[n + 8, :])

            # IMU gyroscope bias
            axs5[0].plot(time, cov_hist[n + 9, :], label=imu_label)
            axs5[1].plot(time, cov_hist[n + 10, :])
            axs5[2].plot(time, cov_hist[n + 11, :])

        axs2[0].legend()
        axs2[0].grid(True)
        axs2[1].grid(True)
        axs2[2].grid(True)
        axs2[0].set_ylabel("X [m]")
        axs2[1].set_ylabel("Y [m]")
        axs2[2].set_ylabel("Z [m]")
        axs2[2].set_xlabel("Time [s]")
        temp_title = "IMU Positional Offset Covariance"
        fig2.suptitle(temp_title)
        fig2.canvas.set_window_title(temp_title)
        fig2.tight_layout(rect=[0, 0.03, 1, 0.95])
        save_figure_to_dir(fig2, "mc_pos_off_cov")

        axs3[0].legend()
        axs3[0].grid(True)
        axs3[1].grid(True)
        axs3[2].grid(True)
        axs3[2].grid(True)
        axs3[0].set_ylabel("X [rad]")
        axs3[1].set_ylabel("Y [rad]")
        axs3[2].set_ylabel("Z [rad]")
        axs3[2].set_xlabel("Time [s]")
        temp_title = "IMU Angular Offset Covariance"
        fig3.suptitle(temp_title)
        fig3.canvas.set_window_title(temp_title)
        fig3.tight_layout(rect=[0, 0.03, 1, 0.95])
        save_figure_to_dir(fig3, "mc_ang_off_cov")

        axs4[0].legend()
        axs4[0].grid(True)
        axs4[1].grid(True)
        axs4[2].grid(True)
        axs4[2].grid(True)
        axs4[0].set_ylabel("X [m/s/s]")
        axs4[1].set_ylabel("Y [m/s/s]")
        axs4[2].set_ylabel("Z [m/s/s]")
        axs4[2].set_xlabel("Time [s]")
        temp_title = "IMU Accelerometer Bias Covariance"
        fig4.suptitle(temp_title)
        fig4.canvas.set_window_title(temp_title)
        fig4.tight_layout(rect=[0, 0.03, 1, 0.95])
        save_figure_to_dir(fig4, "mc_acc_bias_cov")

        axs5[0].legend()
        axs5[0].grid(True)
        axs5[1].grid(True)
        axs5[2].grid(True)
        axs5[0].set_ylabel("X [rad/s]")
        axs5[1].set_ylabel("Y [rad/s]")
        axs5[2].set_ylabel("Z [rad/s]")
        axs5[2].set_xlabel("Time [s]")
        temp_title = "IMU Gyroscope Bias Covariance"
        fig5.suptitle(temp_title)
        fig5.canvas.set_window_title(temp_title)
        fig5.tight_layout(rect=[0, 0.03, 1, 0.95])
        save_figure_to_dir(fig5, "mc_omg_bias_cov")


def expand_by_time(req_time, val_hist):
    j = 0
    val_out = np.zeros([len(req_time), 3])
    for i in range(len(req_time)):
        if (j == len(val_hist)):
            val_out[i, :] = val_hist[j][1]
        else:
            while ((j < len(val_hist) - 1) and
                   (req_time[i] >= val_hist[j+1][0])):
                j += 1
            val_out[i, :] = val_hist[j][1]
    return val_out


def moving_average(values, n):
    if (n <= 1):
        return values
    expanded_values = np.concatenate(
        (
            np.zeros([n - 1]),
            values,
        )
    )
    cumsum = np.cumsum(expanded_values)
    return (cumsum[(n-1):] - cumsum[:-(n-1)]) / n


def plot_mc_states(ekfs: List[EKF]):
    fig1, axs1 = plt.subplots(3, 1)
    fig2, axs2 = plt.subplots(3, 1)
    fig3, axs3 = plt.subplots(3, 1)
    fig4, axs4 = plt.subplots(3, 1)

    for ekf in ekfs:
        [time, state] = ekf.get_state_history()

        if ekf.get_sensor_count() > 1:

            imus = ekf.get_sensors()
            for i in range(1, ekf.get_sensor_count()):
                imu = imus[i]
                n = (i * 12) - 3
                # id = imu.get_id()

                ang_offset = expand_by_time(time, imu.get_ang_offset_hist())
                pos_offset = expand_by_time(time, imu.get_pos_offset_hist())
                acc_bias = expand_by_time(time, imu.get_acc_bias_hist())
                omg_bias = expand_by_time(time, imu.get_omg_bias_hist())

                # IMU positional offset error
                axs1[0].plot(time, (state[n + 0, :] - pos_offset[:, 0])
                             * M_TO_MM, color="tab:blue", alpha=0.1)
                axs1[1].plot(time, (state[n + 1, :] - pos_offset[:, 1])
                             * M_TO_MM, color="tab:blue", alpha=0.1)
                axs1[2].plot(time, (state[n + 2, :] - pos_offset[:, 2])
                             * M_TO_MM, color="tab:blue", alpha=0.1)

                # IMU angular offset error
                axs2[0].plot(time, (state[n + 3, :] - ang_offset[:, 0])
                             * M_TO_MM, color="tab:blue", alpha=0.1)
                axs2[1].plot(time, (state[n + 4, :] - ang_offset[:, 1])
                             * M_TO_MM, color="tab:blue", alpha=0.1)
                axs2[2].plot(time, (state[n + 5, :] - ang_offset[:, 2])
                             * M_TO_MM, color="tab:blue", alpha=0.1)

                # IMU accelerometer bias error
                axs3[0].plot(time, (state[n + 6, :] - acc_bias[:, 0])
                             * M_TO_MM, color="tab:blue", alpha=0.1)
                axs3[1].plot(time, (state[n + 7, :] - acc_bias[:, 1])
                             * M_TO_MM, color="tab:blue", alpha=0.1)
                axs3[2].plot(time, (state[n + 8, :] - acc_bias[:, 2])
                             * M_TO_MM, color="tab:blue", alpha=0.1)

                # IMU gyroscope bias error
                axs4[0].plot(time, (state[n + 9, :] - omg_bias[:, 0])
                             * M_TO_MM, color="tab:blue", alpha=0.1)
                axs4[1].plot(time, (state[n + 10, :] - omg_bias[:, 1])
                             * M_TO_MM, color="tab:blue", alpha=0.1)
                axs4[2].plot(time, (state[n + 11, :] - omg_bias[:, 2])
                             * M_TO_MM, color="tab:blue", alpha=0.1)

    temp_title = "IMU Positional Offset Error"
    axs1[0].set_ylabel('X [mm]')
    axs1[1].set_ylabel('Y [mm]')
    axs1[2].set_ylabel('Z [mm]')
    axs1[2].set_xlabel('Time [s]')
    axs1[0].grid(True)
    axs1[1].grid(True)
    axs1[2].grid(True)
    fig1.suptitle(temp_title)
    fig1.canvas.set_window_title(temp_title)
    fig1.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig1, "mc_pos_off_err")

    temp_title = "IMU Angular Offset Error"
    axs2[0].set_ylabel('X [mm]')
    axs2[1].set_ylabel('Y [mm]')
    axs2[2].set_ylabel('Z [mm]')
    axs2[2].set_xlabel('Time [s]')
    axs2[0].grid(True)
    axs2[1].grid(True)
    axs2[2].grid(True)
    fig2.suptitle(temp_title)
    fig2.canvas.set_window_title(temp_title)
    fig2.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig2, "mc_ang_off_err")

    temp_title = "IMU Accelerometer Bias Error"
    axs3[0].set_ylabel('X [mm]')
    axs3[1].set_ylabel('Y [mm]')
    axs3[2].set_ylabel('Z [mm]')
    axs3[2].set_xlabel('Time [s]')
    axs3[0].grid(True)
    axs3[1].grid(True)
    axs3[2].grid(True)
    fig3.suptitle(temp_title)
    fig3.canvas.set_window_title(temp_title)
    fig3.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig3, "mc_acc_bias_err")

    temp_title = "IMU Gyroscope Bias Error"
    axs4[0].set_ylabel('X [mm]')
    axs4[1].set_ylabel('Y [mm]')
    axs4[2].set_ylabel('Z [mm]')
    axs4[2].set_xlabel('Time [s]')
    axs4[0].grid(True)
    axs4[1].grid(True)
    axs4[2].grid(True)
    fig4.suptitle(temp_title)
    fig4.canvas.set_window_title(temp_title)
    fig4.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig4, "mc_omg_bias_err")

    plt.show()


def plot_mc_residuals(ekfs: List[EKF]):
    fig1, axs1 = plt.subplots(3, 1)
    fig2, axs2 = plt.subplots(3, 1)

    for ekf in ekfs:
        residuals = ekf.get_residual_history()

        if ekf.get_sensor_count() > 1:

            imus = ekf.get_sensors()
            for i in range(1, ekf.get_sensor_count()):
                imu = imus[i]
                # n = (i * 12) - 3
                id = imu.get_id()

                # IMU Measurement Residuals
                residual_t = np.array([x[0] for x in residuals[id]])
                residual_v = np.array([x[1] for x in residuals[id]])

                # IMU Accelerometer Residuals
                axs1[0].plot(residual_t, residual_v[:, 0],
                             color="tab:blue", alpha=0.1)
                axs1[1].plot(residual_t, residual_v[:, 1],
                             color="tab:blue", alpha=0.1)
                axs1[2].plot(residual_t, residual_v[:, 2],
                             color="tab:blue", alpha=0.1)

                # IMU Gyroscope Residuals
                axs2[0].plot(residual_t, residual_v[:, 3],
                             color="tab:blue", alpha=0.1)
                axs2[1].plot(residual_t, residual_v[:, 4],
                             color="tab:blue", alpha=0.1)
                axs2[2].plot(residual_t, residual_v[:, 5],
                             color="tab:blue", alpha=0.1)

    temp_title = "IMU Accelerometer Residuals"
    axs1[0].set_ylabel('X Error\n[m/s/s]')
    axs1[1].set_ylabel('Y Error\n[m/s/s]')
    axs1[2].set_ylabel('Z Error\n[m/s/s]')
    axs1[2].set_xlabel('Time [s]')
    axs1[0].grid(True)
    axs1[1].grid(True)
    axs1[2].grid(True)
    fig1.suptitle(temp_title)
    fig1.canvas.set_window_title(temp_title)
    fig1.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig1, "mc_acc_residuals")

    temp_title = "IMU Gyroscope Residuals"
    axs2[0].set_ylabel('X Error\n[rad/s]')
    axs2[1].set_ylabel('Y Error\n[rad/s]')
    axs2[2].set_ylabel('Z Error\n[rad/s]')
    axs2[2].set_xlabel('Time [s]')
    axs2[0].grid(True)
    axs2[1].grid(True)
    axs2[2].grid(True)
    fig2.suptitle(temp_title)
    fig2.canvas.set_window_title(temp_title)
    fig2.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig2, "mc_omg_residuals")


def plot_mc_tests(ekfs: List[EKF]):
    fig1, axs1 = plt.subplots(3, 1)
    fig2, axs2 = plt.subplots(3, 1)
    fig3, axs3 = plt.subplots(3, 1)
    fig4, axs4 = plt.subplots(3, 1)
    fig5, axs5 = plt.subplots(3, 1)
    fig6, axs6 = plt.subplots(3, 1)

    for ekf in ekfs:
        [shift_tests, settle_tests] = ekf.get_test_history()

        if ekf.get_sensor_count() > 1:

            imus = ekf.get_sensors()
            for i in range(1, ekf.get_sensor_count()):
                imu = imus[i]
                id = imu.get_id()

                # IMU Shift Tests
                shift_test_t = np.array([x[0] for x in shift_tests[id]])
                shift_test_v = np.array([x[1] for x in shift_tests[id]])

                # Break if insufficient data found
                if (len(shift_test_t) == 0) or (len(shift_test_v) == 0):
                    continue

                # IMU Accelerometer Tests
                axs1[0].plot(shift_test_t, abs(shift_test_v[:, 0]),
                             color="tab:blue", alpha=0.1)
                axs1[1].plot(shift_test_t, abs(shift_test_v[:, 1]),
                             color="tab:blue", alpha=0.1)
                axs1[2].plot(shift_test_t, abs(shift_test_v[:, 2]),
                             color="tab:blue", alpha=0.1)

                # IMU Gyroscope Tests
                axs2[0].plot(shift_test_t, abs(shift_test_v[:, 3]),
                             color="tab:blue", alpha=0.1)
                axs2[1].plot(shift_test_t, abs(shift_test_v[:, 4]),
                             color="tab:blue", alpha=0.1)
                axs2[2].plot(shift_test_t, abs(shift_test_v[:, 5]),
                             color="tab:blue", alpha=0.1)

                # IMU Settle Tests
                settle_test_t = np.array([x[0] for x in settle_tests[id]])
                settle_test_v = np.array([x[1] for x in settle_tests[id]])

                # IMU Positional Offset Covariance Tests
                axs3[0].plot(settle_test_t, abs(
                    settle_test_v[:, 0]), color="tab:blue", alpha=0.1)
                axs3[1].plot(settle_test_t, abs(
                    settle_test_v[:, 1]), color="tab:blue", alpha=0.1)
                axs3[2].plot(settle_test_t, abs(
                    settle_test_v[:, 2]), color="tab:blue", alpha=0.1)

                # IMU Angular Offset Covariance Tests
                axs4[0].plot(settle_test_t, abs(
                    settle_test_v[:, 3]), color="tab:blue", alpha=0.1)
                axs4[1].plot(settle_test_t, abs(
                    settle_test_v[:, 4]), color="tab:blue", alpha=0.1)
                axs4[2].plot(settle_test_t, abs(
                    settle_test_v[:, 5]), color="tab:blue", alpha=0.1)

                # IMU Accelerometer Bias Covariance Tests
                axs5[0].plot(settle_test_t, abs(
                    settle_test_v[:, 6]), color="tab:blue", alpha=0.1)
                axs5[1].plot(settle_test_t, abs(
                    settle_test_v[:, 7]), color="tab:blue", alpha=0.1)
                axs5[2].plot(settle_test_t, abs(
                    settle_test_v[:, 8]), color="tab:blue", alpha=0.1)

                # IMU Gyroscope Bias Covariance Tests
                axs6[0].plot(settle_test_t, abs(
                    settle_test_v[:, 9]),  color="tab:blue", alpha=0.1)
                axs6[1].plot(settle_test_t, abs(
                    settle_test_v[:, 10]), color="tab:blue", alpha=0.1)
                axs6[2].plot(settle_test_t, abs(
                    settle_test_v[:, 11]), color="tab:blue", alpha=0.1)

    temp_title = "IMU Accelerometer Shift Tests"
    axs1[2].set_xlabel('Time [s]')
    axs1[0].set_ylabel('X')
    axs1[1].set_ylabel('Y')
    axs1[2].set_ylabel('Z')
    axs1[0].grid(True)
    axs1[1].grid(True)
    axs1[2].grid(True)
    fig1.suptitle(temp_title)
    fig1.canvas.set_window_title(temp_title)
    fig1.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig1, "mc_acc_shift_test")

    temp_title = "IMU Gyroscope Shift Tests"
    axs2[2].set_xlabel('Time [s]')
    axs2[0].set_ylabel('X')
    axs2[1].set_ylabel('Y')
    axs2[2].set_ylabel('Z')
    axs2[0].grid(True)
    axs2[1].grid(True)
    axs2[2].grid(True)
    fig2.suptitle(temp_title)
    fig2.canvas.set_window_title(temp_title)
    fig2.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig2, "mc_omg_shift_test")

    temp_title = "IMU Positional Offset Covariance Tests"
    axs3[2].set_xlabel('Time [s]')
    axs3[0].set_ylabel('X')
    axs3[1].set_ylabel('Y')
    axs3[2].set_ylabel('Z')
    axs3[0].grid(True)
    axs3[1].grid(True)
    axs3[2].grid(True)
    fig3.suptitle(temp_title)
    fig3.canvas.set_window_title(temp_title)
    fig3.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig3, "mc_pos_off_settle_test")

    temp_title = "IMU Angular Offset Covariance Tests"
    axs4[2].set_xlabel('Time [s]')
    axs4[0].set_ylabel('X')
    axs4[1].set_ylabel('Y')
    axs4[2].set_ylabel('Z')
    axs4[0].grid(True)
    axs4[1].grid(True)
    axs4[2].grid(True)
    fig4.suptitle(temp_title)
    fig4.canvas.set_window_title(temp_title)
    fig4.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig4, "mc_ang_off_settle_test")

    temp_title = "IMU Accelerometer Bias Covariance Tests"
    axs5[2].set_xlabel('Time [s]')
    axs5[0].set_ylabel('X')
    axs5[1].set_ylabel('Y')
    axs5[2].set_ylabel('Z')
    axs5[0].grid(True)
    axs5[1].grid(True)
    axs5[2].grid(True)
    fig5.suptitle(temp_title)
    fig5.canvas.set_window_title(temp_title)
    fig5.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig5, "mc_acc_settle_test")

    temp_title = "IMU Gyroscope Bias Covariance Tests"
    axs6[2].set_xlabel('Time [s]')
    axs6[0].set_ylabel('X')
    axs6[1].set_ylabel('Y')
    axs6[2].set_ylabel('Z')
    axs6[0].grid(True)
    axs6[1].grid(True)
    axs6[2].grid(True)
    fig6.suptitle(temp_title)
    fig6.canvas.set_window_title(temp_title)
    fig6.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig6, "mc_omg_settle_test")


def plot_mc_flags(ekfs: List[EKF]):
    fig1, axs1 = plt.subplots(1, 1)
    fig2, axs2 = plt.subplots(1, 1)

    for ekf in ekfs:
        [settled_hist, re_init_hist] = ekf.get_flag_history()

        if ekf.get_sensor_count() > 1:

            imus = ekf.get_sensors()
            for i in range(1, ekf.get_sensor_count()):
                imu = imus[i]
                id = imu.get_id()

                # IMU Settle Flag History
                settled_hist_t = np.array([x[0] for x in settled_hist[id]])
                settled_hist_v = np.array([x[1] for x in settled_hist[id]])
                axs1.plot(settled_hist_t, settled_hist_v,
                          color="tab:blue", alpha=0.1)

                # IMU Shift Flag History
                re_init_hist_t = np.array([x[0] for x in re_init_hist[id]])
                re_init_hist_v = np.array([x[1] for x in re_init_hist[id]])
                axs2.plot(re_init_hist_t, re_init_hist_v,
                          color="tab:blue", alpha=0.1)

    temp_title = "IMU Settle Flag History"
    axs1.set_xlabel('Time [s]')
    axs1.set_ylabel('Settle Flag')
    axs1.set_yticks([0, 1])
    axs1.set_yticklabels(['False', 'True'])
    axs1.set_ylim([-1, 2])
    axs1.grid(True)
    fig1.suptitle(temp_title)
    fig1.canvas.set_window_title(temp_title)
    fig1.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig1, "mc_settle_flag")

    temp_title = "IMU Shift Flag History"
    axs2.set_xlabel('Time [s]')
    axs2.set_ylabel('Shift Flag')
    axs2.set_yticks([0, 1])
    axs2.set_yticklabels(['False', 'True'])
    axs2.set_ylim([-1, 2])
    axs2.grid(True)
    fig2.suptitle(temp_title)
    fig2.canvas.set_window_title(temp_title)
    fig2.tight_layout(rect=[0, 0.03, 1, 0.95])
    save_figure_to_dir(fig2, "mc_shift_flag")
