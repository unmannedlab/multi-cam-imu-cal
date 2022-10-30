#!/usr/bin/env python3

from multi_imu.truth import TruthModel
from multi_imu.imu import IMU
from multi_imu.ekf import EKF
from multi_imu.cf  import CF

import os
import numpy as np

def sim_run(run: int, time_max, bump_time = None):
    np.random.seed(int.from_bytes(os.urandom(4), byteorder='little'))

    truth_model = TruthModel(
        np.random.uniform(-1.0, 1.0),
        np.random.uniform(-1.0, 1.0),
        np.random.uniform(-1.0, 1.0),
        np.random.uniform(-1.0, 1.0),
        np.random.uniform(-1.0, 1.0),
        np.random.uniform(-1.0, 1.0),
    )

    imu0 = IMU(0, truth_model)
    imu0.set_hz(400)
    imu0.set_acc_sigma(0.001, 0.001, 0.001)
    imu0.set_omg_sigma(0.01, 0.01, 0.01)

    imu1 = IMU(1, truth_model)
    imu1.set_hz(100)
    imu1.set_acc_sigma(0.01, 0.01, 0.01)
    imu1.set_omg_sigma(0.1, 0.1, 0.1)

    imu2 = IMU(2, truth_model)
    imu2.set_hz(100)
    imu2.set_acc_sigma(0.01, 0.01, 0.01)
    imu2.set_omg_sigma(0.1, 0.1, 0.1)

    imus = [imu0, imu1, imu2]
    ekf = EKF(imus, run)

    id  = np.zeros([0])
    t   = np.zeros([0])
    acc = np.zeros([0, 3])
    omg = np.zeros([0, 3])
    r   = np.zeros([0, 6])

    for imu in ekf.get_sensors(): 
        if (imu.get_id() != 0):
            imu.set_pos_offset(
                np.random.uniform(-0.1, 0.1),
                np.random.uniform(-0.1, 0.1),
                np.random.uniform(-0.1, 0.1),
                time = 0
            )
            imu.set_ang_offset(
                np.random.uniform(-0.1, 0.1),
                np.random.uniform(-0.1, 0.1),
                np.random.uniform(-0.1, 0.1),
                time = 0
            )

        if (bump_time is not None):
            [id0, t0, acc0, omg0, r0] = imu.get_measurements(bump_time)
        else:
            [id0, t0, acc0, omg0, r0] = imu.get_measurements(time_max)

        if (imu.get_id() != 0):
            pos_off = imu.get_pos_offset()
            ang_off = imu.get_ang_offset()
            imu.set_pos_offset(
                pos_off[0,0] + np.random.uniform(-0.2, 0.2),
                pos_off[0,1] + np.random.uniform(-0.2, 0.2),
                pos_off[0,2] + np.random.uniform(-0.2, 0.2),
                time = bump_time
            )
            imu.set_ang_offset(
                ang_off[0,0] + np.random.uniform(-0.2, 0.2),
                ang_off[0,1] + np.random.uniform(-0.2, 0.2),
                ang_off[0,2] + np.random.uniform(-0.2, 0.2),
                time = bump_time
            )

        id  = np.concatenate((id,  id0 ))
        t   = np.concatenate((t,   t0  ))
        acc = np.concatenate((acc, acc0))
        omg = np.concatenate((omg, omg0))
        r   = np.concatenate((r,   r0  ))

        if (bump_time is not None):
            [id1, t1, acc1, omg1, r1] = imu.get_measurements(time_max, bump_time + imu.get_period())
            id  = np.concatenate((id,  id1 ))
            t   = np.concatenate((t,   t1  ))
            acc = np.concatenate((acc, acc1))
            omg = np.concatenate((omg, omg1))
            r   = np.concatenate((r,   r1  ))

    sort_indices = t.argsort()
    id  = np.take(id,  sort_indices)
    t   = np.take(t,   sort_indices)
    acc = np.take(acc, sort_indices, axis=0)
    omg = np.take(omg, sort_indices, axis=0)
    r   = np.take(r,   sort_indices, axis=0)

    ekf.init_state_history(len(t))
    for i in range(0, len(t)):
        ekf.update(id[i], t[i], acc[i], omg[i], r[i])

    cf = CF(imus, t, acc, omg, r, run)

    return ekf, cf