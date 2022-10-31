#!/usr/bin/env python3

import multiprocessing
from functools import partial

from multi_imu.sim import sim_run
from multi_imu.stats import evaluate_ekfs
from multi_imu.plot import plot_ekf_state_history, plot_ekf_cov_history, \
    plot_mc_states, plot_mc_residuals, plot_mc_tests, plot_mc_flags


num_sims = 5
time_max = 10
bump_time = 5

pool = multiprocessing.Pool(multiprocessing.cpu_count() - 1)
mp_output = pool.map(partial(sim_run, time_max=time_max,
                     bump_time=bump_time), range(num_sims))

ekfs = [tup[0] for tup in mp_output]
cfs = [tup[1] for tup in mp_output]

plot_ekf_state_history(ekfs[0])
plot_ekf_cov_history(ekfs[0])
plot_mc_states(ekfs)
plot_mc_residuals(ekfs)
plot_mc_tests(ekfs)
plot_mc_flags(ekfs)
evaluate_ekfs(ekfs, cfs)
