#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2013 Stephane Caron <stephane.caron@normalesup.org>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""
Run a benchmark for the inverted pendulum with torque limits (13, 5) Nm.

Planners:
    - VIP-RRT
    - 1NN-RRT
    - 10NN-RRT
    - 100NN-RRT
"""

import datetime
import numpy
import os
import pickle
import sys

sys.path.append('.')
sys.path.append('..')

from multiprocessing import Process
from rrtcmp import TestBed
from rrtcmp import TorqueSampleState
from rrtcmp import RRT, VIP_RRT


class Benchmark(object):
    pass

benchmark = Benchmark()

benchmark.tunings = {
    'sims_per_case':      1,      # Number of runs
    'random_seed':       42,      # seed for random number generation

    # Precision
    'spatial_prec':     1e-2,     # [m]
    'time_prec':        1e-2,     # [s] smallest integration timestep

    # Extension parameters
    'max_iter':         6000,     # max num. of extensions
    'max_simu_duration': 1e4,     # max duration for each run in seconds
    'modulo':              5,     # steer to goal every $modulo extensions
    'rrt_neigh_size':     40,     # num. of neighbors
    'nb_traj_samples':    20,     # num. of traj. tested / neighbor / extension
    'max_traj_duration': 1.0,     # max traj duration for each trajectory

    # Pendulum characteristics
    'torque_limits':  [13., 5.],  # [N.m]
    'Vmax':  50,                  # [rad/s^2]

    # RRT-specific
    'try_exact_steering': False,  # use LinAcc steering to goal at every step?
}

benchmark.sims_per_case_max = 40
benchmark.max_iter_max = 10000

benchmark.cases = [
    {'run_vip': False},  # defaults            # (40 neighbor + VIP) x 40 sims
    {'rrt_neigh_size': 1, 'sims_per_case': 20},
    {'rrt_neigh_size': 1, 'sims_per_case': 20, 'start_index': 20},
    {'rrt_neigh_size': 10, 'sims_per_case': 20},
    {'rrt_neigh_size': 10, 'sims_per_case': 20, 'start_index': 20},
    {'rrt_neigh_size': 100, 'sims_per_case': 20},
    {'rrt_neigh_size': 100, 'sims_per_case': 20, 'start_index': 20},
]

benchmark.trace_dir = './traces/%s' % str(datetime.datetime.today())

sims_per_case_max = benchmark.sims_per_case_max
max_iter_max = benchmark.max_iter_max
Vmax, nb_dof = benchmark.tunings['Vmax'], 2  # the pendulum has two DOFs

X1 = numpy.random.random((sims_per_case_max, max_iter_max, nb_dof))
X2 = numpy.random.random((sims_per_case_max, max_iter_max, nb_dof))
benchmark.rand_poses = numpy.pi * (2. * X1 - 1.)
benchmark.rand_velocities = Vmax * (2. * X2 - 1.)


def run_and_log(benchmark, case_params):
    dump_file = '%s/dump-%d.pkl' % (benchmark.trace_dir, os.getpid())
    StatePlannerList = [(TorqueSampleState, RRT)]
    if 'run_vip' in case_params.keys() and case_params['run_vip']:
        StatePlannerList.append((TorqueSampleState, VIP_RRT))
    test = TestBed(dump_file, benchmark.tunings, custom_tunings=case_params,
                   StatePlannerList=StatePlannerList)
    test.run(benchmark.rand_poses, benchmark.rand_velocities)


if __name__ == "__main__":
    os.mkdir(benchmark.trace_dir)
    for test_case in benchmark.cases:
        Process(target=run_and_log, args=(benchmark, test_case)).start()
    with open('%s/poses.pkl' % benchmark.trace_dir, 'w') as f:
        pickle.dump(benchmark.rand_poses, f)
    with open('%s/velocities.pkl' % benchmark.trace_dir, 'w') as f:
        pickle.dump(benchmark.rand_velocities, f)
