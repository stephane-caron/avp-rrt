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


import numpy
import pylab
import sys

sys.path.append('.')
sys.path.append('..')

from rrtcmp import VIP_RRT, TorqueSampleState, TestBed


tunings = {
    'random_seed':          111,
    'spatial_prec':        1e-2,    # [m]
    'time_prec':           1e-2,    # [s] smallest integration timestep
    'max_iter':          100000,    # max num. of extensions
    'max_simu_duration':   3600,    # [s]
    'modulo':                 5,    # steer to goal every $modulo extensions
    'rrt_neigh_size':         1,    # num. of neighbors
    'max_traj_duration':     1.,    # [s] max traj duration for each trajectory
    'torque_limits':  [11., 7.],    # [N.m]
    'Vmax':                  50,    # [rad/s^2]
    'try_exact_steering': True,
    'lqr_torque_weight':     1.,    # [N^{-1}.m^{-1}]
}

testbed = TestBed('singlerun.log', tunings,
                  StatePlannerList=[(TorqueSampleState, VIP_RRT)])

planner = testbed.get_new_planner('VIP_RRT')


if __name__ == '__main__':
    pylab.ion()
    nb_dof, max_iter = 2, tunings['max_iter']
    X1 = numpy.random.random((max_iter, nb_dof))
    X2 = numpy.random.random((max_iter, nb_dof))
    poses = numpy.pi * (2. * X1 - 1.)
    velocities = tunings['Vmax'] * (2. * X2 - 1.)

    if False:
        traj = planner.run(poses, velocities)
    else:
        print "\nThe following variables are defined:"
        print "- max_iter =", max_iter
        print "- planner: your VIP-RRT planner object"
        print "- poses: (%d, 2) array of random joint-angles" % max_iter
        print "- velocities: (%d, 2) array of random velocities" % max_iter
        print ""
        print "To run:"
        print ""
        print "    traj = planner.run(poses, velocities)"

    try:
        __IPYTHON__
    except NameError:
        print ""
        import IPython
        IPython.embed()
