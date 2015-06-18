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
import time

import vippy.MintimeTrajectory as MintimeTrajectory
import vippy.MintimeProblemTorque as MintimeProblemTorque
import vippy.MintimeProfileIntegrator as MintimeProfileIntegrator
import vippy.Kinodynamic as Kinodynamic
from trajectory import Trajectory


class VIP_RRT(object):
    @staticmethod
    def get_name():
        return 'VIP_RRT'

    def __init__(self, robot, init_state, goal_state, tunings):
        self.robot = robot
        self.init_state = init_state
        self.goal_state = goal_state
        self.tunings = tunings
        self.plotting = None
        self.trajres = None
        self.nb_ext = 0
        self.nb_succ_ext = 0
        self.has_found = False
        self.start_time = time.time()
        self.trace = []
        self.State = init_state.__class__

    def run(self, samples_list, *args):
        env = self.robot.rave.GetEnv()
        tuningsvip = Kinodynamic.Tunings()
        tau0 = self.tunings.torque_limits[0]
        tau1 = self.tunings.torque_limits[1]
        tuningsvip.limits = [numpy.array([-tau0, -tau1]),
                             numpy.array([tau0, tau1])]
        tuningsvip.grav = env.GetPhysicsEngine().GetGravity()
        tuningsvip.checkcoll = False
        tuningsvip.t_step = 0.005
        # time step to integrate the limiting curves:
        tuningsvip.dt_integ = tuningsvip.t_step / 5
        tuningsvip.disc_thr = 1e15
        tuningsvip.width = 10
        tuningsvip.palier = 10
        tuningsvip.tolerance_ends = 1e-2

        tuningsvip.dicho_steps = 10
        tuningsvip.n_close = 10
        tuningsvip.n_rand = 0
        tuningsvip.try_zero = True

        q_start = self.init_state.q
        v_start = 0  # dummy
        q_target = self.goal_state.q
        v_target = numpy.linalg.norm(self.goal_state.qd)
        robot = self.robot.rave
        q_range = []  # dummy
        K = self.tunings.max_iter

        # Search for solution
        res = Kinodynamic.Bi_CKRRT(robot, q_start, v_start, q_target, v_target,
                                   q_range, K, tuningsvip, bidir=False,
                                   samples_list=samples_list,
                                   dirty_backref=self)

        if res[0] == -1:
            return None

        # Process the results
        ver_list = res[3]
        pwp_traj_list = res[1]
        traj_list = [pwp_traj.GetSampleTraj(pwp_traj.duration,
                                            tuningsvip.t_step)
                     for pwp_traj in pwp_traj_list]

        # Construct the trajectories
        pb_list = []
        T_list = [pwp_traj.duration for pwp_traj in pwp_traj_list]
        traj2_list = []
        for i in range(len(T_list)):
            traj = traj_list[i]
            pb = MintimeProblemTorque.MintimeProblemTorque(robot, traj)
            pb.set_dynamics_limits(tuningsvip.limits)
            pb.disc_thr = tuningsvip.disc_thr
            pb.preprocess()
            pb_list.append(pb)
            # Integrate profiles
            algo = MintimeProfileIntegrator.MintimeProfileIntegrator(pb)
            algo.dt_integ = tuningsvip.dt_integ / 10
            algo.width = tuningsvip.width
            algo.palier = tuningsvip.palier
            algo.tolerance_ends = tuningsvip.tolerance_ends
            algo.sdot_init = 1e-4
            algo.sdot_final = 1e-4
            algo.integrate_all_profiles()
            algo.integrate_final()
            #algo.plot_profiles(sum(T_list[0:i]))
            #algo.plot_profiles()
            # Resample the trajectory
            s_res = algo.s_res
            sdot_res = algo.sdot_res
            undersample_coef = int(round(traj.t_step / algo.dt_integ))
            s_res_u = s_res[range(1, len(s_res), undersample_coef)]
            sdot_res_u = sdot_res[range(1, len(s_res), undersample_coef)]
            traj2 = pwp_traj_list[i].ResampleTraj(s_res_u, sdot_res_u,
                                                  traj.t_step)
            traj2_list.append(traj2)

        traj2_final = MintimeTrajectory.Concat(list(traj2_list))

        trajres = Trajectory(traj2_final.duration,
                             q_fun=traj2_final.value,
                             qd_fun=traj2_final.velocity,
                             qdd_fun=traj2_final.acceleration)

        self.plotting = (ver_list, q_start, q_target, traj2_list)
        x = trajres.last_state()
        last_state = self.State(q=x[:2], qd=x[2:])

        if last_state == self.goal_state:
            self.trajres = trajres
            self.has_found = True
            return trajres

        return None

    def plot(self):
        assert self.plotting
        ver_list, q_start, q_target, traj_list = self.plotting
        pylab.figure(1)
        Kinodynamic.plot_tree(ver_list, q_start, q_target, traj_list)
        pylab.axis([-4, 4, -4, 4])
        pylab.grid(True)
        pylab.draw()

    def get_dict_repr(self):
        return {
            'nodes': self.trace,
            'nb_ext': self.nb_ext,
            'nb_succ_ext': self.nb_succ_ext,
            'start_time': self.start_time,
            'has_found': self.has_found
        }

    def get_trajectory(self):
        return self.trajres
