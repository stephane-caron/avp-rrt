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
import time


class RobotModel(object):
    """Robot model including collision checking and inverse dynamics."""
    def set_torque_limits(self, tau):
        raise NotImplementedError

    def get_current_pose(self):
        raise NotImplementedError

    def self_collides(self, pose):
        raise NotImplementedError

    def generate_pose(self):
        raise NotImplementedError

    def check_dynamics(self, pose, vel, acc):
        raise NotImplementedError


class RaveRobotModel(RobotModel):
    def __init__(self, rave_robot, tunings):
        self.rave = rave_robot
        self.tunings = tunings
        self.dof = rave_robot.GetDOF()
        v = numpy.ones(self.dof)
        self.rave.SetDOFLimits(-100 * v, +100 * v)   # no DOF limit
        self.rave.SetDOFVelocityLimits(1e40 * v)  # no velocity limit
        self.rave.SetDOFTorqueLimits(tunings.torque_limits)

    def get_current_pose(self):
        return self.rave.GetDOFValues()

    def self_collides(self, pose=None):
        raise NotImplementedError  # TODO: later
        #if pose is None:
        #    return self.rave.CheckSelfCollision()
        #cur_pose = self.get_current_pose()
        #self.rave.SetDOFValues(pose)
        #pose_coll = self.rave.CheckSelfCollision()
        #self.rave.SetDOFValues(cur_pose)
        #return pose_coll

    def generate_pose(self):
        x = numpy.random.random(self.dof)
        return (2 * x - 1.) * numpy.pi

    def compute_inertia_matrix(self, pose):
        M = numpy.zeros((self.dof, self.dof))
        self.rave.SetDOFValues(pose)
        for (i, qdd) in enumerate(numpy.eye(self.dof)):
            tm, _, _ = self.rave.ComputeInverseDynamics(qdd, None,
                                                        returncomponents=True)
            M[:, i] = tm
        return M

    def compute_forward_dynamics(self, pose, vel, torque):
        with self.rave:
            self.rave.SetDOFValues(pose)
            self.rave.SetDOFVelocities(vel)
            zero_acc = numpy.zeros((self.dof,))
            _, tc, tg = self.rave.ComputeInverseDynamics(zero_acc, None,
                                                         returncomponents=True)
            M = self.compute_inertia_matrix(pose)
            qdd = numpy.linalg.solve(M, torque - tc - tg)
        return qdd

    def compute_inverse_dynamics(self, pose, vel, acc):
        with self.rave:
            self.rave.SetDOFValues(pose)
            self.rave.SetDOFVelocities(vel)
            tau = self.rave.ComputeInverseDynamics(acc)
        return tau

    def check_dynamics(self, pose, vel, acc):
        tau = self.compute_inverse_dynamics(pose, vel, acc)
        return (abs(tau) < self.tunings.torque_limits).all()

    def check_traj_dynamics(self, traj):
        dt = self.tunings.time_prec
        q, qd, qdd = traj.q, traj.qd, traj.qdd
        trange = numpy.arange(0, traj.duration + dt, dt)
        for t in trange:
            if not self.check_dynamics(q(t), qd(t), qdd(t)):
                return t - dt
        return traj.duration

    def show_trajectory(self, traj):
        dt = self.tunings.time_prec
        for t in numpy.arange(0, traj.duration + dt, dt):
            pose = traj.q(t)
            self.rave.SetDOFValues(pose)
            time.sleep(self.tunings.time_prec)
