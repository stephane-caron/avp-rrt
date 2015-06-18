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
from numpy.random import random, randint

from trajectory import Trajectory
from misc import center_angle


class VirtualState(object):
    """States provide the metric (dist) and steering (steer) functions required
    by RRT planners."""

    Factory = None     # set through the @add_state_factory decorator
    Trajectory = None  # automatically added to curried state classes

    def dist(self, other_state):
        raise NotImplementedError

    def steer(self, dest_state):
        raise NotImplementedError


def add_state_factory(State):
    """This decorator adds the Factory attribute to a State class."""
    @staticmethod
    def Factory(tunings):
        class CurriedState(State):
            def __init__(self, *args, **kwargs):
                kwargs['tunings'] = tunings
                return super(CurriedState, self).__init__(*args, **kwargs)
        return CurriedState
    State.Factory = Factory
    return State


@add_state_factory
class EuclideanState(VirtualState):
    def __init__(self, q, qd, tunings=None):
        assert len(q) == len(qd)
        self.q = numpy.array([center_angle(qi) for qi in q])
        self.qd = numpy.array(qd)
        self.tunings = tunings

    def get_pose(self):
        return self.q

    def dist(self, other_state):
        from numpy import sqrt, cos
        dq = sum(sqrt(1. - cos(self.q - other_state.q))) / 4.
        dv = sum(abs(self.qd - other_state.qd)) / (4. * self.tunings.Vmax)
        return (dq + dv) / 2

    def __eq__(self, other_state):
        return self.dist(other_state) < self.tunings.spatial_prec

    def __ne__(self, other_state):
        return self.dist(other_state) > self.tunings.spatial_prec

    def __str__(self):
        return "State(q=%s, qd=%s)" % (str(self.q), str(self.qd))

    def steer(self, dest_state):
        raise NotImplementedError


@add_state_factory
class LinAccState(EuclideanState):
    """Here we interpolate a (joint-wise) linear acceleration profile,
    from 'self' to a target state. Trajectory duration is optimized
    through dichotomy."""

    def steer(self, dest_state, robot):
        # Let qd(t) = a T^2 + b T + v0
        # Solve for qd(T) = qd1, q(0) = q0, q(T) = q1
        q0, v0 = self.q, self.qd
        q1, v1 = dest_state.q, dest_state.qd
        Dq, Dv = (q1 - q0), (v1 - v0)
        dt = self.tunings.time_prec

        def solve_with_duration(T):
            b = 2. / T * (3. * (Dq / T - v0) - Dv)
            a = (Dv / T - b) / T
            qdd_fun = lambda t: 2 * a * t + b
            qd_fun = lambda t: a * t ** 2. + b * t + v0
            q_fun = lambda t: a * t ** 3. / 3. + b * t ** 2. / 2 + v0 * t + q0
            traj = Trajectory(T, q_fun, qd_fun, qdd_fun)
            max_duration = robot.check_traj_dynamics(traj)
            if max_duration > 2 * dt:
                traj.duration = max_duration
                return traj
            return None

        # optimized for local steering:
        Tsamples = [k * dt for k in [3, 5, 10, 20, 50, 100]]
        local_trajectories = [solve_with_duration(T) for T in Tsamples]
        local_trajectories = filter(None, local_trajectories)
        if len(local_trajectories) < 1:
            return None

        def traj_score(traj):
            x = traj.last_state()
            q, qd = x[:2], x[2:]
            last_state = self.__class__(q, qd, tunings=self.tunings)
            return last_state.dist(dest_state)

        best_traj = min(local_trajectories, key=traj_score)
        return best_traj


@add_state_factory
class TorqueSampleState(EuclideanState):
    """..."""

    def steer(self, dest_state, robot):
        """..."""
        dt = self.tunings.time_prec
        nb_traj_samples = self.tunings.nb_traj_samples
        max_traj_duration = self.tunings.max_traj_duration
        max_timesteps = int(max_traj_duration / dt)
        assert max_timesteps > 3

        def integrate_cst_torque(duration, tau):
            # TODO; put in robot
            q, qd = self.q, self.qd
            q_list, qd_list = [q], [qd]
            for t in numpy.arange(0, duration, dt):
                qdd = robot.compute_forward_dynamics(q, qd, tau)
                q = q + qd * dt     # NB: not the same semantic as +=
                qd = qd + qdd * dt  # same here
                q_list.append(numpy.array([center_angle(qi) for qi in q]))
                qd_list.append(qd)
            q_fun = lambda t: q_list[int(t / dt)] if t < duration \
                else q_list[-1]
            qd_fun = lambda t: qd_list[int(t / dt)] if t < duration \
                else qd_list[-1]
            tau_fun = lambda t: tau
            traj = Trajectory(duration, q_fun, qd_fun, tau_fun)
            return traj

        local_trajectories = []
        for _ in xrange(nb_traj_samples):
            duration = randint(3, max_timesteps) * dt
            tau = (2. * random(2) - 1.) * self.tunings.torque_limits
            local_trajectories.append(integrate_cst_torque(duration, tau))

        def traj_score(traj):
            x = traj.last_state()
            q, qd = x[:2], x[2:]
            last_state = self.__class__(q, qd, tunings=self.tunings)
            return last_state.dist(dest_state)

        best_traj = min(local_trajectories, key=traj_score)

        return best_traj
