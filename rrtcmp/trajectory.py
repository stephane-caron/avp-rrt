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


import bisect
import numpy


class Trajectory(object):
    def __init__(self, duration, q_fun, qd_fun, qdd_fun=None, tau_fun=None):
        assert not (qdd_fun is None and tau_fun is None)
        self.duration = duration
        self.q = q_fun
        self.qd = qd_fun
        self.qdd = qdd_fun
        self.tau = tau_fun

    def sample_poses(self, trange):
        for t in trange:
            yield self.q(t)

    def last_state(self):
        q, qd = self.q(self.duration), self.qd(self.duration)
        return numpy.hstack([q, qd])


class TrajectoryList(object):
    def __init__(self, traj_list):
        nb_traj = len(traj_list)
        traj_durations = [traj.duration for traj in traj_list]
        self.traj_list = traj_list
        self.duration = sum(traj_durations)
        self.cum_durations = [sum(traj_durations[0:i]) for i in range(nb_traj)]
        #for field in ['q', 'qd', 'qdd', 'tau', 'sample_state']:
        #    self.__dict__[field] = lambda t: self._pass_to_traj(t, field)

    def _traj_for_time(self, t):
        i = bisect.bisect(self.cum_durations, t)
        assert i > 0, "The first cumulative time should be zero..."
        traj_index = i - 1
        t_start = self.cum_durations[traj_index]
        traj = self.traj_list[traj_index]
        return traj, (t - t_start)

    def q(self, t):
        traj, t2 = self._traj_for_time(t)
        return traj.q(t2)
