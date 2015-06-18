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
import openravepy
import os
import pickle
import time

from robot import RaveRobotModel
from rrt import RRT
from state import TorqueSampleState
from tunings import Tunings


def get_openrave_robot(display=False):
    rave_env = openravepy.Environment()
    fpath = 'model/pendulum.env.xml'
    if os.path.isfile('./' + fpath):
        rave_env.Load('./' + fpath)
    elif os.path.isfile('../' + fpath):
        rave_env.Load('../' + fpath)
    else:
        assert False, "Where is %s?" % fpath
    if display:
        rave_env.SetViewer('qtcoin')
        viewer = rave_env.GetViewer()
        cam_trans = numpy.array([
            [0, 0, -1, 1.1], [1, 0, 0, 0],
            [0, -1, 0, 0.3], [0, 0, 0, 1]])
        viewer.SetCamera(cam_trans)
    rave_robot = rave_env.GetRobots()[0]
    rave_robot.GetEnv().GetPhysicsEngine().SetGravity([0, 0, -9.81])
    rave_robot.SetTransform(numpy.array([
        [0, 0, 1, 0], [0, 1, 0, 0],
        [-1., 0, 0, 0.3], [0, 0, 0, 1]]))
    return rave_robot


class TestBed(object):
    def __init__(self, dump_file=None, tunings_dict=None, custom_tunings=None,
                 StatePlannerList=[], display=False):
        self.tunings = Tunings(tunings_dict)
        if custom_tunings:
            self.tunings.update(custom_tunings)
        numpy.random.seed(self.tunings.random_seed)

        self.start_time = time.time()
        self.pid = os.getpid()
        self.dump_file = dump_file
        self.plan_log = []

        self.robot = RaveRobotModel(get_openrave_robot(display=display),
                                    self.tunings)

        if len(StatePlannerList) < 1:
            StatePlannerList.append((RRT, TorqueSampleState))
        self.Planners = self._curry_planners(StatePlannerList)

    def _curry_planners(self, StatePlannerList):
        def porky_curry(StateClass, PlannerClass):
            tunings = self.tunings
            State = StateClass.Factory(tunings)
            init_state = State(q=[0., 0.], qd=[0., 0.])
            goal_state = State(q=[-numpy.pi, 0.], qd=[0., 0.])
            robot = self.robot

            class CurriedRRT(PlannerClass):
                def __init__(self):
                    kron = [robot, init_state, goal_state, tunings]
                    return super(CurriedRRT, self).__init__(*kron)
            return CurriedRRT

        CurriedPlanners = {}
        for (State, Planner) in StatePlannerList:
            pname = Planner.get_name()
            CurriedPlanners[pname] = porky_curry(State, Planner)
        return CurriedPlanners

    def get_new_planner(self, pname):
        PClass = self.Planners[pname]
        return PClass()

    def run(self, rand_poses, rand_velocities):
        sims_per_case = self.tunings.sims_per_case
        start_index = 0
        if 'start_index' in self.tunings.__dict__.keys():
            start_index = self.tunings.start_index
        with open('../logs/process-%d.log' % self.pid, 'w') as fh:
            def log_msg(msg):
                dt = time.time() - self.start_time
                msg = msg.replace("xxx", "%.2f" % dt)
                fh.write("%s\n" % msg)
                fh.flush()
            log_msg("Test Case: %s\n" % str(self.tunings))
            log_msg("Start:")
            for rr in xrange(sims_per_case):
                r = rr + start_index
                run_dict = {}
                log_msg(" | Run #%d" % r)
                for pname, PClass in self.Planners.iteritems():
                    log_msg(" |  | [xxx] %s: starting..." % pname)
                    pinst = PClass()
                    pinst.run(rand_poses[r], rand_velocities[r])
                    log_msg(" |  | [xxx] %s: done." % pname)
                    run_dict[pname] = pinst.get_dict_repr()
                self.plan_log.append(run_dict)
                self.dump()
            log_msg("All done.\n")

    def dump(self):
        assert self.dump_file is not None
        tun_dic = self.tunings.__dict__.copy()
        dump_dic = {'tunings': tun_dic, 'planners': self.plan_log}
        with open(self.dump_file, 'w') as f:
            pickle.dump(dump_dic, f)


sample_tunings = {
    'spatial_prec':       1e-2,
    'time_prec':          1e-2,
    'max_iter':           1000,
    'max_simu_duration':    20,
    'modulo':                5,
    'rrt_neigh_size':        1,
    'nb_traj_samples':       1,
    'max_traj_duration':    1.,
    'torque_limits': [13., 5.],
    'Vmax':                 50,
    'lqr_torque_weight':   10.,
}
