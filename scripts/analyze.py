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
import os
import pickle
import pylab
import sys

sys.path.append('.')
sys.path.append('..')

from numpy import array, cos, pi, sqrt
from rrtcmp import Tunings, TorqueSampleState


class Dump(object):
    def __init__(self, dump_dict):
        self.tunings = Tunings(dump_dict['tunings'])
        State = TorqueSampleState.Factory(self.tunings)
        goal_state = State(q=[-pi, 0], qd=[0, 0])

        class Node(object):
            def __init__(self, node_dict):
                self.__dict__.update(node_dict)
                if 'qd' in node_dict:  # RRT
                    self.state = State(q=self.q, qd=self.qd)
                    self.goal_dist = goal_state.dist(self.state)
                else:  # VIP-RRT
                    dq = sum(sqrt(1. - cos(self.q - goal_state.q))) / 4.
                    dv = self.v_min
                    self.goal_dist = (dq + dv) / 2.

        class Run(object):
            def __init__(self, run_dict):
                self.__dict__.update(run_dict)
                self.nodes = [Node(node) for node in run_dict['nodes']]
                if len(self.nodes) > 0:
                    self.score = min([node.goal_dist for node in self.nodes])
                else:
                    self.score = 2.0

        class VIPRun(Run):
            def __init__(self, run_dict):
                super(VIPRun, self).__init__(run_dict)
                forgotten_root = {
                    'q': array([0., 0.]),
                    'parent': None,
                    'v_min': 0.,
                    'v_max': 0.,
                    'iterstep': 0,
                    'timestamp': self.start_time + 1e-3}
                if len(self.nodes) > 0:
                    max_iterstep = max([node.iterstep for node in self.nodes])
                    max_timestamp = max([node.timestamp for node in self.nodes])
                else:
                    max_iterstep = 0
                    max_timestamp = 0
                forgotten_goal = {
                    'q': array([-pi, 0]),
                    'parent': None,
                    'v_min': 0.,
                    'v_max': 0.,
                    'iterstep': max_iterstep + 1,
                    'timestamp': max_timestamp + 1e-3}
                self.nodes = [Node(forgotten_root)] + self.nodes
                self.nodes.append(Node(forgotten_goal))

        self.rrt_runs = [Run(p['van_rrt']) for p in dump_dict['planners'] if 'van_rrt' in p.keys()]
        self.vip_runs = [VIPRun(p['vip_rrt']) for p in dump_dict['planners'] if 'vip_rrt' in p.keys()]
        # TODO: LQR-RRT here
        self.goal_state = goal_state


def plot_run(run, color):
    # Hack to exclude the case when VIP has not been run
    if len(run.nodes) < 3:
        return
    xvals, yvals, cur_dist = [], [], 1e15
    for node in run.nodes:
        cur_dist = node.goal_dist if node.goal_dist < cur_dist else cur_dist
        xvals.append(node.timestamp - run.start_time)
        yvals.append(cur_dist)
    pylab.plot(xvals, yvals, color)


def plot_dump(dump, subplot=111):
    pylab.subplot(subplot)
    map(lambda run: plot_run(run, 'g-'), dump.rrt_runs)
    map(lambda run: plot_run(run, 'b-'), dump.vip_runs)
    xvals, yvals, _ = compute_means(dump.rrt_runs)
    pylab.plot(xvals, yvals, 'g-', linewidth=5)
    xvals, yvals, _ = compute_means(dump.vip_runs)
    pylab.plot(xvals, yvals, 'b-', linewidth=5)
    pylab.xscale('log')
    pylab.yscale('linear')
    pylab.grid(True)
    pylab.xlim(1., 2e4)
    pylab.ylim(0, 2e-1)
    title = str(dump.tunings)
    title = title.replace('[', '\n[')
    if 'run_vip = False' in title:
        title = title.replace('\n[run_vip = False]', '')
        title = title.replace('Tunings with', 'RRT')
    else:
        title = title.replace('Default tunings', 'VIP-RRT')
    pylab.title(title)


def first_entry(run, threshold, default):
    for i in range(len(run.nodes)):
        if run.nodes[i].goal_dist < threshold:
            return run.nodes[i].timestamp - run.start_time
    return default


def cumul(runs):
    nbruns = len(runs)+0.
    maxtime = max([run.nodes[-1].timestamp-run.start_time for run in runs])+100
    endtime = 2e4
    x = [first_entry(run, 1e-2, endtime) for run in runs]
    n = x.count(endtime)
    for i in range(n):
        x.remove(endtime)
    x.sort()
    y = [(len(filter(lambda t: t <= xi, x))) / nbruns for xi in x]
    x.insert(0, 0)
    y.insert(0, 0)
    x = x*2
    y = y*2
    x.sort()
    y.sort()
    x.pop(0)
    y.pop()
    x.append(maxtime)
    y.append(y[-1])
    return numpy.array(x), numpy.array(y)


def compute_means(runs):
    """
    Compute the time-series (t, mean(dist_to_goal), std(dist_to_goal))
    for a set of runs.

    """
    # exclude the case when VIP has not been run
    if len(runs[0].nodes) < 3:
        return [0], [0], [0]
    nruns = len(runs)
    times = map(lambda run: [node.timestamp - run.start_time for node in
                             run.nodes], runs)
    dists = map(lambda run: [node.goal_dist for node in run.nodes], runs)

    def mindist(distvect):
        res = numpy.zeros(len(distvect)+1)
        m = 1e10
        for i in xrange(len(distvect)):
            m = min(distvect[i], m)
            res[i] = m
        if(res[-2] > 1e-2):
            res[-1] = res[-2]
        return res

    mindists = [mindist(dv) for dv in dists]
    maxtime = max([t[-1] for t in times])
    time_vect = numpy.arange(0, maxtime + 10, 1)

    def interpolate_dist(timesv, distsv, t):
        idx = bisect.bisect_left(timesv, t)
        return distsv[min(idx, len(distsv) - 1)]

    dist_matrix = numpy.zeros((nruns, len(time_vect)))
    for i in range(len(time_vect)):
        for j in range(len(times)):
            dist_matrix[j, i] = interpolate_dist(times[j], mindists[j],
                                                 time_vect[i])
    return time_vect, numpy.mean(dist_matrix, 0), numpy.std(dist_matrix, 0)


if __name__ == '__main__':
    assert len(sys.argv) > 1, "Usage: %s <trace_path>" % sys.argv[0]
    trace_dir = sys.argv[1]

    print "\n--\nLoading dumps from %s..." % trace_dir
    files = os.listdir(trace_dir)
    with open('%s/poses.pkl' % trace_dir, 'r') as f:
        poses = pickle.load(f)
    with open('%s/velocities.pkl' % trace_dir, 'r') as f:
        velocities = pickle.load(f)

    dump_files = filter(lambda s: s.find("dump") >= 0, files)
    dump_dicts = []
    for dump_file in dump_files:
        with open('%s/%s' % (trace_dir, dump_file), 'r') as f:
            dump_dicts.append(pickle.load(f))
    dumps = [Dump(dd) for dd in dump_dicts]

    for i, dump in enumerate(dumps):
        print "Dump %d: %s" % (i, str(dump.tunings))
        maxtime = lambda run: (run.nodes[-1].timestamp-run.start_time) < 2e4
        dump.rrt_runs = filter(maxtime, dump.rrt_runs)
        dump.vip_runs = filter(maxtime, dump.vip_runs)

    print "All dumps loaded in `dumps` list."

    if "with-plots" in sys.argv:
        pylab.ion()
        pylab.clf()
        for (i, dump) in enumerate(dumps):
            plot_dump(dump, subplot=241 + i)
