#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2013 Quang-Cuong Pham <cuong.pham@normalesup.org>
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


import analyze
import numpy
import os
import pickle
import pylab
import sys

sys.path.append('.')
sys.path.append('..')


# Load traces
assert len(sys.argv) > 1, "Usage: %s <trace_path>" % sys.argv[0]
trace_dir = sys.argv[1]
files = os.listdir(trace_dir)
with open('%s/poses.pkl' % trace_dir, 'r') as f:
    poses = pickle.load(f)
with open('%s/velocities.pkl' % trace_dir, 'r') as f:
    velocities = pickle.load(f)
dump_files = filter(lambda s: s.find("dump") >= 0, files)
dump_files.sort()
dump_dicts = []
for dump_file in dump_files:
    with open('%s/%s' % (trace_dir, dump_file), 'r') as f:
        dump_dicts.append(pickle.load(f))
dumps = [analyze.Dump(dd) for dd in dump_dicts]


if "traj-per-extension" in trace_dir:
    dump10, dump30, dump80, _, _, _, _, _ = dumps
    assert dump10.tunings.nb_traj_samples == 10
    assert dump30.tunings.nb_traj_samples == 30
    assert dump80.tunings.nb_traj_samples == 80
    vip = dump10.vip_runs
    rrt10 = dump10.rrt_runs
    rrt30 = dump30.rrt_runs
    rrt80 = dump80.rrt_runs
    ferrt10x, ferrt10y = analyze.cumul(rrt10)
    ferrt30x, ferrt30y = analyze.cumul(rrt30)
    ferrt80x, ferrt80y = analyze.cumul(rrt80)
    xrrt10, yrrt10, errt10 = analyze.compute_means(rrt10)
    xrrt30, yrrt30, errt30 = analyze.compute_means(rrt30)
    xrrt80, yrrt80, errt80 = analyze.compute_means(rrt80)
elif "max-traj-duration" in trace_dir:
    dump05, dump10, dump20 = dumps
    assert abs(dump05.tunings.max_traj_duration - 0.5) < 1e-10
    assert abs(dump10.tunings.max_traj_duration - 1.0) < 1e-10
    assert abs(dump20.tunings.max_traj_duration - 2.0) < 1e-10
    rrt05 = dump05.rrt_runs
    rrt10 = dump10.rrt_runs
    rrt20 = dump20.rrt_runs
    ferrt05x, ferrt05y = analyze.cumul(rrt05)
    ferrt10x, ferrt10y = analyze.cumul(rrt10)
    ferrt20x, ferrt20y = analyze.cumul(rrt20)
    xrrt05, yrrt05, errt05 = analyze.compute_means(rrt05)
    xrrt10, yrrt10, errt10 = analyze.compute_means(rrt10)
    xrrt20, yrrt20, errt20 = analyze.compute_means(rrt20)
elif "modulo" in trace_dir:
    dump10, dump5, dump40 = dumps
    assert dump5.tunings.modulo == 5
    assert dump10.tunings.modulo == 10
    assert dump40.tunings.modulo == 40
    rrt5 = dump5.rrt_runs
    rrt10 = dump10.rrt_runs
    rrt40 = dump40.rrt_runs
    ferrt5x, ferrt5y = analyze.cumul(rrt5)
    ferrt10x, ferrt10y = analyze.cumul(rrt10)
    ferrt40x, ferrt40y = analyze.cumul(rrt40)
    xrrt5, yrrt5, errt5 = analyze.compute_means(rrt5)
    xrrt10, yrrt10, errt10 = analyze.compute_means(rrt10)
    xrrt40, yrrt40, errt40 = analyze.compute_means(rrt40)
    a = 1
elif "11-07" in trace_dir:
    dump1, dump10, dump40a, dump40b, dump100a, dump100b, dump100c = dumps
    assert dump1.tunings.rrt_neigh_size == 1
    assert dump10.tunings.rrt_neigh_size == 10
    assert dump40a.tunings.rrt_neigh_size == 40
    assert dump40b.tunings.rrt_neigh_size == 40
    assert dump100a.tunings.rrt_neigh_size == 100
    assert dump100b.tunings.rrt_neigh_size == 100
    assert dump100c.tunings.rrt_neigh_size == 100
    vip = dump1.vip_runs
    rrt1 = dump1.rrt_runs
    rrt10 = dump10.rrt_runs
    rrt40 = dump40a.rrt_runs
    rrt40.extend(dump40b.rrt_runs)
    rrt100 = dump100a.rrt_runs
    rrt100.extend(dump100b.rrt_runs)
    rrt100.extend(dump100c.rrt_runs)
    fevipx, fevipy = analyze.cumul(vip)
    ferrt1x, ferrt1y = analyze.cumul(rrt1)
    ferrt10x, ferrt10y = analyze.cumul(rrt10)
    ferrt40x, ferrt40y = analyze.cumul(rrt40)
    ferrt100x, ferrt100y = analyze.cumul(rrt100)
elif "11-05" in trace_dir:
    dump40, dump10, dump1a, dump1b, dump100a, dump100b = dumps
    assert dump1a.tunings.rrt_neigh_size == 1
    assert dump1b.tunings.rrt_neigh_size == 1
    assert dump10.tunings.rrt_neigh_size == 10
    assert dump40.tunings.rrt_neigh_size == 40
    assert dump100a.tunings.rrt_neigh_size == 100
    assert dump100b.tunings.rrt_neigh_size == 100
    vip = dump40.vip_runs
    rrt1 = dump1a.rrt_runs
    rrt1.extend(dump1b.rrt_runs)
    rrt10 = dump10.rrt_runs
    rrt40 = dump40.rrt_runs
    rrt100 = dump100a.rrt_runs
    rrt100.extend(dump100b.rrt_runs)
    fevipx, fevipy = analyze.cumul(vip)
    ferrt1x, ferrt1y = analyze.cumul(rrt1)
    ferrt10x, ferrt10y = analyze.cumul(rrt10)
    ferrt40x, ferrt40y = analyze.cumul(rrt40)
    ferrt100x, ferrt100y = analyze.cumul(rrt100)


#####################################################
# Plotting


# Nb trajs
if "traj-per-extension" in trace_dir:
    pylab.ion()
    pylab.figure(1)
    pylab.clf()
    pylab.fill_between(xrrt10, yrrt10-errt10, yrrt10+errt10, facecolor='r',
                       alpha=0.3)
    pylab.fill_between(xrrt30, yrrt30-errt30, yrrt30+errt30, facecolor='g',
                       alpha=0.3)
    pylab.fill_between(xrrt80, yrrt80-errt80, yrrt80+errt80, facecolor='b',
                       alpha=0.3)
    pylab.plot(xrrt10, yrrt10, 'r-', linewidth=3, label="10 local trajs")
    pylab.plot(xrrt30, yrrt30, 'g-', linewidth=3, label="30 local trajs")
    pylab.plot(xrrt80, yrrt80, 'b-', linewidth=3, label="80 local trajs")
    pylab.plot([1e-2, 1e4], [1e-2, 1e-2], 'k--', linewidth=1)
    pylab.legend(loc=1)
    pylab.xscale('log')
    pylab.yscale('linear')
    pylab.grid(True)
    pylab.xlim(1., 1e4)
    pylab.ylim(0, 2e-1)


# Traj dur
if "max-traj-duration" in trace_dir:
    pylab.ion()
    pylab.figure(1)
    pylab.clf()
    pylab.fill_between(xrrt05, yrrt05-errt05, yrrt05+errt05, facecolor='r',
                       alpha=0.3)
    pylab.fill_between(xrrt10, yrrt10-errt10, yrrt10+errt10, facecolor='g',
                       alpha=0.3)
    pylab.fill_between(xrrt20, yrrt20-errt20, yrrt20+errt20, facecolor='b',
                       alpha=0.3)
    pylab.plot(xrrt05, yrrt05, 'r-', linewidth=3,
               label="Max local traj duration 0.5 s")
    pylab.plot(xrrt10, yrrt10, 'g-', linewidth=3,
               label="Max local traj duration 1.0 s")
    pylab.plot(xrrt20, yrrt20, 'b-', linewidth=3,
               label="Max local traj duration 2.0 s")
    pylab.plot([1e-2, 1e4], [1e-2, 1e-2], 'k--', linewidth=1)
    pylab.legend(loc=1)
    pylab.xscale('log')
    pylab.yscale('linear')
    pylab.grid(True)
    pylab.xlim(1., 1e4)
    pylab.ylim(0, 2e-1)


# Modulo
if "modulo" in trace_dir:
    pylab.ion()
    pylab.figure(1)
    pylab.clf()
    pylab.fill_between(xrrt5, yrrt5-errt5, yrrt5+errt5, facecolor='r',
                       alpha=0.3)
    pylab.fill_between(xrrt10, yrrt10-errt10, yrrt10+errt10, facecolor='g',
                       alpha=0.3)
    pylab.fill_between(xrrt40, yrrt40-errt40, yrrt40+errt40, facecolor='b',
                       alpha=0.3)
    pylab.plot(xrrt5, yrrt5, 'r-', linewidth=3,
               label="Steer-to-goal every 5 steps")
    pylab.plot(xrrt10, yrrt10, 'g-', linewidth=3,
               label="Steer-to-goal every 10 steps")
    pylab.plot(xrrt40, yrrt40, 'b-', linewidth=3,
               label="Steer-to-goal every 40 steps")
    pylab.plot([1e-2, 1e4], [1e-2, 1e-2], 'k--', linewidth=1)
    pylab.legend(loc=1)
    pylab.xscale('log')
    pylab.yscale('linear')
    pylab.grid(True)
    pylab.xlim(1., 1e4)
    pylab.ylim(0, 2e-1)


# 7 and 5
if "11-07" in trace_dir or "11-05" in trace_dir:
    pylab.ion()
    pylab.figure(1)
    pylab.clf()
    pylab.plot(fevipx, 100*fevipy, 'r-', linewidth=3, label="VIP")
    pylab.plot(ferrt1x, 100*ferrt1y, 'g-', linewidth=3, label="RRT with 1 NN")
    pylab.plot(ferrt10x, 100*ferrt10y, 'm-', linewidth=3, label="RRT with 10 NN")
    pylab.plot(ferrt40x, 100*ferrt40y, 'b-', linewidth=3, label="RRT with 40 NN")
    pylab.plot(ferrt100x, 100*ferrt100y, 'c-', linewidth=3, label="RRT with 100 NN")
    pylab.legend(loc=4)
    pylab.xlim(1., 1e4)
    pylab.ylim(0, 105)
    pylab.xlabel("Computation time (s)", fontsize=18)
    pylab.ylabel("Percentage of success (%)", fontsize=18)
    pylab.grid(True)
    pylab.gca().get_legend().get_texts()[0].set_fontsize(18)
    for k in pylab.gca().get_xticklabels():
        k.set_fontsize(18)

    for k in pylab.gca().get_yticklabels():
        k.set_fontsize(18)

    rrt = rrt40
    pylab.figure(2)
    pylab.clf()
    map(lambda run: analyze.plot_run(run, 'r-'), vip)
    map(lambda run: analyze.plot_run(run, 'b-'), rrt)
    pylab.plot([1e-2, 1e4], [1e-2, 1e-2], 'k--', linewidth=1)
    pylab.plot([1e-2, 1e4], [0, 0], 'k', linewidth=2)
    pylab.plot([analyze.first_entry(run, 1e-2, 1e4) for run in vip],
               len(vip)*[-0.02], 'ro', markersize=8, label='VIP')
    pylab.plot([analyze.first_entry(run, 1e-2, 1e4) for run in rrt],
               len(rrt)*[-0.01], 'bo', markersize=8, label='RRT with 40 NN')
    mvip = numpy.mean([analyze.first_entry(run, 1e-2, 1e4) for run in vip])
    mrrt = numpy.mean([analyze.first_entry(run, 1e-2, 1e4) for run in rrt])
    pylab.plot([mvip], [-0.02], 'r*', markersize=25)
    pylab.plot([mrrt], [-0.01], 'b*', markersize=25)
    pylab.legend()
    pylab.xscale('linear')
    pylab.yscale('linear')
    pylab.grid(True)
    pylab.xlim(1., 1e4)
    pylab.ylim(-0.03, 0.2)
    pylab.xlabel("Computation time (s)", fontsize=18)
    pylab.ylabel("Distance to target (dimensionless)", fontsize=18)
    pylab.gca().get_legend().get_texts()[0].set_fontsize(18)
    for k in pylab.gca().get_xticklabels():
        k.set_fontsize(18)

    for k in pylab.gca().get_yticklabels():
        k.set_fontsize(18)


# Analysis of max_traj_duration
if "max-traj-duration" in trace_dir and False:
    xvip, yvip, evip = analyze.compute_means(vip)
    xrrt1, yrrt1, errt1 = analyze.compute_means(rrt1)
    xrrt10, yrrt10, errt10 = analyze.compute_means(rrt10)
    xrrt40, yrrt40, errt40 = analyze.compute_means(rrt40)
    xrrt100, yrrt100, errt100 = analyze.compute_means(rrt100)

    pylab.ion()
    pylab.figure(1)
    pylab.clf()
    pylab.fill_between(xvip, yvip-evip, yvip+evip, facecolor='r', alpha=0.2)
    pylab.fill_between(xrrt1, yrrt1-errt1, yrrt1+errt1, facecolor='g',
                       alpha=0.2)
    pylab.fill_between(xrrt10, yrrt10-errt10, yrrt10+errt10, facecolor='b',
                       alpha=0.2)
    pylab.fill_between(xrrt40, yrrt40-errt40, yrrt40+errt40, facecolor='m',
                       alpha=0.2)
    pylab.fill_between(xrrt100, yrrt100-errt100, yrrt100+errt100, facecolor='c',
                       alpha=0.2)
    pylab.plot(xvip, yvip, 'r-', linewidth=3, label="VIP")
    pylab.plot(xrrt1, yrrt1, 'g-', linewidth=3, label="RRT with 1 NN")
    pylab.plot(xrrt10, yrrt10, 'b-', linewidth=3, label="RRT with 10 NN")
    pylab.plot(xrrt40, yrrt40, 'm-', linewidth=3, label="RRT with 40 NN")
    pylab.plot(xrrt100, yrrt100, 'c-', linewidth=3, label="RRT with 100 NN")
    pylab.plot([1e-2, 1e4], [1e-2, 1e-2], 'k--', linewidth=1)
    pylab.legend()
    pylab.xscale('log')
    pylab.yscale('linear')
    pylab.grid(True)
    pylab.xlim(1., 1e4)
    pylab.ylim(0, 2e-1)
