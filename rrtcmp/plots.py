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

from matplotlib.pyplot import clf, plot, title, xlabel, ylabel, xlim, ylim
from matplotlib.pyplot import grid, arrow
from matplotlib.patches import Ellipse

from rrtcmp.trajectory import TrajectoryList


def plot_cspace(rrt, vel=False):
    clf()
    for node in rrt.nodes:
        if node.traj_from_parent:
            traj = node.traj_from_parent
            trange = numpy.arange(0, traj.duration, rrt.tunings.time_prec)
            traj_poses = traj.sample_poses(trange)
            xvals, yvals = zip(*traj_poses)
            plot(xvals, yvals, color='k')
            if vel:  # include velocity vectors
                q, v = node.state.q, (1. / rrt.tunings.Vmax * node.state.qd)
                arrow(q[0], q[1], v[0], v[1], head_width=0.05,
                      head_length=0.05, fc='g', ec='g')
    xvals, yvals = zip(*[node.state.get_pose() for node in rrt.nodes])
    plot(xvals, yvals, 'go', markersize=6)
    plot([rrt.goal_state.q[0]], [rrt.goal_state.q[1]], 'g*', markersize=20)
    title("C-space after %d extensions" % (rrt.nb_ext))
    xlabel('$\\theta_1$')
    ylabel('$\\theta_2$')
    xlim(-3.5, +3.5)
    ylim(-3.5, +3.5)
    grid(True)


def plot_vspace(rrt):
    clf()
    xvals, yvals = zip(*[node.state.qd for node in rrt.nodes])
    plot(xvals, yvals, 'bo', markersize=6)
    title("Velocity space after %d extensions" % (rrt.nb_ext))
    xlabel('$\\dot{\\theta}_1$')
    ylabel('$\\dot{\\theta}_2$')
    xlim(-rrt.tunings.Vmax, +rrt.tunings.Vmax)
    ylim(-rrt.tunings.Vmax, +rrt.tunings.Vmax)
    grid(True)


def plot_c0v0(rrt, clear=True, colors=['#999999', '#EE9999']):
    if clear:
        clf()
    for node in rrt.nodes:
        if node.traj_from_parent:
            trange = numpy.linspace(0, node.traj_from_parent.duration, 40)
            q0l = [node.traj_from_parent.q(t)[0] for t in trange]
            qd0l = [node.traj_from_parent.qd(t)[0] for t in trange]
            plot(q0l, qd0l, color=colors[0])
    for node in rrt.nodes:
        plot(node.state.q[0], node.state.qd[0], 'ro', color=colors[1], markersize=6)
    plot([rrt.goal_state.q[0]], [rrt.goal_state.qd[0]], 'r*', markersize=10)
    title("Phase-plot for first joint after %d extensions" % (rrt.nb_ext))
    xlim(-3.5, +3.5)
    ylim(-rrt.tunings.Vmax, +rrt.tunings.Vmax)
    xlabel('$\\theta_1$')
    ylabel('$\\dot{\\theta}_1$')
    goal_zone = Ellipse(xy=(rrt.goal_state.q[0], rrt.goal_state.qd[0]),
                        width=(numpy.pi / 0.17 * rrt.tunings.spatial_prec),
                        height=(rrt.tunings.Vmax / 0.17 * rrt.tunings.spatial_prec),
                        color=colors[1])
    pylab.gcf().gca().add_artist(goal_zone)
    circle = pylab.Circle((rrt.goal_state.q[0], rrt.goal_state.qd[0]), .2, color='r')
    grid(True)


def plot_tree(rrt):
    clf()
    for node in rrt.nodes:
        if node.parent:
            xvals, yvals = zip(*[s.get_pose()
                                 for s in [node.state, node.parent.state]])
            plot(xvals, yvals, 'b--', color='#9999FF')
    xvals, yvals = zip(*[node.state.get_pose() for node in rrt.nodes])
    plot(xvals, yvals, 'go', color='#42DD42', markersize=6)
    plot([rrt.goal_state.q[0]], [rrt.goal_state.q[1]], 'g*', markersize=10)
    goal_zone = Ellipse(xy=(rrt.goal_state.q[0], rrt.goal_state.qd[0]),
                        width=(numpy.pi / 0.17 * rrt.tunings.spatial_prec),
                        height=(numpy.pi / 0.17 * rrt.tunings.spatial_prec),
                        color='#EE9999')
    pylab.gcf().gca().add_artist(goal_zone)
    title("Tree in C-space after %d extensions" % (rrt.nb_ext))
    xlabel('$\\theta_1$')
    ylabel('$\\theta_2$')
    xlim(-3.5, +3.5)
    ylim(-3.5, +3.5)
    grid(True)


def plot_trajectory(traj, color='r-'):
    if isinstance(traj, TrajectoryList):
        map(plot_trajectory, traj.traj_list)
    else:
        trange = numpy.arange(0, traj.duration, 1e-3)
        xvals, yvals = zip(*traj.sample_poses(trange))
        plot(xvals, yvals, color)
