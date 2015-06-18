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


import time

from rrtcmp import LinAccState, TrajectoryList
import plots


class RRT(object):
    @staticmethod
    def get_name():
        return 'RRT'

    class Node(object):
        def __init__(self, state, parent, traj_from_parent, iterstep):
            self.state = state
            self.parent = None
            self.traj_from_parent = None
            self.path_duration = None
            self.update_parent(parent, traj_from_parent)
            self.iterstep = iterstep
            self.timestamp = time.time()

        def update_parent(self, parent, traj_from_parent):
            self.parent = parent
            self.traj_from_parent = traj_from_parent
            self.path_duration = \
                parent.path_duration + traj_from_parent.duration if parent \
                else 0.

    def __init__(self, robot, init_state, goal_state, tunings):
        """Initiate a new roadmap.

        robot -- robot model
        init_state -- root of the random tree
        goal_state -- target of the planning

        """
        self.robot = robot
        self.tunings = tunings
        self.nodes = [self.Node(init_state, None, None, 0)]
        self.init_state = init_state
        self.goal_state = goal_state
        self.nb_ext = 0
        self.nb_succ_ext = 0
        self.start_time = time.time()
        self.has_found = False
        self.State = init_state.__class__
        self._is_running = False
        self.find_time = None

    def closest_node(self, state):
        node_dist = lambda node: state.dist(node.state)
        return min(self.nodes, key=node_dist)

    def closest_nodes(self, state):
        import heapq
        n = self.tunings.rrt_neigh_size
        nodecmp = lambda node: state.dist(node.state)
        return heapq.nsmallest(n, self.nodes, key=nodecmp)

    def extend(self, target):
        """Extend the roadmap using the local planner towards a target state.
        Since the extension is performed from the k nearest neighbors,
        returns a list of (at most k) nodes added to the
        roadmap.

        """
        if self.has_found:
            return  # laziness
        self.nb_ext += 1

        def steer_node(node, target):
            traj = node.state.steer(target, self.robot)
            if traj is None:
                return None
            iterstep = self.nb_succ_ext + 1  # here we anticipate a bit
            x = traj.last_state()
            last_state = self.State(q=x[:2], qd=x[2:])
            return self.Node(last_state, node, traj, iterstep)

        neighbors = self.closest_nodes(target)
        candidates = [steer_node(neigh, target) for neigh in neighbors]
        candidates = filter(None, candidates)
        if len(candidates) < 1:
            return

        self.nb_succ_ext += 1
        # beware: dist(.) function is asymmetric in LQR-RRT*
        dist_to_target = lambda node: target.dist(node.state)
        dist_to_goal = lambda node: self.goal_state.dist(node.state)
        best_new_node = min(candidates, key=dist_to_target)
        self.nodes.append(best_new_node)
        if dist_to_goal(best_new_node) < self.tunings.spatial_prec:
            self.has_found = True
            self.find_time = time.time()
            return

        if self.tunings.try_exact_steering:
            self.try_exact_steering(best_new_node)
        else:
            reach_for_goal = steer_node(best_new_node, self.goal_state)
            if reach_for_goal and \
               dist_to_goal(reach_for_goal) < self.tunings.spatial_prec:
                self.nodes.append(reach_for_goal)
                self.has_found = True

    def try_exact_steering(self, node):
        linacc_state = LinAccState(node.state.q, node.state.qd, tunings=self.tunings)
        traj = linacc_state.steer(self.goal_state, self.robot)
        if traj:
            x = traj.last_state()
            last_state = self.State(q=x[:2], qd=x[2:])
            iterstep = node.iterstep + 1
            if self.goal_state.dist(last_state) < self.tunings.spatial_prec:
                reaching_node = self.Node(last_state, node, traj, iterstep)
                self.nodes.append(reaching_node)
                self.has_found = True

    def run(self, rand_poses, rand_vel):
        def check_shape(shape):
            assert len(shape) == 2
            assert shape[0] >= self.tunings.max_iter
            assert shape[1] == self.robot.dof
        check_shape(rand_poses.shape)
        check_shape(rand_vel.shape)
        self._is_running = True
        for i in xrange(self.tunings.max_iter):
            if time.time() - self.start_time > self.tunings.max_simu_duration:
                break
            q, qd = rand_poses[i], rand_vel[i]
            self.extend(self.State(q, qd))
            if i % self.tunings.modulo == 0:
                self.extend(self.goal_state)
        self._is_running = False

    def _get_traj_to_node(self, node=None):
        def get_traj_list(node, traj_list=[]):
            if node.parent:
                get_traj_list(node.parent, traj_list)
                traj_list.append(node.traj_from_parent)
            return traj_list
        return TrajectoryList(get_traj_list(node))

    def get_trajectory(self):
        closest_node = self.closest_node(self.goal_state)
        if closest_node.state != self.goal_state:
            return None
        return self._get_traj_to_node(closest_node)

    def get_dict_repr(self):
        return {
            'nodes': [{
                'q': node.state.q,
                'qd': node.state.qd,
                'parent': node.parent.iterstep if node.parent else None,
                'iterstep': node.iterstep,
                'timestamp': node.timestamp
            } for node in self.nodes],
            'nb_ext': self.nb_ext,
            'nb_succ_ext': self.nb_succ_ext,
            'start_time': self.start_time,
            'find_time': self.find_time,
            'has_found': self.has_found,
        }

    def is_running(self):
        if self.has_found:
            self._is_running = False
        return self._is_running

    def plot_tree(self, **kwargs):
        return plots.plot_tree(self, **kwargs)

    def plot_c0v0(self, **kwargs):
        return plots.plot_c0v0(self, **kwargs)
