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

import pendulum
from state import EuclideanState, add_state_factory
from rrt import RRT
from trajectory import Trajectory

from slycot import sb02md
from misc import center_angle

from numpy import array, diag, dot, eye, hstack, log, zeros

wx, kx = array([0., 0.]), 0
wx2, kx2 = array([0., 0.]), 0
ns, kns, knsp = 0., 0, 0


@add_state_factory
class LQR_State(EuclideanState):
    def __init__(self, q, qd, tunings=None):
        super(LQR_State, self).__init__(q, qd, tunings)
        self.x = numpy.hstack([q, qd])

        q_cost = 1. / numpy.pi
        qd_cost = 1. / tunings.Vmax

        torque_lim = array(tunings.torque_limits)
        Q = diag([q_cost] * 2 + [qd_cost] * 2)
        R_inv = diag(torque_lim) / tunings.lqr_torque_weight

        df = pendulum.df(q, qd, tau=numpy.array([0, 0]))
        A = zeros((4, 4))  # d(f_state)/d(q, qd)
        A[0:2, 2:4] = eye(2)
        A[2:4, 0:4] = df[:, 0:4]
        B = zeros((4, 2))  # d(f_state)/d(tau)
        B[2:4, 0:2] = df[:, 4:]
        dico = 'C'  # continuous Riccati equation
        n = 4  # order of A, Q, G and (the resulting) S
        G = dot(B, dot(R_inv, B.transpose()))
        try:
            self.S, _, _, _, _, _ = sb02md(n, A, G, Q, dico)
            self.minus_K = - dot(R_inv, dot(B.transpose(), self.S))
        except:
            print "Ooops! Debug info:"
            print "q =", q
            print "qd =", qd
            print "A ="
            print A
            print "G ="
            print G
            print "Q ="
            print Q
            raise
        if False:
            print "S ="
            print self.S
            print "K ="
            print self.K

    def V(self, x):
        """Value function from LQR (beware: includes centering on self.x)."""
        xc = x - self.x
        xc[0] = center_angle(xc[0])
        xc[1] = center_angle(xc[1])
        return dot(xc.transpose(), dot(self.S, xc))

    def policy(self, x):
        """Policy function from LQR (beware: includes centering on self.x)."""
        xc = x - self.x
        xc[0] = center_angle(xc[0])
        xc[1] = center_angle(xc[1])
        return dot(self.minus_K, xc)

    def dist(self, other_state):
        return self.V(other_state.x)

    def steer(self, dest_state, robot):
        lqr_dest = dest_state  # LQR_State(dest_state)
        dt = self.tunings.time_prec
        #nb_traj_samples = self.tunings.nb_traj_samples
        max_traj_duration = self.tunings.max_traj_duration
        max_timesteps = int(max_traj_duration / dt)
        assert max_timesteps > 3

        q, qd = self.q, self.qd
        q_list, qd_list = [q], [qd]
        duration = max_traj_duration
        for tstep in xrange(max_timesteps):
            x = hstack([q, qd])
            tau = lqr_dest.policy(x)
            if False and tstep < 1:
                global wx, kx
                wx += abs(tau)
                kx += 1
                print "  avg. initial torque =", (wx / kx)
            elif False and tstep < 2:
                global wx2, kx2
                wx2 += abs(tau)
                kx2 += 1
                print "avg. successful torque =", (wx2 / kx2)
            if lqr_dest.V(x) < self.tunings.spatial_prec \
               or (abs(tau) > self.tunings.torque_limits).any():
                # TODO: find a proper epsilon here
                # goal reached
                duration = tstep * dt
                break
            qdd = robot.compute_forward_dynamics(q, qd, tau)
            q = q + qd * dt     # NB: not the same semantic as +=
            qd = qd + qdd * dt  # same here
            q_list.append(array([center_angle(qi) for qi in q]))
            qd_list.append(qd)

        if duration < 2 * dt:
            return None

        q_fun = lambda t: q_list[int(t / dt)] if t < duration \
            else q_list[-1]
        qd_fun = lambda t: qd_list[int(t / dt)] if t < duration \
            else qd_list[-1]
        tau_fun = lambda t: tau
        traj = Trajectory(duration, q_fun, qd_fun, tau_fun)
        return traj

    def __str__(self):
        return "LQRState(q=%s, qd=%s)" % (str(self.q), str(self.qd))


class LQR_RRT(RRT):
    @staticmethod
    def get_name():
        return 'LQR_RRT'

    def __init__(self, robot, init_state, goal_state, tunings):
        super(LQR_RRT, self).__init__(robot, init_state, goal_state, tunings)
        assert isinstance(init_state, LQR_State)
        assert isinstance(goal_state, LQR_State)
        assert tunings.rrt_neigh_size == 1
        if tunings.try_exact_steering:
            sep = "=" * 55
            print "\n%s" % sep
            print "Warning: exact steering is not in the standard LQR-RRT"
            print "%s\n" % sep


class LQR_RRT_star(LQR_RRT):
    @staticmethod
    def get_name():
        return 'LQR_RRT_star'

    def nearest(self, dest_state):
        # same as in RRT, since the distance is encoded in LQR_State
        return self.closest_node(dest_state)

    def near(self, dest_node):
        n = len(self.nodes)
        d = 4  # dimension of the space
        gamma = self.tunings.lqr_gamma
        thres = gamma * (log(n) / n) ** (1. / d)
        if False:
            print "thres =", thres
            print "dists =", sorted([dest_node.state.dist(node.state) for node in
                                    self.nodes])
        return [node for node in self.nodes
                if dest_node.state.dist(node.state) < thres]

    def extend(self, target):
        if self.has_found:
            return  # laziness
        self.nb_ext += 1

        def steer_node(node, target):
            traj = node.state.steer(target, self.robot)
            if traj is None:
                return None
            iterstep = self.nb_succ_ext + 1  # here we anticipate a bit
            x = traj.last_state()
            last_state = self.State(x[:2], x[2:])
            return self.Node(last_state, node, traj, iterstep)

        nearest_node = self.nearest(target)
        new_node = steer_node(nearest_node, target)
        if new_node is None:  # steering failed
            return None

        neighborhood = self.near(new_node)
        if True:
            global ns, kns, knsp
            kns += 1
            if False and len(neighborhood) > 0:
                knsp += 1
                ns += len(neighborhood)
                print "avg. neighborhood length  =", ns / kns
                print "avg. neighborhood length (> 0) =", ns / knsp
                print "no neighborhood %.2f of the time" % (1. - knsp * 1. / kns)

        self.choose_parent(new_node, neighborhood)
        self.rewire(neighborhood, new_node)
        self.nodes.append(new_node)
        self.nb_succ_ext += 1

    def choose_parent(self, new_node, nodes):
        min_cost = 1e5
        best_parent, best_traj = None, None
        for node in nodes:
            traj = node.state.steer(new_node.state, self.robot)
            if traj \
               and node.path_duration + traj.duration < min_cost:
                min_cost = node.path_duration + traj.duration
                best_parent = node
                best_traj = traj
        if best_parent is not None:
            new_node.update_parent(best_parent, best_traj)

    def rewire(self, nodes, new_node):
        for node in nodes:
            traj = new_node.state.steer(node.state, self.robot)
            if traj \
               and new_node.path_duration + traj.duration < node.path_duration:
                node.update_parent(new_node, traj)
