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
from numpy import array, cos, eye, sin, zeros

l = 0.2   # [m], as in the paper
m = 8.    # [kg], as in the paper
g = 9.81  # grav. acceleration

__ml2 = m * l ** 2.
__mgl = m * g * l


def M(q):
    m00 = 5. / 3. + cos(q[1])
    m01 = 1. / 3. + .5 * cos(q[1])
    m10 = 1. / 3. + .5 * cos(q[1])
    m11 = 1. / 3.
    return __ml2 * array([[m00, m01], [m10, m11]])


def dM(q):
    Der = zeros((2, 2, 2))
    Der[:, :, 1] = - array([[1., .5], [.5, 0.]]) * sin(q[1])  # dM/dq1
    return __ml2 * Der


def C(q, qd):
    c00 = - sin(q[1]) * qd[1]
    c01 = - .5 * sin(q[1]) * qd[1]
    c10 = + .5 * sin(q[1]) * qd[0]
    c11 = 0
    return __ml2 * array([[c00, c01], [c10, c11]])


def dC(q, qd):
    Der = zeros((2, 2, 4))
    Der[0, 0, 1] = - cos(q[1]) * qd[1]            # d(c00)/d(q1)
    Der[0, 0, 3] = - sin(q[1])                    # d(c00)/d(qd1)
    Der[0, 1, 1] = - .5 * cos(q[1]) * qd[1]       # d(c01)/d(q1)
    Der[0, 1, 3] = - .5 * sin(q[1])               # d(c01)/d(qd1)
    Der[1, 0, 1] = + .5 * cos(q[1]) * qd[0]       # d(c10)/d(q1)
    Der[1, 0, 2] = + .5 * sin(q[1])               # d(c10)/d(qd0)
    return __ml2 * Der


def G(q):
    g0 = 1.5 * sin(q[0]) + .5 * sin(q[0] + q[1])
    g1 = .5 * sin(q[0] + q[1])
    return __mgl * array([g0, g1])


def dG(q):
    Der = zeros((2, 2))
    Der[0, 0] = 1.5 * cos(q[0]) + .5 * cos(q[0] + q[1])  # d(g0)/d(q0)
    Der[0, 1] = .5 * cos(q[0] + q[1])                    # d(g0)/d(q1)
    Der[1, 0] = .5 * cos(q[0] + q[1])                    # d(g1)/d(q0)
    Der[1, 1] = .5 * cos(q[0] + q[1])                    # d(g1)/d(q1)
    return __mgl * Der


def f(q, qd, tau):
    """Dynamics of the pendulum follow: qdd = f(q, qd, tau)."""
    M_inv = numpy.linalg.inv(M(q))
    qdd = numpy.dot(M_inv, tau - G(q) - numpy.dot(C(q, qd), qd))
    assert qdd.shape == (2,)
    return qdd


def df(q, qd, tau):
    M_inv = numpy.linalg.inv(M(q))
    M_inv_sq = numpy.dot(M_inv, M_inv)
    dg_dq = dG(q)
    dc = numpy.tensordot(dC(q, qd), qd, axes=[1, 0])
    dc_dq = dc[:, :2]
    dc_dqd = dc[:, 2:]
    a = tau - G(q) - numpy.dot(C(q, qd), qd)
    dA_dq = numpy.dot(- M_inv, numpy.tensordot(dM(q), numpy.dot(M_inv, a), axes=[1, 0]))
    dB_dq = numpy.dot(M_inv, - dg_dq - dc_dq)
    df_dq = dA_dq + dB_dq
    df_dqd = numpy.dot(M_inv, - dc_dqd - C(q, qd))
    df_dtau = M_inv
    Der = zeros((2, 6))
    Der[:, :2] = df_dq
    Der[:, 2:4] = df_dqd
    Der[:, 4:] = df_dtau
    return Der


if False:  # test
    def M(q):
        m00 = q[0]
        m01 = 1.
        m10 = 1.
        m11 = 2.
        return array([[m00, m01], [m10, m11]])

    def dM(q):
        Der = zeros((2, 2, 2))
        Der[:, :, 0] = array([[1., 0.], [0., 0.]]) * 1.
        return Der

    w = array([0., 1.])

    def f(q, qd, tau):
        """Dynamics of the pendulum follow: qdd = f(q, qd, tau)."""
        qdd = numpy.linalg.solve(M(q), w)
        qdd = numpy.dot(numpy.linalg.inv(M(q)), w)
        print "f(", q, ") = ", qdd
        return qdd

    def df(q, qd, tau):
        M_inv = numpy.linalg.inv(M(q))
        M_inv_sq = numpy.dot(M_inv, M_inv)
        print "M_inv"
        print M_inv
        print "M_inv_sq"
        print M_inv_sq
        print "dM(q).w"
        print numpy.tensordot(dM(q), w, axes=[1, 0])
        #df_dq = numpy.dot(- M_inv_sq, numpy.tensordot(dM(q), w, axes=[1, 0]))
        df_dq = numpy.dot(- M_inv, numpy.tensordot(dM(q), numpy.dot(M_inv, w), axes=[1, 0]))
        df_dq = numpy.dot(numpy.dot(- M_inv, numpy.tensordot(dM(q), M_inv, axes=[1, 0])), w)
        print "df_dq"
        print df_dq
        print "============="
        print ""
        df_dqd = zeros((2, 2))
        df_dtau = zeros((2, 2))
        Der = zeros((2, 6))
        Der[:, :2] = df_dq
        Der[:, 2:4] = df_dqd
        Der[:, 4:] = df_dtau
        return Der
