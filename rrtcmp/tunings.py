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


required_keys = ['max_traj_duration', 'nb_traj_samples', 'rrt_neigh_size',
                 'spatial_prec', 'time_prec', 'torque_limits']

unmutable_keys = ['max_iter', 'Vmax']


class Tunings(object):
    def __init__(self, tunings_dict):
        if False:  # disabling key checks
            for key in required_keys:
                assert key in tunings_dict.keys(), "Missing tuning '%s'." % key
        for key, value in tunings_dict.iteritems():
            if key[0] != '_':
                self.__dict__[key] = value
        if '_defaults' in tunings_dict:
            self._defaults = tunings_dict['_defaults'].copy()
        else:
            self._defaults = tunings_dict.copy()

    def update(self, custom_tunings):
        for param, value in custom_tunings.iteritems():
            assert param not in unmutable_keys, "%s is unmutable" % param
            self.__dict__[param] = value

    def revert_to_defaults(self):
        self.__dict__.update(self._defaults)

    def __str__(self):
        s = ''
        for key, value in self.__dict__.iteritems():
            if key in self._defaults.keys() and value == self._defaults[key]:
                continue
            if key[0] != '_':
                s += " [%s = %s]" % (key, str(value))
        if len(s) > 0:
            return "Tunings with%s" % s
        return "Default tunings"

    def print_description(self):
        print "Tunings:"
        for key, value in self.__dict__.iteritems():
            if key[0] <> '_':
                print "- %s = %s" % (key, str(value))
