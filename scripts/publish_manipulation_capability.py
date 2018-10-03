#!/usr/bin/env python

## Publishes marker: camera frustum.
# @ingroup utilities
# @file publish_camera_frustum.py
# @namespace scripts.publish_camera_frustum Publishes marker: camera frustum

# Copyright (c) 2017, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import roslib; roslib.load_manifest('rcprg_ros_utils')

import rospy
import threading
import math
import numpy as np
import PyKDL
from sensor_msgs.msg import JointState
from geometry_msgs.msg import *
from visualization_msgs.msg import *
import tf
import tf_conversions.posemath as pm

import xml.dom.minidom as minidom
from rcprg_ros_utils import marker_publisher

def spherePoints(R, step):
    points = []
    u_steps = max(1, int(math.pi*R/step))
    for u in np.linspace(-math.pi/2, math.pi/2, u_steps):
        # calculate radius of circle
        r = R * math.cos(u)
        L = 2*math.pi*r
        v_steps = max(1, int(L/step))
        y = R * math.sin(u)
        for v in np.linspace(-math.pi, math.pi, v_steps):
            x = math.cos(v) * r
            z = math.sin(v) * r
            points.append(PyKDL.Vector(x,y,z))
    return points

def icosphereEdges(R):
    edges = [(30, 26), (20, 25), (28, 41), (0, 1), (18, 19), (28, 26), (2, 5), (37, 35),
            (6, 7), (20, 35), (11, 22), (27, 28), (7, 19), (17, 23), (40, 38), (35, 36),
            (17, 36), (37, 41), (29, 30), (8, 21), (6, 0), (19, 24), (11, 21), (27, 40),
            (8, 38), (33, 34), (34, 41), (3, 1), (24, 36), (15, 16), (6, 16), (39, 35),
            (41, 40), (14, 33), (36, 34), (23, 12), (10, 3), (14, 22), (21, 27), (12, 13),
            (34, 32), (17, 24), (1, 10), (33, 29), (37, 34), (8, 25), (16, 17), (11, 9),
            (24, 15), (11, 26), (16, 23), (36, 32), (8, 3), (38, 39), (34, 31), (27, 38),
            (14, 12), (23, 33), (13, 9), (32, 23), (22, 30), (13, 22), (4, 8), (36, 37),
            (5, 13), (39, 37), (30, 31), (17, 15), (32, 33), (29, 22), (2, 13), (5, 0),
            (14, 23), (35, 24), (6, 19), (31, 29), (11, 30), (25, 18), (1, 4), (38, 25),
            (14, 29), (7, 0), (5, 16), (20, 24), (39, 40), (22, 9), (28, 40), (18, 7),
            (6, 15), (33, 31), (4, 7), (10, 11), (5, 6), (4, 18), (19, 15), (10, 21),
            (40, 37), (1, 2), (2, 9), (5, 12), (25, 39), (30, 28), (20, 39), (20, 18),
            (26, 21), (17, 32), (2, 10), (9, 10), (4, 25), (19, 20), (16, 12), (31, 28),
            (1, 7), (26, 27), (31, 41), (13, 14), (21, 3), (2, 0), (4, 3), (8, 27)]

    points = [(0.0, 0.0, -0.5), (0.212661, -0.154506, -0.425327), (-0.081228, -0.249998, -0.425327),
            (0.361804, -0.262863, -0.22361), (0.425324, 0.0, -0.262868), (-0.262865, 0.0, -0.425326),
            (-0.081228, 0.249998, -0.425327), (0.212661, 0.154506, -0.425327), (0.475529, -0.154506, 0.0),
            (-0.138194, -0.425325, -0.22361), (0.131434, -0.404506, -0.262869), (0.0, -0.5, 0.0),
            (-0.447213, 0.0, -0.223608), (-0.344095, -0.249998, -0.262868), (-0.475529, -0.154506, 0.0),
            (-0.138194, 0.425325, -0.22361), (-0.344095, 0.249998, -0.262868), (-0.293893, 0.404508, 0.0),
            (0.361804, 0.262863, -0.22361), (0.131434, 0.404506, -0.262869), (0.293893, 0.404508, 0.0),
            (0.293893, -0.404508, 0.0), (-0.293893, -0.404508, 0.0), (-0.475529, 0.154506, 0.0),
            (0.0, 0.5, 0.0), (0.475529, 0.154506, 0.0), (0.138194, -0.425325, 0.22361),
            (0.344095, -0.249998, 0.262868), (0.081228, -0.249998, 0.425327), (-0.361804, -0.262863, 0.22361),
            (-0.131434, -0.404506, 0.262869), (-0.212661, -0.154506, 0.425327), (-0.361804, 0.262863, 0.22361),
            (-0.425324, 0.0, 0.262868), (-0.212661, 0.154506, 0.425327), (0.138194, 0.425325, 0.22361),
            (-0.131434, 0.404506, 0.262869), (0.081228, 0.249998, 0.425327), (0.447213, 0.0, 0.223608),
            (0.344095, 0.249998, 0.262868), (0.262865, 0.0, 0.425326), (0.0, 0.0, 0.5)]
    points222 = [
        (-0.131434, -0.404506, 0.262869) , (0.138194, -0.425325, 0.22361),
        (0.293893, 0.404508, 0.0) , (0.475529, 0.154506, 0.0),
        (0.081228, -0.249998, 0.425327) , (0.0, 0.0, 0.5),
        (0.0, 0.0, -0.5) , (0.212661, -0.154506, -0.425327),
        (0.361804, 0.262863, -0.22361) , (0.131434, 0.404506, -0.262869),
        (0.081228, -0.249998, 0.425327) , (0.138194, -0.425325, 0.22361),
        (-0.081228, -0.249998, -0.425327) , (-0.262865, 0.0, -0.425326),
        (0.081228, 0.249998, 0.425327) , (0.138194, 0.425325, 0.22361),
        (-0.081228, 0.249998, -0.425327) , (0.212661, 0.154506, -0.425327),
        (0.293893, 0.404508, 0.0) , (0.138194, 0.425325, 0.22361),
        (0.0, -0.5, 0.0) , (-0.293893, -0.404508, 0.0),
        (0.344095, -0.249998, 0.262868) , (0.081228, -0.249998, 0.425327),
        (0.212661, 0.154506, -0.425327) , (0.131434, 0.404506, -0.262869),
        (-0.293893, 0.404508, 0.0) , (-0.475529, 0.154506, 0.0),
        (0.262865, 0.0, 0.425326) , (0.447213, 0.0, 0.223608),
        (0.138194, 0.425325, 0.22361) , (-0.131434, 0.404506, 0.262869),
        (-0.293893, 0.404508, 0.0) , (-0.131434, 0.404506, 0.262869),
        (0.081228, 0.249998, 0.425327) , (0.0, 0.0, 0.5),
        (-0.361804, -0.262863, 0.22361) , (-0.131434, -0.404506, 0.262869),
        (0.475529, -0.154506, 0.0) , (0.293893, -0.404508, 0.0),
        (-0.081228, 0.249998, -0.425327) , (0.0, 0.0, -0.5),
        (0.131434, 0.404506, -0.262869) , (0.0, 0.5, 0.0),
        (0.0, -0.5, 0.0) , (0.293893, -0.404508, 0.0),
        (0.344095, -0.249998, 0.262868) , (0.262865, 0.0, 0.425326),
        (0.475529, -0.154506, 0.0) , (0.447213, 0.0, 0.223608),
        (-0.425324, 0.0, 0.262868) , (-0.212661, 0.154506, 0.425327),
        (-0.212661, 0.154506, 0.425327) , (0.0, 0.0, 0.5),
        (0.361804, -0.262863, -0.22361) , (0.212661, -0.154506, -0.425327),
        (0.0, 0.5, 0.0) , (-0.131434, 0.404506, 0.262869),
        (-0.138194, 0.425325, -0.22361) , (-0.344095, 0.249998, -0.262868),
        (-0.081228, 0.249998, -0.425327) , (-0.344095, 0.249998, -0.262868),
        (0.344095, 0.249998, 0.262868) , (0.138194, 0.425325, 0.22361),
        (0.0, 0.0, 0.5) , (0.262865, 0.0, 0.425326),
        (-0.475529, -0.154506, 0.0) , (-0.425324, 0.0, 0.262868),
        (-0.131434, 0.404506, 0.262869) , (-0.212661, 0.154506, 0.425327),
        (-0.475529, 0.154506, 0.0) , (-0.447213, 0.0, -0.223608),
        (0.131434, -0.404506, -0.262869) , (0.361804, -0.262863, -0.22361),
        (-0.475529, -0.154506, 0.0) , (-0.293893, -0.404508, 0.0),
        (0.293893, -0.404508, 0.0) , (0.344095, -0.249998, 0.262868),
        (-0.447213, 0.0, -0.223608) , (-0.344095, -0.249998, -0.262868),
        (-0.212661, 0.154506, 0.425327) , (-0.361804, 0.262863, 0.22361),
        (-0.293893, 0.404508, 0.0) , (0.0, 0.5, 0.0),
        (0.212661, -0.154506, -0.425327) , (0.131434, -0.404506, -0.262869),
        (-0.425324, 0.0, 0.262868) , (-0.361804, -0.262863, 0.22361),
        (0.081228, 0.249998, 0.425327) , (-0.212661, 0.154506, 0.425327),
        (0.475529, -0.154506, 0.0) , (0.475529, 0.154506, 0.0),
        (-0.344095, 0.249998, -0.262868) , (-0.293893, 0.404508, 0.0),
        (0.0, -0.5, 0.0) , (-0.138194, -0.425325, -0.22361),
        (0.0, 0.5, 0.0) , (-0.138194, 0.425325, -0.22361),
        (0.0, -0.5, 0.0) , (0.138194, -0.425325, 0.22361),
        (-0.344095, 0.249998, -0.262868) , (-0.475529, 0.154506, 0.0),
        (-0.131434, 0.404506, 0.262869) , (-0.361804, 0.262863, 0.22361),
        (0.475529, -0.154506, 0.0) , (0.361804, -0.262863, -0.22361),
        (0.447213, 0.0, 0.223608) , (0.344095, 0.249998, 0.262868),
        (-0.212661, 0.154506, 0.425327) , (-0.212661, -0.154506, 0.425327),
        (0.344095, -0.249998, 0.262868) , (0.447213, 0.0, 0.223608),
        (-0.475529, -0.154506, 0.0) , (-0.447213, 0.0, -0.223608),
        (-0.475529, 0.154506, 0.0) , (-0.425324, 0.0, 0.262868),
        (-0.344095, -0.249998, -0.262868) , (-0.138194, -0.425325, -0.22361),
        (-0.361804, 0.262863, 0.22361) , (-0.475529, 0.154506, 0.0),
        (-0.293893, -0.404508, 0.0) , (-0.131434, -0.404506, 0.262869),
        (-0.344095, -0.249998, -0.262868) , (-0.293893, -0.404508, 0.0),
        (0.425324, 0.0, -0.262868) , (0.475529, -0.154506, 0.0),
        (-0.131434, 0.404506, 0.262869) , (0.081228, 0.249998, 0.425327),
        (-0.262865, 0.0, -0.425326) , (-0.344095, -0.249998, -0.262868),
        (0.344095, 0.249998, 0.262868) , (0.081228, 0.249998, 0.425327),
        (-0.131434, -0.404506, 0.262869) , (-0.212661, -0.154506, 0.425327),
        (-0.293893, 0.404508, 0.0) , (-0.138194, 0.425325, -0.22361),
        (-0.361804, 0.262863, 0.22361) , (-0.425324, 0.0, 0.262868),
        (-0.361804, -0.262863, 0.22361) , (-0.293893, -0.404508, 0.0),
        (-0.081228, -0.249998, -0.425327) , (-0.344095, -0.249998, -0.262868),
        (-0.262865, 0.0, -0.425326) , (0.0, 0.0, -0.5),
        (-0.475529, -0.154506, 0.0) , (-0.475529, 0.154506, 0.0),
        (0.138194, 0.425325, 0.22361) , (0.0, 0.5, 0.0),
        (-0.081228, 0.249998, -0.425327) , (0.131434, 0.404506, -0.262869),
        (-0.212661, -0.154506, 0.425327) , (-0.361804, -0.262863, 0.22361),
        (0.0, -0.5, 0.0) , (-0.131434, -0.404506, 0.262869),
        (0.475529, 0.154506, 0.0) , (0.361804, 0.262863, -0.22361),
        (0.212661, -0.154506, -0.425327) , (0.425324, 0.0, -0.262868),
        (0.447213, 0.0, 0.223608) , (0.475529, 0.154506, 0.0),
        (-0.475529, -0.154506, 0.0) , (-0.361804, -0.262863, 0.22361),
        (0.212661, 0.154506, -0.425327) , (0.0, 0.0, -0.5),
        (-0.262865, 0.0, -0.425326) , (-0.344095, 0.249998, -0.262868),
        (0.293893, 0.404508, 0.0) , (0.0, 0.5, 0.0),
        (0.344095, 0.249998, 0.262868) , (0.262865, 0.0, 0.425326),
        (-0.293893, -0.404508, 0.0) , (-0.138194, -0.425325, -0.22361),
        (0.081228, -0.249998, 0.425327) , (0.262865, 0.0, 0.425326),
        (0.361804, 0.262863, -0.22361) , (0.212661, 0.154506, -0.425327),
        (-0.081228, 0.249998, -0.425327) , (-0.138194, 0.425325, -0.22361),
        (-0.425324, 0.0, 0.262868) , (-0.212661, -0.154506, 0.425327),
        (0.425324, 0.0, -0.262868) , (0.212661, 0.154506, -0.425327),
        (0.131434, -0.404506, -0.262869) , (0.0, -0.5, 0.0),
        (-0.262865, 0.0, -0.425326) , (-0.081228, 0.249998, -0.425327),
        (0.425324, 0.0, -0.262868) , (0.361804, 0.262863, -0.22361),
        (0.131434, 0.404506, -0.262869) , (-0.138194, 0.425325, -0.22361),
        (0.131434, -0.404506, -0.262869) , (0.293893, -0.404508, 0.0),
        (0.262865, 0.0, 0.425326) , (0.081228, 0.249998, 0.425327),
        (0.212661, -0.154506, -0.425327) , (-0.081228, -0.249998, -0.425327),
        (-0.081228, -0.249998, -0.425327) , (-0.138194, -0.425325, -0.22361),
        (-0.262865, 0.0, -0.425326) , (-0.447213, 0.0, -0.223608),
        (0.475529, 0.154506, 0.0) , (0.344095, 0.249998, 0.262868),
        (-0.131434, -0.404506, 0.262869) , (0.081228, -0.249998, 0.425327),
        (0.293893, 0.404508, 0.0) , (0.344095, 0.249998, 0.262868),
        (0.293893, 0.404508, 0.0) , (0.361804, 0.262863, -0.22361),
        (0.138194, -0.425325, 0.22361) , (0.293893, -0.404508, 0.0),
        (-0.293893, 0.404508, 0.0) , (-0.361804, 0.262863, 0.22361),
        (-0.081228, -0.249998, -0.425327) , (0.131434, -0.404506, -0.262869),
        (-0.138194, -0.425325, -0.22361) , (0.131434, -0.404506, -0.262869),
        (0.425324, 0.0, -0.262868) , (0.475529, 0.154506, 0.0),
        (0.131434, 0.404506, -0.262869) , (0.293893, 0.404508, 0.0),
        (-0.344095, 0.249998, -0.262868) , (-0.447213, 0.0, -0.223608),
        (-0.212661, -0.154506, 0.425327) , (0.081228, -0.249998, 0.425327),
        (0.212661, -0.154506, -0.425327) , (0.212661, 0.154506, -0.425327),
        (0.138194, -0.425325, 0.22361) , (0.344095, -0.249998, 0.262868),
        (-0.212661, -0.154506, 0.425327) , (0.0, 0.0, 0.5),
        (-0.344095, -0.249998, -0.262868) , (-0.475529, -0.154506, 0.0),
        (0.293893, -0.404508, 0.0) , (0.361804, -0.262863, -0.22361),
        (-0.081228, -0.249998, -0.425327) , (0.0, 0.0, -0.5),
        (0.425324, 0.0, -0.262868) , (0.361804, -0.262863, -0.22361),
        (0.475529, -0.154506, 0.0) , (0.344095, -0.249998, 0.262868),
    ]
    points_kdl = []
    for e in edges:
        pt0 = points[e[0]]
        pt1 = points[e[1]]
        points_kdl.append(R*PyKDL.Vector(pt0[0], pt0[1], pt0[2]))
        points_kdl.append(R*PyKDL.Vector(pt1[0], pt1[1], pt1[2]))
    return points_kdl

if __name__ == "__main__":
    rospy.init_node('publish_manipulation_capability', anonymous=False)

    coordinates = [
    0.000000,0.000000,-0.500000,
    0.212661,-0.154506,-0.425327,
    -0.081228,-0.249998,-0.425327,
    0.361804,-0.262863,-0.223610,
    0.212661,-0.154506,-0.425327,
    0.425324,0.000000,-0.262868,
    0.000000,0.000000,-0.500000,
    -0.081228,-0.249998,-0.425327,
    -0.262865,0.000000,-0.425326,
    0.000000,0.000000,-0.500000,
    -0.262865,0.000000,-0.425326,
    -0.081228,0.249998,-0.425327,
    0.000000,0.000000,-0.500000,
    -0.081228,0.249998,-0.425327,
    0.212661,0.154506,-0.425327,
    0.361804,-0.262863,-0.223610,
    0.425324,0.000000,-0.262868,
    0.475529,-0.154506,0.000000,
    -0.138194,-0.425325,-0.223610,
    0.131434,-0.404506,-0.262869,
    0.000000,-0.500000,0.000000,
    -0.447213,0.000000,-0.223608,
    -0.344095,-0.249998,-0.262868,
    -0.475529,-0.154506,0.000000,
    -0.138194,0.425325,-0.223610,
    -0.344095,0.249998,-0.262868,
    -0.293893,0.404508,0.000000,
    0.361804,0.262863,-0.223610,
    0.131434,0.404506,-0.262869,
    0.293893,0.404508,0.000000,
    0.361804,-0.262863,-0.223610,
    0.475529,-0.154506,0.000000,
    0.293893,-0.404508,0.000000,
    -0.138194,-0.425325,-0.223610,
    0.000000,-0.500000,0.000000,
    -0.293893,-0.404508,0.000000,
    -0.447213,0.000000,-0.223608,
    -0.475529,-0.154506,0.000000,
    -0.475529,0.154506,0.000000,
    -0.138194,0.425325,-0.223610,
    -0.293893,0.404508,0.000000,
    0.000000,0.500000,0.000000,
    0.361804,0.262863,-0.223610,
    0.293893,0.404508,0.000000,
    0.475529,0.154506,0.000000,
    0.138194,-0.425325,0.223610,
    0.344095,-0.249998,0.262868,
    0.081228,-0.249998,0.425327,
    -0.361804,-0.262863,0.223610,
    -0.131434,-0.404506,0.262869,
    -0.212661,-0.154506,0.425327,
    -0.361804,0.262863,0.223610,
    -0.425324,0.000000,0.262868,
    -0.212661,0.154506,0.425327,
    0.138194,0.425325,0.223610,
    -0.131434,0.404506,0.262869,
    0.081228,0.249998,0.425327,
    0.447213,0.000000,0.223608,
    0.344095,0.249998,0.262868,
    0.262865,0.000000,0.425326,
    0.262865,0.000000,0.425326,
    0.081228,0.249998,0.425327,
    0.000000,0.000000,0.500000,
    0.262865,0.000000,0.425326,
    0.344095,0.249998,0.262868,
    0.081228,0.249998,0.425327,
    0.344095,0.249998,0.262868,
    0.138194,0.425325,0.223610,
    0.081228,0.249998,0.425327,
    0.081228,0.249998,0.425327,
    -0.212661,0.154506,0.425327,
    0.000000,0.000000,0.500000,
    0.081228,0.249998,0.425327,
    -0.131434,0.404506,0.262869,
    -0.212661,0.154506,0.425327,
    -0.131434,0.404506,0.262869,
    -0.361804,0.262863,0.223610,
    -0.212661,0.154506,0.425327,
    -0.212661,0.154506,0.425327,
    -0.212661,-0.154506,0.425327,
    0.000000,0.000000,0.500000,
    -0.212661,0.154506,0.425327,
    -0.425324,0.000000,0.262868,
    -0.212661,-0.154506,0.425327,
    -0.425324,0.000000,0.262868,
    -0.361804,-0.262863,0.223610,
    -0.212661,-0.154506,0.425327,
    -0.212661,-0.154506,0.425327,
    0.081228,-0.249998,0.425327,
    0.000000,0.000000,0.500000,
    -0.212661,-0.154506,0.425327,
    -0.131434,-0.404506,0.262869,
    0.081228,-0.249998,0.425327,
    -0.131434,-0.404506,0.262869,
    0.138194,-0.425325,0.223610,
    0.081228,-0.249998,0.425327,
    0.081228,-0.249998,0.425327,
    0.262865,0.000000,0.425326,
    0.000000,0.000000,0.500000,
    0.081228,-0.249998,0.425327,
    0.344095,-0.249998,0.262868,
    0.262865,0.000000,0.425326,
    0.344095,-0.249998,0.262868,
    0.447213,0.000000,0.223608,
    0.262865,0.000000,0.425326,
    0.475529,0.154506,0.000000,
    0.344095,0.249998,0.262868,
    0.447213,0.000000,0.223608,
    0.475529,0.154506,0.000000,
    0.293893,0.404508,0.000000,
    0.344095,0.249998,0.262868,
    0.293893,0.404508,0.000000,
    0.138194,0.425325,0.223610,
    0.344095,0.249998,0.262868,
    0.000000,0.500000,0.000000,
    -0.131434,0.404506,0.262869,
    0.138194,0.425325,0.223610,
    0.000000,0.500000,0.000000,
    -0.293893,0.404508,0.000000,
    -0.131434,0.404506,0.262869,
    -0.293893,0.404508,0.000000,
    -0.361804,0.262863,0.223610,
    -0.131434,0.404506,0.262869,
    -0.475529,0.154506,0.000000,
    -0.425324,0.000000,0.262868,
    -0.361804,0.262863,0.223610,
    -0.475529,0.154506,0.000000,
    -0.475529,-0.154506,0.000000,
    -0.425324,0.000000,0.262868,
    -0.475529,-0.154506,0.000000,
    -0.361804,-0.262863,0.223610,
    -0.425324,0.000000,0.262868,
    -0.293893,-0.404508,0.000000,
    -0.131434,-0.404506,0.262869,
    -0.361804,-0.262863,0.223610,
    -0.293893,-0.404508,0.000000,
    0.000000,-0.500000,0.000000,
    -0.131434,-0.404506,0.262869,
    0.000000,-0.500000,0.000000,
    0.138194,-0.425325,0.223610,
    -0.131434,-0.404506,0.262869,
    0.293893,-0.404508,0.000000,
    0.344095,-0.249998,0.262868,
    0.138194,-0.425325,0.223610,
    0.293893,-0.404508,0.000000,
    0.475529,-0.154506,0.000000,
    0.344095,-0.249998,0.262868,
    0.475529,-0.154506,0.000000,
    0.447213,0.000000,0.223608,
    0.344095,-0.249998,0.262868,
    0.293893,0.404508,0.000000,
    0.000000,0.500000,0.000000,
    0.138194,0.425325,0.223610,
    0.293893,0.404508,0.000000,
    0.131434,0.404506,-0.262869,
    0.000000,0.500000,0.000000,
    0.131434,0.404506,-0.262869,
    -0.138194,0.425325,-0.223610,
    0.000000,0.500000,0.000000,
    -0.293893,0.404508,0.000000,
    -0.475529,0.154506,0.000000,
    -0.361804,0.262863,0.223610,
    -0.293893,0.404508,0.000000,
    -0.344095,0.249998,-0.262868,
    -0.475529,0.154506,0.000000,
    -0.344095,0.249998,-0.262868,
    -0.447213,0.000000,-0.223608,
    -0.475529,0.154506,0.000000,
    -0.475529,-0.154506,0.000000,
    -0.293893,-0.404508,0.000000,
    -0.361804,-0.262863,0.223610,
    -0.475529,-0.154506,0.000000,
    -0.344095,-0.249998,-0.262868,
    -0.293893,-0.404508,0.000000,
    -0.344095,-0.249998,-0.262868,
    -0.138194,-0.425325,-0.223610,
    -0.293893,-0.404508,0.000000,
    0.000000,-0.500000,0.000000,
    0.293893,-0.404508,0.000000,
    0.138194,-0.425325,0.223610,
    0.000000,-0.500000,0.000000,
    0.131434,-0.404506,-0.262869,
    0.293893,-0.404508,0.000000,
    0.131434,-0.404506,-0.262869,
    0.361804,-0.262863,-0.223610,
    0.293893,-0.404508,0.000000,
    0.475529,-0.154506,0.000000,
    0.475529,0.154506,0.000000,
    0.447213,0.000000,0.223608,
    0.475529,-0.154506,0.000000,
    0.425324,0.000000,-0.262868,
    0.475529,0.154506,0.000000,
    0.425324,0.000000,-0.262868,
    0.361804,0.262863,-0.223610,
    0.475529,0.154506,0.000000,
    0.212661,0.154506,-0.425327,
    0.131434,0.404506,-0.262869,
    0.361804,0.262863,-0.223610,
    0.212661,0.154506,-0.425327,
    -0.081228,0.249998,-0.425327,
    0.131434,0.404506,-0.262869,
    -0.081228,0.249998,-0.425327,
    -0.138194,0.425325,-0.223610,
    0.131434,0.404506,-0.262869,
    -0.081228,0.249998,-0.425327,
    -0.344095,0.249998,-0.262868,
    -0.138194,0.425325,-0.223610,
    -0.081228,0.249998,-0.425327,
    -0.262865,0.000000,-0.425326,
    -0.344095,0.249998,-0.262868,
    -0.262865,0.000000,-0.425326,
    -0.447213,0.000000,-0.223608,
    -0.344095,0.249998,-0.262868,
    -0.262865,0.000000,-0.425326,
    -0.344095,-0.249998,-0.262868,
    -0.447213,0.000000,-0.223608,
    -0.262865,0.000000,-0.425326,
    -0.081228,-0.249998,-0.425327,
    -0.344095,-0.249998,-0.262868,
    -0.081228,-0.249998,-0.425327,
    -0.138194,-0.425325,-0.223610,
    -0.344095,-0.249998,-0.262868,
    0.425324,0.000000,-0.262868,
    0.212661,0.154506,-0.425327,
    0.361804,0.262863,-0.223610,
    0.425324,0.000000,-0.262868,
    0.212661,-0.154506,-0.425327,
    0.212661,0.154506,-0.425327,
    0.212661,-0.154506,-0.425327,
    0.000000,0.000000,-0.500000,
    0.212661,0.154506,-0.425327,
    -0.081228,-0.249998,-0.425327,
    0.131434,-0.404506,-0.262869,
    -0.138194,-0.425325,-0.223610,
    -0.081228,-0.249998,-0.425327,
    0.212661,-0.154506,-0.425327,
    0.131434,-0.404506,-0.262869,
    0.212661,-0.154506,-0.425327,
    0.361804,-0.262863,-0.223610,
    0.131434,-0.404506,-0.262869,
    ]

    edges = set()
    points_map = {}
    points = []
    pt_idx = 0
    for i in range(0, len(coordinates), 9):
        p1 = (coordinates[i], coordinates[i+1], coordinates[i+2])
        p2 = (coordinates[i+3], coordinates[i+4], coordinates[i+5])
        p3 = (coordinates[i+6], coordinates[i+7], coordinates[i+8])
        if not p1 in points_map:
            points_map[p1] = pt_idx
            pt_idx = pt_idx + 1
            points.append( p1 )
        if not p2 in points_map:
            points_map[p2] = pt_idx
            pt_idx = pt_idx + 1
            points.append( p2 )
        if not p3 in points_map:
            points_map[p3] = pt_idx
            pt_idx = pt_idx + 1
            points.append( p3 )
        if not (points_map[p2], points_map[p1]) in edges:
            edges.add( (points_map[p1], points_map[p2]) )
        if not (points_map[p3], points_map[p2]) in edges:
            edges.add( (points_map[p2], points_map[p3]) )
        if not (points_map[p1], points_map[p3]) in edges:
            edges.add( (points_map[p3], points_map[p1]) )
    print len(edges)
    print edges
    print len(points)

    for p in points:
        print p

    for e in edges:
        #p1 = points[e[0]]
        #p2 = points[e[1]]
        print e[0], ',', e[1]


    rospy.sleep(0.5)

    pub = marker_publisher.MarkerPublisher('/reachability_range')

    inner_size = 0.75
    outer_size = 1.65

    inner_pts = spherePoints(inner_size*0.5, 0.2)
    outer_pts = spherePoints(outer_size*0.5, 0.2)

    inner_pts = icosphereEdges(inner_size)
    outer_pts = icosphereEdges(outer_size)

    while not rospy.is_shutdown():

        m_id = 0
#        m_id = pub.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0, g=1, b=0, a=0.5, namespace='right',
#                frame_id='right_arm_2_link', m_type=Marker.SPHERE, scale=Vector3(outer_size, outer_size, outer_size), T=None)
#        m_id = pub.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0, g=0, b=1, a=0.5, namespace='right',
#                frame_id='right_arm_2_link', m_type=Marker.SPHERE, scale=Vector3(inner_size, inner_size, inner_size), T=None)



#        m_id = pub.publishMultiPointsMarker(inner_pts, m_id, r=0, g=0, b=1, a=1, namespace='right', frame_id='right_arm_2_link', m_type=Marker.SPHERE, scale=Vector3(0.01, 0.01, 0.01), T=None)
#        m_id = pub.publishMultiPointsMarker(outer_pts, m_id, r=0, g=1, b=0, a=1, namespace='right', frame_id='right_arm_2_link', m_type=Marker.SPHERE, scale=Vector3(0.01, 0.01, 0.01), T=None)
#        m_id = pub.publishLineStripMarker(inner_pts, m_id, r=0, g=0, b=1, a=1, namespace='default', frame_id='right_arm_2_link', width=0.01, T=None)
#        m_id = pub.publishLineStripMarker(outer_pts, m_id, r=0, g=1, b=0, a=1, namespace='default', frame_id='right_arm_2_link', width=0.01, T=None)

        m_id = pub.publishLineListMarker(inner_pts, m_id, r=0, g=0, b=1, a=1, namespace='default', frame_id='right_arm_2_link', width=0.01, T=None)
        m_id = pub.publishLineListMarker(outer_pts, m_id, r=0, g=1, b=0, a=1, namespace='default', frame_id='right_arm_2_link', width=0.01, T=None)

        try:
            rospy.sleep(0.1)
        except:
            break

    exit(0)






    tf_listener = tf.TransformListener()

    try:
        robot_description = rospy.get_param("/robot_description")
    except KeyError as e:
        print "Some ROS parameters are not provided:"
        print e
        exit(1)

    shown_joints = None
    try:
        shown_joints = rospy.get_param("~shown_joints")
    except KeyError as e:
        print "ROS param 'shown_joints' is not set. All joints are visualized."

    dom = minidom.parseString(robot_description)
    robot = dom.getElementsByTagName("robot")
    if len(robot) != 1:
        print "robot_description does not contain 'robot' node"
        exit(1)

    joints = []

    for j in robot[0].childNodes:
        if j.localName != "joint":
            continue
        name = j.getAttribute("name")
        joint_type = j.getAttribute("type")
        if joint_type == "fixed":
            continue
        origin = j.getElementsByTagName("origin")
        if len(origin) != 1:
            print "joint '" + name + "' does contain wrong number of 'origin' nodes: " + str(len(origin))
            exit(1)
        axis = j.getElementsByTagName("axis")
        if len(axis) != 1:
            print "joint '" + name + "' contain wrong number of 'axis' nodes: " + str(len(axis))
            exit(1)
        limit = j.getElementsByTagName("limit")
        if len(limit) != 1:
            print "joint '" + name + "' contain wrong number of 'limit' nodes: " + str(len(limit))
            exit(1)
        parent = j.getElementsByTagName("parent")
        if len(parent) != 1:
            print "joint '" + name + "' contain wrong number of 'parent' nodes: " + str(len(parent))
            exit(1)

        xyz = origin[0].getAttribute("xyz")
        rpy = origin[0].getAttribute("rpy")
        xyz = xyz.split()
        rpy = rpy.split()
        orig = PyKDL.Frame(PyKDL.Rotation.RPY(float(rpy[0]), float(rpy[1]), float(rpy[2])), PyKDL.Vector(float(xyz[0]), float(xyz[1]), float(xyz[2])))

        axis_xyz = axis[0].getAttribute("xyz")
        axis_xyz = axis_xyz.split()
        ax = PyKDL.Vector(float(axis_xyz[0]), float(axis_xyz[1]), float(axis_xyz[2]))

        limit_lo = float(limit[0].getAttribute("lower"))
        limit_up = float(limit[0].getAttribute("upper"))
        print name + ": " + str(limit_lo) + " " + str(limit_up) + "  axis: " + str(ax.x()) + " " + str(ax.y()) + " " + str(ax.z()) + " "

        parent_link = parent[0].getAttribute("link")

        axis_z = ax
        if abs(axis_z.z()) > 0.7:
            axis_x = PyKDL.Vector(1,0,0)
        else:
            axis_x = PyKDL.Vector(0,0,1)
        axis_y = axis_z * axis_x
        axis_x = axis_y * axis_z
        axis_x.Normalize()
        axis_y.Normalize()
        axis_z.Normalize()
        rot = PyKDL.Frame(PyKDL.Rotation(axis_x,axis_y,axis_z))

        joints.append( (name, parent_link, orig, ax, limit_lo, limit_up, rot) )

    listener_joint_states = rospy.Subscriber('/joint_states', JointState, jointStatesCallback)

    limit_scale = 0.1

    pub = marker_publisher.MarkerPublisher('/joints_vis')
    points_list = {}
    for j in joints:
        pl = []
        rot = j[6]
        angle = j[4]
        while angle < j[5]-0.15:
            angle_prev = angle
            angle += 0.1
            pl.append(PyKDL.Vector())
            pl.append(rot*PyKDL.Vector(math.cos(angle_prev), math.sin(angle_prev),0)*limit_scale)
            pl.append(rot*PyKDL.Vector(math.cos(angle), math.sin(angle),0)*limit_scale)
        pl.append(PyKDL.Vector())
        pl.append(rot*PyKDL.Vector(math.cos(angle), math.sin(angle),0)*limit_scale)
        pl.append(rot*PyKDL.Vector(math.cos(j[5]), math.sin(j[5]),0)*limit_scale)
        points_list[j[0]] = pl

    dest_frame_name = 'right_HandGripLink'
    dest_frame_offset = PyKDL.Frame()

    while not rospy.is_shutdown():
        try:
            rospy.sleep(0.1)
        except:
            break

        js = {}
        with joint_states_lock:
            if joint_state != None:
                joint_idx = 0
                for joint_name in joint_state.name:
                    js[joint_name] = joint_state.position[joint_idx]
                    joint_idx += 1
        m_id = 0
#        for j in joints:
#            if shown_joints != None and not j[0] in shown_joints:
#                continue
#            # show axis
#            m_id = pub.publishVectorMarker(j[2]*PyKDL.Vector(), j[2]*(j[3]*0.2), m_id, 1, 0, 0, a=1.0, namespace="axes", frame=j[1], scale=0.01)
#            m_id = pub.publishTriangleListMarker(points_list[j[0]], m_id, r=0, g=1, b=0, a=1.0, namespace="limits", frame_id=j[1], T=j[2])
#            if j[0] in js:
#                rot = j[6]
#                vec = rot*PyKDL.Vector(math.cos(js[j[0]]), math.sin(js[j[0]]),0)            
#                m_id = pub.publishVectorMarker(j[2]*PyKDL.Vector(), j[2]*(vec*0.15), m_id, 0, 0, 1, a=1.0, namespace="limits", frame=j[1], scale=0.01)

        used_joints = [
            'torso_0_joint',
            'right_arm_0_joint',
            'right_arm_1_joint',
            'right_arm_2_joint',
            'right_arm_3_joint',
            'right_arm_4_joint',
            'right_arm_5_joint',
            'right_arm_6_joint',
        ]
        colors = [
            (1,1,1),
            (1,0,0),
            (1,1,0),
            (1,0,1),
            (0,1,0),
            (0,1,1),
            (0,0,1),
            (0.5,0.5,0.5),
        ]

        for jnt_idx in range(len(used_joints)):
            joint_name = used_joints[jnt_idx]
            color = colors[jnt_idx]
            j = None
            for jnt in joints:
                if jnt[0] == joint_name:
                    j = jnt
                    break
            time_now = rospy.Time(0)#.now()

            T_B_L = None
            T_B_D = None
            try:
                #tf_listener.waitForTransform('torso_base', j[1], time_now, rospy.Duration(1.0))
                #tf_listener.waitForTransform('torso_base', dest_frame_name, time_now, rospy.Duration(1.0))
                T_B_L = pm.fromTf(tf_listener.lookupTransform('torso_base', j[1], time_now))
                T_B_D = pm.fromTf(tf_listener.lookupTransform('torso_base', dest_frame_name, time_now))
            except:
                print "tf failed"

            if T_B_L is None or T_B_D is None:
                continue

            j_pos = js[j[0]]
            limit_lo = max(j[4]-j_pos, - 20.4)
            limit_up = min(j[5]-j_pos, + 20.4)

            # express destination point in parent link of the joint and rotate it around joint axis
            ptD_L = T_B_L.Inverse() * T_B_D * PyKDL.Vector()
            T_L_J = j[2]

            pts = []
            for angle in np.linspace(limit_lo,limit_up, 50):
                ptDh_B = T_B_L * T_L_J * PyKDL.Frame(PyKDL.Rotation.Rot(j[3], angle)) * T_L_J.Inverse() * ptD_L
                pts.append(ptDh_B)

            m_id = pub.publishLineStripMarker(pts, m_id, r=color[0], g=color[1], b=color[2], a=1, namespace='default', frame_id='torso_base', width=0.01, T=None)
            rospy.sleep(0.01)
