#!/usr/bin/env python

## Publishes marker: range of arm.
# @ingroup utilities
# @file reachability_range.py

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
import math
import numpy as np
import PyKDL
from geometry_msgs.msg import *
from visualization_msgs.msg import *
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

    points_kdl = []
    for e in edges:
        pt0 = points[e[0]]
        pt1 = points[e[1]]
        points_kdl.append(R*PyKDL.Vector(pt0[0], pt0[1], pt0[2]))
        points_kdl.append(R*PyKDL.Vector(pt1[0], pt1[1], pt1[2]))
    return points_kdl

if __name__ == "__main__":
    rospy.init_node('reachability_range', anonymous=False)

    visualization_mode = rospy.get_param('~visualization_mode', 'icosphere')
    assert visualization_mode == 'solid_sphere' or visualization_mode == 'uvsphere' or visualization_mode == 'icosphere'

    inner_size = float(rospy.get_param('~inner_size'))
    outer_size = float(rospy.get_param('~outer_size'))
    marker_namespace = rospy.get_param('~marker_namespace')
    frame_id = rospy.get_param('~frame_id')
    cr = rospy.get_param('~color_r')
    cg = rospy.get_param('~color_g')
    cb = rospy.get_param('~color_b')

    rospy.sleep(0.5)

    pub = marker_publisher.MarkerPublisher('/reachability_range')

    if visualization_mode == 'solid_sphere' or visualization_mode == 'uvsphere':
        inner_pts = spherePoints(inner_size*0.5, 0.2)
        outer_pts = spherePoints(outer_size*0.5, 0.2)
    elif visualization_mode == 'icosphere':
        inner_pts = icosphereEdges(inner_size)
        outer_pts = icosphereEdges(outer_size)

    while not rospy.is_shutdown():
        m_id = 0
        if visualization_mode == 'solid_sphere':
            # visualize as solid spheres with alpha=0.5
            m_id = pub.publishSinglePointMarker(PyKDL.Vector(), m_id, r=cr*0.7, g=cg*0.7, b=cb*0.7, a=0.5, namespace=marker_namespace,
                    frame_id=frame_id, m_type=Marker.SPHERE, scale=Vector3(inner_size, inner_size, inner_size), T=None)
            m_id = pub.publishSinglePointMarker(PyKDL.Vector(), m_id, r=cr, g=cg, b=cb, a=0.5, namespace=marker_namespace,
                    frame_id=frame_id, m_type=Marker.SPHERE, scale=Vector3(outer_size, outer_size, outer_size), T=None)
        elif visualization_mode == 'uvsphere':
            # visualize as lines of uv sphere
            m_id = pub.publishLineStripMarker(inner_pts, m_id, r=cr*0.7, g=cg*0.7, b=cb*0.7, a=1, namespace=marker_namespace, frame_id=frame_id, width=0.01, T=None)
            m_id = pub.publishLineStripMarker(outer_pts, m_id, r=cr, g=cg, b=cb, a=1, namespace=marker_namespace, frame_id=frame_id, width=0.01, T=None)
        elif visualization_mode == 'icosphere':
            # visualize as lines of icosphere
            m_id = pub.publishLineListMarker(inner_pts, m_id, r=cr*0.7, g=cg*0.7, b=cb*0.7, a=1, namespace=marker_namespace, frame_id=frame_id, width=0.01, T=None)
            m_id = pub.publishLineListMarker(outer_pts, m_id, r=cr, g=cg, b=cb, a=1, namespace=marker_namespace, frame_id=frame_id, width=0.01, T=None)

        try:
            rospy.sleep(0.1)
        except:
            break

