#!/usr/bin/env python

## Publishes marker: camera frustum.
# @ingroup utilities
# @file publish_camera_frustum.py

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
import PyKDL

from rcprg_ros_utils import marker_publisher

if __name__ == "__main__":

    rospy.init_node('publish_camera_frustum', anonymous=True)

    rospy.sleep(0.5)

    ros_parameter_names = [('horizontal_fov', float), ('aspect_w_h', float),
        ('min_z', float), ('max_z', float), ('frame_id', str)]

    params = {}
    for param_name, param_type in ros_parameter_names:
        param_value_str = rospy.get_param('~{}'.format(param_name))
        params[param_name] = (param_type)( param_value_str )

    pub = marker_publisher.MarkerPublisher('/camera_frustum')

    dx = math.tan(params['horizontal_fov']/2)
    dy = dx/params['aspect_w_h']
    min_z = params['min_z']
    max_z = params['max_z']
    frame_id = params['frame_id']
    while not rospy.is_shutdown():
        m_id = 0
        m_id = pub.publishVectorMarker(PyKDL.Vector(dx*min_z, dy*min_z, min_z), PyKDL.Vector(dx*max_z, dy*max_z, max_z), m_id, 1, 0, 0, frame=frame_id, namespace=frame_id, scale=0.01)
        m_id = pub.publishVectorMarker(PyKDL.Vector(-dx*min_z, dy*min_z, min_z), PyKDL.Vector(-dx*max_z, dy*max_z, max_z), m_id, 1, 0, 0, frame=frame_id, namespace=frame_id, scale=0.01)
        m_id = pub.publishVectorMarker(PyKDL.Vector(-dx*min_z, -dy*min_z, min_z), PyKDL.Vector(-dx*max_z, -dy*max_z, max_z), m_id, 1, 0, 0, frame=frame_id, namespace=frame_id, scale=0.01)
        m_id = pub.publishVectorMarker(PyKDL.Vector(dx*min_z, -dy*min_z, min_z), PyKDL.Vector(dx*max_z, -dy*max_z, max_z), m_id, 1, 0, 0, frame=frame_id, namespace=frame_id, scale=0.01)

        m_id = pub.publishVectorMarker(PyKDL.Vector(dx*min_z, dy*min_z, min_z), PyKDL.Vector(-dx*min_z, dy*min_z, min_z), m_id, 1, 0, 0, frame=frame_id, namespace=frame_id, scale=0.01)
        m_id = pub.publishVectorMarker(PyKDL.Vector(-dx*min_z, dy*min_z, min_z), PyKDL.Vector(-dx*min_z, -dy*min_z, min_z), m_id, 1, 0, 0, frame=frame_id, namespace=frame_id, scale=0.01)
        m_id = pub.publishVectorMarker(PyKDL.Vector(-dx*min_z, -dy*min_z, min_z), PyKDL.Vector(dx*min_z, -dy*min_z, min_z), m_id, 1, 0, 0, frame=frame_id, namespace=frame_id, scale=0.01)
        m_id = pub.publishVectorMarker(PyKDL.Vector(dx*min_z, -dy*min_z, min_z), PyKDL.Vector(dx*min_z, dy*min_z, min_z), m_id, 1, 0, 0, frame=frame_id, namespace=frame_id, scale=0.01)

        m_id = pub.publishVectorMarker(PyKDL.Vector(dx*max_z, dy*max_z, max_z), PyKDL.Vector(-dx*max_z, dy*max_z, max_z), m_id, 1, 0, 0, frame=frame_id, namespace=frame_id, scale=0.01)
        m_id = pub.publishVectorMarker(PyKDL.Vector(-dx*max_z, dy*max_z, max_z), PyKDL.Vector(-dx*max_z, -dy*max_z, max_z), m_id, 1, 0, 0, frame=frame_id, namespace=frame_id, scale=0.01)
        m_id = pub.publishVectorMarker(PyKDL.Vector(-dx*max_z, -dy*max_z, max_z), PyKDL.Vector(dx*max_z, -dy*max_z, max_z), m_id, 1, 0, 0, frame=frame_id, namespace=frame_id, scale=0.01)
        m_id = pub.publishVectorMarker(PyKDL.Vector(dx*max_z, -dy*max_z, max_z), PyKDL.Vector(dx*max_z, dy*max_z, max_z), m_id, 1, 0, 0, frame=frame_id, namespace=frame_id, scale=0.01)
        try:
            rospy.sleep(0.1)
        except:
            break