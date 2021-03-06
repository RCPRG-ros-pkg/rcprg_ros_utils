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
import PyKDL
from sensor_msgs.msg import JointState

import xml.dom.minidom as minidom
from rcprg_ros_utils import marker_publisher

joint_states_lock = threading.Lock()
joint_state = None

def jointStatesCallback(data):
    global joint_state
    with joint_states_lock:
        joint_state = data

if __name__ == "__main__":
    rospy.init_node('publish_joints_visualization', anonymous=False)

    rospy.sleep(0.5)

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

    while not rospy.is_shutdown():
        js = {}
        with joint_states_lock:
            if joint_state != None:
                joint_idx = 0
                for joint_name in joint_state.name:
                    js[joint_name] = joint_state.position[joint_idx]
                    joint_idx += 1
        m_id = 0
        for j in joints:
            if shown_joints != None and not j[0] in shown_joints:
                continue
            # show axis
            m_id = pub.publishVectorMarker(j[2]*PyKDL.Vector(), j[2]*(j[3]*0.2), m_id, 1, 0, 0, a=1.0, namespace="axes", frame=j[1], scale=0.01)
            m_id = pub.publishTriangleListMarker(points_list[j[0]], m_id, r=0, g=1, b=0, a=1.0, namespace="limits", frame_id=j[1], T=j[2])
            if j[0] in js:
                rot = j[6]
                vec = rot*PyKDL.Vector(math.cos(js[j[0]]), math.sin(js[j[0]]),0)            
                m_id = pub.publishVectorMarker(j[2]*PyKDL.Vector(), j[2]*(vec*0.15), m_id, 0, 0, 1, a=1.0, namespace="limits", frame=j[1], scale=0.01)
        try:
            rospy.sleep(0.1)
        except:
            break


