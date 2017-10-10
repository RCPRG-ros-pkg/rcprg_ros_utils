#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')

import rospy
import math
import PyKDL

from ros_utils import marker_publisher

if __name__ == "__main__":

    rospy.init_node('publish_camera_frustrum', anonymous=True)

    rospy.sleep(0.5)

    try:
        horizontal_fov = float(rospy.get_param("~horizontal_fov"))
        aspect_w_h = float(rospy.get_param("~aspect_w_h"))
        min_z = float(rospy.get_param("~min_z"))
        max_z = float(rospy.get_param("~max_z"))
        frame_id = rospy.get_param("~frame_id")
    except KeyError as e:
        print "Some ROS parameters are not provided:"
        print e
        exit(1)

    #horizontal_fov = 1.047
    #aspect_w_h = 1.33333
    #min_z = 0.5
    #max_z = 3.0
    #frame_id = "head_kinect_rgb_optical_frame"

    pub = marker_publisher.MarkerPublisher('/camera_frustrum')

    while not rospy.is_shutdown():
        dx = math.tan(horizontal_fov/2)
        dy = dx/aspect_w_h
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


