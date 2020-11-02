// Copyright (c) 2015, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//

#ifndef MARKER_PUBLISHER_H
#define MARKER_PUBLISHER_H

#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include <kdl/frames.hpp>
#include <octomap/octomap.h>

class MarkerPublisher {
public:
    MarkerPublisher(ros::NodeHandle &nh, const std::string& topic_name="/velma_markers");

    ~MarkerPublisher();

    void publish();

    void clear();

    int addLineListMarker(int m_id, const std::vector<KDL::Vector > &pts, const KDL::Frame &fr,
            double r, double g, double b, double a, double size, const std::string& frame_id,
            const std::string& namesp=std::string());

    int addSinglePointMarker(int m_id, const KDL::Vector &pos, double r, double g, double b,
            double a, double size, const std::string &frame_id,
            const std::string& namesp=std::string());

    int addSinglePointMarkerCube(int m_id, const KDL::Vector &pos, double r, double g, double b,
            double a, double size_x, double size_y, double size_z, const std::string &frame_id,
            const std::string& namesp=std::string());

    int addSphereListMarker(int m_id, const std::vector<KDL::Vector > &pos, double r, double g,
            double b, double a, double size, const std::string &frame_id,
            const std::string& namesp=std::string());

    int addMeshMarker(int m_id, const KDL::Vector &pos, double r, double g, double b, double a,
            double size_x, double size_y, double size_z, const std::string &mesh_path,
            const std::string& frame_id, const std::string& namesp=std::string());

    int addVectorMarker(int m_id, const KDL::Vector &v1, const KDL::Vector &v2, double r, double g,
            double b, double a, double size, const std::string& frame_id,
            const std::string& namesp=std::string());

    int addCapsule(int m_id, const KDL::Frame &fr, double r, double g, double b, double a,
            double length, double radius, const std::string &frame_id,
            const std::string& namesp=std::string());

    int addOctomap(int m_id, const octomap::OcTree &om, const std::string &frame_id,
            const std::string& namesp=std::string());

    std_msgs::ColorRGBA heightMapColor(double h);

    void addEraseMarkers(int from, int to, const std::string& namesp=std::string());

    uint32_t getNumSubscribers() const;

protected:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    visualization_msgs::MarkerArray marker_array_;
    const std::string default_namesp_;
};

#endif	// MARKER_PUBLISHER_H


