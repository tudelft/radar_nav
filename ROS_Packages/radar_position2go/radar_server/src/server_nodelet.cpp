/**
 * This file is part of the odroid_ros_radar package - MAVLab TU Delft
 * 
 *   MIT License
 *
 *   Copyright (c) 2020 Julien Dupeyroux
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 *   
 * @author Julien Dupeyroux
 * */

#include <pluginlib/class_list_macros.h>
#include "radar_server/server_nodelet.h"

namespace radar_server{

    void RadarServerNodelet::onInit(){
      server = new radar_server::Server(getNodeHandle(), getPrivateNodeHandle());
      NODELET_INFO_STREAM("Initialized " <<  getName() << " nodelet.");
    }

    #ifndef PLUGINLIB_EXPORT_CLASS
        PLUGINLIB_DECLARE_CLASS(radar_Server, RadarServerNodelet, radar_server::RadarServerNodelet, nodelet::Nodelet);
    #else
        PLUGINLIB_EXPORT_CLASS(radar_server::RadarServerNodelet, nodelet::Nodelet);
    #endif

}
