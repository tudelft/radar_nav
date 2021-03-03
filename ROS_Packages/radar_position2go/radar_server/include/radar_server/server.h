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

#ifndef RADAR_SERVER_H_
#define RADAR_SERVER_H_

#include <ros/ros.h>
#include <string>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <iostream>
#include <fstream>

#include "radar_msgs/Event.h"
#include "radar_targets_msgs/Event.h"

namespace radar_server{

    class Server {
        public:
        Server(ros::NodeHandle & nh, ros::NodeHandle nh_private);
        virtual ~Server();

        private:
        ros::NodeHandle nh_;
        ros::Subscriber radar_sub_;
	ros::Subscriber radar_sub_targets_;
        std::ofstream RADAR_rec_file;
        std::ofstream RADAR_rec_file_targets;

        void radarCallback(const radar_msgs::Event::ConstPtr& msg);
	void radarCallbackTarget(const radar_targets_msgs::Event::ConstPtr& msg);

    };

    } // namespace

#endif // RADAR_SERVER_H_
