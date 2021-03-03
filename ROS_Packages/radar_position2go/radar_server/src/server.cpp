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
 * @authors Julien Dupeyroux, Federico Corradi
 * */

#include "radar_server/server.h"

namespace radar_server {

	Server::Server(ros::NodeHandle & nh, ros::NodeHandle nh_private) : nh_(nh){
		radar_sub_ = nh_.subscribe("radar", 10, &Server::radarCallback, this);
		radar_sub_targets_ = nh_.subscribe("radar_targets", 10, &Server::radarCallbackTarget, this);
		RADAR_rec_file.open("RADAR_recording.txt");
		RADAR_rec_file_targets.open("RADAR_recording_targets.txt");
	}

	Server::~Server(){
		RADAR_rec_file.close();
		RADAR_rec_file_targets.close();
		std::cout << "Server stopped." << std::endl;
	}

	void Server::radarCallback(const radar_msgs::Event::ConstPtr& msg){
		RADAR_rec_file << msg->ts << ",";
		RADAR_rec_file << (uint32_t) msg->dimx << ",";
		RADAR_rec_file << (uint32_t) msg->dimy << ",";
		int dimension = (int)(msg->dimx * msg->dimy);
		for(int i=0; i<dimension; i++){
			RADAR_rec_file << (double)(msg->data_rx1_re[i]);
			if (i < dimension - 1)
				RADAR_rec_file << ","; 
		}
		for(int i=0; i<dimension; i++){
			RADAR_rec_file << (double)(msg->data_rx1_im[i]);
			if (i < dimension - 1)
				RADAR_rec_file << ","; 
		}
		for(int i=0; i<dimension; i++){
			RADAR_rec_file << (double)(msg->data_rx2_re[i]);
			if (i < dimension - 1)
				RADAR_rec_file << ","; 
		}
		for(int i=0; i<dimension; i++){
			RADAR_rec_file << (double)(msg->data_rx2_im[i]);
			if (i < dimension - 1)
				RADAR_rec_file << ","; 
		}
		RADAR_rec_file << std::endl;
	}

	void Server::radarCallbackTarget(const radar_targets_msgs::Event::ConstPtr& msg){
		int dimension = (int)(msg->num_current_targets);
		for(int i=0; i<dimension; i++){
			RADAR_rec_file_targets << (float) msg->angle[i] << ",";
			RADAR_rec_file_targets << (float) msg->speed[i] << ",";
			RADAR_rec_file_targets << (float) msg->range[i] << ",";
			RADAR_rec_file_targets << (float) msg->speed_th[i] << ",";
			RADAR_rec_file_targets << msg->ts;
			RADAR_rec_file_targets << std::endl;
		}
	}

} // namespace
