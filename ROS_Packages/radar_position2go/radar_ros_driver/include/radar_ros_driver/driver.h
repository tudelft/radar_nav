/**
 * This file is part of the odroid_ros_dvs package - MAVLab TU Delft
 * 
 *   MIT License
 *
 *   Copyright (c) 2020 MAVLab TU Delft
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
 * */

#include "radar_ros_driver/ofxRadar24Ghz.h"

#include <ros/ros.h>
#include <string>
#include <sstream>

#include "radar_msgs/Event.h"
#include "radar_avoid_msgs/Command.h"
#include "radar_targets_msgs/Event.h"
#include "rc_msgs/RcData.h"

// Sampling frequency
#define FREQ 30.0

namespace radar_ros_driver {

    class RadarRosDriver {
        public:

            ofxRadar24Ghz myradar;

            RadarRosDriver(ros::NodeHandle & nh, ros::NodeHandle nh_private);
            virtual ~RadarRosDriver();
            void readout(uint16_t count);
            void mav_st_callback(const rc_msgs::RcData::ConstPtr& rc_msg);

        private: 

            // Data size
            uint32_t num_samples_per_chirp;
            uint32_t num_chirps;

            // Raw data
            double *adc_real_tx1rx1;	// REAL
            double *adc_imag_tx1rx1;	// IMG
            double *adc_real_tx1rx2;	// REAL
            double *adc_imag_tx1rx2;	// IMG

            //Tracking_Params_t *tracking_list2; // target info
            int8_t test_ros;
            boost::array<float, 2> dv_;

            boost::array<float, 2048> adc_real_tx1rx1_f;	// REAL
            boost::array<float, 2048> adc_imag_tx1rx1_f;	// IMG
            boost::array<float, 2048> adc_real_tx1rx2_f;	// REAL
            boost::array<float, 2048> adc_imag_tx1rx2_f;	// IMG

            ros::NodeHandle     nh_;
            ros::Publisher      radar_pub_;
            ros::Publisher      radar_pub_command_;
            ros::Publisher      radar_pub_targets_;
            ros::Subscriber     sub_state;
            volatile bool       running_;
            std::string         ns;
            
            // Targets 
            //vector<Measurement_elem_t> current_targets_driver;	    
            uint32_t num_targets_now;
            
            //boost::array<uint32_t, 256> is_associated_f;
            boost::array<float, 5> angle_f;
            boost::array<float, 5> speed_f;
            boost::array<float, 5> range_f;
            boost::array<float, 5> speed_th_f;
            //boost::array<float, 256> rx1_angle_arg_re;
            //boost::array<float, 256> rx1_angle_arg_im;
            //boost::array<float, 256> rx2_angle_arg_re;
            //boost::array<float, 256> rx2_angle_arg_im;
            
    };

} // namespace
