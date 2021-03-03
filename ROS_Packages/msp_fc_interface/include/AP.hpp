#pragma once
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "msp.hpp"
#include "msp_node.hpp"
#include <math.h>
#include "controller.hpp"
#include "natnet.hpp"

#include "radar_avoid_msgs/Command.h"
#include "std_msgs/Int32.h"
#include "rc_msgs/RcData.h"

#include <algorithm>
#include <cmath>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <thread>

extern Controller *controller;

//extern msp_node *msp;
//extern NatNet *gps;
/*
//extern state_mc *st_mc;
//extern Controller *controller;
//extern NatNet *gps;
//extern msp_node *msp;

class AP {
    public:
        std::thread AP_thread;
        float curr_time = 0;
        float dt = 0;
        AP();
        //~user_ai();
        void do_nothing();
};


// for accessing curr_time and dt..
//extern AP *ap;
*/
