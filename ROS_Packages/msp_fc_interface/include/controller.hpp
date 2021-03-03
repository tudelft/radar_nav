#pragma once
#include <thread>
#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include "state.h"
#include "utils.h"

#define USE_NATNET 0
#define HEADING_FROM_OPTITRACK 0
#define USE_VO 0


/* remap attitude signals, -1 to 1 is mapped to 1000 to 2000 
 * If this function returns value tmpval = 0, zero attitude command is sent to betaflight. */
inline uint16_t remap_attitude_signals(float val, uint16_t min, uint16_t max) {
	float avg = ((float) min + (float) max) * 0.5;
	uint16_t tmpval = round((float) avg + (val) * ((float)max - (float)min) * 0.5);
	if (tmpval > max) {
		tmpval = max;
	}
	if (tmpval < min) {
		tmpval = min;
	}
	return tmpval;
}

class Controller {
    private:
        std::thread control_job_;
        //Internal avoidance state:
        int controller_avoid = 0;

        //Internal avoidance velocity vector:
        double controller_vx = 0;// same same but different
        double controller_vy = 0;

        //yaw setpoint when switching over to autonomous control:
        float yaw_setpoint;

    public:
        uint8_t input = 1;
        robot_t robot;

        //Avoidance state:
        int avoid = 0;// -1 = left, 0 = straight, 1 = right MODIFIED EXTERNALLY

        //Avoidance velocity vector:
        double v_xa_des = 0;// velocity obstacles desired velcmdbody_x
        double v_ya_des = 0;

        uint8_t loop_index = 0;

        //for DI control:
        float last_error_vel_x = 0;
        float last_error_vel_y = 0;
        float dt = 0.02;

        // For monitoring when switching to autonomous control
        uint16_t channel3_curr;// to see when switched to onboard ctrl
        uint16_t channel2_curr;// to see when switched to onboard ctrl
        uint16_t channel2_prev;

        //Current velocity vector to transmit to radar
        float vel_x_est_velFrame;
        float vel_y_est_velFrame;
        
		/* signals_f for thrust must be between 0 to +1
		   signals_f for attitude must be between -1 to +1 */
        signals<float> signals_f;

		/* these signals are directly picked up by uart.
		   carefully bound these between 1000 to 2000 */
        signals<uint16_t> signals_i;
        Controller();
        ~Controller();
        void control_job();
        void change_input(char key);
        void set_keys(char key);
        int getch(void);
        //void control_job();
        //void altitude_control();
		//float position_control();
        void attitude_control();
        void avoid_obstacles();
		void velocity_control(float, float);
        void toActuators();
		//void rateBound(signals<float> *signal);
};
