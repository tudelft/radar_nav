#include "controller.hpp"

#define ATT_RCMIN 1200
#define ATT_RCMAX 1800

#define ATT_YAW_RCMIN 1300
#define ATT_YAW_RCMAX 1700

#define THRUST_FLOAT_MAX 0.8  // MAX THROTTLE 
#define THRUST_FLOAT_MIN 0.1  // MIN SAFE THROTTLE
#define KP_ALT 0.32
#define KI_ALT 0.0
#define KD_ALT 0.15
#define HOVERTHRUST 0.5
#define SETPOINT_ALT (-1.5)

#define KP_POS      1.0
#define KP_VEL      3
#define KD_VEL      1
#define KI_VEL      1
#define MAX_BANK    0.65   // 26 deg max bank
#define K_FF        0.0
#define MAX_VEL     2.5

#define MAX_YAW_RATE 0.5
#define KP_YAW       1.0

// min 1050 max 1950
#define THRUST_RCMIN 1000
#define THRUST_RCMAX 2000


//========================================== NAVIGATION ===============================================
void Controller::avoid_obstacles() {
    this->signals_f.thr = 0.3;
    float velcmdbody_x = 0.8;// ms-1
    float velcmdbody_y = 0;// ms-1

    #ifdef USE_VO
    // If sensors give avoidance command, and not in avoidance state:
    if (this->avoid != 0 && this->controller_avoid == 0) {
        this->controller_avoid = this->avoid;
        this->avoid = 0;
        this->loop_index = 0;
        this->controller_vx = bound_f(this->v_xa_des, 0, 0.5);
        this->controller_vy = bound_f(this->v_ya_des, -0.5, 0.5);
    }

    // If in avoid state:
    if (this->controller_avoid != 0) {
        if (this->loop_index <= 10) {
            velcmdbody_x = 0;
            velcmdbody_y = 0;
        } else if (this->loop_index > 10 && this->loop_index >= 100) {
            velcmdbody_x = this->controller_vx;
            velcmdbody_y = this->controller_vy;
        } else if (this->loop_index > 100 && this->loop_index >= 120) {
            velcmdbody_x = 0;
            velcmdbody_y = 0;
        } else if (this->loop_index > 120) {// 120 cycles at 50 Hz ~ 2.4s
            this->controller_avoid = 0;
        }
        this->loop_index += 1;
    }

    #else
    // Simple avoidance algorithm that translates to the side by approx 1m
    // If sensors give avoidance command, and not in avoidance state:
    if (this->avoid != 0 && this->controller_avoid == 0) {
        //printf("Avoiding %d\n", this->avoid);
        this->controller_avoid = this->avoid;
        this->avoid = 0;
        this->loop_index = 0;
    }

    // If in avoid state:
    if (this->controller_avoid != 0) {
        velcmdbody_x = 0;
        if (this->controller_avoid == 1) {
            velcmdbody_y = -0.5;// y axis points left
        } else {
            velcmdbody_y = 0.5;
        }
        this->loop_index += 1;
        if (this->loop_index > 75) {//1.5 seconds at 50hz
            this->controller_avoid = 0;
        }
    }
    #endif

    #ifdef USE_NATNET
    this->velocity_control(velcmdbody_x, velcmdbody_y);
    #else
    # warning CAUTION: THIS SHOULD NOT BE USED, ONLY FOR TESTING
    // should try to determine (crude) velocity state from IMU and radar/dvs (assuming static env, or using OPTICAL FLOW sensor)
    this->signals_f.yb = 1500 + velcmdbody_x*40;
    this->signals_f.xb = 1500 + velcmdbody_y*40;
    #endif
}

void Controller::velocity_control(float velcmdbody_x, float velcmdbody_y) {

    this->vel_x_est_velFrame =  cos(robot.att.yaw) * robot.vel.x - sin(robot.att.yaw) * robot.vel.y;
    this->vel_y_est_velFrame =  sin(robot.att.yaw) * robot.vel.x + cos(robot.att.yaw) * robot.vel.y;
    
    float curr_error_vel_x = velcmdbody_x - this->vel_x_est_velFrame;
    float curr_error_vel_y = velcmdbody_y - this->vel_y_est_velFrame;
    
    float D_error_x = (curr_error_vel_x - last_error_vel_x)/dt;
    float D_error_y = (curr_error_vel_y - last_error_vel_y)/dt;

    float I_error_x = (curr_error_vel_x + last_error_vel_x)*0.5*dt;
    float I_error_y = (curr_error_vel_y + last_error_vel_y)*0.5*dt;

    last_error_vel_x = curr_error_vel_x;
    last_error_vel_y = curr_error_vel_y;

    // PID + FF
    float accx_cmd_velFrame = curr_error_vel_x * KP_VEL + D_error_x * KD_VEL + I_error_x * KI_VEL;//K_FF * velcmdbody_x;
    float accy_cmd_velFrame = curr_error_vel_y * KP_VEL + D_error_y * KD_VEL + I_error_y * KI_VEL;//K_FF * velcmdbody_y;

    this->signals_f.yb = bound_f(accx_cmd_velFrame, -MAX_BANK, MAX_BANK);
    this->signals_f.xb = -1.0 * bound_f(accy_cmd_velFrame, -MAX_BANK, MAX_BANK);
    //this->signals_f.zb = 0; // TODO: yaw towards goal -> atan2(curr_error_pos_w_y, curr_error_pos_w_x);
}

/**
 * Function to control (fixed) yaw
 */
void Controller::attitude_control() {
    //float setpoint = -80.0 * D2R;
    float yawerror = this->yaw_setpoint - this->robot.att.yaw;
    // Set this to zero if alignment causes problems: (IMU should be good enough to handle)
    this->signals_f.zb = bound_f(KP_YAW * yawerror, -MAX_YAW_RATE, MAX_YAW_RATE);
}

// bound rate of change of these signals 
void rateBound(signals<float> *signal) {

    // printf("in: %.05f,%.05f,%.05f,%.05f\n", signal->thr, signal->xb, signal->yb, signal->zb);
    
    // 0.03 = 30 on radio
    #define MAXRATE 0.1

    // do static structs exist?
    static signals<float> prev_signal = *signal;

    float d_thr = bound_f(signal->thr - prev_signal.thr, -MAXRATE, MAXRATE);
    float d_xb  = bound_f(signal->xb - prev_signal.xb, -MAXRATE, MAXRATE);
    float d_yb  = bound_f(signal->yb - prev_signal.yb, -MAXRATE, MAXRATE);
    float d_zb  = bound_f(signal->zb - prev_signal.zb, -MAXRATE, MAXRATE);

    // new bounded signal output
    signal->thr = prev_signal.thr + d_thr;
    signal->xb = prev_signal.xb + d_xb;
    signal->yb = prev_signal.yb + d_yb;
    signal->zb = prev_signal.zb + d_zb;

    // prev_signal = *signal;
    prev_signal.thr = signal->thr;
    prev_signal.xb = signal->xb;
    prev_signal.yb = signal->yb;
    prev_signal.zb = signal->zb;

    // printf("out: %.05f,%.05f,%.05f,%.05f\n", signal->thr, signal->xb, signal->yb, signal->zb);

}

// send control signals
void Controller::toActuators() {

    // auto signals = this_hal->get_nav()->get_signals();
    rateBound(&this->signals_f);

    bool chk = !(isfinite(this->signals_f.thr));
    chk |= !(isfinite(this->signals_f.xb));
    chk |= !(isfinite(this->signals_f.yb));
    chk |= !(isfinite(this->signals_f.zb));
    chk |= isnan(this->signals_f.thr);
    chk |= isnan(this->signals_f.xb);
    chk |= isnan(this->signals_f.yb);
    chk |= isnan(this->signals_f.zb);
    
    if (chk) {
        // NaN or Inf in calculations before sending to betaflight
        printf(COLOR_FBLACK);
        printf(COLOR_BRED);
        printf("[control] NaN or Inf sent to betaflight\n");
        printf(COLOR_NONE);
        printf("\n");
        signals_f.thr = 0.0;
        signals_f.xb = 0.0;
        signals_f.yb = 0.0;
        signals_f.zb = 0.0;
        return;
    }

    // float to uint16_t for sending over MSP UART
    // Check if surface mode is engaged, otherwise put thrust low:
    if (this->channel3_curr < 1600) {
        this->signals_i.thr = 1200;
    } else {
        this->signals_i.thr = 1500;
    }
	this->signals_i.xb  = remap_attitude_signals(this->signals_f.xb,   ATT_RCMIN, ATT_RCMAX);  // roll
	this->signals_i.yb  = remap_attitude_signals(this->signals_f.yb,   ATT_RCMIN, ATT_RCMAX);  // pitch
	this->signals_i.zb  = remap_attitude_signals(this->signals_f.zb,   ATT_YAW_RCMIN, ATT_YAW_RCMAX);  // yaw

    // TODO: mutex control signals with MSP
	// this_hal->get_nav()->update_signals(signals);
	// this_hal->get_nav()->send_signals();
    //printf("%.02f,%.02f,%.02f,%.02f\n", signals_f.thr, signals_f.xb, signals_f.yb, signals_f.zb);
    //printf("%d,%d,%d,%d\n", signals_i.thr, signals_i.xb, signals_i.yb, signals_i.zb);
}

//======================================== KEYBOARD ====================================================
/**
 * Function to change input from controller to keyboard control
 * Will also check if channel override is engaged, and set yawpoint
 */
void Controller::change_input(char key) {
    // Switch to MSP override when flipping switch (set on aux2):
    if (this->channel2_curr > 1700 && this->channel2_prev < 1700){
        this->yaw_setpoint = this->robot.att.yaw;//from natnet or current orientation from inav
        printf("switched to onboard control\n"); 
        printf("setting fixed yaw orientation...\n");
    }
    #ifdef USE_NATNET
    // Switch to control with keyboard: default not (input == 1)
    if (key == 'U' || key == 'u'){
	    this->controller_avoid = 0;
        this->signals_i.thr = 1300;
        this->signals_i.yb = 1500; //p | e
        this->signals_i.xb = 1500; //r | a
        this->signals_i.zb = 1500; //y | r
        if (this->input == 0) {
            this->input = 1;
        } else if (this->input == 1) {
            printf("switched control\n");
            this->input = 0;
        }
    #else
    if (key == 'U' || key == 'u'){
        printf("\nWARNING: natnet not defined, cannot switch\n");
    #endif
    // Force controller to avoid left or right (only possible with crude controller, not VO)
    } else if (key == 'X' || key == 'x') {
        this->avoid = 1;
    } else if (key == 'Z' || key == 'z') {
        this->avoid = -1;
    }
}

/*
* Function to modify desired state using asdw (use with caution)
*/
void Controller::set_keys(char key) {
    if ((key == 'D' || key == 'd') && this->signals_i.yb < 1700){
        this->signals_i.xb += 30;
    } else if ((key == 'A' || key == 'a') && this->signals_i.yb > 1300) {
        this->signals_i.xb -= 30;
    } else if ((key == 'W' || key == 'w') && this->signals_i.xb < 1700) {
        this->signals_i.yb += 30;
    } else if ((key == 'S' || key == 's') && this->signals_i.xb > 1300) {
        this->signals_i.yb -= 30;
    } else if ((key == 'Q' || key == 'q') && this->signals_i.zb > 1300) {
        this->signals_i.zb -= 30;
    } else if ((key == 'E' || key == 'e') && this->signals_i.zb < 1700) {
        this->signals_i.zb += 30;
    } else if (key == '\x03') {
        printf("stopping...\n");//ROS_INFO_STREAM
        ros::shutdown();
    }
}

/*
* Function to get keyboard keys
*/
int Controller::getch(void){
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    //newt.c_lflag &= ~(ECHO | ICANON);
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();

    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

//============================================ MAIN ====================================================

void Controller::control_job() {
    ros::Rate rate_1(50);
    this->signals_i.thr = 1300;
    this->signals_i.yb = 1500; //p | e
    this->signals_i.xb = 1500; //r | a
    this->signals_i.zb = 1500; //y | r
    this->yaw_setpoint = this->robot.att.yaw;
    while(1) {
        char key(' ');
        key = this->getch();
        this->change_input(key);
        if (this->input == 0) {
            this->set_keys(key);
        } else {
            this->avoid_obstacles();
            this->attitude_control();
            this->toActuators();
        }
        
        rate_1.sleep();
    }
}

// constructor
Controller::Controller() {
    try {
        control_job_ = std::thread(&Controller::control_job, this);
        printf("[ctrl] thread spawned!\n");
    } catch (...) {
        printf("[ctrl] can't start thread!\n");
    }
}

Controller::~Controller() {
    // fflush all files
    // if (control_job_.joinable()) {
        control_job_.detach();
    // }
    printf("[ctrl] thread killed!\n");
}
