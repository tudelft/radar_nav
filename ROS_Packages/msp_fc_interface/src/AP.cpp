#include "AP.hpp"


/*
//main program constructor 
// Can be used in the future if it becomes more complex
void AP::do_nothing() {
    ros::Rate rate_1(50);
    while(1) {
        rate_1.sleep();
    }
}

AP::AP() {
    AP_thread = std::thread(&AP::do_nothing, this);
}

*/

// Called when receiving avoid msg from DVS
void dvs_cmd_callback(const std_msgs::Int32::ConstPtr& msg) {
    controller->avoid = msg->data;
    ROS_INFO("Subscribed DVS command: [%d]", msg->data);
}

// Called when receiving avoid msg from Radar (using simple controller or full VO)
void radar_cmd_callback(const radar_avoid_msgs::Command::ConstPtr& command_msg) {
    controller->avoid = command_msg->avoid_state;
    controller->v_xa_des = (double)command_msg->dv[0];
    controller->v_ya_des = (double)command_msg->dv[1];
    //ROS_INFO("Subscribed Radar command: [%d]", command_msg->avoid_state);
}

// For publising state of MAV to radar (for VO)
void publish_state_to_radar(ros::Publisher pub_st) {
    rc_msgs::RcData rc_msg;
    rc_msg.state[0] = controller->vel_x_est_velFrame;
    rc_msg.state[1] = controller->vel_y_est_velFrame;;
    pub_st.publish(rc_msg);
}

Controller *controller;
msp_node *msp;
NatNet *gps;
int main(int argc, char** argv) {
    ros::init(argc, argv, "msp_fc_interface");
    ros::NodeHandle n;
    ros::Rate rate(50);//50 Hz
    ros::Publisher pub_st;

    controller = new Controller();// controller in seperate thread
    #ifdef USE_NATNET
    gps = new NatNet();// Optitrack thread
    #endif
    msp = new msp_node();// MSP comminication handled in this thread
    
    // All ROS communication handled here (not how its supposed to be but ok):
    pub_st = n.advertise<rc_msgs::RcData>("/MAV_state", 1, true);
    ros::Subscriber sub_dvs_cmd  = n.subscribe("/roll_command", 1, dvs_cmd_callback);
    ros::Subscriber sub_radar_cmd  = n.subscribe("radar_commands", 1, radar_cmd_callback);

    int i = 0;
    while (ros::ok()) {
        publish_state_to_radar(pub_st);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
