/** \file natnet2ivy.c
*  \brief NatNet (GPS) to RPI forwarder
*
* This receives aircraft position information through the Optitrack system
* NatNet UDP stream and forwards it to the Raspberry Pi. An aircraft with the gps
* subsystem "datalink" is then able to parse the GPS position and use it to
* navigate inside the Optitrack system.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <ros/ros.h>

#include "udp_socket.h"
#include "utils.h"
#include "natnet.hpp"
#include "AP.hpp"

/** Debugging options */
uint8_t verbose = 0;
#define printf_natnet(...)   if(verbose > 1) fprintf (stderr, __VA_ARGS__)
#define printf_debug(...)    if(verbose > 0) fprintf (stderr, __VA_ARGS__)

/* rx udp */
char const *rx_addr = "127.0.0.1";
uint16_t rx_port = 5000;

FILE *optitrack_f;

robot_t probot;

/** Natnet socket connections */
struct UdpSocket rx_data;

void NatNet::natnet_rx() {
	// start loop before arming
	ros::Rate rate_2(500);
	while(1) {
                //print("running\n");
		if (1){//st_mc->arm_status == ARM) {
			
			// TODO: circ buffer?
			static unsigned char buffer_data[50];
			static int bytes_data = 0;
			bytes_data += udp_socket_recv(&rx_data, buffer_data, 50);
			// Parse NatNet data
			if (bytes_data > (int) sizeof(robot_t)) {
				if (buffer_data[0] == '$') {
					// TODO: endianness check of processor
					memcpy(&probot, &buffer_data[1], sizeof(robot_t));

					// check if inf or NaN
					bool chk = !(isfinite(probot.pos.x));
					chk |= !(isfinite(probot.pos.y));
					chk |= !(isfinite(probot.pos.z));
					chk |= isnan(probot.pos.x);
					chk |= isnan(probot.pos.y);
					chk |= isnan(probot.pos.z);

					chk |= !(isfinite(probot.vel.x));
					chk |= !(isfinite(probot.vel.y));
					chk |= !(isfinite(probot.vel.z));
					chk |= isnan(probot.vel.x);
					chk |= isnan(probot.vel.y);
					chk |= isnan(probot.vel.z);

					chk |= !(isfinite(probot.att.roll));
					chk |= !(isfinite(probot.att.pitch));
					chk |= !(isfinite(probot.att.yaw));
					chk |= isnan(probot.att.roll);
					chk |= isnan(probot.att.pitch);
					chk |= isnan(probot.att.yaw);

					// if it is a safe number; then copy it to controller class
					if (!chk) {
                                                //printf("%f\n", probot.pos.x);
						controller->robot.pos = probot.pos;
						controller->robot.vel = probot.vel;
						#ifdef HEADING_FROM_OPTITRACK
						//# warning CAUTION: HEADING FROM OPTITRACK NOT TESTED!! 
						controller->robot.att.yaw = probot.att.yaw;
						#endif
					} else {
						// NaN or Inf
						printf(COLOR_FBLACK);
						printf(COLOR_BRED);
						printf("[natnet-err] NaN or Inf in sent states\n");
						printf(COLOR_NONE);
						printf("\n");
						continue;
					}
					// printf("%f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f, %.02f\n", 
					// ai->curr_time, robot.pos.x, robot.pos.y, robot.pos.z, robot.vel.x, robot.vel.y, robot.vel.z, robot.att.roll, robot.att.pitch, robot.att.yaw);
					
					// fprintf(optitrack_f, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", 
					// ai->curr_time, robot.pos.x, robot.pos.y, robot.pos.z, robot.vel.x, robot.vel.y, robot.vel.z, robot.att.roll, robot.att.pitch, robot.att.yaw);
				}
				bytes_data = 0;
			}
		}
		rate_2.sleep();
		//std::this_thread::sleep_for(std::chrono::milliseconds(2)); // 500 Hz listenin
	}
}

NatNet::NatNet() {

	// rx udp
	udp_socket_create(&rx_data, (char *) rx_addr, -1, rx_port, 0);
	udp_socket_set_recvbuf(&rx_data, 0x100000); // 1MB rx loopback buffer

	// start the natnet thread
	try {
		natnet_thread_ = std::thread(&NatNet::natnet_rx, this);
		optitrack_f = fopen("optitrack.csv", "w+");
		printf("[gps] thread spawned!\n");
	} catch (...) {
		printf("[gps] thread couldn't be spawned!\n");
	}

}

NatNet::~NatNet() {
	// if (natnet_thread_.joinable()) {
	fflush(optitrack_f);
	fclose(optitrack_f);
	natnet_thread_.detach();
	printf("[gps] thread killed!\n");
}
