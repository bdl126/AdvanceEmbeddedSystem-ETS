/*
 *  Mavlink.h
 *
 *  Librairie de communication pour le AR Drone 2.0
 *
 *  Date :	2012-08-31
 *  Auteur :	Vincent Trudel-Lapierre
 *  Contact : 	vincent.trudel-lapierre.1@ens.etsmtl.ca
 *
 */

#ifndef _DRONE_MAVLINK_H_
#define _DRONE_MAVLINK_H_

#include "mavlink/v1.0/common/mavlink.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h> 
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <semaphore.h>
#include <string.h>

#include "Control.h"

#define MAVLINK_STATUS_PERIOD	2
#define MAVLINK_RECEIVE_PERIOD	1

#define BUFFER_LENGTH 	1024
#define TARGET_IP 		"192.168.1.2" //IP de la station au sol QGroundControl
#define SYSTEM_ID 		1
#define COMPONENT_ID 	200

#define PITCH_MAX 		15.0
#define ROLL_MAX 		20.0
#define YAW_MAX 		90.0
#define HEIGHT_MAX 		6.0

typedef struct mavlink_struct {
	char 				target_ip[20];
	int					sock;
	struct sockaddr_in	gcAddr;
	struct sockaddr_in	locAddr;
	AttitudeData		*AttitudeDesire;
	AttitudeData		*AttitudeMesure;
	pthread_t 			MavlinkReceiveTask;
	pthread_t 			MavlinkStatusTask;
} MavlinkStruct;

int  MavlinkInit(MavlinkStruct *Mavlink, AttitudeData *AttitudeDesire, AttitudeData *AttitudeMesure, char *TargetIP);
int  MavlinkStart(void);
int  MavlinkStop(MavlinkStruct *Mavlink);


#endif
