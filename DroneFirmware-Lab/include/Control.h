/*
 * Control.h
 *
 *  Created on: 26 sept. 2013
 *      Author: bruno
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include <stdio.h>   /* Standard input/output definitions */
#include <stdlib.h>
#include <stdint.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <signal.h>
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <pthread.h>
#include <semaphore.h>
#include <sys/time.h>

#include "Sensor.h"
#include "Motor.h"
#include "Attitude.h"

#define POLICY SCHED_FIFO
#define THREADSTACK  65536

enum { HEIGHT, ROLL, PITCH, YAW };

#define CONTROL_PERIOD	1

#define TS 			(0.005)	// Ts = 5 ms
#define DEADZONE	(0.10)

typedef struct control_struct {
	AttitudeData		*AttitudeDesire;
	AttitudeData		*AttitudeMesure;
	MotorStruct			*Motor;
	pthread_t 			ControlThread;
} ControlStruct;

#define CONTROL_INIT { .AttitudeDesire 	= &AttitudeDesire, \
					   .AttitudeMesure	= &AttitudeMesure, \
					   .Motor 			= &Motor \
				   	 }

int ControlInit (ControlStruct *Control);
int ControlStart (void);
int ControlStop (ControlStruct *Control);

#endif /* CONTROL_H_ */
