/*
 * Attitude.h
 *
 *  Created on: 19 déc. 2012
 *      Author: bruno
 */

#ifndef ATTITUDE_H_
#define ATTITUDE_H_

#define _POSIX_C_SOURCE 200809L
#define _XOPEN_SOURCE 500

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <poll.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <semaphore.h>
#include <string.h>
#include <math.h>

#include <Sensor.h>

#define POLICY SCHED_RR
#define THREADSTACK  65536

enum { X_AXE, Y_AXE, Z_AXE };

#define FC_ACCEL 	5.0
#define FC_GYRO 	5.0
#define FC_SONAR 	1.0
#define FC_BAROM 	1.0
#define FC_MAGNETO 	1.0


typedef struct AttData_struct {
	double 		Pitch;
	double 		Roll;
	double 		Yaw;
	double 		Elevation;
} AttData;

typedef struct AttitudeData_struct {
	uint32_t	timestamp_s;
	uint32_t	timestamp_n;
	AttData		Data;
	AttData		Speed;
	double 		Throttle;
	pthread_spinlock_t 	AttitudeLock;
} AttitudeData;


typedef struct AttitudeStruct_struct {
	AttitudeData	*AttitudeDesire;
	AttitudeData	*AttitudeMesure;
	SensorStruct	*Sensor;
	double			XYZTau;
	pthread_t 		AttitudeThread;
} AttitudeStruct;


#define ACCEL_ATT_INIT { .AttitudeDesire = &AttitudeDesire, \
					 	 .AttitudeMesure = &AttitudeMesure, \
					 	 .Sensor 		 = &SensorTab[ACCELEROMETRE], \
					 	 .XYZTau 		 = (1.0/(2.0*M_PI*FC_ACCEL)) \
				   	   }

#define GYRO_ATT_INIT { .AttitudeDesire = &AttitudeDesire, \
					 	.AttitudeMesure	= &AttitudeMesure, \
					 	.Sensor 		= &SensorTab[GYROSCOPE], \
					 	.XYZTau 		= (1.0/(2.0*M_PI*FC_GYRO)) \
				   	  }

#define SONAR_ATT_INIT { .AttitudeDesire = &AttitudeDesire, \
					 	 .AttitudeMesure = &AttitudeMesure, \
					 	 .Sensor 		 = &SensorTab[SONAR], \
					 	 .XYZTau 		 = (1.0/(2.0*M_PI*FC_SONAR)) \
				   	   }

#define BAROM_ATT_INIT { .AttitudeDesire = &AttitudeDesire, \
					 	 .AttitudeMesure = &AttitudeMesure, \
					 	 .Sensor 		 = &SensorTab[BAROMETRE], \
					 	 .XYZTau 		 = (1.0/(2.0*M_PI*FC_BAROM)) \
				   	   }

#define MAGNETO_ATT_INIT { .AttitudeDesire = &AttitudeDesire, \
					 	   .AttitudeMesure = &AttitudeMesure, \
					 	   .Sensor 		   = &SensorTab[MAGNETOMETRE], \
						   .XYZTau 		   = (1.0/(2.0*M_PI*FC_MAGNETO)) \
				   	     }

void *AttitudeTask(void *ptr);

int   AttitudeInit (AttitudeStruct AttitudeTab[NUM_SENSOR]);
int   AttitudeStart (void);
int   AttitudeStop (AttitudeStruct AttitudeTab[NUM_SENSOR]);

#endif /* ATTITUDE_H_ */
