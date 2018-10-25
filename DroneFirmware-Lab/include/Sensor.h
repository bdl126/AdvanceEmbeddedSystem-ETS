/*
 * Sensor.h
 *
 *  Created on: 19 déc. 2012
 *      Author: bruno
 */

#ifndef SENSOR_H_
#define SENSOR_H_

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

#define POLICY SCHED_RR
#define THREADSTACK  65536

#define INVALID_CHECKSUM -1
#define OLD_SAMPLE        0
#define NEW_SAMPLE        1

#define NUM_SENSOR 		5
#define DATABUFSIZE 	   100
#define GYROCALMAX		1000
#define SONARCALMAX		100

enum { ACCELEROMETRE, GYROSCOPE, SONAR, BAROMETRE, MAGNETOMETRE };

#define ACCEL_RATE		1
#define GYRO_RATE		1
#define SONAR_RATE		8
#define BAROM_RATE		2
#define MAGNETO_RATE	2

typedef struct SensorRawData_struct {
	int16_t		status;
	uint32_t	timestamp_s;
	uint32_t	timestamp_n;
	uint32_t	ech_num;
	uint16_t	type;
	int16_t		data[3];
} SensorRawData;

typedef struct SensorData_struct {
	uint32_t	TimeDelay;
	double		Data[3];
} SensorData;

typedef struct SensorParam_struct {
	int16_t		AbsErrorMax;
	int16_t		RelErrorMax;
	double 		minVal, centerVal, maxVal, Conversion;
	double		alpha[3][3], beta[3];
} SensorParam;

typedef struct Sensor_struct {
	const char 			*DevName;
	const char 			*Name;
	int					File;
	uint16_t			type;
	pthread_spinlock_t 	DataLock;
	sem_t				DataSem;
	pthread_mutex_t 	DataSampleMutex;
	pthread_cond_t  	DataNewSampleCondVar;
	pthread_t 			SensorThread;
	pthread_t 			LogThread;
	uint16_t			DoLog;
	uint16_t			DataIdx;
	SensorParam			*Param;
	SensorRawData		*RawData;
	SensorData			*Data;
} SensorStruct;

#define ACCEL_PARAM { .AbsErrorMax 	        = 25, \
		 	 	 	  .RelErrorMax   		= 25, \
		 	 	 	  .minVal 		 		= 0.0, \
		 	 	 	  .centerVal 	 		= 2048.0, \
		 	 	 	  .maxVal 		 		= 4095.0, \
					  .Conversion 	 		= 4.0/2048.0, \
					  .alpha 		 		= {{0.98081557, -0.05644951, -0.02957259},{-0.05646295, 0.98907009, -0.03478460},{-0.02957224, -0.03477524, 1.02502064}}, \
					  .beta 		 		= {-0.03472795, -0.00820768, 0.01836154} \
					}

#define GYRO_PARAM { .AbsErrorMax		   = 25, \
	 	 	 	 	 .RelErrorMax		   = 25, \
	 	 	 	 	 .minVal			   = -4096.0, \
	 	 	 	 	 .centerVal			   = 0.0, \
	 	 	 	 	 .maxVal			   = 4095.0, \
	 	 	 	 	 .Conversion		   = M_PI/(180.0*16.375), \
	 	 	 	 	 .alpha				   = {{1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{0.0, 0.0, 1.0}}, \
	 	 	 	 	 .beta				   = {0.0, 0.0, 0.0} \
				   }

#define SONAR_PARAM { .AbsErrorMax			= 25, \
					  .RelErrorMax			= 25, \
					  .minVal				= 0.0, \
					  .centerVal			= 0.0, \
					  .maxVal				= 0.0, \
					  .Conversion			= 0.00034454, \
					  .alpha				= {{1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{0.0, 0.0, 1.0}}, \
					  .beta					= {0.0, 0.0, 0.0} \
					}

#define BAROM_PARAM { .AbsErrorMax			= 25, \
					  .RelErrorMax			= 25, \
					  .minVal				= 0.0, \
					  .centerVal			= 0.0, \
					  .maxVal				= 0.0, \
					  .Conversion			= 1.0, \
					  .alpha				= {{1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{0.0, 0.0, 1.0}}, \
					  .beta					= {0.0, 0.0, 0.0} \
					}

#define MAGNETO_PARAM { .AbsErrorMax  		  = 25, \
						.RelErrorMax  		  = 25, \
						.minVal		  		  = -300.0, \
						.centerVal	  		  = 0.0, \
						.maxVal		  		  = 300.0, \
						.Conversion	  		  = 12.0/4096.0, \
						.alpha		  		  = {{2.11841928, 0.00351888, 0.00397052},{0.00331451, 2.07320710, -0.01264231},{0.00518187, -0.00926185, 2.32147984}}, \
						.beta		  		  = {-0.12157171, -0.16832892, -1.31929740}, \
					  }


#define ACCEL_INIT { .DevName 	= "/dev/accel", \
					 .Name 		= "ACCELEROMETRE (m/s^2)", \
					 .File 		= -1, \
					 .type 		= ACCELEROMETRE, \
					 .DoLog     = 0, \
					 .DataIdx   = 0, \
					 .Param		= &ParamData[ACCELEROMETRE], \
					 .RawData	= &(RawData[ACCELEROMETRE][0]), \
					 .Data		= &(NavData[ACCELEROMETRE][0]) \
				   }

#define GYRO_INIT  { .DevName 	= "/dev/gyro", \
					 .Name 		= "GYROSCOPE", \
					 .File 		= -1, \
					 .type 		= GYROSCOPE, \
					 .DoLog     = 0, \
					 .DataIdx   = 0, \
					 .Param		= &ParamData[GYROSCOPE], \
					 .RawData	= &(RawData[GYROSCOPE][0]), \
					 .Data		= &(NavData[GYROSCOPE][0]) \
				   }

#define SONAR_INIT { .DevName 	= "/dev/sonar", \
					 .Name 		= "SONAR", \
					 .File 		= -1, \
					 .type 		= SONAR, \
					 .DoLog     = 0, \
					 .DataIdx   = 0, \
					 .Param		= &ParamData[SONAR], \
					 .RawData	= &(RawData[SONAR][0]), \
					 .Data		= &(NavData[SONAR][0]) \
				   }

#define BAROM_INIT { .DevName 	= "/dev/barom", \
					 .Name 		= "BAROMETRE", \
					 .File 		= -1, \
					 .type 		= BAROMETRE, \
					 .DataIdx   = 0, \
					 .Param		= &ParamData[BAROMETRE], \
					 .RawData	= &(RawData[BAROMETRE][0]), \
					 .Data		= &(NavData[BAROMETRE][0]) \
				   }

#define MAGNETO_INIT { .DevName  = "/dev/magneto", \
					   .Name 	 = "MAGNETOMETRE", \
					   .File 	 = -1, \
					   .type 	 = MAGNETOMETRE, \
					   .DoLog    = 0, \
					   .DataIdx  = 0, \
					   .Param	 = &ParamData[MAGNETOMETRE], \
					   .RawData	 = &(RawData[MAGNETOMETRE][0]), \
					   .Data	 = &(NavData[MAGNETOMETRE][0]) \
				     }


void *SensorTask(void *ptr);
void *SensorLogTask(void *ptr);

int   SensorsInit(SensorStruct SensorTab[]);
int   SensorsStart (void);
int   SensorsStop (SensorStruct SensorTab[]);
int   SensorsLogsInit (SensorStruct SensorTab[]);
int   SensorsLogsStart (void);
int   SensorsLogsStop (SensorStruct SensorTab[]);

#endif /* SENSOR_H_ */
