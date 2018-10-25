/*
 *  Mavlink.c
 *
 *  Module de communication avec la station au sol QGroundControl
 *
 *  Auteur :    Vincent Trudel-Lapierre
 *  Contact :   vincent.trudel-lapierre.1@ens.etsmtl.ca
 *
 */

#include "Mavlink.h"

extern sem_t	MavlinkReceiveTimerSem;
extern sem_t	MavlinkStatusTimerSem;
extern int		MavlinkActivated;

pthread_barrier_t 	MavlinkStartBarrier;

extern ControlStruct	Control;

void *MavlinkStatusTask(void *ptr) {
	MavlinkStruct		*Mavlink = (MavlinkStruct *) ptr;
	AttitudeData	*AttitudeDesire = Control.AttitudeDesire;
	AttitudeData		*AttitudeMesure = Mavlink->AttitudeMesure;
	AttData				Data, Speed;
	AttData			DataD, SpeedD;
	AttData			DataM, SpeedM;
	double			Error[4]    = {0.0, 0.0, 0.0, 0.0};
	uint32_t			TimeStamp;
	mavlink_message_t 	msg;
	uint16_t 			len;
	uint8_t 			buf[BUFFER_LENGTH];
	int 				bytes_sent;
	uint32_t 			sensor = 0xf00f;

	printf("%s : Mavlink Status démarré\n", __FUNCTION__);
	pthread_barrier_wait(&(MavlinkStartBarrier));

	while (MavlinkActivated) {
		sem_wait(&MavlinkStatusTimerSem);
		if (MavlinkActivated == 0)
			break;
		memset(buf, 0, BUFFER_LENGTH);
		pthread_spin_lock(&(AttitudeMesure->AttitudeLock));
		memcpy((void *) &Data, (void *) &(AttitudeMesure->Data), sizeof(AttData));
		memcpy((void *) &Speed, (void *) &(AttitudeMesure->Speed), sizeof(AttData));
		TimeStamp = AttitudeMesure->timestamp_s*1000 + AttitudeMesure->timestamp_n/1000000L;
		pthread_spin_unlock(&(AttitudeMesure->AttitudeLock));

		//Send Heartbeat
		mavlink_msg_heartbeat_pack(SYSTEM_ID, COMPONENT_ID, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(Mavlink->sock, buf, len, 0, (struct sockaddr *)&Mavlink->gcAddr, sizeof(struct sockaddr_in));

		// Send Status
		mavlink_msg_sys_status_pack(SYSTEM_ID, COMPONENT_ID, &msg, sensor, sensor, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(Mavlink->sock, buf, len, 0, (struct sockaddr *)&Mavlink->gcAddr, sizeof (struct sockaddr_in));

		// Send Local Position
		mavlink_msg_local_position_ned_pack(SYSTEM_ID, COMPONENT_ID, &msg, TimeStamp, 0, 0, (float) Data.Elevation, 0, 0, (float) Speed.Elevation);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(Mavlink->sock, buf, len, 0, (struct sockaddr *)&Mavlink->gcAddr, sizeof(struct sockaddr_in));

		pthread_spin_lock(&(AttitudeDesire->AttitudeLock));
		memcpy((void *) &DataD, (void *) &(AttitudeDesire->Data), sizeof(AttData));
		memcpy((void *) &SpeedD, (void *) &(AttitudeDesire->Speed), sizeof(AttData));
		pthread_spin_unlock(&(AttitudeDesire->AttitudeLock));

		pthread_spin_lock(&(AttitudeMesure->AttitudeLock));
		memcpy((void *) &DataM, (void *) &(AttitudeMesure->Data), sizeof(AttData));
		memcpy((void *) &SpeedM, (void *) &(AttitudeMesure->Speed), sizeof(AttData));
		pthread_spin_unlock(&(AttitudeMesure->AttitudeLock));
		Error[HEIGHT]    = DataD.Elevation - DataM.Elevation;
		Error[ROLL]      = DataD.Roll - DataM.Roll;
		Error[PITCH]     = DataD.Pitch - DataM.Pitch;
		Error[YAW]       = DataD.Yaw - DataM.Yaw;
		// Send Attitude
		mavlink_msg_attitude_pack(SYSTEM_ID, COMPONENT_ID, &msg, TimeStamp, (float) Data.Roll, (float) Data.Pitch, (float) Data.Yaw, (float) Speed.Roll, (float) Speed.Pitch, (float) Speed.Yaw);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		bytes_sent = sendto(Mavlink->sock, buf, len, 0, (struct sockaddr *)&Mavlink->gcAddr, sizeof(struct sockaddr_in));

	}
	printf("%s : Mavlink Status Arrêté\n", __FUNCTION__);
	pthread_exit(NULL);
}


void *MavlinkReceiveTask(void *ptr) {
	MavlinkStruct			*Mavlink = (MavlinkStruct *) ptr;
	AttitudeData			*AttitudeDesire = Mavlink->AttitudeDesire;
	AttData			 		 Data, Speed;
	double					 Pitch, Roll, Yaw, Elevation;
	mavlink_message_t		 msg;
	mavlink_status_t		 status;
	mavlink_manual_control_t man_control;
	socklen_t 				 fromlen;
	ssize_t 				 recsize;
	uint8_t 				 buf[BUFFER_LENGTH];
	int 					 i = 0;

	printf("%s : Mavlink Receive démarré\n", __FUNCTION__);
	pthread_barrier_wait(&(MavlinkStartBarrier));

	while (MavlinkActivated) {
		sem_wait(&MavlinkReceiveTimerSem);
		if (MavlinkActivated == 0)
			break;

		memset(buf, 0, BUFFER_LENGTH);
		recsize = recvfrom(Mavlink->sock, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&Mavlink->gcAddr, &fromlen);
		if (recsize > 0) {
			// Something received - print out all bytes and parse packet
			for (i = 0; i < recsize; ++i) {
				if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
					// Packet received
				}
			}
			if (msg.msgid == MAVLINK_MSG_ID_MANUAL_CONTROL) {
				mavlink_msg_manual_control_decode(&msg, &man_control);
				Pitch		= -((double)man_control.y/1000.0)*PITCH_MAX*M_PI/180.0;
				Roll		= ((double)man_control.x/1000.0)*ROLL_MAX*M_PI/180.0;
				Yaw			= -((double)man_control.r/1000.0)*YAW_MAX*M_PI/180.0;
				Elevation	= HEIGHT_MAX*((double)man_control.z/1000.0);
				Speed.Pitch	    = (Pitch - Data.Pitch)/TS;
				Speed.Roll	    = (Roll - Data.Roll)/TS;
				Speed.Yaw	    = (Yaw - Data.Yaw)/TS;
				Speed.Elevation = (Elevation - Data.Elevation)/TS;
				Data.Pitch	    = Pitch;
				Data.Roll	    = Roll;
				Data.Yaw	    = Yaw;
				Data.Elevation  = Elevation;
				pthread_spin_lock(&(AttitudeDesire->AttitudeLock));
				memcpy((void *) &(AttitudeDesire->Data), (void *) &Data, sizeof(AttData));
				memcpy((void *) &(AttitudeDesire->Speed), (void *) &Speed, sizeof(AttData));
				AttitudeDesire->Throttle = (float)man_control.z/1000;
				pthread_spin_unlock(&(AttitudeDesire->AttitudeLock));
			} else {	// Un message non attendu a ete recu
			}
		}
	}
	printf("%s : Mavlink Receive Arrêté\n", __FUNCTION__);
	pthread_exit(NULL);

}


int MavlinkInit(MavlinkStruct *Mavlink, AttitudeData *AttitudeDesire, AttitudeData *AttitudeMesure, char *TargetIP) {
	pthread_attr_t		attr;
	struct sched_param	param;
	int					minprio, maxprio;
	int retval = 0;

	Mavlink->AttitudeDesire = AttitudeDesire;
	Mavlink->AttitudeMesure = AttitudeMesure;

	strcpy(Mavlink->target_ip, TargetIP);
	Mavlink->sock 		= socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	memset((void *) &(Mavlink->locAddr), 0, sizeof(struct sockaddr_in));
	Mavlink->locAddr.sin_family 	 = AF_INET;
	Mavlink->locAddr.sin_addr.s_addr = INADDR_ANY;
	Mavlink->locAddr.sin_port 		 = htons(14551);

	/* Bind the socket to port 14551 - necessary to receive packets from qgroundcontrol */ 
	retval = bind(Mavlink->sock,(struct sockaddr *)&(Mavlink->locAddr), sizeof(struct sockaddr));
	if (retval == -1) {
		printf("%s : erreur bind échoué", __FUNCTION__);
		close(Mavlink->sock);
		return retval;
	} 

	/* Attempt to make it non blocking */
	if ((retval = fcntl(Mavlink->sock, F_SETFL, fcntl(Mavlink->sock, F_GETFL) | O_ASYNC | O_NONBLOCK)) < 0) {
		printf("%s : erreur ouverture non-bloquante", __FUNCTION__);
		close(Mavlink->sock);
		return retval;
	}

	memset(&Mavlink->gcAddr, 0, sizeof(struct sockaddr_in));
	Mavlink->gcAddr.sin_family 		= AF_INET;
	Mavlink->gcAddr.sin_addr.s_addr = inet_addr(Mavlink->target_ip);
	Mavlink->gcAddr.sin_port 		= htons(14550);

	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
	minprio = sched_get_priority_min(POLICY);
	maxprio = sched_get_priority_max(POLICY);
	pthread_attr_setschedpolicy(&attr, POLICY);
	param.sched_priority = minprio + (maxprio - minprio)/4;
	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);

	printf("Creating Mavlink thread\n");
	pthread_barrier_init(&MavlinkStartBarrier, NULL, 3);

	sem_init(&MavlinkReceiveTimerSem, 0, 0);
	sem_init(&MavlinkStatusTimerSem, 0, 0);

	retval = pthread_create(&Mavlink->MavlinkReceiveTask, &attr, MavlinkReceiveTask, Mavlink);
	if (retval) {
		printf("pthread_create : Impossible de créer le thread Mavlink_reception_thread\n");
		return retval;
	}

	retval = pthread_create(&Mavlink->MavlinkStatusTask, &attr, MavlinkStatusTask, Mavlink);
	if (retval) {
		printf("pthread_create : Impossible de créer le thread MavlinkStatusTask\n");
		return retval;
	}

	pthread_attr_destroy(&attr);

	return retval;
}


int MavlinkStart (void) {
	int retval = 0;

	MavlinkActivated = 1;
	pthread_barrier_wait(&(MavlinkStartBarrier));
	pthread_barrier_destroy(&MavlinkStartBarrier);
	printf("%s Mavlink démarré\n", __FUNCTION__);

	return retval;
}


int MavlinkStop(MavlinkStruct *Mavlink) {
	int err;

	MavlinkActivated = 0;
	sem_post(&MavlinkReceiveTimerSem);
	sem_post(&MavlinkStatusTimerSem);

	err = pthread_join(Mavlink->MavlinkStatusTask, NULL);
	if (err) {
		printf("pthread_join(MavlinkStatusTask) : Erreur\n");
		return err;
	}

	err = pthread_join(Mavlink->MavlinkReceiveTask, NULL);
	if (err) {
		printf("pthread_join(MavlinkReceiveTask) : Erreur\n");
		return err;
	}

	sem_destroy(&MavlinkReceiveTimerSem);
	sem_destroy(&MavlinkStatusTimerSem);

	return err;
}

