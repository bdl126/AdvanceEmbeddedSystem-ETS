/*
 * Control.c
 *
 *  Created on: 28 sept. 2013
 *      Author: bruno
 */

#include "Control.h"

extern sem_t 		ControlTimerSem;
extern int			ControlActivated;


pthread_barrier_t 	ControlStartBarrier;

//				 ROLL	PITCH	YAW		HEIGHT
double KP[4] = {0.125,  0.125,  0.07,    0.0  };
double KI[4] = {0.0,    0.0,    0.001,   0.0  };
double KD[4] = {0.004,  0.004,  0.00004, 0.0  };


void *ControlTaskOld ( void *ptr ) {
	ControlStruct	*Control  = (ControlStruct *) ptr;
	AttitudeData	*AttitudeDesire = Control->AttitudeDesire;
	AttitudeData	*AttitudeMesure = Control->AttitudeMesure;
	AttData			*DataDesire = &(AttitudeDesire->Data);
	AttData			*DataMesure = &(AttitudeMesure->Data);
	MotorStruct		*Motor	  = Control->Motor;
	double			Error[4]  = {0.0, 0.0, 0.0, 0.0};
	double			IError[4] = {0.0, 0.0, 0.0, 0.0};
	double			DError[4] = {0.0, 0.0, 0.0, 0.0};
	double			OldError[4] = {0.0, 0.0, 0.0, 0.0};
	double			AttAdj[4] = {0.0, 0.0, 0.0, 0.0};
	double			Throttle;
	int				i;

	printf("%s : Control démarré\n", __FUNCTION__);
	pthread_barrier_wait(&ControlStartBarrier);

	while (ControlActivated) {
		sem_wait(&ControlTimerSem);
		if (ControlActivated == 0)
			break;

		pthread_spin_lock(&(AttitudeMesure->AttitudeLock));
		Error[ROLL]    = DataMesure->Roll;
		Error[PITCH]   = DataMesure->Pitch;
		Error[YAW]     = DataMesure->Yaw;
		Error[HEIGHT]  = DataMesure->Elevation;
		pthread_spin_unlock(&(AttitudeMesure->AttitudeLock));
		pthread_spin_lock(&(AttitudeDesire->AttitudeLock));
		Error[ROLL]   -= DataDesire->Roll;
		Error[PITCH]  -= DataDesire->Pitch;
		Error[YAW]    -= DataDesire->Yaw;
		Error[HEIGHT] -= DataDesire->Elevation;
		Throttle 	   = AttitudeDesire->Throttle;
		pthread_spin_unlock(&(AttitudeDesire->AttitudeLock));
		if (Throttle <= 2.0*DEADZONE) {
			for (i = ROLL; i < YAW; i++)
				AttAdj[i]   = 0.0;
			Throttle = 0.0;
			pthread_spin_lock(&(Motor->MotorLock));
			for (i = 0; i < 4; i++)
				Motor->pwm[i] = 0;
		   	pthread_spin_unlock(&(Motor->MotorLock));
			continue;
		}

		for (i = ROLL; i < YAW; i++) {
			IError[i]   = IError[i] + 0.5*TS*(Error[i]+OldError[i]);
			DError[i]   = (Error[i]-OldError[i])/TS;
			AttAdj[i]   = KP[i]*Error[i] + KI[i]*IError[i] + KD[i]*DError[i];
			OldError[i] = Error[i];
		}

		pthread_spin_lock(&(Motor->MotorLock));
	   	Motor->pwm[0] = (uint16_t)((Throttle + AttAdj[ROLL] + AttAdj[PITCH] + AttAdj[YAW])*0x01ff); //(uint16_t)(AttitudeDesiree->Throttle*0x01ff);
	   	Motor->pwm[1] = (uint16_t)((Throttle - AttAdj[ROLL] + AttAdj[PITCH] - AttAdj[YAW])*0x01ff); //(uint16_t)(AttitudeDesiree->Throttle*0x01ff);
	   	Motor->pwm[2] = (uint16_t)((Throttle - AttAdj[ROLL] - AttAdj[PITCH] + AttAdj[YAW])*0x01ff); //(uint16_t)(AttitudeDesiree->Throttle*0x01ff);
	   	Motor->pwm[3] = (uint16_t)((Throttle + AttAdj[ROLL] - AttAdj[PITCH] - AttAdj[YAW])*0x01ff); //(uint16_t)(AttitudeDesiree->Throttle*0x01ff);
	   	Motor->pwm[0] = (Motor->pwm[0] < DEADZONE) ? DEADZONE : Motor->pwm[0];
	   	Motor->pwm[1] = (Motor->pwm[1] < DEADZONE) ? DEADZONE : Motor->pwm[1];
	   	Motor->pwm[2] = (Motor->pwm[2] < DEADZONE) ? DEADZONE : Motor->pwm[2];
	   	Motor->pwm[3] = (Motor->pwm[3] < DEADZONE) ? DEADZONE : Motor->pwm[3];
	   	Motor->led[0] = (Motor->pwm[0] > 0.0) ? MOTOR_LEDGREEN : MOTOR_LEDRED;
	   	Motor->led[1] = (Motor->pwm[1] > 0.0) ? MOTOR_LEDGREEN : MOTOR_LEDRED;
	   	Motor->led[2] = (Motor->pwm[2] > 0.0) ? MOTOR_LEDGREEN : MOTOR_LEDRED;
	   	Motor->led[3] = (Motor->pwm[3] > 0.0) ? MOTOR_LEDGREEN : MOTOR_LEDRED;
	   	pthread_spin_unlock(&(Motor->MotorLock));
	}

   	printf("%s : Control Arrêté\n", __FUNCTION__);

	pthread_exit(0); /* exit thread */
}

#define Ixx	(0.0241/2.0)					//	[kg*m^2]	Moment d'inertie autour de X
#define Iyy	(0.0232/2.0)					//	[kg*m^2]	Moment d'inertie autour de Y
#define Izz	(0.0451/2.0)					//	[kg*m^2]	Moment d'inertie autour de Z
#define b	(0.00001418713138124377/3.0)	//	[N*s^2]		Coefficient de poussée des moteurs    (Note : valeur suggérée par Guillaume - Dronolab)
#define d	(0.00000184)				//	[N*m*s^2]	Coefficient de trainée des moteurs  (Note : valeur suggérée par Guillaume - Dronolab)
#define l	(0.18)						//	[m]			Longueur des bras
#define m	(0.457)						//	[kg]		Poids du Drone
#define Jr	(0.000078187598)			//	[kg*m^2]	Inertie du moteur  (Note : valeur suggérée par Guillaume - Dronolab)
#define g	(9.80665)					//	[m/s^2]		Accélération gravitationnelle

#define a1	((Iyy-Izz)/Ixx)
#define a2	(Jr/Ixx)
#define a3	((Izz-Ixx)/Iyy)
#define a4	(Jr/Iyy)
#define a5	((Ixx-Iyy)/Izz)
#define b1	(1.0/Ixx)
#define b2	(1.0/Iyy)
#define b3	(1.0/Izz)

#define aa	(2.828427124)  //	2*sqrt(2)

#define AAA 5.0
#define BBB 3.0

#define ka0	(AAA)		//	Gain ka	->	Altitude
#define kb0	(BBB)		//	Gain kb	->	Altitude

#define ka1	(AAA)		//	Gain ka	->	Roulis
#define kb1	(BBB)		//	Gain kb	->	Roulis

#define ka2	(AAA)		//	Gain ka	->	Tangage
#define kb2	(BBB)		//	Gain kb	->	Tangage

#define ka3	(AAA)		//	Gain ka	->	Lacet
#define kb3	(BBB)		//	Gain kb	->	Lacet


#define WtoPWM_Gain		(0.0026854)
#define WtoPWM_Offset	(0.341465)

void *ControlTask ( void *ptr ) {
	ControlStruct	*Control  = (ControlStruct *) ptr;
	AttitudeData	*AttitudeDesire = Control->AttitudeDesire;
	AttitudeData	*AttitudeMesure = Control->AttitudeMesure;
	AttData			DataD, SpeedD;
	AttData			DataM, SpeedM;
	MotorStruct		*Motor	  = Control->Motor;
	double			Error[4]    = {0.0, 0.0, 0.0, 0.0};
	double			DotError[4] = {0.0, 0.0, 0.0, 0.0};
	double			U[4]   = {0.0, 0.0, 0.0, 0.0};
	double			W[4]   = {0.0, 0.0, 0.0, 0.0};
	double			PWM[4] = {0.0, 0.0, 0.0, 0.0};
	double			AA[4][4] = {{ 1.0/(4.0*b),  1.0/(aa*b*l),  1.0/(aa*b*l), -1.0/(4.0*d) },
								{ 1.0/(4.0*b), -1.0/(aa*b*l),  1.0/(aa*b*l),  1.0/(4.0*d) },
								{ 1.0/(4.0*b), -1.0/(aa*b*l), -1.0/(aa*b*l), -1.0/(4.0*d) },
								{ 1.0/(4.0*b),  1.0/(aa*b*l), -1.0/(aa*b*l),  1.0/(4.0*d) }};
	double			Wr;

	printf("%s : Control démarré\n", __FUNCTION__);
	pthread_barrier_wait(&ControlStartBarrier);

	while (ControlActivated) {
		sem_wait(&ControlTimerSem);
		if (ControlActivated == 0)
			break;

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
		Error[YAW]       = -DataM.Yaw;
		DotError[HEIGHT] = SpeedD.Elevation - SpeedM.Elevation;
		DotError[ROLL]   = SpeedD.Roll - SpeedM.Roll;
		DotError[PITCH]  = SpeedD.Pitch - SpeedM.Pitch;
		DotError[YAW]    = DataD.Yaw - SpeedM.Yaw;

		U[0] = DataD.Elevation;
		Wr   = -W[0] + W[1] - W[2] + W[3];
		U[1] = (1.0/b1)*(-a1*SpeedM.Pitch*SpeedM.Yaw - a2*SpeedM.Pitch*Wr + ka1*Error[ROLL] + kb1*DotError[ROLL]);
		Wr   = W[0] - W[1] + W[2] - W[3];
		U[2] = (1.0/b2)*(-a3*SpeedM.Roll*SpeedM.Yaw - a4*SpeedM.Roll*Wr + ka2*Error[PITCH] + kb2*DotError[PITCH]);

		U[3] = 1.0*DotError[YAW];

		W[0] = AA[0][0]*U[0] + AA[0][1]*U[1] + AA[0][2]*U[2] + AA[0][3]*U[3];
		W[1] = AA[1][0]*U[0] + AA[1][1]*U[1] + AA[1][2]*U[2] + AA[1][3]*U[3];
		W[2] = AA[2][0]*U[0] + AA[2][1]*U[1] + AA[2][2]*U[2] + AA[2][3]*U[3];
		W[3] = AA[3][0]*U[0] + AA[3][1]*U[1] + AA[3][2]*U[2] + AA[3][3]*U[3];
		W[0] = (W[0] > 0.0) ? sqrt(W[0]) : 0.0;
		W[1] = (W[1] > 0.0) ? sqrt(W[1]) : 0.0;
		W[2] = (W[2] > 0.0) ? sqrt(W[2]) : 0.0;
		W[3] = (W[3] > 0.0) ? sqrt(W[3]) : 0.0;

		PWM[0] = WtoPWM_Gain*W[0] - WtoPWM_Offset;
		PWM[1] = WtoPWM_Gain*W[1] - WtoPWM_Offset;
		PWM[2] = WtoPWM_Gain*W[2] - WtoPWM_Offset;
		PWM[3] = WtoPWM_Gain*W[3] - WtoPWM_Offset;

		PWM[0] = (PWM[0] < DEADZONE) ? DEADZONE : PWM[0];
		PWM[1] = (PWM[1] < DEADZONE) ? DEADZONE : PWM[1];
		PWM[2] = (PWM[2] < DEADZONE) ? DEADZONE : PWM[2];
		PWM[3] = (PWM[3] < DEADZONE) ? DEADZONE : PWM[3];

		PWM[0] = (PWM[0] > 0.95) ? 0.95 : PWM[0];
		PWM[1] = (PWM[1] > 0.95) ? 0.95 : PWM[1];
		PWM[2] = (PWM[2] > 0.95) ? 0.95 : PWM[2];
		PWM[3] = (PWM[3] > 0.95) ? 0.95 : PWM[3];

		PWM[1] = PWM[2] = PWM[3] = PWM[0];

		pthread_spin_lock(&(Motor->MotorLock));
   	Motor->pwm[0] = (uint16_t)(PWM[0]*0x01ff);
   	Motor->pwm[1] = (uint16_t)(PWM[1]*0x01ff);
   	Motor->pwm[2] = (uint16_t)(PWM[2]*0x01ff);
   	Motor->pwm[3] = (uint16_t)(PWM[3]*0x01ff);
   	Motor->led[0] = (Motor->pwm[0] > 0.0) ? MOTOR_LEDGREEN : MOTOR_LEDRED;
   	Motor->led[1] = (Motor->pwm[1] > 0.0) ? MOTOR_LEDGREEN : MOTOR_LEDRED;
   	Motor->led[2] = (Motor->pwm[2] > 0.0) ? MOTOR_LEDGREEN : MOTOR_LEDRED;
   	Motor->led[3] = (Motor->pwm[3] > 0.0) ? MOTOR_LEDGREEN : MOTOR_LEDRED;
   	pthread_spin_unlock(&(Motor->MotorLock));
	}

   	printf("%s : Control Arrêté\n", __FUNCTION__);

	pthread_exit(0); /* exit thread */
}


int ControlInit (ControlStruct *Control) {
/* A faire! */
/* Ici, vous devriez initialiser le contrôleur du drone.      */
/* C'est-à-dire de faire les initialisations requises et      */
/* de créer la Tâche ControlTask() qui va faire calculer      */
/* les nouvelles vitesses des moteurs, basé sur l'attitude    */
/* désirée du drone et son attitude actuelle (voir capteurs). */
	int retval=0;
	pthread_attr_t		attr;
	struct sched_param	param;
	int					minprio, maxprio;

	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
	minprio = sched_get_priority_min(POLICY);
	maxprio = sched_get_priority_max(POLICY);
	pthread_attr_setschedpolicy(&attr, POLICY);
	param.sched_priority = minprio + (maxprio - minprio)/2;
	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);

	sem_init(&ControlTimerSem, 0, 0);

	retval = pthread_barrier_init(&ControlStartBarrier,NULL,2);
	retval = pthread_create(&(Control->ControlThread), &attr,ControlTask ,Control);


	return 0;
}


int ControlStart (void) {
/* A faire! */
/* Ici, vous devriez le travail du contrôleur (loi de commande) du drone. */ 
/* Les capteurs ainsi que tout le reste du système devrait être           */
/* prêt à faire leur travail et il ne reste plus qu'à tout démarrer.      */
	//return retval;
	int retval=0;
	ControlActivated=1;
	pthread_barrier_wait(&ControlStartBarrier);
	return retval;
}



int ControlStop (ControlStruct *Control) {
/* A faire! */
/* Ici, vous devriez arrêter le contrôleur du drone.    */ 
	ControlActivated=0;
	pthread_barrier_destroy(&ControlStartBarrier);
	pthread_join((Control->ControlThread), NULL);
	return 0;
}


