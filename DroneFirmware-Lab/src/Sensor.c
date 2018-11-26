/*
 * Sensor.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */

#include "Sensor.h"

#define ABS(x) (((x) < 0.0) ? -(x) : (x))


#define MAX_TOT_SAMPLE 100

extern SensorStruct	SensorTab[NUM_SENSOR];

pthread_barrier_t   SensorStartBarrier;
pthread_barrier_t   LogStartBarrier;
pthread_mutex_t 	  Log_Mutex;

uint8_t  SensorsActivated 	= 0;
uint8_t  LogActivated  	  	= 0;
uint8_t  numLogOutput 	  	= 0;


void *SensorTask ( void *ptr ) {
/* A faire! */
/* Tache qui sera instancié pour chaque sensor. Elle s'occupe d'aller */
/* chercher les donnees du sensor.                                    */
	int i = 0;
	int j = 0;
	int k = 0;
	SensorStruct * ptr2=(SensorStruct *)ptr;
	SensorRawData LocalRawData;
	SensorRawData PrevRawData;
	SensorData LocalData;
	SensorData LocalDataCorriger;
	double Naughtydata[3];
	double sample[3][100];
	int LocalIdx=0;
	int File=ptr2->File;

	printf("%s pthread_barrier_wait !!!\n", __FUNCTION__);
	pthread_barrier_wait(&SensorStartBarrier);
	while (SensorsActivated) {

		if(read(File,&LocalRawData,sizeof(LocalRawData))==sizeof(LocalRawData)){
			if(LocalRawData.status == NEW_SAMPLE){

				for(i=0;i<3;i++){
					LocalData.Data[i] = ((double)(((LocalRawData.data[i])-(ptr2->Param->centerVal))*(ptr2->Param->Conversion)));
				}
			/*	for(i=0;i<3;i++){
					LocalDataCorriger.Data[i] = (LocalData.Data[0])*(ptr2->Param->alpha[i][0])+(LocalData.Data[1])*(ptr2->Param->alpha[i][1])+(LocalData.Data[2])*(ptr2->Param->alpha[i][2])+(ptr2->Param->beta[i]);
				}
*/
				LocalData.TimeDelay = (uint32_t) (((LocalRawData.timestamp_s * 1000000000)+(LocalRawData.timestamp_n))-(((PrevRawData.timestamp_s * 1000000000))+PrevRawData.timestamp_n));
			}
			else if(LocalRawData.status == OLD_SAMPLE){
				printf("%s that's some old stuff son! ",  __FUNCTION__);
			}
			else {
				printf("%s you done messed up A-Aron !!!\n", __FUNCTION__);
				printf("%s LocalRawData.status=%d\n", __FUNCTION__,LocalRawData.status);
			}
			/*if(ptr2->type == GYROSCOPE){
				//fetch data
				for (i=0; i<3 ; i++){
					sample[i][j]=LocalData.Data[i];
				}

				if(j == 99){
					//calcul de moyenne
					for(k=0;k<100;k++){
						for(i=0;i<3;i++){
							Naughtydata[i]+=sample[i][k];
						}
					}
					//affichage de moyenne
					for (i=0; i<3 ; i++){
						printf("%s LocalMeanData indice=%d valeur=%lf\n", __FUNCTION__,i,Naughtydata[i]/100.0);
						Naughtydata[i]=0;
					}
				}
				j=(j+1)%100;
			}*/

			pthread_mutex_lock(&(ptr2->DataLockMutex));
			ptr2->DataIdx=(ptr2->DataIdx+1)%MAX_TOT_SAMPLE;
			LocalIdx = (int)(ptr2->DataIdx);
			pthread_mutex_unlock(&(ptr2->DataLockMutex));
			pthread_mutex_lock(&(ptr2->DataSampleMutex));

			memcpy((void *) &(ptr2->Data[LocalIdx]), (void *) &LocalData, sizeof(SensorData));
			memcpy((void *) &(ptr2->RawData[LocalIdx]), (void *) &LocalRawData, sizeof(SensorRawData));

			pthread_cond_broadcast(&(ptr2->DataNewSampleCondVar));
			pthread_mutex_unlock(&(ptr2->DataSampleMutex));

			memcpy((void *) &PrevRawData, (void *) &LocalRawData, sizeof(SensorRawData));
		}
		else {
			//La structure n'a pas ete copier en entier
		}



	}
	printf("%s EXIT:! %s\n",  __FUNCTION__,ptr2->Name);
	pthread_exit(0); /* exit thread */
}


int SensorsInit (SensorStruct SensorTab[NUM_SENSOR]) {
/* A faire! */
/* Ici, vous devriez faire l'initialisation de chacun des capteurs.  */ 
/* C'est-à-dire de faire les initialisations requises, telles que    */
/* ouvrir les fichiers des capteurs, et de créer les Tâches qui vont */
/* s'occuper de réceptionner les échantillons des capteurs.          */
	int retval=0;
	int i =0;
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

	retval = pthread_barrier_init(&SensorStartBarrier,NULL,NUM_SENSOR+1);
	for(i=0;i<NUM_SENSOR;i++){
		SensorTab[i].File=open(SensorTab[i].DevName,O_RDONLY);
		retval = pthread_mutex_init(&(SensorTab[i].DataLockMutex),NULL);
		retval = pthread_mutex_init(&(SensorTab[i].DataSampleMutex),NULL);
		retval = pthread_cond_init(&(SensorTab[i].DataNewSampleCondVar),NULL);
		retval = pthread_create(&(SensorTab[i].SensorThread), &attr, SensorTask, &SensorTab[i]);
	}
	pthread_attr_destroy(&attr);
	printf("%s ça démarre !!!\n", __FUNCTION__);

	return 0;
};


int SensorsStart (void) {
/* A faire! */
/* Ici, vous devriez démarrer l'acquisition sur les capteurs.        */ 
/* Les capteurs ainsi que tout le reste du système devrait être      */
/* prêt à faire leur travail et il ne reste plus qu'à tout démarrer. */
	SensorsActivated=1;
	printf("%s pthread_barrier_wait !!!\n", __FUNCTION__);
	pthread_barrier_wait(&SensorStartBarrier);
	printf("%s 2 ça démarre !!!\n", __FUNCTION__);

	return 0;
}


int SensorsStop (SensorStruct SensorTab[NUM_SENSOR]) {
/* A faire! */
/* Ici, vous devriez défaire ce que vous avez fait comme travail dans */
/* SensorsInit() (toujours verifier les retours de chaque call)...    */ 
	int i =0;
	SensorsActivated=0;
	pthread_barrier_destroy(&SensorStartBarrier);
	printf("%s pthread_gon_stawp !!!\n", __FUNCTION__);
	for(i=0;i<NUM_SENSOR;i++){
			pthread_join(SensorTab[i].SensorThread, NULL);
			pthread_mutex_destroy(&(SensorTab[i].DataLockMutex));
			pthread_mutex_destroy(&(SensorTab[i].DataSampleMutex));
			pthread_cond_destroy(&(SensorTab[i].DataNewSampleCondVar));
			close(SensorTab[i].File);
	}
	printf("%s arreter\n", __FUNCTION__);
	return 0;
}



/* Le code ci-dessous est un CADEAU !!!	*/
/* Ce code permet d'afficher dans la console les valeurs reçues des capteurs.               */
/* Évidemment, celà suppose que les acquisitions sur les capteurs fonctionnent correctement. */
/* Donc, dans un premier temps, ce code peut vous servir d'exemple ou de source d'idées.     */
/* Et dans un deuxième temps, peut vous servir pour valider ou vérifier vos acquisitions.    */
/*                                                                                           */
/* NOTE : Ce code suppose que les échantillons des capteurs sont placés dans un tampon       */
/*        circulaire de taille DATABUFSIZE, tant pour les données brutes (RawData) que       */
/*        les données converties (NavData) (voir ci-dessous)                                 */
void *SensorLogTask ( void *ptr ) {
	SensorStruct	*Sensor    = (SensorStruct *) ptr;
	uint16_t		*Idx       = &(Sensor->DataIdx);
	uint16_t		LocalIdx   = DATABUFSIZE;
	SensorData	 	*NavData   = NULL;
	SensorRawData	*RawData   = NULL;
	SensorRawData   tpRaw;
	SensorData 	    tpNav;
	double			norm;

	printf("%s : Log de %s prêt à démarrer\n", __FUNCTION__, Sensor->Name);
	pthread_barrier_wait(&(LogStartBarrier));

	while (LogActivated) {
		pthread_mutex_lock(&(Sensor->DataSampleMutex));
		while (LocalIdx == *Idx)
			pthread_cond_wait(&(Sensor->DataNewSampleCondVar), &(Sensor->DataSampleMutex));
	    pthread_mutex_unlock(&(Sensor->DataSampleMutex));

	    pthread_mutex_lock(&(Sensor->DataLockMutex));
    	NavData   = &(Sensor->Data[LocalIdx]);
    	RawData   = &(Sensor->RawData[LocalIdx]);
		memcpy((void *) &tpRaw, (void *) RawData, sizeof(SensorRawData));
		memcpy((void *) &tpNav, (void *) NavData, sizeof(SensorData));
		pthread_mutex_unlock(&(Sensor->DataLockMutex));

	   	pthread_mutex_lock(&Log_Mutex);
		if (numLogOutput == 0)
			printf("Sensor  :     TimeStamp      SampleDelay  Status  SampleNum   Raw Sample Data  =>        Converted Sample Data               Norme\n");
		else switch (tpRaw.type) {
				case ACCELEROMETRE :	norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Accel   : (%5u.%09d)-(  0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
				case GYROSCOPE :		norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Gyro    : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
				case SONAR :			printf("Sonar   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X              =>  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], tpNav.Data[0]);
										break;
				case BAROMETRE :		printf("Barom   : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X              =>  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], tpNav.Data[0]);
										break;
				case MAGNETOMETRE :		norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Magneto : (%5u.%09d)-(0.%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp_s, tpRaw.timestamp_n, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
			 }
		LocalIdx = *Idx;
		numLogOutput++;
		if (numLogOutput > 20)
			numLogOutput = 0;
		pthread_mutex_unlock(&Log_Mutex);
	}

	printf("%s : %s Terminé\n", __FUNCTION__, Sensor->Name);

	pthread_exit(0); /* exit thread */
}


int InitSensorLog (SensorStruct *Sensor) {
	pthread_attr_t		attr;
	struct sched_param	param;
	int					retval;
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

	printf("Creating Log thread : %s\n", Sensor->Name);
	if ((retval = pthread_create(&(Sensor->LogThread), &attr, SensorLogTask, (void *) Sensor)) != 0)
		printf("%s : Impossible de créer Tâche Log de %s => retval = %d\n", __FUNCTION__, Sensor->Name, retval);


	pthread_attr_destroy(&attr);

	return 0;
}


int SensorsLogsInit (SensorStruct SensorTab[]) {
	int16_t	  i, numLog = 0;
	int16_t	  retval = 0;
	printf("%s 1 ça démarre !!!\n", __FUNCTION__);
	for (i = 0; i < NUM_SENSOR; i++) {
		if (SensorTab[i].DoLog == 1) {
			if ((retval = InitSensorLog(&SensorTab[i])) < 0) {
				printf("%s : Impossible d'initialiser log de %s => retval = %d\n", __FUNCTION__, SensorTab[i].Name, retval);
				return -1;
			}
			numLog++;
		}
	}
	pthread_barrier_init(&LogStartBarrier, NULL, numLog+1);
	pthread_mutex_init(&Log_Mutex, NULL);
	printf("%s 2 ça démarre !!!\n", __FUNCTION__);
	return 0;
};


int SensorsLogsStart (void) {
	LogActivated = 1;
	pthread_barrier_wait(&(LogStartBarrier));
	pthread_barrier_destroy(&LogStartBarrier);
	printf("%s NavLog démarré\n", __FUNCTION__);

	return 0;
};


int SensorsLogsStop (SensorStruct SensorTab[]) {
	int16_t	i;

	LogActivated = 0;
	for (i = 0; i < NUM_SENSOR; i++) {
		if (SensorTab[i].DoLog == 1) {
			pthread_join(SensorTab[i].LogThread, NULL);
			SensorTab[i].DoLog = 0;
		}
	}
	pthread_mutex_destroy(&Log_Mutex);
	printf("%s arreter\n", __FUNCTION__);

	return 0;
};


