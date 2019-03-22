#include <string.h>
#include <iostream>
#include "low.h"



TSense sense;

void InitLowSlam(ArRobot *robot)
{
	int i, j;   //��������
	odometry.lastsumtheta = 0;
	odometry.lastsumx= 0;
	odometry.lastsumy= 0;
	// All angle values will remain static ������������Ƕ�
	for (i = 0; i < SENSE_NUMBER; i++)
		sense[i].theta = ((double)i*M_PI / 360.0) - M_PI * 3.0 / 4.0;

}

void LowSlam(ArRobot *robot)
{
	
	LowInitializeWorldMap();
	GetSensation(odometry, sense, &robot);

	GetOdometry(odometry, &robot);
	buildlowrawmap();
	
}