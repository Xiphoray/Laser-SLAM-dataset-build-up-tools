#include <string.h>
#include <iostream>
#include "low.h"



TSense sense;

void InitLowSlam(ArRobot *robot)
{
	int i, j;   //辅助增量
	odometry.lastsumtheta = 0;
	odometry.lastsumx= 0;
	odometry.lastsumy= 0;
	// All angle values will remain static 分配各传感器角度
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