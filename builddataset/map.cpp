#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "map.h"
#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;
unsigned char rawmap[MAP_WIDTH][MAP_HEIGHT];
unsigned char allmap[ALLMAP_WIDTH][ALLMAP_HEIGHT];
const char *csv_addr = "input\\input.csv";
extern int DataNumber ;
//
// Initializes the lowMap and the observationArray.
// Always returns 0 to indicate that it was successful.
//
void LowInitializeWorldMap()
{
	int x, y;

	for (y = 0; y < MAP_HEIGHT; y++)
		for (x = 0; x < MAP_WIDTH; x++) {
			// The map is a set of pointers. Null represents that it is unobserved.
			rawmap[x][y] = 1;
		}

	for (y = 0; y < ALLMAP_HEIGHT; y++)
		for (x = 0; x < ALLMAP_WIDTH; x++) {
			// The map is a set of pointers. Null represents that it is unobserved.
			allmap[x][y] = 1;
		}
}


void buildlowrawmap()
{
	int i;
	for (i = 0; i < SENSE_NUMBER; i++) {
		if (sense[i].distance < FarestRange && sense[i].distance > ClosetRange) {
			rawmap[(int)(-sin(sense[i].theta)*sense[i].distance) + (int)(MAP_WIDTH / 2)][(int)(MAP_HEIGHT / 2) - (int)(cos(sense[i].theta)*sense[i].distance)]= 0;
		}
	}
	imgflash = true;
}