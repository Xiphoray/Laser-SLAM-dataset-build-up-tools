#pragma once
#include "ThisRobot.h"
// We need to know, for various purposes, how big our map is allowed to be
#define MAP_WIDTH  250
#define MAP_HEIGHT 250

#define ALLMAP_WIDTH  600
#define ALLMAP_HEIGHT 600

#define SWPARATOR_BIG ','
#define SWPARATOR_SMALL ';'

extern unsigned char rawmap[MAP_WIDTH][MAP_HEIGHT];
extern unsigned char allmap[ALLMAP_WIDTH][ALLMAP_HEIGHT];
extern bool imgflash;
void LowInitializeWorldMap();
void buildlowrawmap();