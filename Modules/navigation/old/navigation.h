#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <stdbool.h>
#include <float.h>
#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include "lauxlib.h"
#include <unistd.h>
#include <set>

#define PI 3.14159265
#define DEG_TO_RAD (PI/180.0)



extern int8_t smap[2048*2048];
extern int smap_x, smap_y;
extern float smap_x0,smap_y0,smap_z0,smap_res;
extern int lomap_size;
extern float lomap_res, lomap_x0, lomap_y0;

extern int8_t lomap[201*201];
extern int8_t temp_lomap[201*201];
extern int8_t dirmap[201*201];
extern float lomapv[201*201]; //value map



extern float *xpath, *ypath, *xpaths, *ypaths;
extern int path_num,path_num_s;



void init_lomap();
void downsample_smap();
void downsample_smap2(); //for turtlebot!!
void update_valuemap(float xpose,float ypose, float apose, int maxdepth);
int pathplan(float xpos, float ypos, float xtarget, float ytarget, int enhance);

#endif
