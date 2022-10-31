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
// #define LOMAP_SIZE 201//30 by 30 meter with 15cm resolution
#define LOMAP_SIZE 601//30 by 30 meter with 5cm resolution

extern int8_t smap[2048*2048];
extern int smap_x, smap_y;
extern float smap_x0,smap_y0,smap_z0,smap_res;
extern int lomap_size;
extern float lomap_res, lomap_x0, lomap_y0;

extern int8_t lomap[LOMAP_SIZE*LOMAP_SIZE ];
extern int8_t temp_lomap[LOMAP_SIZE *LOMAP_SIZE ];
extern int8_t dirmap[LOMAP_SIZE*LOMAP_SIZE ];
extern float lomapv[LOMAP_SIZE*LOMAP_SIZE ]; //value map



extern float *xpath, *ypath, *xpaths, *ypaths;
extern int path_num,path_num_s;

int get_lomap_index(float xpos, float ypos,int *xindex, int *yindex);

void init_lomap();
void downsample_smap(int downsample_factor,int increase_factor);
void downsample_smap2(); //for turtlebot!!
void update_valuemap(float xpose,float ypose, float apose, float max_c);
int pathplan(float xpos, float ypos, float xtarget, float ytarget, int enhance);
void add_obstacle(float x, float y);
void add_obstacle_direct(float x, float y,float increase_factor);
int check_empty_space(float x, float y, float radius);

#endif
