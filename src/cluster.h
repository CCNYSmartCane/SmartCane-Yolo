#ifndef CLUSTER_H
#define CLUSTER_H

#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <string.h>
#include <math.h>
#include "darknet.h"
#include <stdlib.h>
#include <stdio.h>

typedef struct{
    box detect;
    char box_id[2];   //alpha-numeric id (e.g. A0 - Z9)
}box_new;

typedef struct{
    box_new* frame1;
    size_t size1;

    box_new* frame2;
    size_t size2;

    box_new* frame3;
    size_t size3;
    
    box_new* frame4;
    size_t size4;

    box_new* frame5;
    size_t size5;
    
    box_new* frame6;
    size_t size6;

    box_new* frame7;
    size_t size7;
    
    box_new* frame8;
    size_t size8;

    box_new* frame9;
    size_t size9;

    box_new* frame10;
    size_t size10;

    size_t init; 
}framewindow;

typedef struct{
    box_new *crsd_dets;
    double x;
    double y;
    size_t detect_tracker;
} centroid;

centroid* cluster(framewindow input, FILE *fp, FILE *fp2);
void frame_switch(framewindow *input, box_new* bn, int frame_size_ten);
void frames_init(framewindow *input, box_new* bn, int frame_size_ten);
void box_conversion(box* input, box_new* output, int counter);

#endif
