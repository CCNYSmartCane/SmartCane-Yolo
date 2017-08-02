#ifndef CLUSTER_H
#define CLUSTER_H

#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <string.h>
#include <math.h>
#include "darknet.h"
#include <stdbool.h>

typedef struct{
    double x;
    double y;
    double width;
    double height;

    double depth;

    int cluster_id;  // boxes in frame:1-9, the values are assigned after clustering, for frame 10 is 0xFFFFFFFF
    double timestamp;
    int local_id;  // e.g for a local frame, we got 3 boxes, assign them as 0, 1, 2 respectively
}box_new;

// as tracking output from JNI call
typedef struct{
    double x; //correspond to final centroid values
    double y;
    double depth;
    double width;
    double height;
    int class_type;
    int cluster_id;
}object;


bool clustering(FILE *fp, FILE *fp2, int cluster_num); // return: cluster[] with size
void frame_switch(box_new* bn, int detectionCount);
void frames_init(box_new* bn, int detectionCount);
bool set_framewindow_size(int input_size);
void convert_clusts_2_objs(object* objs, int detectionCount);
void create_framewindow();


#endif
