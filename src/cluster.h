#ifndef CLUSTER_H
#define CLUSTER_H

#include "cluster_ex.h"
#include "darknet.h"

typedef struct{
    box_new* detections;
    size_t numDetections;
}frame;

typedef struct{
    box_new *crsd_dets; //a way to do comparison func without??
    size_t num_dets;
    box_new newest_dets;
    double x;
    double y;
    double z;
    size_t detect_tracker; // can we do without?
    int cluster_id;
}cluster;


bool clustering(FILE *fp, FILE *fp2, int cluster_num); // return: cluster[] with size
void frame_switch(box_new* bn, int detectionCount);
void frames_init(box_new* bn, int detectionCount);
bool set_framewindow_size(int input_size);
void convert_clusts_2_objs(object* objs, int detectionCount);
void create_framewindow();


#endif
