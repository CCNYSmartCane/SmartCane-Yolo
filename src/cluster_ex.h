#ifndef CLUSTER_EX_H
#define CLUSTER_EX_H

#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

typedef struct{
	// todo: change the type of those variable which can be float, no need for double
    double x;
    double y;
    double width;
    double height;

    double depth;

    int cluster_id;  // boxes in frame:1-9, the values are assigned after clustering, for frame 10 is 0xFFFFFFFF
    double timestamp;
    int local_id;  // e.g for a local frame, we got 3 boxes, assign them as 0, 1, 2 respectively
    int class_id;  // in 1st, it's assigned by local_id
                   // then, it's assigned by the mean of it's belonging cluster
}box_new;

// as tracking output from JNI call
typedef struct{
    double x; //correspond to final centroid values
    double y;
    double depth;
    double width;
    double height;
    int class_type; // like: person, cup, ...
    int class_id;   // like: 0, 1, 2, ... (as a continuous & unique id)
    int cluster_id; // diff with in box_new?
}object;

// are we able to use structure for JNI?
// since point is different in Java and C

// only JNI calls
void tracking_vars_init(int framewindow_size); //intialize global variable: int FRAMEWINDOWSIZE

// framewindow_init() should be internal?
// void framewindow_init(int len, box_new* box_input);

// todo: write a description of parameter for the function
//bool tracking(char* folder, int len, box_new* box_input, object* obs); // obs is output, length of obs[] is len
bool tracking(int len, box_new* box_input, object* obs, FILE *fp, FILE *fp2);
// tracking()
//      frame* framewindow = malloc(10*sizeof(frame));
//      frames_init
//      frame_switch
//      cluster
//      corresponding labelling

#endif
