#ifndef CLUSTER_EX_H
#define CLUSTER_EX_H

#include "cluster.h"


// only JNI calls
void tracking_vars_init(int framewindow_size); //intialize global variable: int FRAMEWINDOWSIZE

void framewindow_init(int len, box_new* box_input);

//bool tracking(char* folder, int len, box_new* box_input, object* obs); // obs is output, length of obs[] is len
bool tracking(int len, box_new* box_input, object* obs, FILE *fp, FILE *fp2);
// tracking()
//      frame* framewindow = malloc(10*sizeof(frame));
//      frames_init
//      frame_switch
//      cluster
//      corresponding labelling

#endif