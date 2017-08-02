#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>
#include <math.h>
#include "darknet.h"
#include "cluster.h"

void tracking_vars_init(int framewindow_size){
	set_framewindow_size(framewindow_size);
	create_framewindow();
}

//To be called after first detection
//Or if after zero detections are made (i.e. essentially a reset)
void framewindow_init(int len, box_new* box_input){
	frames_init(box_input, len);
}

//To be called for all other subsequent detections
bool tracking(int len, box_new* box_input, object* obs, FILE *fp, FILE *fp2){
	bool flag = false;
	frame_switch(box_input, len);
	clustering(fp, fp2, len);
	//objs_init(obs,len);
	convert_clusts_2_objs(obs,len); //objs are update and can now be visualized
	flag = true;
	return flag;
}
