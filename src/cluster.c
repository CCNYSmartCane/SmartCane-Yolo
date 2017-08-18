
#include "cluster.h"


//#define DEBUG_RECORD
//#define DEBUG_RECORD_FOLDER  /path/..

int FRAMEWINDOWSIZE;
frame* FRAMEWINDOW;
cluster* cluster_ptr;
int CONTINUOUSCNT = 0;
int max_class_id = 1; //the next class id to be assigned; should never be zero

void tracking_vars_init(int framewindow_size){
	set_framewindow_size(framewindow_size);
	create_framewindow();
}

//To be called after first detection
//Or if after zero detections are made (i.e. essentially a reset)
void framewindow_init(int len, box_new* box_input){
	frames_init(box_input, len);
	CONTINUOUSCNT = 0;
}

// 'demo' uses this tracking(), while 'test' uses cluster2()?
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

int mode(int local_index){
	int i,j,k,detectionNumber,tmp, maxCount = 0, modeVal = 0;
	int * class_counts;
	int newest_frame = FRAMEWINDOWSIZE-1;
	class_counts = calloc(max_class_id,  sizeof(int) ); //sets allocated memory to zero
	for(i = 0; i <  newest_frame; ++i){ //don't check last frame, class_id's have not been assigned
		detectionNumber = FRAMEWINDOW[i].numDetections;
		for(j = 0; j < detectionNumber; ++j){
			if(FRAMEWINDOW[i].detections[j].local_id == local_index){
				tmp = FRAMEWINDOW[i].detections[j].class_id;
				++class_counts[tmp];
			}
		}
	}

	for(i = 0; i < max_class_id; ++i){
		if(class_counts[i] > maxCount){
			maxCount = class_counts[i];
			modeVal = i;
		}
	}
	free(class_counts);
	//printf("maxcount: %d\n",maxCount);
	return modeVal;
}

bool clustering(FILE *fp, FILE *fp2, int cluster_num){

    int s = 0;
    int t = 0;
    int u = 0;
    int detection_counter;
    int detections_per_frame = 0;
    size_t total = 0;
    int newest_frame = FRAMEWINDOWSIZE-1;
    double numer_x = 0.0;
    double numer_y = 0.0;
    double numer_z = 0.0;
    double denom = 0.0;
    int cent_counter = 0;
    double distance_temp = 0.0;
    double distance_min = 99999.99;
    //find total number of detections to cluster
    for(s = 0; s < FRAMEWINDOWSIZE; ++s){
        total += (FRAMEWINDOW[s].numDetections);
    }

    int old_clust_indices[total]; // old set of cluster-indices for the previous cluster iteration
    old_clust_indices[0] = 1000; //need this to overcome zero initialization


    cluster_ptr = malloc((cluster_num)*sizeof(cluster));   //pointer for final centroids


    printf("intialized for clustering...\n");
    
    /*INITIAL GUESSES FOR CENTROIDS*/
    for(s = 0; s < cluster_num; ++s){
        cluster_ptr[s].x = FRAMEWINDOW[newest_frame].detections[s].x;
        cluster_ptr[s].y = FRAMEWINDOW[newest_frame].detections[s].y;
        cluster_ptr[s].z = 0; //FRAMEWINDOW[newest_frame].detections[s].depth;        

        printf("cluster %d: x: %f y: %f z: %f\n",s,cluster_ptr[s].x,cluster_ptr[s].y,cluster_ptr[s].z);
    }
    
    printf("created initial guesses for centroids...\n");

    size_t iters = 0;
    size_t flag = 0;
    
    // assign the cluster_id to the box_new

    while(1){
        detection_counter = 0;
        distance_temp = 0.0;
        int new_clust_indices[total]; //indices of centroids
        ++iters;
        //assignment step
        for(s = 0; s < FRAMEWINDOWSIZE; ++s){
            detections_per_frame = FRAMEWINDOW[s].numDetections;
            printf("number of detections per frame: %d\n", detections_per_frame);
            for(t = 0; t < detections_per_frame; ++t){
                distance_min = 99999.99;
                //printf("detection count in frame: %d\n", t);
                for(u = 0; u < cluster_num; ++u){
                    //printf("x coord:%f\n",detections[s].x);
                    distance_temp = (pow(FRAMEWINDOW[s].detections[t].x-cluster_ptr[u].x,2)+
                                     pow(FRAMEWINDOW[s].detections[t].y-cluster_ptr[u].y,2)); //ignore Z component for now
                    if(distance_temp<=distance_min){
                        //printf("assigning index value: %d\n", u);
                        new_clust_indices[detection_counter] = u;
                        FRAMEWINDOW[s].detections[t].local_id = u;
                        distance_min = distance_temp;
                    }
                }
                //printf("distance min: %f\n", distance_min);
                //printf("detection count: %d corresponding cluster: %d\n",detection_counter,new_clust_indices[detection_counter]);
                ++detection_counter;
            }    
            //printf("%d\nx:%f\ny:%f\n",d[s],detections[s].x,detections[s].y);
        }
        printf("assignment step complete...\n");  

        detection_counter = 0;


        /*UPDATE CENTROIDS TO MEANS OF EACH NEW CLUSTER*/
        for(s = 0; s < cluster_num; ++s){
            numer_x = 0.0;
            numer_y = 0.0;
            numer_z = 0.0;
            denom = 0.0;
            cent_counter = 0;
            //printf("Total detections: %d\n",total);
            for(t = 0; t < FRAMEWINDOWSIZE; ++t){
                detections_per_frame = FRAMEWINDOW[t].numDetections;
                for(u = 0; u < detections_per_frame; ++u){
                	// go through each box_new in the framewindow
                    if(new_clust_indices[detection_counter] == s){

                    	// save s to local_id. This local_id is used in the mode function
                    	FRAMEWINDOW[t].detections[u].local_id = s + 1;

                        // save x/y of detections for future calculation of obj func
                        numer_x += FRAMEWINDOW[t].detections[u].x;
                        numer_y += FRAMEWINDOW[t].detections[u].y;
                        numer_z += 0.0; // FRAMEWINDOW[t].detections[u].depth;
                        denom += 1.0;

                        fprintf(fp2,"%d\t%d\t%f\t%f\t%f\n",iters,s,(FRAMEWINDOW[t].detections[u].x),(FRAMEWINDOW[t].detections[u].y),0.0);

                        ++cent_counter;
                        if (0 ==  CONTINUOUSCNT) { //if this is the first time the algorithm is running
						// when initially, or reset from none of detection
						// in 1st, it's assigned by local_id
						FRAMEWINDOW[t].detections[u].class_id = FRAMEWINDOW[t].detections[u].local_id;
						}
                    }
                    //printf("Updated based on detection number %d\n", detection_counter);
                    ++detection_counter;
                }
            }


            detection_counter = 0;
            //printf("numerx: %f, numery: %f, denom: %f\n", numer_x, numer_y, denom);

            cluster_ptr[s].x = numer_x/denom;
            cluster_ptr[s].y = numer_y/denom;
            cluster_ptr[s].z = numer_z/denom;

            //printf("cluster %d: x: %f y: %f z: %f\n",s,cluster_ptr[s].x,cluster_ptr[s].y,cluster_ptr[s].z);

            /*FOR COMPARISON FUNCTION
            cluster_ptr[s].crsd_dets = malloc(cent_counter*sizeof(*box_new));
            cluster_ptr[s].detect_tracker = 0;*/

            fprintf(fp2,"\t%d\t%f\t%f\n",s,cluster_ptr[s].x,cluster_ptr[s].y); //might need to stick an extra new line here
        }
        printf("update step complete...\n");
        //printf("iteration %d completed...\n",iters);

        flag = 0;
        printf("flag set to 0\n");

        while(new_clust_indices[flag] == old_clust_indices[flag]){
            //printf("entered while loop\n");
            //printf("total-1 = %d\n", total-1);
            //printf("flag = %d\n",flag);
            if(flag == total-1){
                printf("entered flag == total-1 loop\n");

                for(s = 0; s < cluster_num; ++s){

                	if(CONTINUOUSCNT > 0){
						int tmp,local_index, a, new_frame_detects;
						local_index = s + 1;
						tmp = mode(local_index);
						printf("mode value: %d\nlocal index: %d\n",tmp,local_index);
						//go through newest frame
						new_frame_detects = FRAMEWINDOW[newest_frame].numDetections;
						for(a = 0; a < new_frame_detects; ++a){
							if(FRAMEWINDOW[newest_frame].detections[a].local_id == local_index){
								if(tmp == 0){
									cluster_ptr[s].class_id = max_class_id;
									FRAMEWINDOW[newest_frame].detections[a].class_id = max_class_id;
									max_class_id++;
								}
								else{
									cluster_ptr[s].class_id = tmp;
									FRAMEWINDOW[newest_frame].detections[a].class_id = tmp;
								}
							}
						}
					}
					else{
						cluster_ptr[s].class_id = max_class_id;
						max_class_id++;
					}


                    printf("total:%d\n",total);
                    printf("cluster number:%d\n",s+1);
                    printf("iterations to cluster:%d\n",iters);
                    printf("Class ID: %d\n",cluster_ptr[s].class_id);
                    printf("x:%f\n",cluster_ptr[s].x);
                    printf("y:%f\n",cluster_ptr[s].y);
                    printf("z:%f\n",cluster_ptr[s].z);
                    //save final centroids to txt file 
                    fprintf(fp,"\t%f\t%f\t%f\n",cluster_ptr[s].x,cluster_ptr[s].y,cluster_ptr[s].z);
                }
                fprintf(fp2,"done\n");
                CONTINUOUSCNT++;
                return true;
            }
            else{
                ++flag;
            }
        }
        printf("flag = %d old[flag] = %d new[flag] = %d\n",flag, old_clust_indices[flag],new_clust_indices[flag]);
        for(s = 0; s < total; ++s){
            old_clust_indices[s] = new_clust_indices[s];
        }
        /*for(t = 0; t < cluster_num; ++t){
            cluster_ptr[t].num_dets = 0;
            //free(cluster_ptr[t].crsd_dets);
        }*/
        fprintf(fp2,"\n");
    }
}

void convert_clusts_2_objs(object* objs, int detectionCount){
    int i;
    for(i = 0; i < detectionCount; ++i){
        objs[i].x = cluster_ptr[i].x;
        objs[i].y = cluster_ptr[i].y;
        objs[i].depth = cluster_ptr[i].z;
    }
    free(cluster_ptr);
    printf("cluster to object conversion completed\n");
}

// int FRAMEWINDOW_SIZE, which is initlized in tracking_init
// get rid of: int framewindow_size, 
void frame_switch(box_new* bn, int detectionCount){ //change to memory address copy
    int i;
    int boxes_mem_size;
    int newest_frame = FRAMEWINDOWSIZE-1;

    //free what the first frame points to
    free(FRAMEWINDOW[0].detections);

    //shift frames 2-10 to the left once
    for(i = 0; i < newest_frame; ++i){
        FRAMEWINDOW[i].detections = FRAMEWINDOW[i+1].detections;
        FRAMEWINDOW[i].numDetections = FRAMEWINDOW[i+1].numDetections;
    }

    boxes_mem_size = detectionCount * sizeof(box_new);
    //copy new detections into last frame in frame array
    FRAMEWINDOW[newest_frame].detections = malloc(boxes_mem_size);
    FRAMEWINDOW[newest_frame].numDetections = detectionCount;
    memcpy(FRAMEWINDOW[newest_frame].detections, bn, boxes_mem_size);    

}

void frames_init(box_new* bn, int detectionCount){
//Want to intialize each frame in frame array to its own copy of first detected frame
    int i;
    int boxes_mem_size;

    for(i = 0; i < FRAMEWINDOWSIZE; ++i){
        FRAMEWINDOW[i].numDetections = detectionCount;
        boxes_mem_size = detectionCount * sizeof(box_new);
        FRAMEWINDOW[i].detections = malloc(boxes_mem_size);
        memcpy(FRAMEWINDOW[i].detections, bn, boxes_mem_size);
    }

}
bool set_framewindow_size(int input_size){
    return (FRAMEWINDOWSIZE = input_size);
}

void create_framewindow(){
    FRAMEWINDOW = malloc(FRAMEWINDOWSIZE*sizeof(frame));
}
