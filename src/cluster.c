#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <string.h>
#include <math.h>
#include "darknet.h"
#include "cluster.h"
#include <stdlib.h>
#include <stdio.h>

void box_conversion(box* input, box_new*output, int counter){ //input output sizes should already be allocated
    int i;
    for(i = 0; i < counter; ++i){
        output[i].detect = input[i];
    }
}

centroid* cluster(framewindow input, FILE *fp, FILE *fp2){
    //box detections[845];
    int s = 0;
    int t = 0;
    size_t total = input.size1+input.size2+input.size3
                    +input.size4+input.size5+input.size6
                    +input.size7+input.size8+input.size9+input.size10;
    int d_old[total];
    d_old[0] = 1000; //need this to overcome zero initialization
    
    box_new* detections = malloc(total*sizeof(box_new));

    //double *centroids = malloc(2*input.size10*sizeof(double));           // [2*input.size3]; //double the orignal size (one for x, one for y)

    centroid *final_cents = malloc(input.size10*sizeof(centroid));   //pointer for final centroids


    printf("intialized for clustering...\n");

    memcpy(detections,input.frame1,sizeof(box_new)*input.size1);

    memcpy(&detections[input.size1],input.frame2,sizeof(box_new)*input.size2);
 
    memcpy(&detections[input.size1+input.size2],input.frame3,sizeof(box_new)*input.size3);

    memcpy(&detections[input.size1+input.size2+input.size3],input.frame4,sizeof(box_new)*input.size4);

    memcpy(&detections[input.size1+input.size2+input.size3+input.size4],input.frame5,sizeof(box_new)*input.size5);

    memcpy(&detections[input.size1+input.size2+input.size3+input.size4+input.size5],input.frame6,sizeof(box_new)*input.size6);

    memcpy(&detections[input.size1+input.size2+input.size3+input.size4+input.size5
                      +input.size6],input.frame7,sizeof(box_new)*input.size7);

    memcpy(&detections[input.size1+input.size2+input.size3+input.size4+input.size5
                      +input.size6+input.size7],input.frame8,sizeof(box_new)*input.size8);

    memcpy(&detections[input.size1+input.size2+input.size3+input.size4+input.size5
                      +input.size6+input.size7+input.size8],input.frame9,sizeof(box_new)*input.size9);

    memcpy(&detections[input.size1+input.size2+input.size3+input.size4+input.size5
                      +input.size6+input.size7+input.size8+input.size9],input.frame10,sizeof(box_new)*input.size10);

    printf("copied all dectections to detection buffer...\n");
    
    /*INITIAL GUESSES FOR CENTROIDS*/
    for(s = 0; s < input.size10; ++s){
        //centroids[2*s] = input.frame10[s].detect.x;
        //centroids[2*s+1] = input.frame10[s].detect.y;
        final_cents[s].x = input.frame10[s].detect.x;
        final_cents[s].y = input.frame10[s].detect.y;        

        //printf("init x:%f\n",centroids[2*i]);
         //printf("init y:%f\n",centroids[2*i+1]);
    }
    
    printf("created initial guesses for centroids...\n");

    size_t iters = 0;
    
    while(1){
        int d[total]; //indices of centroids
        double d_temp = 0.0;
        ++iters;
        //assignment step
        for(s = 0; s < total; ++s){
            double d_min = 99999.99;
            for(t = 0; t < input.size10; ++t){
                //printf("x coord:%f\n",detections[s].x);
                d_temp = (pow(detections[s].detect.x-final_cents[t].x,2)+
                              pow(detections[s].detect.y-final_cents[t].y,2));
                if(d_temp<d_min){
                    d[s] = t;
                    d_min = d_temp;
                }
            }
            //printf("%d\nx:%f\ny:%f\n",d[s],detections[s].x,detections[s].y);
        }
        printf("assignment step complete...\n");  

        /*UPDATE CENTROIDS TO MEANS OF EACH NEW CLUSTER*/
        for(s = 0; s < input.size10; ++s){
            //printf("size10: %d\n",input.size10);
            double numer_x = 0.0;
            double numer_y = 0.0;
            double denom = 0.0;
            int cent_counter = 0;
            //printf("Total detections: %d\n",total);
            for(t = 0; t < total; ++t){
                if(d[t] == s){
                    // save x/y of detections for future calculation of obj func
                    numer_x += detections[t].detect.x;
                    numer_y += detections[t].detect.y;
                    denom += 1.0;

                    fprintf(fp2,"%d\t%d\t%f\t%f\n",iters,s,detections[t].detect.x,detections[t].detect.y);

                    ++cent_counter;
                }
            }

            //printf("found all corresponding detections...\n");
           // save all centroids to txt file (1)
            //centroids[2*s] = numer_x/denom;
            //centroids[2*s+1] = numer_y/denom;
            final_cents[s].x = numer_x/denom;
            final_cents[s].y = numer_y/denom;

            final_cents[s].crsd_dets = malloc(cent_counter*sizeof(box_new));
            final_cents[s].detect_tracker = 0;

            //printf("x:%f\n",centroids[2*s]);
            //printf("y:%f\n",centroids[2*s+1]);
            fprintf(fp2,"\t%d\t%f\t%f\n",s,final_cents[s].x,final_cents[s].y); //might need to stick an extra new line here
        }
        printf("update step complete...\n");
        size_t flag = 0;

        while(d[flag] == d_old[flag]){

            memcpy(&final_cents[d[flag]].crsd_dets[final_cents[d[flag]].detect_tracker],&detections[flag],sizeof(box_new));
            ++final_cents[d[flag]].detect_tracker;

            if(flag == total-1){
                for(s = 0; s < input.size10; ++s){
                    printf("total:%d\n",total);
                    printf("cluster number:%d\n",s+1);
                    printf("iterations to cluster:%d\n",iters);
                    printf("x:%f\n",final_cents[s].x);
                    printf("y:%f\n",final_cents[s].y);
                    //save final centroids to txt file 
                    fprintf(fp,"\t%f\t%f\n",final_cents[s].x,final_cents[s].y);

                }
                fprintf(fp2,"done\n");
                free(detections);
                return (centroid*) final_cents;
            }
            else{
                ++flag;
            }
        }
        for(s = 0; s <total; ++s){
            d_old[s] = d[s];
        }
        for(t = 0; t < input.size10; ++t){
            free(final_cents[t].crsd_dets);
        }
        fprintf(fp2,"\n");
    }
}

void frame_switch(framewindow *input, box_new* bn, int size_frame_ten){
    free((*input).frame1);
    (*input).frame1 = malloc((*input).size2*sizeof(box_new));
    memcpy((*input).frame1,(*input).frame2,(*input).size2*sizeof(box_new));
    (*input).size1 = (*input).size2;
    
    free((*input).frame2);
    (*input).frame2 = malloc((*input).size3*sizeof(box_new));
    memcpy((*input).frame2,(*input).frame3,(*input).size3*sizeof(box_new));
    (*input).size2 = (*input).size3;

    free((*input).frame3);
    (*input).frame3 = malloc((*input).size4*sizeof(box_new));       
    memcpy((*input).frame3,(*input).frame4,(*input).size4*sizeof(box_new));
    (*input).size3 = (*input).size4;

    free((*input).frame4);
    (*input).frame4 = malloc((*input).size5*sizeof(box_new));
    memcpy((*input).frame4,(*input).frame5,(*input).size5*sizeof(box_new));
    (*input).size4 = (*input).size5;

    free((*input).frame5);
    (*input).frame5 = malloc((*input).size6*sizeof(box_new));
    memcpy((*input).frame5,(*input).frame6,(*input).size6*sizeof(box_new));
    (*input).size5 = (*input).size6;

    free((*input).frame6);
    (*input).frame6 = malloc((*input).size7*sizeof(box_new));
    memcpy((*input).frame6,(*input).frame7,(*input).size7*sizeof(box_new));
    (*input).size6 = (*input).size7;

    free((*input).frame7);
    (*input).frame7 = malloc((*input).size8*sizeof(box_new));
    memcpy((*input).frame7,(*input).frame8,(*input).size8*sizeof(box_new));
    (*input).size7 = (*input).size8;

    free((*input).frame8);
    (*input).frame8 = malloc((*input).size9*sizeof(box_new));
    memcpy((*input).frame8,(*input).frame9,(*input).size9*sizeof(box_new));
    (*input).size8 = (*input).size9;

    free((*input).frame9);
    (*input).frame9 = malloc((*input).size10*sizeof(box_new));
    memcpy((*input).frame9,(*input).frame10,(*input).size10*sizeof(box_new));
    (*input).size9 = (*input).size10;

    free((*input).frame10);
    (*input).size10 = size_frame_ten;
    (*input).frame10 = malloc(size_frame_ten*sizeof(box_new));
    memcpy((*input).frame10,bn,size_frame_ten*sizeof(box_new));    

}

void frames_init(framewindow *input, box_new* bn, int frame_size_ten){

    int i;

    (*input).size10 = frame_size_ten;      
    (*input).frame10 = malloc((*input).size10*sizeof(box_new)); //if bs changes, then so does frame10
    memcpy((*input).frame10,bn,frame_size_ten*sizeof(box_new));

    //init box_ids
    for(i = 0; i < frame_size_ten; ++i){
        //(*input).frame10[i].box_id[0] = (char)(i);
        //(*input).frame10[i].box_id[1] = ("%c",i);
        sprintf((*input).frame10[i].box_id,"%c%d",(char)i,i);
    }


    (*input).frame9 = malloc((*input).size10*sizeof(box_new));
    memcpy((*input).frame9,(*input).frame10,(*input).size10*sizeof(box_new));
    (*input).size9 = (*input).size10;

    (*input).frame8 = malloc((*input).size10*sizeof(box_new));
    memcpy((*input).frame8,(*input).frame9,(*input).size9*sizeof(box_new));
    (*input).size8 = (*input).size9;

    (*input).frame7 = malloc((*input).size10*sizeof(box_new));
    memcpy((*input).frame7,(*input).frame8,(*input).size8*sizeof(box_new));
    (*input).size7 = (*input).size8;

    (*input).frame6 = malloc((*input).size10*sizeof(box_new));
    memcpy((*input).frame6,(*input).frame7,(*input).size7*sizeof(box_new));
    (*input).size6 = (*input).size7;

    (*input).frame5 = malloc((*input).size10*sizeof(box_new));
    memcpy((*input).frame5,(*input).frame6,(*input).size6*sizeof(box_new));
    (*input).size5 = (*input).size6;

    (*input).frame4 = malloc((*input).size10*sizeof(box_new));
    memcpy((*input).frame4,(*input).frame5,(*input).size5*sizeof(box_new));
    (*input).size4 = (*input).size5;

    (*input).frame3 = malloc((*input).size10*sizeof(box_new));
    memcpy((*input).frame3,(*input).frame4,(*input).size4*sizeof(box_new));
    (*input).size3 = (*input).size4;

    (*input).frame2 = malloc((*input).size10*sizeof(box_new));
    memcpy((*input).frame2,(*input).frame3,(*input).size3*sizeof(box_new));
    (*input).size2 = (*input).size3;

    (*input).frame1 = malloc((*input).size10*sizeof(box_new));
    memcpy((*input).frame1,(*input).frame2,(*input).size2*sizeof(box_new));
    (*input).size1 = (*input).size2;
}
