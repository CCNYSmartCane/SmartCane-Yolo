#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "demo.h"
#include "string.h"
#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#define DEMO 1

#ifdef OPENCV

typedef struct{
    box* frame1;
    size_t size1;

    box* frame2;
    size_t size2;

    box* frame3;
    size_t size3;
    
    box* frame4;
    size_t size4;

    box* frame5;
    size_t size5;
    
    box* frame6;
    size_t size6;

    box* frame7;
    size_t size7;
    
    box* frame8;
    size_t size8;

    box* frame9;
    size_t size9;

    box* frame10;
    size_t size10;

    size_t init; 
}framewindow;

typedef struct{
    double x;
    double y;
    char name;
}centroid;

static char **demo_names;
static image **demo_alphabet;
static int demo_classes;

static float **probs;
static box *boxes;
static network net;
static image buff [3];
static image buff_letter[3];
static int buff_index = 0;
static CvCapture * cap;
static IplImage  * ipl;

static float fps = 0;
static float demo_thresh = 0;
static float demo_hier = .5;
static int running = 0;

static int demo_delay = 0;
static int demo_frame = 3;
static int demo_detections = 0;
static float **predictions;
static int demo_index = 0;
static int demo_done = 0;
static float *last_avg2;
static float *last_avg;
static float *avg;
double demo_time;
static framewindow fw;
box bs[100];
box *temp;
double *out;
double *out_old;
centroid *old_cents;
centroid *new_cents;
int cent_length;
FILE *fp;
FILE *fp2;
FILE *fpa;
FILE *fpb;
int count;
double cent_buff[200];
int m_cent = 0;
int n_cent;
int sx;
int sy;
char* cluster_names;
char max_name = 'a';

void compare_centroids(centroid* cents_a, size_t size_a, centroid* cents_b, size_t size_b){
    int k;
    int l;
    
    double c_min = 0.012;
    double c_tmp;

    for(k = 0; k < size_a; ++k){    
        c_min = 0.012;
        for(l = 0; l < size_b; ++l){
            c_tmp = sqrt(pow(cents_a[k].x-cents_b[l].x,2)+
                         pow(cents_a[k].y-cents_b[l].y,2));       
            if(c_tmp <= 0.012){
              c_min = c_tmp;  
              cents_a[k].name = cents_b[l].name; 
            }
        }
        if(c_min == 0.012){
            cents_a[k].name = max_name;
            max_name++;
        }
        printf("cluster %c is shown\n",cents_a[k].name);      

    }
    return;
}

double* cluster(framewindow input, FILE *fp, FILE *fp2){
    box detections[845];
    int s = 0;
    int t = 0;
    size_t total = input.size1+input.size2+input.size3
                    +input.size4+input.size5+input.size6
                    +input.size7+input.size8+input.size9+input.size10;
    int d_old[total];
    d_old[0] = 1000; //need this to overcome zero initialization
    
    

    double *centroids = malloc(2*input.size10*sizeof(double));           // [2*input.size3]; //double the orignal size (one for x, one for y)
    //out = malloc(2*input.size10*sizeof(double));

    printf("intialized for clustering...\n");

    memcpy(detections,input.frame1,sizeof(box)*input.size1);

    memcpy(&detections[input.size1],input.frame2,sizeof(box)*input.size2);
 
    memcpy(&detections[input.size1+input.size2],input.frame3,sizeof(box)*input.size3);

    memcpy(&detections[input.size1+input.size2+input.size3],input.frame4,sizeof(box)*input.size4);

    memcpy(&detections[input.size1+input.size2+input.size3+input.size4],input.frame5,sizeof(box)*input.size5);

    memcpy(&detections[input.size1+input.size2+input.size3+input.size4+input.size5],input.frame6,sizeof(box)*input.size6);

    memcpy(&detections[input.size1+input.size2+input.size3+input.size4+input.size5
                      +input.size6],input.frame7,sizeof(box)*input.size7);

    memcpy(&detections[input.size1+input.size2+input.size3+input.size4+input.size5
                      +input.size6+input.size7],input.frame8,sizeof(box)*input.size8);

    memcpy(&detections[input.size1+input.size2+input.size3+input.size4+input.size5
                      +input.size6+input.size7+input.size8],input.frame9,sizeof(box)*input.size9);

    memcpy(&detections[input.size1+input.size2+input.size3+input.size4+input.size5
                      +input.size6+input.size7+input.size8+input.size9],input.frame10,sizeof(box)*input.size10);

    printf("copied all dectections to detection buffer...\n");
    
    /*INITIAL GUESSES FOR CENTROIDS*/
    for(s = 0; s < input.size10; ++s){
        centroids[2*s] = input.frame10[s].x;
        centroids[2*s+1] = input.frame10[s].y;
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
                d_temp = (pow(detections[s].x-centroids[2*t],2)+
                              pow(detections[s].y-centroids[2*t+1],2));
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
            //printf("Total detections: %d\n",total);
            for(t = 0; t < total; ++t){
                if(d[t] == s){
                    // save x/y of detections for future calculation of obj func
                    numer_x += detections[t].x;
                    numer_y += detections[t].y;
                    denom += 1.0;
                    //printf("writing to file...\n");
                    fprintf(fp2,"%d\t%d\t%f\t%f\n",iters,s,detections[t].x,detections[t].y);
                }
            }
            //printf("found all corresponding detections...\n");
           // save all centroids to txt file (1)
            centroids[2*s] = numer_x/denom;
            centroids[2*s+1] = numer_y/denom;
            //printf("x:%f\n",centroids[2*s]);
            //printf("y:%f\n",centroids[2*s+1]);
            fprintf(fp2,"\t%d\t%f\t%f\n",s,centroids[2*s],centroids[2*s+1]); //might need to stick an extra new line here
        }
        printf("update step complete...\n");
        size_t flag = 0;

        while(d[flag] == d_old[flag]){
            if(flag == total-1){
                new_cents = malloc(input.size10*24); //alloc space for new_cents pointer

                for(s = 0; s < input.size10; ++s){
                    printf("total:%d\n",total);
                    printf("cluster number:%d\n",s+1);
                    printf("iterations to cluster:%d\n",iters);
                    printf("x:%f\n",centroids[2*s]);
                    printf("y:%f\n",centroids[2*s+1]);
                    //save final centroids to txt file 
                    fprintf(fp,"%d\t%f\t%f\n",count,centroids[2*s],centroids[2*s+1]);

                    //save final centroids to new_cents pointer
                    new_cents[s].x = centroids[2*s];
                    new_cents[s].y = centroids[2*s+1];
                }
                    fprintf(fp2,"done\n");
          
                return (double *)centroids;
            }
            else{
                ++flag;
            }
        }
        for(s = 0; s <total; ++s){
            d_old[s] = d[s];
        }
        fprintf(fp2,"\n");
    }
    
}

double get_wall_time()
{
    struct timeval time;
    if (gettimeofday(&time,NULL)){
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

void *detect_in_thread(void *ptr)
{
    int detects = 0;
    int i = 0;
    sx = 0;
    sy = 0;

    //fw.length++;
    running = 1;
    float nms = .4;

    layer l = net.layers[net.n-1];
    float *X = buff_letter[(buff_index+2)%3].data;
    float *prediction = network_predict(net, X);

    memcpy(predictions[demo_index], prediction, l.outputs*sizeof(float));
    mean_arrays(predictions, demo_frame, l.outputs, avg);
    l.output = last_avg2;
    if(demo_delay == 0) l.output = avg;
    if(l.type == DETECTION){
        get_detection_boxes(l, 1, 1, demo_thresh, probs, boxes, 0);
    } else if (l.type == REGION){
        get_region_boxes(l, buff[0].w, buff[0].h, net.w, net.h, demo_thresh, probs, boxes, 0, 0, demo_hier, 1);
    } else {
        error("Last layer must produce detections\n");
    }
    if (nms > 0) do_nms_obj(boxes, probs, l.w*l.h*l.n, l.classes, nms);

    printf("\033[2J");
    printf("\033[1;1H");
    printf("\nFPS:%.1f\n",fps);
    printf("Objects:\n\n");
    image display = buff[(buff_index+2) % 3];

            
    

    draw_detections(display, demo_detections, demo_thresh, boxes, probs, demo_names, demo_alphabet, demo_classes, &detects, bs);

    if(detects > 0){
        if(fw.init >= 1){
    
            free(fw.frame1);
            fw.frame1 = malloc(fw.size2*sizeof(box));
            memcpy(fw.frame1,fw.frame2,fw.size2*sizeof(box));
            fw.size1 = fw.size2;
            
            free(fw.frame2);
            fw.frame2 = malloc(fw.size3*sizeof(box));
            memcpy(fw.frame2,fw.frame3,fw.size3*sizeof(box));
            fw.size2 = fw.size3;

            free(fw.frame3);
            fw.frame3 = malloc(fw.size4*sizeof(box));       
            memcpy(fw.frame3,fw.frame4,fw.size4*sizeof(box));
            fw.size3 = fw.size4;

            free(fw.frame4);
            fw.frame4 = malloc(fw.size5*sizeof(box));
            memcpy(fw.frame4,fw.frame5,fw.size5*sizeof(box));
            fw.size4 = fw.size5;

            free(fw.frame5);
            fw.frame5 = malloc(fw.size6*sizeof(box));
            memcpy(fw.frame5,fw.frame6,fw.size6*sizeof(box));
            fw.size5 = fw.size6;

            free(fw.frame6);
            fw.frame6 = malloc(fw.size7*sizeof(box));
            memcpy(fw.frame6,fw.frame7,fw.size7*sizeof(box));
            fw.size6 = fw.size7;

            free(fw.frame7);
            fw.frame7 = malloc(fw.size8*sizeof(box));
            memcpy(fw.frame7,fw.frame8,fw.size8*sizeof(box));
            fw.size7 = fw.size8;

            free(fw.frame8);
            fw.frame8 = malloc(fw.size9*sizeof(box));
            memcpy(fw.frame8,fw.frame9,fw.size9*sizeof(box));
            fw.size8 = fw.size9;

            free(fw.frame9);
            fw.frame9 = malloc(fw.size10*sizeof(box));
            memcpy(fw.frame9,fw.frame10,fw.size10*sizeof(box));
            fw.size9 = fw.size10;
  
            free(fw.frame10);
            fw.size10 = detects;
            fw.frame10 = malloc(detects*sizeof(box));
            memcpy(fw.frame10,bs,detects*sizeof(box));
            out = cluster(fw,fpa,fpb);

            if(fw.init == 1){
                free(old_cents);
                old_cents = malloc(fw.size10*24);
                memcpy(old_cents,new_cents,fw.size10*24);
                free(new_cents);

                //initialize names
                printf("initializing names\n");
                for(i = 0; i < fw.size10; ++i){
                    old_cents[i].name = 'a' + i;
                    max_name = (++max_name)%26;
                }
                printf("initialized names\n");
            }

            if(fw.init > 1){
                compare_centroids(new_cents,fw.size10,old_cents,fw.size9);

                free(old_cents);
                old_cents = malloc(fw.size10*24);
                memcpy(old_cents,new_cents,fw.size10*24);
                free(new_cents);
            }

            ++fw.init;

            for(i = 0; i < fw.size10; ++i){
               cent_buff[2*m_cent] = out[2*i];
               cent_buff[2*m_cent+1] = out[2*i+1];
               ++m_cent;
                if(m_cent == 100) {
                 m_cent = 0;
                }
            } 
            n_cent = 0;
            while(n_cent < 100)
            // while(abs(cent_buff[n_cent]) > 0.0000001 && n_cent < 100)
            {
                double xn = cent_buff[2*n_cent];
                double yn = cent_buff[2*n_cent+1];
                // printf("xn: %f, yn: %f\n", xn, yn);

                int sxTmp = xn * display.w;
                int syTmp = yn * display.h;
                // printf("sxTmp: %f, syTmp: %f\n", sxTmp, syTmp);
          
                sx = sxTmp;
                sy = syTmp * display.w;
                // printf("sx   : %d, sy   : %d\n", sx, sy);

                // sy = cent_buff[2*n_cent+1]*(double)(display.h*display.w);
                display.data[sx + sy + 0*display.w*display.h] = 0.196;
                display.data[sx + sy + 1*display.w*display.h] = 0.8;
                display.data[sx + sy + 2*display.w*display.h] = 0.196;
                n_cent++;
            }
            /* printf("sx: %f\nsy: %f\n",cent_buff[2*(m_cent-1)]*display.w, cent_buff[2*(m_cent-1)+1]*display.h);
            printf("sx: %f\nsy: %f\n",out[0]*display.w,out[1]*display.h);
            printf("sx: %d\nsy: %d\n", sx, sy / display.w); */

            free(out);
        }
        else{
            ++fw.init;
                 
            fw.size10 = detects;      
            fw.frame10 = malloc(fw.size10*sizeof(box)); //if bs changes, then so does frame10
            memcpy(fw.frame10,bs,detects*sizeof(box));


            fw.frame9 = malloc(fw.size10*sizeof(box));
            memcpy(fw.frame9,fw.frame10,fw.size10*sizeof(box));
            fw.size9 = fw.size10;

            fw.frame8 = malloc(fw.size10*sizeof(box));
            memcpy(fw.frame8,fw.frame9,fw.size9*sizeof(box));
            fw.size8 = fw.size9;

            fw.frame7 = malloc(fw.size10*sizeof(box));
            memcpy(fw.frame7,fw.frame8,fw.size8*sizeof(box));
            fw.size7 = fw.size8;

            fw.frame6 = malloc(fw.size10*sizeof(box));
            memcpy(fw.frame6,fw.frame7,fw.size7*sizeof(box));
            fw.size6 = fw.size7;

            fw.frame5 = malloc(fw.size10*sizeof(box));
            memcpy(fw.frame5,fw.frame6,fw.size6*sizeof(box));
            fw.size5 = fw.size6;

            fw.frame4 = malloc(fw.size10*sizeof(box));
            memcpy(fw.frame4,fw.frame5,fw.size5*sizeof(box));
            fw.size4 = fw.size5;

            fw.frame3 = malloc(fw.size10*sizeof(box));
            memcpy(fw.frame3,fw.frame4,fw.size4*sizeof(box));
            fw.size3 = fw.size4;

            fw.frame2 = malloc(fw.size10*sizeof(box));
            memcpy(fw.frame2,fw.frame3,fw.size3*sizeof(box));
            fw.size2 = fw.size3;

            fw.frame1 = malloc(fw.size10*sizeof(box));
            memcpy(fw.frame1,fw.frame2,fw.size2*sizeof(box));
            fw.size1 = fw.size2;

            printf("intialized for clustering 1\n");

              
        }

    } 

    else{
        fw.init = 0;
    }
    

    demo_index = (demo_index + 1)%demo_frame;
    running = 0;
    return 0;
}

void *fetch_in_thread(void *ptr)
{
    int status = fill_image_from_stream(cap, buff[buff_index]);
    letterbox_image_into(buff[buff_index], net.w, net.h, buff_letter[buff_index]);
    if(status == 0) demo_done = 1;
    return 0;
}

void *display_in_thread(void *ptr)
{
    show_image_cv(buff[(buff_index + 1)%3], "Demo", ipl);
    int c = cvWaitKey(1);
    if (c != -1) c = c%256;
    if (c == 10){
        if(demo_delay == 0) demo_delay = 60;
        else if(demo_delay == 5) demo_delay = 0;
        else if(demo_delay == 60) demo_delay = 5;
        else demo_delay = 0;
    } else if (c == 27) {
        demo_done = 1;
        return 0;
    } else if (c == 82) {
        demo_thresh += .02;
    } else if (c == 84) {
        demo_thresh -= .02;
        if(demo_thresh <= .02) demo_thresh = .02;
    } else if (c == 83) {
        demo_hier += .02;
    } else if (c == 81) {
        demo_hier -= .02;
        if(demo_hier <= .0) demo_hier = .0;
    }
    return 0;
}

void *display_loop(void *ptr)
{
    while(1){
        display_in_thread(0);
    }
}

void *detect_loop(void *ptr)
{
    while(1){
        detect_in_thread(0);
    }
}

void demo(char *cfgfile, char *weightfile, float thresh, int cam_index, const char *filename, char **names, int classes, int delay, char *prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen)
{
    demo_delay = delay;
    demo_frame = avg_frames;
    predictions = calloc(demo_frame, sizeof(float*));
    image **alphabet = load_alphabet();
    demo_names = names;
    demo_alphabet = alphabet;
    demo_classes = classes;
    demo_thresh = thresh;
    demo_hier = hier;
    printf("Demo\n");
    net = parse_network_cfg(cfgfile);
    if(weightfile){
        load_weights(&net, weightfile);
    }
    set_batch_network(&net, 1);
    pthread_t detect_thread;
    pthread_t fetch_thread;

    srand(2222222);

    if(filename){
        printf("video file: %s\n", filename);
        cap = cvCaptureFromFile(filename);
    }else{
        cap = cvCaptureFromCAM(cam_index);

        if(w){
            cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_WIDTH, w);
        }
        if(h){
            cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_HEIGHT, h);
        }
        if(frames){
            cvSetCaptureProperty(cap, CV_CAP_PROP_FPS, frames);
        }
    }

    if(!cap) error("Couldn't connect to webcam.\n");

    layer l = net.layers[net.n-1];
    demo_detections = l.n*l.w*l.h;
    int j;
    fw.init = 0;
    //fw.frame1 = malloc(sizeof(box[100]));
    //fw.frame2 = malloc(sizeof(box[100])); 

    avg = (float *) calloc(l.outputs, sizeof(float));
    last_avg  = (float *) calloc(l.outputs, sizeof(float));
    last_avg2 = (float *) calloc(l.outputs, sizeof(float));
    for(j = 0; j < demo_frame; ++j) predictions[j] = (float *) calloc(l.outputs, sizeof(float));

    boxes = (box *)calloc(l.w*l.h*l.n, sizeof(box));
    probs = (float **)calloc(l.w*l.h*l.n, sizeof(float *));
    for(j = 0; j < l.w*l.h*l.n; ++j) probs[j] = (float *)calloc(l.classes+1, sizeof(float));

    buff[0] = get_image_from_stream(cap);
    buff[1] = copy_image(buff[0]);
    buff[2] = copy_image(buff[0]); //image buffer is full from initialization
    buff_letter[0] = letterbox_image(buff[0], net.w, net.h);
    buff_letter[1] = letterbox_image(buff[0], net.w, net.h);
    buff_letter[2] = letterbox_image(buff[0], net.w, net.h);
    ipl = cvCreateImage(cvSize(buff[0].w,buff[0].h), IPL_DEPTH_8U, buff[0].c);

    count = 0;
    if(!prefix){
        cvNamedWindow("Demo", CV_WINDOW_NORMAL); 
        if(fullscreen){
            cvSetWindowProperty("Demo", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
        } else {
            cvMoveWindow("Demo", 0, 0);
            cvResizeWindow("Demo", 1352, 1013);
        }
    }

    demo_time = get_wall_time();

    fpa = fopen("FINAL_CENTROIDS.txt","w+");
    fpb = fopen("OBJ_FUNCTION.txt","w+");


    while(!demo_done){
        buff_index = (buff_index + 1) %3;
        if(pthread_create(&fetch_thread, 0, fetch_in_thread, 0)) error("Thread creation failed");
        if(pthread_create(&detect_thread, 0, detect_in_thread, 0)) error("Thread creation failed");
        
        if(!prefix){
            if(count % (demo_delay+1) == 0){
                fps = 1./(get_wall_time() - demo_time);
                demo_time = get_wall_time();
                float *swap = last_avg;
                last_avg  = last_avg2;
                last_avg2 = swap;
                memcpy(last_avg, avg, l.outputs*sizeof(float));
            }
            display_in_thread(0);
        }else{
            char name[256];
            sprintf(name, "%s_%08d", prefix, count);
            save_image(buff[(buff_index + 1)%3], name);
        }
        pthread_join(fetch_thread, 0);
        pthread_join(detect_thread, 0);
        ++count;
    }
        fclose(fpa);
        fclose(fpb);
}
#else
void demo(char *cfgfile, char *weightfile, float thresh, int cam_index, const char *filename, char **names, int classes, int delay, char *prefix, int avg, float hier, int w, int h, int frames, int fullscreen)
{
    fprintf(stderr, "Demo needs OpenCV for webcam images.\n");
}
#endif

