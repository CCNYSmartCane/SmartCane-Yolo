#include "darknet.h"
#include <stdio.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <stdbool.h>
#include <math.h>
typedef struct{
    box* frame1;
    double* depth1;
    size_t size1;

    box* frame2;
    double* depth2;
    size_t size2;

    box* frame3;
    double* depth3;
    size_t size3;
    
    box* frame4;
    double* depth4;
    size_t size4;

    box* frame5;
    double* depth5;
    size_t size5;
    
    box* frame6;
    double* depth6;
    size_t size6;

    box* frame7;
    double* depth7;
    size_t size7;
    
    box* frame8;
    double* depth8;
    size_t size8;

    box* frame9;
    double* depth9;
    size_t size9;

    box* frame10;
    double* depth10;
    size_t size10;

    size_t init; 
}framewindow;

box bs[100];
double* out;
static framewindow fw;
size_t frame_buff = 0;
int count;
FILE *fp;
FILE *fp2;

static int coco_ids[] = {1,2,3,4,5,6,7,8,9,10,11,13,14,15,16,17,18,19,20,21,22,23,24,25,27,28,31,32,33,34,35,36,37,38,39,40,41,42,43,44,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,67,70,72,73,74,75,76,77,78,79,80,81,82,84,85,86,87,88,89,90};

double* cluster2(framewindow input, FILE *fp, FILE *fp2){
    box detections[845];
    double depths[845];
    int s = 0;
    int t = 0;
    size_t total = input.size1+input.size2+input.size3
                    +input.size4+input.size5+input.size6
                    +input.size7+input.size8+input.size9+input.size10;
    int d_old[total];
    d_old[0] = 1000; //need this to overcome zero initialization
    
    

    double *centroids = malloc(3*input.size10*sizeof(double));           // [2*input.size3]; //double the orignal size (one for x, one for y)
    //out = malloc(3*input.size10*sizeof(double));

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

    


    memcpy(depths,input.depth1,sizeof(double)*input.size1);

    memcpy(&depths[input.size1],input.depth2,sizeof(double)*input.size2);
 
    memcpy(&depths[input.size1+input.size2],input.depth3,sizeof(double)*input.size3);

    memcpy(&depths[input.size1+input.size2+input.size3],input.depth4,sizeof(double)*input.size4);

    memcpy(&depths[input.size1+input.size2+input.size3+input.size4],input.depth5,sizeof(double)*input.size5);

    memcpy(&depths[input.size1+input.size2+input.size3+input.size4+input.size5],input.depth6,sizeof(double)*input.size6);

    memcpy(&depths[input.size1+input.size2+input.size3+input.size4+input.size5
                      +input.size6],input.depth7,sizeof(double)*input.size7);

    memcpy(&depths[input.size1+input.size2+input.size3+input.size4+input.size5
                      +input.size6+input.size7],input.depth8,sizeof(double)*input.size8);

    memcpy(&depths[input.size1+input.size2+input.size3+input.size4+input.size5
                      +input.size6+input.size7+input.size8],input.depth9,sizeof(double)*input.size9);

    memcpy(&depths[input.size1+input.size2+input.size3+input.size4+input.size5
                      +input.size6+input.size7+input.size8+input.size9],input.depth10,sizeof(double)*input.size10);

    printf("copied all dectections to detection buffer...\n");
    
    /*INITIAL GUESSES FOR CENTROIDS*/
    for(s = 0; s < input.size10; ++s){
        centroids[3*s] = input.frame10[s].x;
        centroids[3*s+1] = input.frame10[s].y;
        centroids[3*s+2] = input.depth10[s];
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
                d_temp = (pow(detections[s].x-centroids[3*t],2)+
                              pow(detections[s].y-centroids[3*t+1],2)+
                              pow(depths[s]-centroids[3*t+2],2));
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
            double numer_z = 0.0;
            double denom = 0.0;
            //printf("Total detections: %d\n",total);
            for(t = 0; t < total; ++t){
                if(d[t] == s){
                    // save x/y of detections for future calculation of obj func
                    numer_x += detections[t].x;
                    numer_y += detections[t].y;
                    numer_z += depths[t];
                    denom += 1.0;
                    //printf("writing to file...\n");
                    fprintf(fp2,"%d\t%d\t%f\t%f\t%f\n",iters,s,detections[t].x,detections[t].y,depths[t]);
                }
            }
            //printf("found all corresponding detections...\n");
           // save all centroids to txt file (1)
            centroids[3*s] = numer_x/denom;
            centroids[3*s+1] = numer_y/denom;
            centroids[3*s+2] = numer_z/denom;
            //printf("x:%f\n",centroids[2*s]);
            //printf("y:%f\n",centroids[2*s+1]);
            fprintf(fp2,"\t%d\t%f\t%f\t%f\n",s,centroids[3*s],centroids[3*s+1],centroids[3*s+2]); //might need to stick an extra new line here
        }
        printf("update step complete...\n");
        size_t flag = 0;

        while(d[flag] == d_old[flag]){
            if(flag == total-1){
                for(s = 0; s < input.size10; ++s){
                    printf("total:%d\n",total);
                    printf("cluster number:%d\n",s+1);
                    printf("iterations to cluster:%d\n",iters);
                    printf("x:%f\n",centroids[3*s]);
                    printf("y:%f\n",centroids[3*s+1]);
                    printf("z:%f\n",centroids[3*s+2]);
                    //save final centroids to txt file 

                    fprintf(fp,"%d\t%f\t%f\t%f\n",count,centroids[3*s],centroids[3*s+1],centroids[3*s+2]);

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


void train_detector(char *datacfg, char *cfgfile, char *weightfile, int *gpus, int ngpus, int clear)
{
    list *options = read_data_cfg(datacfg);
    char *train_images = option_find_str(options, "train", "data/train.list");
    char *backup_directory = option_find_str(options, "backup", "/backup/");

    srand(time(0));
    char *base = basecfg(cfgfile);
    printf("%s\n", base);
    float avg_loss = -1;
    network *nets = calloc(ngpus, sizeof(network));

    srand(time(0));
    int seed = rand();
    int i;
    for(i = 0; i < ngpus; ++i){
        srand(seed);
#ifdef GPU
        cuda_set_device(gpus[i]);
#endif
        nets[i] = load_network(cfgfile, weightfile, clear);
    }
    srand(time(0));
    network net = nets[0];

    int imgs = net.batch * net.subdivisions * ngpus;
    printf("Learning Rate: %g, Momentum: %g, Decay: %g\n", net.learning_rate, net.momentum, net.decay);
    data train, buffer;

    layer l = net.layers[net.n - 1];

    int classes = l.classes;
    float jitter = l.jitter;

    list *plist = get_paths(train_images);
    //int N = plist->size;
    char **paths = (char **)list_to_array(plist);

    load_args args = {0};
    args.w = net.w;
    args.h = net.h;
    args.paths = paths;
    args.n = imgs;
    args.m = plist->size;
    args.classes = classes;
    args.jitter = jitter;
    args.num_boxes = l.max_boxes;
    args.d = &buffer;
    args.type = DETECTION_DATA;
    args.threads = 8;

    args.angle = net.angle;
    args.exposure = net.exposure;
    args.saturation = net.saturation;
    args.hue = net.hue;

    pthread_t load_thread = load_data(args);
    clock_t time;
    int count = 0;
    //while(i*imgs < N*120){
    while(get_current_batch(net) < net.max_batches){
        if(l.random && count++%10 == 0){
            printf("Resizing\n");
            int dim = (rand() % 10 + 10) * 32;
            if (get_current_batch(net)+200 > net.max_batches) dim = 608;
            //int dim = (rand() % 4 + 16) * 32;
            printf("%d\n", dim);
            args.w = dim;
            args.h = dim;

            pthread_join(load_thread, 0);
            train = buffer;
            free_data(train);
            load_thread = load_data(args);

            for(i = 0; i < ngpus; ++i){
                resize_network(nets + i, dim, dim);
            }
            net = nets[0];
        }
        time=clock();
        pthread_join(load_thread, 0);
        train = buffer;
        load_thread = load_data(args);

        /*
        int k;
        for(k = 0; k < l.max_boxes; ++k){
            box b = float_to_box(train.y.vals[10] + 1 + k*5);
            if(!b.x) break;
            printf("loaded: %f %f %f %f\n", b.x, b.y, b.w, b.h);
        }
        */
        /*
        int zz;
        for(zz = 0; zz < train.X.cols; ++zz){
            image im = float_to_image(net.w, net.h, 3, train.X.vals[zz]);
            int k;
            for(k = 0; k < l.max_boxes; ++k){
                box b = float_to_box(train.y.vals[zz] + k*5);
                printf("%f %f %f %f\n", b.x, b.y, b.w, b.h);
                draw_bbox(im, b, 1, 1,0,0);
            }
            show_image(im, "truth11");
            cvWaitKey(0);
            save_image(im, "truth11");
        }
        */

        printf("Loaded: %lf seconds\n", sec(clock()-time));

        time=clock();
        float loss = 0;
#ifdef GPU
        if(ngpus == 1){
            loss = train_network(net, train);
        } else {
            loss = train_networks(nets, ngpus, train, 4);
        }
#else
        loss = train_network(net, train);
#endif
        if (avg_loss < 0) avg_loss = loss;
        avg_loss = avg_loss*.9 + loss*.1;

        i = get_current_batch(net);
        printf("%d: %f, %f avg, %f rate, %lf seconds, %d images\n", get_current_batch(net), loss, avg_loss, get_current_rate(net), sec(clock()-time), i*imgs);
        if(i%1000==0){
#ifdef GPU
            if(ngpus != 1) sync_nets(nets, ngpus, 0);
#endif
            char buff[256];
            sprintf(buff, "%s/%s.backup", backup_directory, base);
            save_weights(net, buff);
        }
        if(i%10000==0 || (i < 1000 && i%100 == 0)){
#ifdef GPU
            if(ngpus != 1) sync_nets(nets, ngpus, 0);
#endif
            char buff[256];
            sprintf(buff, "%s/%s_%d.weights", backup_directory, base, i);
            save_weights(net, buff);
        }
        free_data(train);
    }
#ifdef GPU
    if(ngpus != 1) sync_nets(nets, ngpus, 0);
#endif
    char buff[256];
    sprintf(buff, "%s/%s_final.weights", backup_directory, base);
    save_weights(net, buff);
}


static int get_coco_image_id(char *filename)
{
    char *p = strrchr(filename, '_');
    return atoi(p+1);
}

static void print_cocos(FILE *fp, char *image_path, box *boxes, float **probs, int num_boxes, int classes, int w, int h)
{
    int i, j;
    int image_id = get_coco_image_id(image_path);
    for(i = 0; i < num_boxes; ++i){
        float xmin = boxes[i].x - boxes[i].w/2.;
        float xmax = boxes[i].x + boxes[i].w/2.;
        float ymin = boxes[i].y - boxes[i].h/2.;
        float ymax = boxes[i].y + boxes[i].h/2.;

        if (xmin < 0) xmin = 0;
        if (ymin < 0) ymin = 0;
        if (xmax > w) xmax = w;
        if (ymax > h) ymax = h;

        float bx = xmin;
        float by = ymin;
        float bw = xmax - xmin;
        float bh = ymax - ymin;

        for(j = 0; j < classes; ++j){
            if (probs[i][j]) fprintf(fp, "{\"image_id\":%d, \"category_id\":%d, \"bbox\":[%f, %f, %f, %f], \"score\":%f},\n", image_id, coco_ids[j], bx, by, bw, bh, probs[i][j]);
        }
    }
}

void print_detector_detections(FILE **fps, char *id, box *boxes, float **probs, int total, int classes, int w, int h)
{
    int i, j;
    for(i = 0; i < total; ++i){
        float xmin = boxes[i].x - boxes[i].w/2. + 1;
        float xmax = boxes[i].x + boxes[i].w/2. + 1;
        float ymin = boxes[i].y - boxes[i].h/2. + 1;
        float ymax = boxes[i].y + boxes[i].h/2. + 1;

        if (xmin < 1) xmin = 1;
        if (ymin < 1) ymin = 1;
        if (xmax > w) xmax = w;
        if (ymax > h) ymax = h;

        for(j = 0; j < classes; ++j){
            if (probs[i][j]) fprintf(fps[j], "%s %f %f %f %f %f\n", id, probs[i][j],
                    xmin, ymin, xmax, ymax);
        }
    }
}

void print_imagenet_detections(FILE *fp, int id, box *boxes, float **probs, int total, int classes, int w, int h)
{
    int i, j;
    for(i = 0; i < total; ++i){
        float xmin = boxes[i].x - boxes[i].w/2.;
        float xmax = boxes[i].x + boxes[i].w/2.;
        float ymin = boxes[i].y - boxes[i].h/2.;
        float ymax = boxes[i].y + boxes[i].h/2.;

        if (xmin < 0) xmin = 0;
        if (ymin < 0) ymin = 0;
        if (xmax > w) xmax = w;
        if (ymax > h) ymax = h;

        for(j = 0; j < classes; ++j){
            int class = j;
            if (probs[i][class]) fprintf(fp, "%d %d %f %f %f %f %f\n", id, j+1, probs[i][class],
                    xmin, ymin, xmax, ymax);
        }
    }
}

void validate_detector_flip(char *datacfg, char *cfgfile, char *weightfile, char *outfile)
{
    int j;
    list *options = read_data_cfg(datacfg);
    char *valid_images = option_find_str(options, "valid", "data/train.list");
    char *name_list = option_find_str(options, "names", "data/names.list");
    char *prefix = option_find_str(options, "results", "results");
    char **names = get_labels(name_list);
    char *mapf = option_find_str(options, "map", 0);
    int *map = 0;
    if (mapf) map = read_map(mapf);

    network net = parse_network_cfg(cfgfile);
    if(weightfile){
        load_weights(&net, weightfile);
    }
    set_batch_network(&net, 2);
    fprintf(stderr, "Learning Rate: %g, Momentum: %g, Decay: %g\n", net.learning_rate, net.momentum, net.decay);
    srand(time(0));

    list *plist = get_paths(valid_images);
    char **paths = (char **)list_to_array(plist);

    layer l = net.layers[net.n-1];
    int classes = l.classes;

    char buff[1024];
    char *type = option_find_str(options, "eval", "voc");
    FILE *fp = 0;
    FILE **fps = 0;
    int coco = 0;
    int imagenet = 0;
    if(0==strcmp(type, "coco")){
        if(!outfile) outfile = "coco_results";
        snprintf(buff, 1024, "%s/%s.json", prefix, outfile);
        fp = fopen(buff, "w");
        fprintf(fp, "[\n");
        coco = 1;
    } else if(0==strcmp(type, "imagenet")){
        if(!outfile) outfile = "imagenet-detection";
        snprintf(buff, 1024, "%s/%s.txt", prefix, outfile);
        fp = fopen(buff, "w");
        imagenet = 1;
        classes = 200;
    } else {
        if(!outfile) outfile = "comp4_det_test_";
        fps = calloc(classes, sizeof(FILE *));
        for(j = 0; j < classes; ++j){
            snprintf(buff, 1024, "%s/%s%s.txt", prefix, outfile, names[j]);
            fps[j] = fopen(buff, "w");
        }
    }


    box *boxes = calloc(l.w*l.h*l.n, sizeof(box));
    float **probs = calloc(l.w*l.h*l.n, sizeof(float *));
    for(j = 0; j < l.w*l.h*l.n; ++j) probs[j] = calloc(classes+1, sizeof(float *));

    int m = plist->size;
    int i=0;
    int t;

    float thresh = .005;
    float nms = .45;

    int nthreads = 4;
    image *val = calloc(nthreads, sizeof(image));
    image *val_resized = calloc(nthreads, sizeof(image));
    image *buf = calloc(nthreads, sizeof(image));
    image *buf_resized = calloc(nthreads, sizeof(image));
    pthread_t *thr = calloc(nthreads, sizeof(pthread_t));

    image input = make_image(net.w, net.h, net.c*2);

    load_args args = {0};
    args.w = net.w;
    args.h = net.h;
    //args.type = IMAGE_DATA;
    args.type = LETTERBOX_DATA;

    for(t = 0; t < nthreads; ++t){
        args.path = paths[i+t];
        args.im = &buf[t];
        args.resized = &buf_resized[t];
        thr[t] = load_data_in_thread(args);
    }
    time_t start = time(0);
    for(i = nthreads; i < m+nthreads; i += nthreads){
        fprintf(stderr, "%d\n", i);
        for(t = 0; t < nthreads && i+t-nthreads < m; ++t){
            pthread_join(thr[t], 0);
            val[t] = buf[t];
            val_resized[t] = buf_resized[t];
        }
        for(t = 0; t < nthreads && i+t < m; ++t){
            args.path = paths[i+t];
            args.im = &buf[t];
            args.resized = &buf_resized[t];
            thr[t] = load_data_in_thread(args);
        }
        for(t = 0; t < nthreads && i+t-nthreads < m; ++t){
            char *path = paths[i+t-nthreads];
            char *id = basecfg(path);
            copy_cpu(net.w*net.h*net.c, val_resized[t].data, 1, input.data, 1);
            flip_image(val_resized[t]);
            copy_cpu(net.w*net.h*net.c, val_resized[t].data, 1, input.data + net.w*net.h*net.c, 1);

            network_predict(net, input.data);
            int w = val[t].w;
            int h = val[t].h;
            get_region_boxes(l, w, h, net.w, net.h, thresh, probs, boxes, 0, map, .5, 0);
            if (nms) do_nms_sort(boxes, probs, l.w*l.h*l.n, classes, nms);
            if (coco){
                print_cocos(fp, path, boxes, probs, l.w*l.h*l.n, classes, w, h);
            } else if (imagenet){
                print_imagenet_detections(fp, i+t-nthreads+1, boxes, probs, l.w*l.h*l.n, classes, w, h);
            } else {
                print_detector_detections(fps, id, boxes, probs, l.w*l.h*l.n, classes, w, h);
            }
            free(id);
            free_image(val[t]);
            free_image(val_resized[t]);
        }
    }
    for(j = 0; j < classes; ++j){
        if(fps) fclose(fps[j]);
    }
    if(coco){
        fseek(fp, -2, SEEK_CUR); 
        fprintf(fp, "\n]\n");
        fclose(fp);
    }
    fprintf(stderr, "Total Detection Time: %f Seconds\n", (double)(time(0) - start));
}


void validate_detector(char *datacfg, char *cfgfile, char *weightfile, char *outfile)
{
    int j;
    list *options = read_data_cfg(datacfg);
    char *valid_images = option_find_str(options, "valid", "data/train.list");
    char *name_list = option_find_str(options, "names", "data/names.list");
    char *prefix = option_find_str(options, "results", "results");
    char **names = get_labels(name_list);
    char *mapf = option_find_str(options, "map", 0);
    int *map = 0;
    if (mapf) map = read_map(mapf);

    network net = parse_network_cfg(cfgfile);
    if(weightfile){
        load_weights(&net, weightfile);
    }
    set_batch_network(&net, 1);
    fprintf(stderr, "Learning Rate: %g, Momentum: %g, Decay: %g\n", net.learning_rate, net.momentum, net.decay);
    srand(time(0));

    list *plist = get_paths(valid_images);
    char **paths = (char **)list_to_array(plist);

    layer l = net.layers[net.n-1];
    int classes = l.classes;

    char buff[1024];
    char *type = option_find_str(options, "eval", "voc");
    FILE *fp = 0;
    FILE **fps = 0;
    int coco = 0;
    int imagenet = 0;
    if(0==strcmp(type, "coco")){
        if(!outfile) outfile = "coco_results";
        snprintf(buff, 1024, "%s/%s.json", prefix, outfile);
        fp = fopen(buff, "w");
        fprintf(fp, "[\n");
        coco = 1;
    } else if(0==strcmp(type, "imagenet")){
        if(!outfile) outfile = "imagenet-detection";
        snprintf(buff, 1024, "%s/%s.txt", prefix, outfile);
        fp = fopen(buff, "w");
        imagenet = 1;
        classes = 200;
    } else {
        if(!outfile) outfile = "comp4_det_test_";
        fps = calloc(classes, sizeof(FILE *));
        for(j = 0; j < classes; ++j){
            snprintf(buff, 1024, "%s/%s%s.txt", prefix, outfile, names[j]);
            fps[j] = fopen(buff, "w");
        }
    }


    box *boxes = calloc(l.w*l.h*l.n, sizeof(box));
    float **probs = calloc(l.w*l.h*l.n, sizeof(float *));
    for(j = 0; j < l.w*l.h*l.n; ++j) probs[j] = calloc(classes+1, sizeof(float *));

    int m = plist->size;
    int i=0;
    int t;

    float thresh = .005;
    float nms = .45;

    int nthreads = 4;
    image *val = calloc(nthreads, sizeof(image));
    image *val_resized = calloc(nthreads, sizeof(image));
    image *buf = calloc(nthreads, sizeof(image));
    image *buf_resized = calloc(nthreads, sizeof(image));
    pthread_t *thr = calloc(nthreads, sizeof(pthread_t));

    load_args args = {0};
    args.w = net.w;
    args.h = net.h;
    //args.type = IMAGE_DATA;
    args.type = LETTERBOX_DATA;

    for(t = 0; t < nthreads; ++t){
        args.path = paths[i+t];
        args.im = &buf[t];
        args.resized = &buf_resized[t];
        thr[t] = load_data_in_thread(args);
    }
    time_t start = time(0);
    for(i = nthreads; i < m+nthreads; i += nthreads){
        fprintf(stderr, "%d\n", i);
        for(t = 0; t < nthreads && i+t-nthreads < m; ++t){
            pthread_join(thr[t], 0);
            val[t] = buf[t];
            val_resized[t] = buf_resized[t];
        }
        for(t = 0; t < nthreads && i+t < m; ++t){
            args.path = paths[i+t];
            args.im = &buf[t];
            args.resized = &buf_resized[t];
            thr[t] = load_data_in_thread(args);
        }
        for(t = 0; t < nthreads && i+t-nthreads < m; ++t){
            char *path = paths[i+t-nthreads];
            char *id = basecfg(path);
            float *X = val_resized[t].data;
            network_predict(net, X);
            int w = val[t].w;
            int h = val[t].h;
            get_region_boxes(l, w, h, net.w, net.h, thresh, probs, boxes, 0, map, .5, 0);
            if (nms) do_nms_sort(boxes, probs, l.w*l.h*l.n, classes, nms);
            if (coco){
                print_cocos(fp, path, boxes, probs, l.w*l.h*l.n, classes, w, h);
            } else if (imagenet){
                print_imagenet_detections(fp, i+t-nthreads+1, boxes, probs, l.w*l.h*l.n, classes, w, h);
            } else {
                print_detector_detections(fps, id, boxes, probs, l.w*l.h*l.n, classes, w, h);
            }
            free(id);
            free_image(val[t]);
            free_image(val_resized[t]);
        }
    }
    for(j = 0; j < classes; ++j){
        if(fps) fclose(fps[j]);
    }
    if(coco){
        fseek(fp, -2, SEEK_CUR); 
        fprintf(fp, "\n]\n");
        fclose(fp);
    }
    fprintf(stderr, "Total Detection Time: %f Seconds\n", (double)(time(0) - start));
}

void validate_detector_recall(char *cfgfile, char *weightfile)
{
    network net = parse_network_cfg(cfgfile);
    if(weightfile){
        load_weights(&net, weightfile);
    }
    set_batch_network(&net, 1);
    fprintf(stderr, "Learning Rate: %g, Momentum: %g, Decay: %g\n", net.learning_rate, net.momentum, net.decay);
    srand(time(0));

    list *plist = get_paths("data/voc.2007.test");
    char **paths = (char **)list_to_array(plist);

    layer l = net.layers[net.n-1];
    int classes = l.classes;

    int j, k;
    box *boxes = calloc(l.w*l.h*l.n, sizeof(box));
    float **probs = calloc(l.w*l.h*l.n, sizeof(float *));
    for(j = 0; j < l.w*l.h*l.n; ++j) probs[j] = calloc(classes+1, sizeof(float *));

    int m = plist->size;
    int i=0;

    float thresh = .001;
    float iou_thresh = .5;
    float nms = .4;

    int total = 0;
    int correct = 0;
    int proposals = 0;
    float avg_iou = 0;

    for(i = 0; i < m; ++i){
        char *path = paths[i];
        image orig = load_image_color(path, 0, 0);
        image sized = resize_image(orig, net.w, net.h);
        char *id = basecfg(path);
        network_predict(net, sized.data);
        get_region_boxes(l, sized.w, sized.h, net.w, net.h, thresh, probs, boxes, 1, 0, .5, 1);
        if (nms) do_nms(boxes, probs, l.w*l.h*l.n, 1, nms);

        char labelpath[4096];
        find_replace(path, "images", "labels", labelpath);
        find_replace(labelpath, "JPEGImages", "labels", labelpath);
        find_replace(labelpath, ".jpg", ".txt", labelpath);
        find_replace(labelpath, ".JPEG", ".txt", labelpath);

        int num_labels = 0;
        box_label *truth = read_boxes(labelpath, &num_labels);
        for(k = 0; k < l.w*l.h*l.n; ++k){
            if(probs[k][0] > thresh){
                ++proposals;
            }
        }
        for (j = 0; j < num_labels; ++j) {
            ++total;
            box t = {truth[j].x, truth[j].y, truth[j].w, truth[j].h};
            float best_iou = 0;
            for(k = 0; k < l.w*l.h*l.n; ++k){
                float iou = box_iou(boxes[k], t);
                if(probs[k][0] > thresh && iou > best_iou){
                    best_iou = iou;
                }
            }
            avg_iou += best_iou;
            if(best_iou > iou_thresh){
                ++correct;
            }
        }

        fprintf(stderr, "%5d %5d %5d\tRPs/Img: %.2f\tIOU: %.2f%%\tRecall:%.2f%%\n", i, correct, total, (float)proposals/(i+1), avg_iou*100/total, 100.*correct/total);
        free(id);
        free_image(orig);
        free_image(sized);
    }
}

char *replace(char *instring,char *oldSubStr,char *newSubStr)
{
    if(!instring || !oldSubStr || !newSubStr){
        return (char*)NULL;
    }

    size_t instring_size=strlen(instring);
    size_t newSubStr_size=strlen(newSubStr);
    size_t oldSubStr_size=strlen(oldSubStr);
    size_t diffsize=newSubStr_size-oldSubStr_size;
    size_t diffsizeAll=diffsize;
    size_t outstring_size=instring_size*2 + 1;
    char *outstring;
    char *test;

    test=(char*)malloc(oldSubStr_size+1);
    outstring =(char*) malloc(outstring_size);

    if(!outstring || !test){
        return (char*)NULL;
    }
    if(instring_size<oldSubStr_size || oldSubStr_size==0)
    {
        strcpy(outstring, instring);
        free(test);
        return outstring;
    }
    outstring[0]='\0';
    int i;
    for(i=0; i <= instring_size; i++)
    {
        strncpy(test,(instring+i),oldSubStr_size);
        test[oldSubStr_size]='\0';
        if(strcmp(test,oldSubStr)==0){
            if((instring_size+diffsizeAll) > outstring_size)
            {
                outstring_size=outstring_size*2+1;
                outstring=realloc(outstring,outstring_size);
                if(!outstring){
                    free(test);
                    return (char*)NULL;
                }
            }
            strcat(outstring,newSubStr);
            i=i+oldSubStr_size-1;
            diffsizeAll=diffsizeAll+diffsize;
        }else{
            test[1]='\0';
            strcat(outstring,test);
        }
    }
    free(test);
    return outstring;
}

char* concat(const char *s1, const char *s2)
{
    char *result = malloc(strlen(s1)+strlen(s2)+1);//+1 for the zero-terminator
    //in real code you would check for errors in malloc here
    strcpy(result, s1);
    strcat(result, s2);

    char* ret = replace(result, "//", "/");
    free(result);

    return ret;
}

bool strstr_extend(char* str, char* suffixs[]) {
    int i = 0;
    int suffixsLen = (int)(sizeof(suffixs) / sizeof(char *));

    for (i = 0; i < suffixsLen; i++) {
        if(strstr(str, suffixs[i]) != NULL) {
            return true;
        }
    }
    return false;
}

int directory_files_cnt(char* folderPath, char* suffixs[]) {
    int fileCnt = 0;

    DIR * d = opendir(folderPath); // open the path
    if(d == NULL) return 0; // if was not able return
    struct dirent * dir; // for the directory entries

    while ((dir = readdir(d)) != NULL) // if we were able to read somehting from the directory
    {
        if(dir-> d_type != DT_DIR) { // if the type is not directory just print it with blue
            // if(strstr(dir->d_name, suffix) != NULL) {
            if(strstr_extend(dir->d_name, suffixs) == true) {
                fileCnt += 1;
            }
        } else if(dir -> d_type == DT_DIR
                  && strcmp(dir->d_name,".")!=0
                  && strcmp(dir->d_name,"..")!=0 ) { // if it is a directory
            // do nothing
        }
    }

    closedir(d);
    return fileCnt;
}

char** directory_files(char* folderPath, char* suffixs[], int* fileCnt)
{
    int i = 0;
    int cnt = 0;

    const int FILECNT = directory_files_cnt(folderPath, suffixs);
    char** filePathListOutput = malloc(sizeof(char *) * FILECNT);
    *fileCnt = FILECNT;

    struct dirent **namelist;
    int FILECNT_ALL;
    FILECNT_ALL = scandir(folderPath, &namelist, 0, alphasort);

    if (FILECNT_ALL < 0) {
        printf("Error: scandir fails: %s\n", folderPath);
    } else {
        for (i = 0; i < FILECNT_ALL; i++) {
            if(strstr_extend(namelist[i]->d_name, suffixs) == true) {
                if (cnt < FILECNT) {
                    filePathListOutput[cnt] = concat(folderPath, namelist[i]->d_name);
                    cnt += 1;
                } else {
                    printf("Error: filePathListOutput cnt is invalid\n");
                    return NULL;
                }
            }
        }
        free(namelist);
    }

    return filePathListOutput;
}

char* folder_path(char* folderPath, char* subfolder) {
    struct stat sb;
    char* folder = concat(folderPath, subfolder);

    // whether "rgb" folder is inside of this path
    if (stat(folder, &sb) == 0 && S_ISDIR(sb.st_mode)) {
        printf("Find data path at:\n  %s\n", folder);
    } else {
        folder = NULL;
    }

    return folder;
}

/* max size of an image */
#define MAX 800

/* RGB color struct with integral types */
typedef struct {unsigned char red;
    unsigned char green;
    unsigned char blue;
    int i;
}RGB_INT;

struct PGMstructure
{
    int maxVal;
    int width;
    int height;
    RGB_INT data[MAX][MAX];
};

typedef struct PGMstructure PGMImage;

/* Gets an ascii pgm image file, store as a color pgm */
void getPGMfile (char filename[], PGMImage *img)
{
    FILE *in_file;
    char ch;
    int row, col, type;
    int ch_int;

    in_file = fopen(filename, "r");
    if (in_file == NULL)
    {
        fprintf(stderr, "Error: Unable to open file %s\n\n", filename);
        exit(8);
    }

    // printf("\nReading image file: %s\n", filename);

    /*determine pgm image type (only type three can be used)*/
    ch = getc(in_file);
    if(ch != 'P')
    {
        printf("ERROR(1): Not valid pgm/ppm file type\n");
        exit(1);
    }
    ch = getc(in_file);
    /*convert the one digit integer currently represented as a character to
      an integer(48 == '0')*/
    type = ch - 48;
    if((type != 2) && (type != 3) && (type != 5) && (type != 6))
    {
        printf("ERROR(2): Not valid pgm/ppm file type\n");
        exit(1);
    }

//    while(getc(in_file) != '\n');             /* skip to end of line*/
//
//    while (getc(in_file) == '#')              /* skip comment lines */
//    {
//        while (getc(in_file) != '\n');          /* skip to end of comment line */
//    }

    /*there seems to be a difference between color and b/w.  This line is needed
      by b/w but doesn't effect color reading...*/
    fseek(in_file, -1, SEEK_CUR);             /* backup one character*/

    int typeTmp = 0;
    fscanf(in_file,"%d", &typeTmp);
    fscanf(in_file,"%d", &((*img).width));
    fscanf(in_file,"%d", &((*img).height));
    fscanf(in_file,"%d", &((*img).maxVal));

    /* printf("\n width  = %d",(*img).width);
    printf("\n height = %d",(*img).height);
    printf("\n maxVal = %d",(*img).maxVal);
    printf("\n"); */

    if (((*img).width  > MAX) || ((*img).height  > MAX))
    {
        printf("\n\n***ERROR - image too big for current image structure***\n\n");
        exit(1);
    }

    if(type == 2) /*uncompressed ascii file (B/W)*/
    {
        for (row=(*img).height-1; row >=0; row--)
            for (col=0; col< (*img).width; col++)
            {
                fscanf(in_file,"%d", &ch_int);
                (*img).data[row][col].red = ch_int;
                (*img).data[row][col].green = ch_int;
                (*img).data[row][col].blue = ch_int;
            }
    }
    else if(type == 3) /*uncompressed ascii file (color)*/
    {
        for (row=(*img).height-1; row >=0; row--)
            for (col=0; col< (*img).width; col++)
            {

                fscanf(in_file,"%d", &ch_int);
                ((*img).data[row][col].red) = (unsigned char)ch_int;

                fscanf(in_file,"%d", &ch_int);
                ((*img).data[row][col].green) = (unsigned char)ch_int;

                fscanf(in_file,"%d", &ch_int);
                ((*img).data[row][col].blue) = (unsigned char)ch_int;
            }
    }
    else if(type == 5) /*compressed file (B/W)*/
        /*note: this type remains untested at this time...*/
    {
        while(getc(in_file) != '\n'); /*skip to end of line*/

        for (row=(*img).height-1; row >=0; row--)
            for (col=0; col< (*img).width; col++)
            {
                unsigned char chHigh = getc(in_file);
                unsigned char chLow = getc(in_file);
                int value = (int)(chHigh << 8) + (int)chLow;
                (*img).data[row][col].i = value;
                /*
                (*img).data[row][col].red = value;
                (*img).data[row][col].green = value;
                (*img).data[row][col].blue = value;
                 */
            }
    }

    else if(type == 6) /*compressed file (color)*/
    {
        while(getc(in_file) != '\n'); /*skip to end of line*/

        for (row=(*img).height-1; row >=0; row--)
            for (col=0; col< (*img).width; col++)
            {
                (*img).data[row][col].red = getc(in_file);
                (*img).data[row][col].green = getc(in_file);
                (*img).data[row][col].blue = getc(in_file);
            }
    }

    fclose(in_file);
    // printf("\nDone reading file.\n");
}


void test_detector(char *datacfg, char *cfgfile, char *weightfile, char *filename, float thresh, float hier_thresh, char *outfile, int fullscreen)
{
    list *options = read_data_cfg(datacfg);
    char *name_list = option_find_str(options, "names", "data/names.list");
    char **names = get_labels(name_list);

    image **alphabet = load_alphabet();
    network net = parse_network_cfg(cfgfile);
    if(weightfile){
        load_weights(&net, weightfile);
    }
    set_batch_network(&net, 1);
    srand(2222222);
    clock_t time;
    char buff[256];
    char *input = buff;
    int j;
    float nms=.4;

    // read files from folder
    int i = 0;
    int m = 0;
    char* folderPath = filename;
    int rgbFileCnt = 0;
    int depthFileCnt = 0;
    char** rgbFilePathList = NULL;
    char** depthFilePathList = NULL;

    char* folderRGB = folder_path(folderPath, "/rgb/");
    char* folderDepth = folder_path(folderPath, "/depth/");
    // if no "rgb" and depth" folder are inside of this path
    // assume this path is for image files of only RGB
    if (NULL == folderRGB && NULL == folderDepth) {
        folderRGB = folderPath;
    }

    printf("folderDepth = %s\n", folderDepth);

    if (NULL != folderRGB) {
        char* suffixs[] = {"ppm", "jpg", "png"};
        rgbFilePathList = directory_files(folderRGB, suffixs, &rgbFileCnt);
        printf("Find %d RGB   .ppm images in path: %s\n", rgbFileCnt, folderRGB);
    }
    if (NULL != folderDepth) {
        char* suffixs[] = {"pgm"};
        depthFilePathList = directory_files(folderDepth, suffixs, &depthFileCnt);
        printf("Find %d Depth .pgm images in path: %s\n", depthFileCnt, folderDepth);
    }

    if (0 == rgbFileCnt || NULL == rgbFilePathList) {
        printf("Error: No %d RGB .ppm images in path: %s, return\n", rgbFileCnt, folderRGB);
        return;
    }
    if (rgbFileCnt != depthFileCnt && 0 != depthFileCnt) {
        printf("Error: rgbFileCnt[%d] != depthFileCnt[%d], return\n", rgbFileCnt, depthFileCnt);
        return;
    }

    // now, you have all files
    /*
    int rgbFileCnt = 0;
    int depthFileCnt = 0; // if no depth image, it will be 0
    char** rgbFilePathList;
    char** depthFilePathList; // if no depth image, it will be NULL
    rgb image file:
    rgbFilePathList[i]
    depth image file:
    depthFilePathList[i]
     */
        
    fp = fopen("FINAL_CENTROIDS.txt","w+");
    fp2 = fopen("OBJ_FUNCTION.txt","w+");

    float* all_depths;
    for(i = 0; i < rgbFileCnt; i++){
        int detects = 0;
        //box bs[5];

        /* if(filename){
            strncpy(input, filename, 256);
        } else {
            printf("Enter Image Path: ");
            fflush(stdout);
            input = fgets(input, 256, stdin);
            if(!input) return;
            strtok(input, "\n");
        } */

        input = rgbFilePathList[i];
        image im = load_image_color(input,0,0);
        // printf("Process RGB   image [w x h: %d x %d]:\n  %s\n", im.w, im.h, input);
        image sized = letterbox_image(im, net.w, net.h);
        //image sized = resize_image(im, net.w, net.h);
        //image sized2 = resize_max(im, net.w);
        //image sized = crop_image(sized2, -((net.w - sized2.w)/2), -((net.h - sized2.h)/2), net.w, net.h);
        //resize_network(&net, sized.w, sized.h);
        layer l = net.layers[net.n-1];

        box *boxes = calloc(l.w*l.h*l.n, sizeof(box));
        float **probs = calloc(l.w*l.h*l.n, sizeof(float *));
        for(j = 0; j < l.w*l.h*l.n; ++j) probs[j] = calloc(l.classes + 1, sizeof(float *));

        float *X = sized.data;
        time=clock();
        network_predict(net, X); //where predictions occurs?
        // printf("%s: Predicted in %f seconds.\n", input, sec(clock()-time));
        get_region_boxes(l, im.w, im.h, net.w, net.h, thresh, probs, boxes, 0, 0, hier_thresh, 1);

        if (nms) do_nms_obj(boxes, probs, l.w*l.h*l.n, l.classes, nms);
        //else if (nms) do_nms_sort(boxes, probs, l.w*l.h*l.n, l.classes, nms);
        draw_detections(im, l.w*l.h*l.n, thresh, boxes, probs, names, alphabet, l.classes, &detects, bs);
        //printf("Number of drawn detections: %d\n",detects);

        if (0 == detects) {
            continue;
        }

        float *depthNormalized = NULL;
        if (NULL != depthFilePathList && depthFileCnt > i) {
            char *inputDepth = depthFilePathList[i];
            PGMImage pgmfile;
            getPGMfile(inputDepth, &pgmfile);

            printf("File(RGB)[%d]: %s\n", i, input);
            printf("  Size: [w x h is %d x %d]\n", im.w, im.h);
            printf("File(Depth)[%d]: %s\n", i, inputDepth);
            printf("  Depth Max = %d\n", pgmfile.maxVal);
            printf("  Size: [w x h is %d x %d]\n", pgmfile.width, pgmfile.height);

            depthNormalized = malloc(sizeof(float) * detects);
            

            printf("Detected boxes number = %d\n", detects);
            for (m = 0; m < detects; m++) {
                int j = (int) (im.h * bs[m].y);
                int i = (int) (im.w * bs[m].x);
                printf("  Box[%d]: Centroid [%d, %d] [xN, yN / w x h is %.6f, %.6f, %.6f x %.6f]\n",
                       m,
                       i, j,
                       bs[m].x, bs[m].y, bs[m].w, bs[m].h);
                if (m != detects) {
                    depthNormalized[m] = (float) (pgmfile.data[j][i].i) / pgmfile.maxVal;
                }
                printf("depthNormalized[%d] = %.6f\n", m, depthNormalized[m]);
            }
           
        }

        if(frame_buff>=10){
            //switch frames

            free(fw.frame1);
                free(fw.depth1);
                fw.frame1 = malloc(fw.size2*sizeof(box));
                fw.depth1 = malloc(fw.size2*sizeof(double));
                memcpy(fw.frame1,fw.frame2,fw.size2*sizeof(box));
                fw.size1 = fw.size2;
                memcpy(fw.depth1,fw.depth2,detects*sizeof(double));
            
                free(fw.frame2);
                free(fw.depth2);
                fw.frame2 = malloc(fw.size3*sizeof(box));
                fw.depth2 = malloc(fw.size3*sizeof(double));
                memcpy(fw.frame2,fw.frame3,fw.size3*sizeof(box));
                fw.size2 = fw.size3;
                memcpy(fw.depth2,fw.depth3,detects*sizeof(double));

                free(fw.frame3);
                free(fw.depth3);
                fw.frame3 = malloc(fw.size4*sizeof(box)); 
                fw.depth3 = malloc(fw.size4*sizeof(double));     
                memcpy(fw.frame3,fw.frame4,fw.size4*sizeof(box));
                fw.size3 = fw.size4;
                memcpy(fw.depth3,fw.depth4,detects*sizeof(double));

                free(fw.frame4);
                free(fw.depth4);
                fw.frame4 = malloc(fw.size5*sizeof(box));
                fw.depth4 = malloc(fw.size5*sizeof(double));
                memcpy(fw.frame4,fw.frame5,fw.size5*sizeof(box));
                fw.size4 = fw.size5;
                memcpy(fw.depth4,fw.depth5,detects*sizeof(double));

                free(fw.frame5);
                free(fw.depth5);
                fw.frame5 = malloc(fw.size6*sizeof(box));
                fw.depth5 = malloc(fw.size6*sizeof(double));
                memcpy(fw.frame5,fw.frame6,fw.size6*sizeof(box));
                fw.size5 = fw.size6;
                memcpy(fw.depth5,fw.depth6,detects*sizeof(double));

                free(fw.frame6);    
                free(fw.depth6);
                fw.frame6 = malloc(fw.size7*sizeof(box));
                fw.depth6 = malloc(fw.size7*sizeof(double));
                memcpy(fw.frame6,fw.frame7,fw.size7*sizeof(box));
                fw.size6 = fw.size7;
                memcpy(fw.depth6,fw.depth7,detects*sizeof(double));

                free(fw.frame7);
                free(fw.depth7);
                fw.frame7 = malloc(fw.size8*sizeof(box));
                fw.depth7 = malloc(fw.size8*sizeof(double));
                memcpy(fw.frame7,fw.frame8,fw.size8*sizeof(box));
                fw.size7 = fw.size8;
                memcpy(fw.depth7,fw.depth8,detects*sizeof(double));

                free(fw.frame8);
                free(fw.depth8);
                fw.frame8 = malloc(fw.size9*sizeof(box));
                fw.depth8 = malloc(fw.size9*sizeof(double));
                memcpy(fw.frame8,fw.frame9,fw.size9*sizeof(box));
                fw.size8 = fw.size9;
                memcpy(fw.depth8,fw.depth9,detects*sizeof(double));

                free(fw.frame9);
                free(fw.depth9);
                fw.frame9 = malloc(fw.size10*sizeof(box));
                fw.depth9 = malloc(fw.size10*sizeof(double));
                memcpy(fw.frame9,fw.frame10,fw.size10*sizeof(box));
                fw.size9 = fw.size10;
                memcpy(fw.depth9,fw.depth10,detects*sizeof(double));
           
                free(fw.frame10);
                free(fw.depth10);
                fw.size10 = detects;
                fw.frame10 = malloc(detects*sizeof(box));
                fw.depth10 = malloc(fw.size10*sizeof(double));
                memcpy(fw.frame10,bs,detects*sizeof(box));
                memcpy(fw.depth10,depthNormalized,detects*sizeof(double));

                printf("Successfully switched frames\n");
                out = cluster2(fw,fp,fp2);
        }
        else{
            ++frame_buff;
            if(frame_buff == 1){
            fw.size10 = detects;      
            fw.frame10 = malloc(fw.size10*sizeof(box)); 
            fw.depth10 = malloc(fw.size10*sizeof(double));
            printf("size10:%d\n",fw.size10);
            memcpy(fw.frame10,bs,detects*sizeof(box));
            memcpy(fw.depth10,depthNormalized,fw.size10*sizeof(double));
                        printf("init first frame\n");

            fw.frame9 = malloc(fw.size10*sizeof(box));
            fw.depth9 = malloc(fw.size10*sizeof(double));
            memcpy(fw.frame9,fw.frame10,fw.size10*sizeof(box));
            fw.size9 = fw.size10;
            memcpy(fw.depth9,fw.depth10,detects*sizeof(double));

            fw.frame8 = malloc(fw.size10*sizeof(box));
            fw.depth8 = malloc(fw.size10*sizeof(double));
            memcpy(fw.frame8,fw.frame9,fw.size9*sizeof(box));
            fw.size8 = fw.size9;
            memcpy(fw.depth8,fw.depth9,detects*sizeof(double));

            fw.frame7 = malloc(fw.size10*sizeof(box));
            fw.depth7 = malloc(fw.size10*sizeof(double));
            memcpy(fw.frame7,fw.frame8,fw.size8*sizeof(box));
            fw.size7 = fw.size8;
            memcpy(fw.depth7,fw.depth8,detects*sizeof(double));

            fw.frame6 = malloc(fw.size10*sizeof(box));
            fw.depth6 = malloc(fw.size10*sizeof(double));
            memcpy(fw.frame6,fw.frame7,fw.size7*sizeof(box));
            fw.size6 = fw.size7;
            memcpy(fw.depth6,fw.depth7,detects*sizeof(double));

            fw.frame5 = malloc(fw.size10*sizeof(box));
            fw.depth5 = malloc(fw.size10*sizeof(double));
            memcpy(fw.frame5,fw.frame6,fw.size6*sizeof(box));
            fw.size5 = fw.size6;
            memcpy(fw.depth5,fw.depth6,detects*sizeof(double));

            fw.frame4 = malloc(fw.size10*sizeof(box));
            fw.depth4 = malloc(fw.size10*sizeof(double));
            memcpy(fw.frame4,fw.frame5,fw.size5*sizeof(box));
            fw.size4 = fw.size5;
            memcpy(fw.depth4,fw.depth5,detects*sizeof(double));

            fw.frame3 = malloc(fw.size10*sizeof(box));
            fw.depth3 = malloc(fw.size10*sizeof(double));
            memcpy(fw.frame3,fw.frame4,fw.size4*sizeof(box));
            fw.size3 = fw.size4;
            memcpy(fw.depth3,fw.depth4,detects*sizeof(double));

            fw.frame2 = malloc(fw.size10*sizeof(box));
            fw.depth2 = malloc(fw.size10*sizeof(double));
            memcpy(fw.frame2,fw.frame3,fw.size3*sizeof(box));
            fw.size2 = fw.size3;
            memcpy(fw.depth2,fw.depth3,detects*sizeof(double));

            fw.frame1 = malloc(fw.size10*sizeof(box));
            fw.depth1 = malloc(fw.size10*sizeof(double));
            memcpy(fw.frame1,fw.frame2,fw.size2*sizeof(box));
            fw.size1 = fw.size2;
            memcpy(fw.depth1,fw.depth2,detects*sizeof(double));
            
        }

            else{
                free(fw.frame1);
                free(fw.depth1);
                fw.frame1 = malloc(fw.size2*sizeof(box));
                fw.depth1 = malloc(fw.size2*sizeof(double));
                memcpy(fw.frame1,fw.frame2,fw.size2*sizeof(box));
                fw.size1 = fw.size2;
                memcpy(fw.depth1,fw.depth2,detects*sizeof(double));
            
                free(fw.frame2);
                free(fw.depth2);
                fw.frame2 = malloc(fw.size3*sizeof(box));
                fw.depth2 = malloc(fw.size3*sizeof(double));
                memcpy(fw.frame2,fw.frame3,fw.size3*sizeof(box));
                fw.size2 = fw.size3;
                memcpy(fw.depth2,fw.depth3,detects*sizeof(double));

                free(fw.frame3);
                free(fw.depth3);
                fw.frame3 = malloc(fw.size4*sizeof(box)); 
                fw.depth3 = malloc(fw.size4*sizeof(double));     
                memcpy(fw.frame3,fw.frame4,fw.size4*sizeof(box));
                fw.size3 = fw.size4;
                memcpy(fw.depth3,fw.depth4,detects*sizeof(double));

                free(fw.frame4);
                free(fw.depth4);
                fw.frame4 = malloc(fw.size5*sizeof(box));
                fw.depth4 = malloc(fw.size5*sizeof(double));
                memcpy(fw.frame4,fw.frame5,fw.size5*sizeof(box));
                fw.size4 = fw.size5;
                memcpy(fw.depth4,fw.depth5,detects*sizeof(double));

                free(fw.frame5);
                free(fw.depth5);
                fw.frame5 = malloc(fw.size6*sizeof(box));
                fw.depth5 = malloc(fw.size6*sizeof(double));
                memcpy(fw.frame5,fw.frame6,fw.size6*sizeof(box));
                fw.size5 = fw.size6;
                memcpy(fw.depth5,fw.depth6,detects*sizeof(double));

                free(fw.frame6);    
                free(fw.depth6);
                fw.frame6 = malloc(fw.size7*sizeof(box));
                fw.depth6 = malloc(fw.size7*sizeof(double));
                memcpy(fw.frame6,fw.frame7,fw.size7*sizeof(box));
                fw.size6 = fw.size7;
                memcpy(fw.depth6,fw.depth7,detects*sizeof(double));

                free(fw.frame7);
                free(fw.depth7);
                fw.frame7 = malloc(fw.size8*sizeof(box));
                fw.depth7 = malloc(fw.size8*sizeof(double));
                memcpy(fw.frame7,fw.frame8,fw.size8*sizeof(box));
                fw.size7 = fw.size8;
                memcpy(fw.depth7,fw.depth8,detects*sizeof(double));

                free(fw.frame8);
                free(fw.depth8);
                fw.frame8 = malloc(fw.size9*sizeof(box));
                fw.depth8 = malloc(fw.size9*sizeof(double));
                memcpy(fw.frame8,fw.frame9,fw.size9*sizeof(box));
                fw.size8 = fw.size9;
                memcpy(fw.depth8,fw.depth9,detects*sizeof(double));

                free(fw.frame9);
                free(fw.depth9);
                fw.frame9 = malloc(fw.size10*sizeof(box));
                fw.depth9 = malloc(fw.size10*sizeof(double));
                memcpy(fw.frame9,fw.frame10,fw.size10*sizeof(box));
                fw.size9 = fw.size10;
                memcpy(fw.depth9,fw.depth10,detects*sizeof(double));
           
                free(fw.frame10);
                free(fw.depth10);
                fw.size10 = detects;
                fw.frame10 = malloc(detects*sizeof(box));
                fw.depth10 = malloc(fw.size10*sizeof(double));
                memcpy(fw.frame10,bs,detects*sizeof(box));
                memcpy(fw.depth10,depthNormalized,detects*sizeof(double));
            }
        printf("frame input init: %d\n",frame_buff);
        }

        /*
        int rgbFileCnt = 0;
        int depthFileCnt = 0; // if no depth image, it will be 0
        char** rgbFilePathList;
        char** depthFilePathList; // if no depth image, it will be NULL
        rgb image file:
            rgbFilePathList[i]
        depth image file:
            depthFilePathList[i]
        detects
        depthNormalized[i] // float value of depth data for the centroid of each box
        */

#if 0
        if(outfile){
            save_image(im, outfile);
            printf("Save output image at: %s\n", outfile);
        }
        else{
            save_image(im, "predictions");
#ifdef OPENCV
            cvNamedWindow("predictions", CV_WINDOW_NORMAL); 
            if(fullscreen){
                cvSetWindowProperty("predictions", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
            }
            show_image(im, "predictions");
            cvWaitKey(0);
            cvDestroyAllWindows();
#endif
        }
#endif

        free(depthNormalized);
        free_image(im);
        free_image(sized);
        free(boxes);
        free_ptrs((void **)probs, l.w*l.h*l.n);
        // if (filename) break;
    }

    for (i = 0; i < rgbFileCnt; i++) {
        free(rgbFilePathList[i]);
    }
    for (i = 0; i < depthFileCnt; i++) {
        free(depthFilePathList[i]);
    }
    fclose(fp);
        fclose(fp2);
}

void run_detector(int argc, char **argv)
{
    char *prefix = find_char_arg(argc, argv, "-prefix", 0);
    float thresh = find_float_arg(argc, argv, "-thresh", .24);
    float hier_thresh = find_float_arg(argc, argv, "-hier", .5);
    int cam_index = find_int_arg(argc, argv, "-c", 0);
    int frame_skip = find_int_arg(argc, argv, "-s", 0);
    int avg = find_int_arg(argc, argv, "-avg", 3);
    if(argc < 4){
        fprintf(stderr, "usage: %s %s [train/test/valid] [cfg] [weights (optional)]\n", argv[0], argv[1]);
        return;
    }
    char *gpu_list = find_char_arg(argc, argv, "-gpus", 0);
    char *outfile = find_char_arg(argc, argv, "-out", 0);
    int *gpus = 0;
    int gpu = 0;
    int ngpus = 0;
    if(gpu_list){
        printf("%s\n", gpu_list);
        int len = strlen(gpu_list);
        ngpus = 1;
        int i;
        for(i = 0; i < len; ++i){
            if (gpu_list[i] == ',') ++ngpus;
        }
        gpus = calloc(ngpus, sizeof(int));
        for(i = 0; i < ngpus; ++i){
            gpus[i] = atoi(gpu_list);
            gpu_list = strchr(gpu_list, ',')+1;
        }
    } else {
        gpu = gpu_index;
        gpus = &gpu;
        ngpus = 1;
    }

    int clear = find_arg(argc, argv, "-clear");
    int fullscreen = find_arg(argc, argv, "-fullscreen");
    int width = find_int_arg(argc, argv, "-w", 0);
    int height = find_int_arg(argc, argv, "-h", 0);
    int fps = find_int_arg(argc, argv, "-fps", 0);

    char *datacfg = argv[3];
    char *cfg = argv[4];
    char *weights = (argc > 5) ? argv[5] : 0;
    char *filename = (argc > 6) ? argv[6]: 0;
    if(0==strcmp(argv[2], "test")) test_detector(datacfg, cfg, weights, filename, thresh, hier_thresh, outfile, fullscreen);
    else if(0==strcmp(argv[2], "train")) train_detector(datacfg, cfg, weights, gpus, ngpus, clear);
    else if(0==strcmp(argv[2], "valid")) validate_detector(datacfg, cfg, weights, outfile);
    else if(0==strcmp(argv[2], "valid2")) validate_detector_flip(datacfg, cfg, weights, outfile);
    else if(0==strcmp(argv[2], "recall")) validate_detector_recall(cfg, weights);
    else if(0==strcmp(argv[2], "demo")) {
        list *options = read_data_cfg(datacfg);
        int classes = option_find_int(options, "classes", 20);
        char *name_list = option_find_str(options, "names", "data/names.list");
        char **names = get_labels(name_list);
        demo(cfg, weights, thresh, cam_index, filename, names, classes, frame_skip, prefix, avg, hier_thresh, width, height, fps, fullscreen);
    }
}
