#include <iostream>
#include "opencv2/opencv.hpp"

#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"

#include "myAprilTag.h"

using namespace std;
using namespace cv;

//getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
//getopt_add_bool(getopt, 'd', "debug", 1, "Enable debugging output (slow)");
//getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
//getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
//getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
//getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
//getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
//getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

#define TAG16H5     0
#define TAG25H9     1
#define TAG36H11    2

int main()
{
    // file path
    string Video_File = "";

    // apriltag parameter
    int family = TAG36H11;
    float quad_decimate = 2.0;
    float quad_sigma = 0.0;
    int nthreads = 1;
    bool refine_edges = true;

    // Initialize tag detector with options
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_family_t *tf = NULL;
    switch(family){
        case TAG16H5:
            tf = tag16h5_create();
            break;
        case TAG25H9:
            tf = tag25h9_create();
            break;
        case TAG36H11:
            tf = tag36h11_create();
            break;
        default:
            cout << "Unrecognized tag family name" << endl;
            return -1;
    }
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = quad_decimate;
    td->quad_sigma = quad_sigma;
    td->nthreads = nthreads;
    td->refine_edges = refine_edges;

    // camera
    VideoCapture video;
    if(Video_File.empty()){
        video.open(0);
    }
    else{
        video.open(Video_File);
    }
    if(!video.isOpened()){
        cout << "failed to open video" << endl;
        return -1;
    }
    double totalTime = 0;
    int totalIterations = 0;
    Mat frame, gray;
    while(video.read(frame)){
        if(!frame.empty()){
            double tick = (double)getTickCount();

            // detection
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            image_u8_t im = { .width = gray.cols, // Make an image_u8_t header for the Mat data
                    .height = gray.rows,
                    .stride = gray.cols,
                    .buf = gray.data
            };
            zarray_t *detections = apriltag_detector_detect(td, &im);

            // calculate detection time
            double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
            totalTime += currentTime;
            totalIterations++;
            if(totalIterations % 30 == 0){
                cout << "Detection Time = " << currentTime * 1000 << " ms "
                     << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
            }
            //cout << "检测到AprilTag数量： " << zarray_size(detections) << endl;

            // draw results
            drawDetectedMarkers(frame, detections, 2, Scalar(0, 0, 255));
            zarray_destroy(detections);
            namedWindow("frame", 0);
            imshow("frame", frame);

        }

        int key = waitKey(10);
        if(key == 32) // Space
        {
            imwrite("../result/result.jpg", frame);
        }
        else if(key == 13){ // Enter
            break;
        }
    }


    // destory variable
    apriltag_detector_destroy(td);
    switch(family){
        case TAG16H5:
            tag16h5_destroy(tf);
            break;
        case TAG25H9:
            tag25h9_destroy(tf);
            break;
        case TAG36H11:
            tag36h11_destroy(tf);
            break;
        default:
            cout << "Unrecognized tag family name" << endl;
            return -1;
    }

    return 0;
}

