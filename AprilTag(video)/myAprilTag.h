//
// Created by lihao on 19-3-5.
//

#ifndef _MYAPRILTAG_H_
#define _MYAPRILTAG_H_

#include <iostream>
#include <opencv2/opencv.hpp>

#include "apriltag.h"

using namespace std;
using namespace cv;

void drawDetectedMarkers(
    Mat                         &img,
    zarray_t                    *detections,
    int                         thickness = 1,
    const Scalar                color = Scalar(0,0,255)
)
{
    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        line(img, Point(det->p[0][0], det->p[0][1]),
             Point(det->p[1][0], det->p[1][1]),
             color, thickness);
        line(img, Point(det->p[0][0], det->p[0][1]),
             Point(det->p[3][0], det->p[3][1]),
             color, thickness);
        line(img, Point(det->p[1][0], det->p[1][1]),
             Point(det->p[2][0], det->p[2][1]),
             color, thickness);
        line(img, Point(det->p[2][0], det->p[2][1]),
             Point(det->p[3][0], det->p[3][1]),
             color, thickness);

        String text = to_string(det->id);
        int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontscale = 1.0;
        int baseline;
        Size textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
        putText(img, text, Point(det->c[0]-textsize.width/2, det->c[1]+textsize.height/2),
                fontface, fontscale, color, thickness);
    }
}



#endif //_MYAPRILTAG_H_
