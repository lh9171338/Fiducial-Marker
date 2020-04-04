//
// Created by lihao on 19-3-5.
//

#ifndef CHROMATAG_LOCATION_MYCHROMATAG_H
#define CHROMATAG_LOCATION_MYCHROMATAG_H

#include <opencv2/opencv.hpp>
#include "JMD_ChromaTag.hpp"

using namespace cv;


namespace JMD{

bool detectChromaTag(
    JMD_ChromaTag   &myChromaTag,
    Mat             &img,
    vector<Point>   &centers,
    int             thresh = 0
)
{
    // detect
    myChromaTag.Detect_BGRToLAB(img);
    myChromaTag.Decode(img);
    myChromaTag.Pose();
    JMD_ChromaTag_Collection *myDetections = myChromaTag.Detections();

    // find center
    centers.clear();
    for(JMD_ChromaTag_Collection::iterator it = myDetections->begin();it != myDetections->end();it++)
    {
        JMD_ChromaTag_Detection *curr_detection = *it;
        if(!curr_detection->IsActive)
        {
            break;
        }

        TagBorderSet curr_borders = curr_detection->TagBorders;

        //outer ring
        TagBorderRing *curr_ring = curr_borders.back();

        vector<Point2f> corners;
        Point2f center(0,0);
        for(TagBorderRing::iterator sit = curr_ring->begin();sit != curr_ring->end();sit++)
        {
            TagBorderSegment *curr_seg = *sit;

            //point values
            float U = curr_seg->myLine.Point1()->U();
            float V = curr_seg->myLine.Point1()->V();
            corners.push_back(Point2f(U, V));
            center += Point2f(U, V);
        }

        // 筛选检测结果
        float width = norm(Mat(corners[0]), Mat(corners[1]));
        float height = norm(Mat(corners[1]), Mat(corners[2]));
        float area = width*height;
//        cout << "area: " << area << endl;
        if(area > thresh){
            center.x /= corners.size();
            center.y /= corners.size();
            centers.push_back(center);
        }
    }

    return centers.size() > 0;
}

bool findChromaTagCorners(
    JMD_ChromaTag               &myChromaTag,
    Mat                         &img,
    vector<vector<Point2f>>     &corners_list,
    int                         thresh = 0
)
{
    // detect
    myChromaTag.Detect_BGRToLAB(img);
    myChromaTag.Decode(img);
    myChromaTag.Pose();
    JMD_ChromaTag_Collection *myDetections = myChromaTag.Detections();

    // find center
    corners_list.clear();
    for(JMD_ChromaTag_Collection::iterator it = myDetections->begin();it != myDetections->end();it++)
    {
        JMD_ChromaTag_Detection *curr_detection = *it;
        if(!curr_detection->IsActive)
        {
            break;
        }

        TagBorderSet curr_borders = curr_detection->TagBorders;

        //outer ring
        TagBorderRing *curr_ring = curr_borders.back();

        vector<Point2f> corners;
        for(TagBorderRing::iterator sit = curr_ring->begin();sit != curr_ring->end();sit++)
        {
            TagBorderSegment *curr_seg = *sit;

            //point values
            float U = curr_seg->myLine.Point1()->U();
            float V = curr_seg->myLine.Point1()->V();
            corners.push_back(Point2f(U, V));
        }

        // 筛选检测结果
        float width = norm(Mat(corners[0]), Mat(corners[1]));
        float height = norm(Mat(corners[1]), Mat(corners[2]));
        float area = width*height;
//        cout << "area: " << area << endl;
        if(area > 1000){
            corners_list.push_back(corners);
        }
    }

    return corners_list.size() > 0;
}

void drawChromaTagCenters(
    Mat             &img,
    vector<Point>   &centers,
    int             radius = 10,
    const Scalar    color = Scalar(0,0,255)
)
{
    for(vector<Point>::iterator it = centers.begin();it != centers.end();it++)
    {
        Point point = *it;
        circle(img, point, radius, color, -1);
    }
}

void drawChromaTagCorners(
    Mat                         &img,
    vector<vector<Point2f>>     &corners_list,
    int                         radius = 10,
    const Scalar                color = Scalar(0,0,255)
)
{
    for(vector<vector<Point2f>>::iterator it = corners_list.begin();it != corners_list.end();it++)
    {
        vector<Point2f> corners = *it;
        for(vector<Point2f>::iterator sit = corners.begin();sit != corners.end();sit++)
        {
            Point2f point = *sit;
            circle(img, point, radius, color, -1);
        }
    }
}


};


#endif //CHROMATAG_LOCATION_MYCHROMATAG_H
