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
#include "apriltag_pose.h"

#include "myAprilTag.h"
#include "myfunction.h"

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

int main(){

    // file path
    string Image_File = "../data/MI8/IMG1.jpg";
    string Param_File = "../data/MI8/calibresult.txt";

    // measurement data
    double theta = 90.0 * PI / 180; // 相机俯仰角，水平朝前为0度，竖直向下为90度
    double yaw = 0.0; // 无人机航偏角
    Vec2d GPSuav(114.351715,30.540224); // 无人机经纬度
    Vec3d  tcuav(0, 0, 0); // 相机坐标到无人机坐标的平移量

    // apriltag parameters
    int family = TAG36H11;
    float quad_decimate = 2.0;
    float quad_sigma = 0.0;
    int nthreads = 1;
    bool refine_edges = true;
    double tagsize = 161;

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

    // camera parameters
    Mat cameraMatrix = Mat(3, 3, CV_64FC1, Scalar::all(0)); // 内参矩阵
    Mat distCoeffs = Mat(1, 5, CV_64FC1, Scalar::all(0)); // 畸变系数k1,k2,p1,p2,k3
    Mat Rcuav = Mat(3, 3, CV_64FC1, Scalar::all(0)); // 相机坐标到无人机坐标的旋转矩阵
    Mat Tcuav = Mat(tcuav); // 相机坐标到无人机坐标的平移向量
    readCameraParameters(Param_File, cameraMatrix, distCoeffs);
    Rcuav.at<double>(0,0) = 1;
    Rcuav.at<double>(1,1) = sin(theta);
    Rcuav.at<double>(1,2) = -cos(theta);
    Rcuav.at<double>(2,1) = cos(theta);
    Rcuav.at<double>(2,2) = sin(theta);
    cout << "内参矩阵:\n" << cameraMatrix << endl;
    cout << "畸变系数:\n" << distCoeffs << endl;

    // read image
    Mat img = imread(Image_File);
    if (img.empty()) {
        cout << "failed to read image" << endl;
        return -1;
    }
    cout << "图像大小： " << img.rows << " * " << img.cols << endl;
    namedWindow("原图", 0);
    imshow("原图", img);

    // detection
    Mat gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    image_u8_t im = { .width = gray.cols, // Make an image_u8_t header for the Mat data
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
    };
    zarray_t *detections = apriltag_detector_detect(td, &im);
    drawDetectedMarkers(img, detections, 5, Scalar(0, 0, 255));
    Point orin(img.cols/2, img.rows/2);
    circle(img, orin, 20, Scalar(0,0,255), -1); // 显示坐标原点
    namedWindow("结果图", 0);
    imshow("结果图", img);
    cout << "检测到AprilTag数量： " << zarray_size(detections) << endl;

    // estimate pose
    apriltag_detection_info_t info;
    info.tagsize = tagsize;
    info.fx = cameraMatrix.at<double>(0,0);
    info.fy = cameraMatrix.at<double>(1,1);
    info.cx = cameraMatrix.at<double>(0,2);
    info.cy = cameraMatrix.at<double>(1,2);
    vector<Mat> tvecs;
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        info.det = det;
        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);
        Mat tvec = Mat(3, 1, CV_64FC1, pose.t->data);
        tvecs.push_back(tvec);
    }

    // calculate coordinate of target respected to UAV
    Mat Pc = Mat(3, 1, CV_64FC1, Scalar::all(0)); // 二维码中心点的相机坐标
    Mat Pu = Mat(3, 1, CV_64FC1, Scalar::all(0)); // 二维码中心点的无人机坐标
    Vec2d GPStar;
    double dist; // 目标到无人机的水平距离
    double angle; // 目标相对于无人机坐标系的角度
    double azimuth; // 目标相对于无人机的方位角
    for(int i = 0;i < zarray_size(detections);i++){
        Pc = tvecs[i];
        Pu = Rcuav * Pc + Tcuav;
        getdistance<double>(Pu, dist, angle);
        azimuth = angle + 90.0 + yaw;
        if(azimuth >= 360){
            azimuth -= 360;
        }
        else if(azimuth <= -360){
            azimuth += 360;
        }
        getcoordinate(GPSuav, dist, azimuth, GPStar);

        cout << "ArUco marker： " << i + 1 << endl;
        cout << "相机坐标(mm)\n" << Pu << endl;
        cout << "无人机坐标(mm)\n" << Pu << endl;
        cout << "距离： " << dist << endl;
        cout << "方位角： " << azimuth << endl;
        cout << "无人机经纬度： " << Mat(GPSuav) << endl;
        cout << "目标经纬度： " << Mat(GPStar) << endl;
    }

    waitKey(0);

    // desodestory
    zarray_destroy(detections);
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
