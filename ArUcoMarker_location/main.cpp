#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "myfunction.h"

using namespace std;
using namespace cv;
using namespace aruco;

#define DICT_4X4_50 0
#define DICT_5X5_50 4
#define DICT_6X6_50 8

int main(){

    // file path
    string Image_File = "../data/MI8/IMG1.jpg";
    string Param_File = "../data/MI8/calibresult.txt";
    string Config_File  = "../Configs/detector_params.yml";

    // measurement data
    double theta = 90.0 * PI / 180; // 相机俯仰角，水平朝前为0度，竖直向下为90度
    double yaw = 0.0; // 无人机航偏角
    Vec2d GPSuav(114.351715,30.540224); // 无人机经纬度
    Vec3d  tcuav(0, 0, 0); // 相机坐标到无人机坐标的平移量

    // aruco parameters
    int dictionaryId = DICT_6X6_50;
    bool showRejected = false;
    double markerLength = 201;
    Ptr<DetectorParameters> detectorParams = DetectorParameters::create();
    if(!readDetectorParameters(Config_File, detectorParams)){
        cout << "failed to read configure file" << endl;
        return -1;
    }

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
    vector<int> ids;
    vector<vector<Point2f>> corners, rejected;
    Ptr<Dictionary> dictionary = getPredefinedDictionary(PREDEFINED_DICTIONARY_NAME(dictionaryId));
    detectMarkers(img, dictionary, corners, ids, detectorParams, rejected);
    drawDetectedMarkers(img, corners, ids);
    if(showRejected){
        drawDetectedMarkers(img, rejected, noArray(), Scalar(0, 0, 255));
    }
    Point orin(img.cols/2, img.rows/2);
    circle(img, orin, 20, Scalar(0,0,255), -1); // 显示坐标原点
    namedWindow("结果图", 0);
    imshow("结果图", img);
    cout << "检测到ArUco marker数量： " << ids.size() << endl;

    // estimate pose
    vector<Vec3d> rvecs, tvecs;
    estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);

    // calculate coordinate of target respected to UAV
    Mat Pc = Mat(3, 1, CV_64FC1, Scalar::all(0)); // 二维码中心点的相机坐标
    Mat Pu = Mat(3, 1, CV_64FC1, Scalar::all(0)); // 二维码中心点的无人机坐标
    Vec2d GPStar;
    double dist; // 目标到无人机的水平距离
    double angle; // 目标相对于无人机坐标系的角度
    double azimuth; // 目标相对于无人机的方位角
    for(int i = 0;i < ids.size();i++){
        Pc = Mat(tvecs[i]);
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
        cout << "相机坐标(mm)\n" << Pc << endl;
        cout << "无人机坐标(mm)\n" << Pu << endl;
        cout << "距离： " << dist << endl;
        cout << "方位角： " << azimuth << endl;
        cout << "无人机经纬度： " << Mat(GPSuav) << endl;
        cout << "目标经纬度： " << Mat(GPStar) << endl;
    }

    waitKey(0);

    return 0;
}
