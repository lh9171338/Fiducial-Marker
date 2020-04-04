#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <signal.h>
#include <libgen.h>
#include <dirent.h>
#include <algorithm>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cmath>

//Utilities
#include "JMD_Utils_Timer.hpp"
#include "JMD_Utils_Writer.hpp"
#include "JMD_Utils_Options.hpp"

//ChromaTag
#include "JMD_ChromaTag.hpp"

//my header file
#include "myChromaTag.h"
#include "myfunction.h"

using namespace JMD;
using namespace cv;
using namespace std;

int main(){

	// 文件路径
	string Image_File = "../data/M210/DJI_N90_300.JPG";

	string Param_File = "../data/M210/calibresult.txt"; // 相机内参文件
    string Config_File  = "../Configs/chromatags_config.yml"; // ChromaTag配置文件

    // 测量数据
    double theta = 90.0 * PI / 180; // 相机俯仰角，水平朝前为0度，竖直向下为90度
    double yaw = 80.0; // 无人机航偏角
    Vec2d GPSuav(114.351715, 30.540224); // 无人机经纬度
    Vec3d tcuav(0, -140, 0);  // 相机坐标到无人机坐标的平移向量
    Size squareSize = Size(153, 153);  // 网格尺寸，单位mm

    // ChromaTag
    JMD_ChromaTag myChromaTag;
    JMD_ChromaTag_Settings mySettings;
    cout << "初始化ChromaTag" << endl;
    if(myChromaTag.Init(&mySettings, Config_File))
    {
        cout << "初始化ChromaTag失败" << endl;
        return -1;
    }

    // 相机参数
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

	// 读取图像
	Mat img = imread(Image_File);
	if (img.empty()) {
		cout << "读取图像失败" << endl;
		return -1;
	}
    cout << "图像大小： " << img.rows << " * " << img.cols << endl;
	namedWindow("原图", 0);
	imshow("原图", img);

	// 提取角点
	vector<Point2f> corners; // 4个点按顺时针顺序排列
    vector<vector<Point2f>> corners_list;
	if (!findChromaTagCorners(myChromaTag, img, corners_list)){
		cout << "没有检测到二维码标志" << endl;
		return -1;
	}
	else{
		cout << "二维码标志数量： " << corners_list.size() << endl;
        // 显示坐标原点
        Point orin(img.cols/2, img.rows/2);
        circle(img, orin, 10, Scalar(0,0,255), -1);
		// 显示二维码角点
		drawChromaTagCorners(img, corners_list, 20, Scalar(0,0,255));
		namedWindow("角点", 0);
		imshow("角点", img);
	}

	// PnP求解相机外参
    vector<Point3f> objectPoints; // 角点的世界坐标，4个点要按顺时针顺序排列
    objectPoints.push_back(Point3f(0,0,0));
    objectPoints.push_back(Point3f(squareSize.width,0,0));
    objectPoints.push_back(Point3f(squareSize.width,squareSize.height,0));
    objectPoints.push_back(Point3f(0,squareSize.height,0));
	vector<Mat> rvecs; // 旋转向量
    vector<Mat> tvecs; // 平移向量
    for(vector<vector<Point2f>>::iterator it = corners_list.begin();it != corners_list.end();it++){
        vector<Point2f> corners = *it;
        Mat rvec, tvec;
        if (!solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec)) {
            cout << "PnP求解失败" << endl;
            return -1;
        }
        rvecs.push_back(rvec);
        tvecs.push_back(tvec);
    }

    // 计算目标坐标
    Mat Pw = Mat(Vec3d(squareSize.width/2.0, squareSize.height/2.0, 0)); // 二维码中心点的世界坐标
    Mat Pc = Mat(3, 1, CV_64FC1, Scalar::all(0)); // 二维码中心点的相机坐标
    Mat Pu = Mat(3, 1, CV_64FC1, Scalar::all(0)); // 二维码中心点的无人机坐标
    Vec2d GPStar;
    double dist; // 目标到无人机的水平距离
    double angle; // 目标相对于无人机坐标系的角度
    double azimuth; // 目标相对于无人机的方位角
    for(int i = 0;i < rvecs.size();i++){
        Mat rvec = rvecs[i];
        Mat tvec = tvecs[i];
        Mat rotationMatrix;
        Rodrigues(rvec, rotationMatrix);
        Pc = rotationMatrix * Pw + tvec;
        Pu = Rcuav * Pc + Tcuav;
        getdistance<double>(Pu, dist, angle);
        azimuth = angle + 90.0 + yaw; // 目标的方位角
        if(azimuth >= 360){
            azimuth -= 360;
        }
        else if(azimuth <= -360){
            azimuth += 360;
        }
        getcoordinate(GPSuav, dist, azimuth, GPStar);

        cout << "二维码标志： " << i + 1 << endl;
//        cout << "旋转向量\n" << rvec << endl;
//        cout << "旋转矩阵\n" << rotationMatrix << endl;
//        cout << "平移向量(mm)\n" << tvec << endl;
        cout << "世界坐标(mm)\n" << Pw << endl;
        cout << "无人机坐标(mm)\n" << Pu << endl;
        cout << "距离： " << dist << endl;
        cout << "方位角： " << azimuth << endl;
        cout << "无人机经纬度： " << Mat(GPSuav) << endl;
        cout << "目标经纬度： " << Mat(GPStar) << endl;
    }

    waitKey(0);

	return 0;
}