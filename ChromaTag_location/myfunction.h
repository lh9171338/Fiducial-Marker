//
// Created by lihao on 19-3-5.
//

#ifndef CHROMATAG_LOCATION_MYFUNCTION_H
#define CHROMATAG_LOCATION_MYFUNCTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <fstream>
#include <cmath>

using namespace cv;
using namespace std;
using namespace aruco;

#define PI  3.1415926


template<typename T>
void matread(
        ifstream	&fin,
        Mat			&mat
)
{
    int rows = mat.rows;
    int cols = mat.cols;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            fin >> mat.at<T>(i, j);
        }
    }
}
template<typename T>
void matwrite(
        ofstream	&fout,
        Mat			&mat
)
{
    int rows = mat.rows;
    int cols = mat.cols;
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            fout << mat.at<T>(i, j) << " ";
        }
        fout << endl;
    }
}

void readCameraParameters(
        string  filename,
        Mat     &cameraMatrix,
        Mat     &distCoeffs
        )
{
    ifstream fin(filename);
    string str;
    fin >> str;
    matread<double>(fin, cameraMatrix);
    fin >> str;
    matread<double>(fin, distCoeffs);
}

void getdistance(
    Vec2d   Pa,
    Vec2d   Pb,
    double  &dist,
    double  &azimuth
    )
{
    // 参数
    const double R = 6371393 * 1000.0; // 地球平均半径，单位mm

    // 经纬度转弧度单位
    double LonA = Pa[0] * PI / 180.0;
    double LatA = Pa[1] * PI / 180.0;
    double LonB = Pb[0] * PI / 180.0;
    double LatB = Pb[1] * PI / 180.0;

    // 近似计算
    double r = R * cos(LatA);
    double dx = R * (LatB - LatA);
    double dy = r * (LonB - LonA);
    dist = sqrt(dx * dx + dy * dy);
    azimuth = atan(dy / dx) * 180 / PI;
    if(dx < 0){ // 第二、三象限
        azimuth += 180;
    }
    else if(dy < 0){ // 第四象限
        azimuth += 360;
    }
}

template<typename T>
void getdistance(
    Mat      &P,
    double   &dist,
    double   &angle
)
{
    double dx = P.at<T>(0);
    double dy = P.at<T>(1);
    dist = sqrt(dx * dx + dy * dy);
    angle = atan(dy / dx) * 180 / PI;
    if(dx < 0){ // 第二、三象限
        angle += 180;
    }
    else if(dy < 0){ // 第四象限
        angle += 360;
    }
}

void getcoordinate(
    Vec2d   Pa,
    double  dist,
    double  azimuth,
    Vec2d   &Pb
    )
{
    // 参数
    const double R = 6371393 * 1000.0; // 地球平均半径，单位mm

    // 经纬度转弧度单位
    double LonA = Pa[0] * PI / 180.0;
    double LatA = Pa[1] * PI / 180.0;
    azimuth = azimuth * PI / 180.0;

    // 近似计算
    double r = R * cos(LatA);
    double dx = dist * cos(azimuth);
    double dy = dist * sin(azimuth);
    double LonB = LonA + dy / r;
    double LatB = LatA + dx / R;
    Pb[0] = LonB * 180 / PI;
    Pb[1] = LatB * 180 / PI;
}


#endif //CHROMATAG_LOCATION_MYFUNCTION_H
