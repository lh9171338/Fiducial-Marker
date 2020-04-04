#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;
using namespace aruco;


//"{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
//"DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
//"DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
//"DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
//"DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
//"{v        |       | Input from video file, if ommited, input comes from camera }"
//"{ci       | 0     | Camera id if input doesnt come from video (-v) }"
//"{c        |       | Camera intrinsic parameters. Needed for camera pose }"
//"{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
//"{dp       |       | File of marker detector parameters }"
//"{r        |       | show rejected candidates too }"
//"{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,"
//"CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}";

#define DICT_4X4_50 0
#define DICT_5X5_50 4
#define DICT_6X6_50 8

bool readDetectorParameters(string filename,Ptr<DetectorParameters> &params);


int main()
{
    // file path
    string Config_File  = "../Configs/detector_params.yml";
    string Video_File = "";

    // parameters
    int dictionaryId = DICT_6X6_50;
    bool showRejected = false;
    Ptr<DetectorParameters> detectorParams = DetectorParameters::create();
    if(!readDetectorParameters(Config_File, detectorParams)){
        cout << "failed to read configure file" << endl;
        return -1;
    }

    // dictionary
    Ptr<Dictionary> dictionary = getPredefinedDictionary(PREDEFINED_DICTIONARY_NAME(dictionaryId));

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
    Mat frame;
    while(video.read(frame)){
        if(!frame.empty()){
            double tick = (double)getTickCount();
            // detection
            vector<int> ids;
            vector<vector<Point2f>> corners, rejected;
            detectMarkers(frame, dictionary, corners, ids, detectorParams, rejected);
            // calculate detection time
            double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
            totalTime += currentTime;
            totalIterations++;
            if(totalIterations % 30 == 0){
                cout << "Detection Time = " << currentTime * 1000 << " ms "
                     << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
            }
            // draw results
            drawDetectedMarkers(frame, corners, ids);
            if(showRejected){
                drawDetectedMarkers(frame, rejected, noArray(), Scalar(0, 0, 255));
            }
            //namedWindow("frame", 0);
            imshow("frame", frame);

        }

        if(waitKey(10) == 13){ // Enter
            break;
        }
    }

    return 0;
}

bool readDetectorParameters(
        string                  filename,
        Ptr<DetectorParameters> &params
)
{
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}
