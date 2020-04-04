//system
#include <stdio.h>
#include <iostream>
#include <string>
#include <signal.h>
#include <libgen.h>
#include <dirent.h>
#include <algorithm>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>

//Utilities
#include "JMD_Utils_Timer.hpp"
#include "JMD_Utils_Writer.hpp"
#include "JMD_Utils_Options.hpp"

//ChromaTag
#include "JMD_ChromaTag.hpp"

//my header file
#include "myChromaTag.h"


using namespace JMD;
using namespace cv;
using namespace std;


int main()
{
	// objects
    JMD_ChromaTag myChromaTag;
	JMD_ChromaTag_Settings mySettings;

	// file path
	string Config_File = "../Configs/chromatags_config.yml";
    string Video_File = "";
    //string Video_File = "/home/lihao/workspace/无人机视频图片/精灵/DJI_0605.mp4";

	// initial
	cout << "init ChromaTag" << endl;
	if(myChromaTag.Init(&mySettings, Config_File))
	{
        cout << "failed to intial ChromaTag" << endl;
		return -1;
	}

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
            //vector<Point> centers;
            //detectChromaTag(myChromaTag, frame, centers);
            vector<vector<Point2f>> corners_list;
            findChromaTagCorners(myChromaTag, frame, corners_list);
            // calculate detection time
            double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
            totalTime += currentTime;
            totalIterations++;
            if(totalIterations % 30 == 0){
                cout << "Detection Time = " << currentTime * 1000 << " ms "
                     << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
            }

            // draw result
            //drawChromaTagCenters(frame, centers);
            drawChromaTagCorners(frame, corners_list);
            namedWindow("video", 0);
            imshow("video", frame);

            if(corners_list.size() > 2) // Space
            {
                imwrite("../result/result.jpg", frame);
            }
        }

        if(waitKey(10) == 13) // Enter
        {
            break;
        }
    }

	return 0;
}
