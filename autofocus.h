#pragma once
#ifndef PLAYER_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define PLAYER_H

#include "stdafx.h"
#include <windows.h>
#include "tchar.h"
#include <iostream>
#include <iomanip>
#include <fstream>

#include "C:\opencv-4.2.0\opencv\build\include\opencv2\opencv.hpp"

//#include "opencv\sources\include\opencv2\opencv.hpp"
//#include "opencv\build\include\opencv2\opencv.hpp"
//
//#include "opencv/build/include/opencv2/core/core.hpp"
//#include "opencv\build\include\opencv2\imgproc\imgproc_c.h"
//#include "opencv\build\include\opencv2\imgcodecs.hpp"
//#include "opencv/build/include/opencv2/core/mat.hpp"

//public:
//	CComboBox CComboCamera;


//Functions from ctrl_obj.cpp that we need
void __init__();
void mov_rel(double mmToMove);
void close_motor();


//Functions from cam_obj.cpp that we need
//void init_cam();
//void binning();
//void start_video();
//void end_video();
//cv::Mat video_frame(CString FilePath);
// 
//Functions from sensor.cpp that we need
int sharpness(cv::Mat img, double scale, std::ofstream& outputFile);

#endif
