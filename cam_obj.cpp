//#include <iostream>
//#include "ASICamera2.h"
//#include <Windows.h>
//#include <atlstr.h>
////#include "opencv/highgui.h"
//
//
//#include "opencv2/imgcodecs.hpp"
////#include "opencv/build/include/opencv2/core/core.hpp"
////#include "opencv\build\include\opencv2\imgproc\imgproc_c.h"
////#include "opencv\build\include\opencv2\imgcodecs.hpp"
////#include "opencv/build/include/opencv2/core/mat.hpp"
//
//#include "autofocus.h"
//
//#define _CRT_SECURE_NO_WARNINGS
//
//using namespace std;
//
//struct ConnectedCam {
//	HANDLE Thr_Display, Thr_Snap, Thr_CapVideo;
//	// CamStatus Status;
//	int iCtrlNum;
//	//		bool bAvail;
//	ASI_CONTROL_CAPS* pControlCaps;
//	ASI_CAMERA_INFO* pASICameraInfo;
//	ASI_SUPPORTED_MODE* pASISupportedMode;
//	bool* pRefreshedIndex;
//	IplImage* pRgb;
//	IplImage* pTempImg;
//	IplImage* pTempImgScaled;
//	int iSnapTime;
//	ASI_IMG_TYPE ImageType;
//	int width, height;
//	bool bSnap;
//	float fOldScale;
//	bool bNewImg;
//	bool bSnapContinuous;
//	LONG lTrigOutputADelay;
//	LONG lTrigOutputADuration;
//	bool bHighLevelValidA;
//	LONG lTrigOutputBDelay;
//	LONG lTrigOutputBDuration;
//	bool bHighLevelValidB;
//};
//ConnectedCam ConnectCamera[ASICAMERA_ID_MAX];
////unsigned char img_data[1228800]; //480 * 640, image size for ASI_IMG_RAW8
//
//
////int main()
////{
//
////	int iNumofConnectCameras = ASIGetNumOfConnectedCameras();
////	if (iNumofConnectCameras > 0) {
////		cout << "Connected to the camera(s)";
////	}
////	else cout << "Failed to connect to the camera(s)";
////
////	ASIOpenCamera(0);
////	ASIInitCamera(0);
//////	ASISetControlValue(0, ASI_HIGH_SPEED_MODE, 1, ASI_FALSE); //Sets to high-speed mode
//////	ASISetROIFormat(0, 640, 480, 2, ASI_IMG_RAW8);
////	ASIStartVideoCapture(0);
//
//	//int width, height, bin;
//	//long lBuffSize;
//	//ASI_IMG_TYPE image_type;
//
//	//ASIGetROIFormat(0, &width, &height, &bin, &image_type);
//
//	//const long size = width * height;
//	//if (ASIGetVideoData(0, img_data, size, -1) == ASI_SUCCESS)
//	//{
//	//	cout << "\nSuccess\n";
//	//};
//	//cv::Mat img = cv::Mat(height, width, CV_8U, img_data);
//	//cv::imwrite("C:\\Users\\Thomas\\Desktop\\HVITiltedCameraImages\\test\\img100.png", img);
//
//	//ASIStopVideoCapture(0);
//	//if (ASICloseCamera(0) == ASI_SUCCESS)
//	//{
//	//	cout << "closed";
//	//}
//
////}
//
//
//void init_cam() {
//	int iNumofConnectCameras = ASIGetNumOfConnectedCameras();
//	if (iNumofConnectCameras > 0) {
//		cout << "Connected to the camera(s)";
//	}
//	else cout << "Failed to connect to the camera(s)";
//
//	ASIOpenCamera(0);
//	ASIInitCamera(0);
//
//	//SET AUTO-EXPOSURE
//	//ASISetControlValue(0, ASI_EXPOSURE,something, ASI_TRUE)
//	//SET AUTO-WHITE BALANCE ... SEE PYTHON
//	//ASISetControlValue(0,ASI_WB,something,ASI_TRUE)
//	//High-speed mode
//	ASISetControlValue(0, ASI_HIGH_SPEED_MODE, 1, ASI_FALSE);
//
//	int width, height, bin;
//	long lBuffSize;
//	ASI_IMG_TYPE image_type;
//
//	ASIGetROIFormat(0, &width, &height, &bin, &image_type);
//
//	ConnectCamera[0].ImageType = image_type;
//	ConnectCamera[0].width = width;
//	ConnectCamera[0].height = height;
//}
//
//void binning() {
//	ASISetROIFormat(0, 640, 480, 2, ASI_IMG_RAW8);
//
//	int width, height, bin;
//	long lBuffSize;
//	ASI_IMG_TYPE image_type;
//	ASIGetROIFormat(0, &width, &height, &bin, &image_type);
//	ConnectCamera[0].ImageType = image_type;
//	ConnectCamera[0].width = width;
//	ConnectCamera[0].height = height;
//
//	//switch (image_type)
//	//{
//	//case ASI_IMG_Y8:
//	//case ASI_IMG_RAW8:
//	//	ConnectCamera[0].pRgb = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1); //pRgb is _lplImage* or void * 640x480 GRAY, 8 depth, everything you want
//	//	lBuffSize = width * height;
//	//	break;
//
//	//case ASI_IMG_RGB24:
//	//	ConnectCamera[0].pRgb = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
//	//	lBuffSize = width * height * 3;
//	//	break;
//
//	//case ASI_IMG_RAW16:
//	//	ConnectCamera[0].pRgb = cvCreateImage(cvSize(width, height), IPL_DEPTH_16U, 1);
//	//	lBuffSize = width * height * 2;
//	//	break;
//	
//	ConnectCamera[0].pTempImg = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1); //pTempImg is all the same things as pRgb
//
//}
//
//void start_video() {
//	ASIStartVideoCapture(0);
//}
//
//void end_video() {
//	ASIStopVideoCapture(0); 
//	ASICloseCamera(0);
//}
//
//cv::Mat GetImageFromMemory(uchar* image, int length, int flag)
//{
//
//	std::vector<uchar> data = std::vector<uchar>(image, image + length);
//	cv::Mat ImMat = cv::imdecode(data, flag);
//	//cv::Mat ImMat = cv::Mat(480, 640, ASI_IMG_RAW8, data);
//	// ASI_IMG_RAW8 vs. CV_8UC1, CV_8U(n), etc.
//	return ImMat;
//}
//
//cv::Mat video_frame(CString FilePath)
//{
//	//SaveImage.Width = ConnectCamera[SaveImage.CamID].width; //640
//	//SaveImage.Height = ConnectCamera[SaveImage.CamID].height; //480
//	//SaveImage.Type = ConnectCamera[SaveImage.CamID].ImageType; //Gets default image type, ASI_IMG_RAW8
//	//long lBuffSize = SaveImage.Width * SaveImage.Height; //307200
//	//int length = ((SaveImage.Width * 8 / 8 + 3) / 4 * 4) * SaveImage.Height; // 307200 I Think
//
//	//// METHOD 1:
//	//cv::Mat img;
//	//ASIGetVideoData(0, SaveImage.pBuf, length, -1); //pBuf comes from 
//	//img = GetImageFromMemory(SaveImage.pBuf, length, 1); 
//	
//	//// METHOD 2: 
//	//ASIGetVideoData(0, ConnectCamera[0].pTempImg, length, -1); //pBuf comes from 
//	//cv::Mat Mat_img = cv::cvarrToMat(ConnectCamera[0].pTempImg, TRUE);
//
//	// METHOD 3:   
//	unsigned char img_data[307200];
//	ASIGetVideoData(0, img_data, 307200, -1); //now img_data is 307200 character array of value 255 (is this black?)
//	cv::Mat TempMat = cv::Mat(480, 640, CV_8U, img_data); //Now TempMat matches SaveImage.pBuf...
//
//	return TempMat;
//	
//	//if(SaveImage.pBuf)
//	//	{
//	//		delete[] SaveImage.pBuf;
//	//		SaveImage.pBuf = NULL;
//	//	} 
//
//	//switch (SaveImage.Type)
//	//{
//	//case ASI_IMG_RAW8:
//
//	//	lBuffSize = SaveImage.Width*SaveImage.Height;
//	//	SaveImage.pBuf = new BYTE[lBuffSize];
//	//	memcpy(SaveImage.pBuf, (BYTE*)ConnectCamera[0].pTempImg->imageData, lBuffSize);
//	//	break;
//	//case ASI_IMG_RGB24:
//	//	lBuffSize = SaveImage.Width*SaveImage.Height*3;
//	//	SaveImage.pBuf = new BYTE[lBuffSize];
//	//	memcpy(SaveImage.pBuf, (BYTE*)ConnectCamera[SaveImage.CamID].pTempImg->imageData, lBuffSize);
//	//	break;
//	//case ASI_IMG_RAW16:
//	//	lBuffSize = SaveImage.Width*SaveImage.Height;
//	//	SaveImage.pBuf = new BYTE[lBuffSize];
//	//	for(long i = 0; i < lBuffSize; i++ )
//	//		SaveImage.pBuf[i] = ((BYTE*)ConnectCamera[SaveImage.CamID].pTempImg->imageData)[2*i + 1];
//	//	break;
//
//	//}
//
//	//const char* pszFileName = "C:\\Users\\Thomas\\Desktop\\HVITiltedCameraImages\\test\\img77.BMP";
//	////pszFileName = (LPSTR)(LPCTSTR)FilePath; //apparently this is 'i'... That would be a problem!
//	//switch(SaveImage.Type)
//	//{
//	//case ASI_IMG_Y8:
//	//case ASI_IMG_RAW8:
//	//case ASI_IMG_RAW16:
//	//	saveBmp(pszFileName, SaveImage.pBuf, SaveImage.Width, SaveImage.Height, 8);
//	//	cout << pszFileName;
//	//	break;
//	//case ASI_IMG_RGB24:
//	//	saveBmp(pszFileName, SaveImage.pBuf, SaveImage.Width, SaveImage.Height, 24);
//	//	break;
//	//}
//	//if(SaveImage.pBuf)
//	//{
//	//	delete[] SaveImage.pBuf;
//	//	SaveImage.pBuf = NULL;
//	//}
//}
//
//
