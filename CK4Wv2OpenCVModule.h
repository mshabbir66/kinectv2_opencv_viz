//#pragma once
// Default IO

#ifndef __CK4Wv2OpenCVModule_H_INCLUDED__
#define __CK4Wv2OpenCVModule_H_INCLUDED__

#include <iostream>
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// Kinect for Windows SDK 2.0
#include "Kinect.h"



// Namespaces
using namespace std;
using namespace cv;


class CK4Wv2OpenCVModule
{
	// Sensor frame data values
	// Color frame resolution

	INT64                   m_nStartTime;
	INT64                   timeStamp;

public:
	static const int COLOR_FRAME_WIDTH = 1920;
	static const int COLOR_FRAME_HEIGHT = 1080;
	// Depth frame resolution
	static const int DEPTH_FRAME_WIDTH = 512;
	static const int DEPTH_FRAME_HEIGHT = 424;
	// Infrared frame resolution
	static const int INFRARED_FRAME_WIDTH = 512;
	static const int INFRARED_FRAME_HEIGHT = 424;

	CK4Wv2OpenCVModule();
	~CK4Wv2OpenCVModule();

	HRESULT InitializeKinectDevice();

	// Image frame Mat
	Mat colorRAWFrameMat;
	CameraSpacePoint* pointCloudptr;
	RGBQUAD* pColorRAWBuffer;
	UINT16* pDepthRAWBuffer;
	ColorSpacePoint* colorSpacePointsPtr;
	Joint headJoint;
	JointOrientation headOrientation;
	bool isBodyTracked;


	// Process frame
	void UpdateData();
	INT64 GetTimeStamp();
	ICoordinateMapper*      m_pCoordinateMapper;

	// Calculate Mapped Frame

	//ushort* pDepthRAWBuffer;
	//ushort* pInfraRAWBuffer;
private:
	// Device
	IKinectSensor* pSensor;




	//float* facialAnimationParametersBuffer;
	//float* bodyBuffer;

	// Frame reader
	IMultiSourceFrameReader* pMultiSourceFrameReader;

	// Release function
	template< class T > void SafeRelease(T** ppT);
	void CK4Wv2OpenCVModule::ProcessBody(IBody** ppBodies);
	void ExtractFaceRotationInDegrees(const Vector4* pQuaternion, float* pPitch, float* pYaw, float* pRoll);
	Point2f CK4Wv2OpenCVModule::BodyToScreen(const CameraSpacePoint& bodyPoint);


};

#endif // __CK4Wv2OpenCVModule_H_INCLUDED__ 