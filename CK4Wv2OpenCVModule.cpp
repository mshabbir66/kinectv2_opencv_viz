#include "CK4Wv2OpenCVModule.h"

// Log tag
static const string TAG = "[K4WOCV]";


CK4Wv2OpenCVModule::CK4Wv2OpenCVModule() :
pSensor(NULL),
pMultiSourceFrameReader(NULL),
m_pCoordinateMapper(NULL),
m_nStartTime(0),
timeStamp(0),
isBodyTracked(false)
{
	cout << TAG << "Initializing Kinect for Windows OpenCV Module." << endl;

	//pSensor = nullptr;
	//pCoordinateMapper = nullptr;
	//pMultiSourceFrameReader = nullptr;

	// Allocate buffers
	pColorRAWBuffer = new RGBQUAD[COLOR_FRAME_WIDTH * COLOR_FRAME_HEIGHT];
	//pDepthRAWBuffer = new ushort[DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT];
	//pInfraRAWBuffer = new ushort[INFRARED_FRAME_WIDTH * INFRARED_FRAME_HEIGHT];
	pointCloudptr = new CameraSpacePoint[DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT];
	pDepthRAWBuffer = new UINT16[DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT];
	colorSpacePointsPtr = new ColorSpacePoint[DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT];


	// Set 0
	memset(pColorRAWBuffer, 0, COLOR_FRAME_WIDTH * COLOR_FRAME_HEIGHT * sizeof(RGBQUAD));
	//memset( pColorRAWBuffer, 0, DEPTH_FRAME_WIDTH * DEPTH_FRAME_HEIGHT * sizeof( ushort ) );
	//memset( pColorRAWBuffer, 0, INFRARED_FRAME_WIDTH * INFRARED_FRAME_HEIGHT * sizeof( ushort ) );

	// Set Mat
	colorRAWFrameMat = Mat(Size(COLOR_FRAME_WIDTH, COLOR_FRAME_HEIGHT), CV_8UC4, (void*)pColorRAWBuffer);
	//depthRAWFrameMat = Mat( Size( DEPTH_FRAME_WIDTH, DEPTH_FRAME_HEIGHT ), CV_16UC1, (void*)pDepthRAWBuffer );
	//infraRAWFrameMat = Mat( Size( INFRARED_FRAME_WIDTH, INFRARED_FRAME_HEIGHT ), CV_16UC1, (void*)pInfraRAWBuffer );

}

CK4Wv2OpenCVModule::~CK4Wv2OpenCVModule()
{
	cout << TAG << "Releasing Kinect for Windows OpenCV Module." << endl;
	// Release buffers
	if (pColorRAWBuffer)
	{
		delete pColorRAWBuffer;
		pColorRAWBuffer = nullptr;
	}

	if (pSensor)
	{
		pSensor->Close();
		SafeRelease(&pSensor);
	}

	if (m_pCoordinateMapper)
	{
		SafeRelease(&m_pCoordinateMapper);
	}

}

HRESULT CK4Wv2OpenCVModule::InitializeKinectDevice()
{
	HRESULT hr;

	// Get default sensor
	hr = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hr))
	{
		cerr << TAG << "Sensor initialization error at - " << __FUNCTIONW__ << endl;
		return hr;
	}

	if (pSensor)
	{
		// Get coordinate mapper
		// Open sensor
		hr = pSensor->Open();
		if (SUCCEEDED(hr))
		{
			char frameFlg = FrameSourceTypes_Color | FrameSourceTypes_Depth | FrameSourceTypes_Body;
			hr = pSensor->OpenMultiSourceFrameReader(frameFlg, &pMultiSourceFrameReader);
		}

		if (SUCCEEDED(hr))
		{
			hr = pSensor->get_CoordinateMapper(&m_pCoordinateMapper);

		}


	}

	if (!pSensor || FAILED(hr))
	{
		cerr << TAG << "No devices ready." << endl;
		return E_FAIL;
	}

	return hr;
}

void CK4Wv2OpenCVModule::UpdateData()
{
	isBodyTracked = false;
	if (!pMultiSourceFrameReader)
		return;

	IMultiSourceFrame* pMultiSourceFrame = nullptr;
	IDepthFrame* pDepthFrame = nullptr;
	IColorFrame* pColorFrame = nullptr;
	IInfraredFrame* pInfraFrame = nullptr;
	IBodyFrame* pBodyFrame = nullptr;
	

	HRESULT hr = pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

	if (SUCCEEDED(hr))
	{
		// Receive color
		IColorFrameReference* pColorFrameReference = nullptr;
		HRESULT hrColor = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);

		if (SUCCEEDED(hrColor))
		{
			pColorFrameReference->get_RelativeTime(&timeStamp);
		}

		if (!m_nStartTime)
		{
			m_nStartTime = timeStamp;
		}


		if (SUCCEEDED(hrColor))
		{
			hrColor = pColorFrameReference->AcquireFrame(&pColorFrame);
			//pColorFrame->get_RelativeTime(&timeStamp);

		}

		if (SUCCEEDED(hrColor))
		{
			unsigned int colorBufferSize = COLOR_FRAME_WIDTH * COLOR_FRAME_HEIGHT * sizeof(RGBQUAD);
			hr = pColorFrame->CopyConvertedFrameDataToArray(colorBufferSize, reinterpret_cast<BYTE*>(pColorRAWBuffer), ColorImageFormat_Bgra);

		}

		SafeRelease(&pColorFrameReference);
		SafeRelease(&pColorFrame);

		IDepthFrameReference* pDepthFrameReference = nullptr;
		HRESULT hrDepth = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		if (SUCCEEDED(hrDepth))
		{
			hrDepth = pDepthFrameReference->AcquireFrame(&pDepthFrame);
		}
		if (SUCCEEDED(hrDepth))
		{ 
			hrDepth = pDepthFrame->CopyFrameDataToArray(DEPTH_FRAME_WIDTH*DEPTH_FRAME_HEIGHT, pDepthRAWBuffer);
		}
		if (SUCCEEDED(hrDepth))
		{
			hrDepth = m_pCoordinateMapper->MapDepthFrameToCameraSpace(DEPTH_FRAME_WIDTH*DEPTH_FRAME_HEIGHT, pDepthRAWBuffer, DEPTH_FRAME_WIDTH*DEPTH_FRAME_HEIGHT, pointCloudptr);

		}
		if (SUCCEEDED(hrDepth))
		{
			hrDepth = m_pCoordinateMapper->MapDepthFrameToColorSpace(DEPTH_FRAME_WIDTH*DEPTH_FRAME_HEIGHT, pDepthRAWBuffer, DEPTH_FRAME_WIDTH*DEPTH_FRAME_HEIGHT, colorSpacePointsPtr);

		}


		IBodyFrameReference* pBodyFrameReference = nullptr;
		HRESULT hrBody = pMultiSourceFrame->get_BodyFrameReference(&pBodyFrameReference);
		IBody* ppBodies[BODY_COUNT] = { 0 };

		if (SUCCEEDED(hrBody))
		{
			hrBody = pBodyFrameReference->AcquireFrame(&pBodyFrame);
		}
		SafeRelease(&pBodyFrameReference);


		if (SUCCEEDED(hrBody))
		{
			hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies);
		}
		if (SUCCEEDED(hrBody))
		{
			ProcessBody(ppBodies);
			for (int i = 0; i < _countof(ppBodies); ++i)
			{
				SafeRelease(&ppBodies[i]);
			}
		}

		SafeRelease(&pDepthFrameReference);
		SafeRelease(&pDepthFrame);

		SafeRelease(&pBodyFrame);
		SafeRelease(&pMultiSourceFrame);

	}
}

void CK4Wv2OpenCVModule::ProcessBody(IBody** ppBodies)
{
	for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
	{
		IBody* pBody = ppBodies[iFace];
		if (pBody)
		{
			BOOLEAN bTracked = false;
			HRESULT hr = pBody->get_IsTracked(&bTracked);

			if (SUCCEEDED(hr) && bTracked)
			{
				Joint joints[JointType_Count];
				hr = pBody->GetJoints(JointType_Count, joints);

				JointOrientation jOs[JointType_Count];
				hr = pBody->GetJointOrientations(JointType_Count, jOs);
				
				headJoint = joints[JointType_Head];
				headJoint = joints[JointType_Head];
				isBodyTracked = true;


				iFace = BODY_COUNT;

			}
		}


	}
}


template< class T > void CK4Wv2OpenCVModule::SafeRelease(T** ppT)
{
	if (*ppT)
	{
		(*ppT)->Release();
		*ppT = nullptr;
	}
}

INT64 CK4Wv2OpenCVModule::GetTimeStamp(){
	return timeStamp - m_nStartTime;

}



void CK4Wv2OpenCVModule::ExtractFaceRotationInDegrees(const Vector4* pQuaternion, float* pPitch, float* pYaw, float* pRoll)
{
	double x = pQuaternion->x;
	double y = pQuaternion->y;
	double z = pQuaternion->z;
	double w = pQuaternion->w;

	// convert face rotation quaternion to Euler angles in degrees		
	//double dPitch, dYaw, dRoll;
	*pPitch = static_cast<float>(atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) / 3.14159265358979323846 * 180.0);
	*pYaw = static_cast<float>(asin(2 * (w * y - x * z)) / 3.14159265358979323846 * 180.0);
	*pRoll = static_cast<float>(atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) / 3.14159265358979323846 * 180.0);

	// clamp rotation values in degrees to a specified range of values to control the refresh rate
	//double increment = 5.0f;
	//*pPitch = static_cast<int>((dPitch + increment / 2.0 * (dPitch > 0 ? 1.0 : -1.0)) / increment) * static_cast<int>(increment);
	//*pYaw = static_cast<int>((dYaw + increment / 2.0 * (dYaw > 0 ? 1.0 : -1.0)) / increment) * static_cast<int>(increment);
	//*pRoll = static_cast<int>((dRoll + increment / 2.0 * (dRoll > 0 ? 1.0 : -1.0)) / increment) * static_cast<int>(increment);
}

Point2f CK4Wv2OpenCVModule::BodyToScreen(const CameraSpacePoint& bodyPoint)
{
	// Calculate the body's position on the screen
	ColorSpacePoint colorPoint = { 0 };
	m_pCoordinateMapper->MapCameraPointToColorSpace(bodyPoint, &colorPoint);

	float screenPointX = static_cast<float>(colorPoint.X);
	float screenPointY = static_cast<float>(colorPoint.Y);

	return Point2f(screenPointX, screenPointY);
}
