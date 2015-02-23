

//#include <Windows.h>
//#include <Kinect.h>


#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>


#include <iostream>

#include "CK4Wv2OpenCVModule.h"
using namespace std;


int main(int argc, char* argv[])
{

	CK4Wv2OpenCVModule myKinect;
	myKinect.InitializeKinectDevice();
	viz::Viz3d myWindow("Coordinate Frame");
	std::vector<Point3f> pointsVec;
	std::vector<int> mesh;
	cv::Mat M(1, myKinect.DEPTH_FRAME_WIDTH* myKinect.DEPTH_FRAME_HEIGHT, CV_8UC3, Scalar(0, 0, 0));
	unsigned char *input = (unsigned char*)(M.data);



	for (int y = 0; y < myKinect.DEPTH_FRAME_HEIGHT; y++)
	{
		for (int x = 0; x < myKinect.DEPTH_FRAME_WIDTH; x++)
		{
			

			pointsVec.push_back(Point3f(x, y, 0));
			//input[(myKinect.DEPTH_FRAME_WIDTH* y +  x)*3 ] = 0;
			//input[(myKinect.DEPTH_FRAME_WIDTH* y + x) * 3 + 1] = 0;
			//input[(myKinect.DEPTH_FRAME_WIDTH* y + x) * 3 + 2] = 0;

		}
	}

	for (int y = 0; y < myKinect.DEPTH_FRAME_HEIGHT-1; y++)
	{
		for (int x = 0; x < myKinect.DEPTH_FRAME_WIDTH-1; x++)
		{
			int jumpSize = myKinect.DEPTH_FRAME_WIDTH;
			mesh.push_back(3);
			mesh.push_back(jumpSize*y + x+1);
			mesh.push_back(jumpSize*y + x);
			mesh.push_back(jumpSize*(y+1) + x);
			mesh.push_back(3);
			mesh.push_back(jumpSize*y + x + 1);
			mesh.push_back(jumpSize*(y + 1) + x);
			mesh.push_back(jumpSize*(y+1) + x+1);
		}
	}

	myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
	//viz::WMesh W(pointsVec, mesh,M);

	
	while (!myWindow.wasStopped())
	{
		myKinect.UpdateData();

		for (int yi = 0; yi < myKinect.DEPTH_FRAME_HEIGHT; yi++)
		{
			for (int xi = 0; xi <myKinect.DEPTH_FRAME_WIDTH; xi++)
			{
				int jumpSize = myKinect.DEPTH_FRAME_WIDTH;
				ColorSpacePoint colorSpacePoint = myKinect.colorSpacePointsPtr[myKinect.DEPTH_FRAME_WIDTH*yi + xi];
				int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
				int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));

				if ((0 <= colorX) && (colorX < myKinect.COLOR_FRAME_WIDTH) && (0 <= colorY) && (colorY < myKinect.COLOR_FRAME_HEIGHT))
				{
					RGBQUAD color = myKinect.pColorRAWBuffer[colorY * myKinect.COLOR_FRAME_WIDTH + colorX];
					input[(myKinect.DEPTH_FRAME_WIDTH* yi + xi) * 3] = color.rgbBlue;
					input[(myKinect.DEPTH_FRAME_WIDTH* yi + xi) * 3 + 1] = color.rgbGreen;
					input[(myKinect.DEPTH_FRAME_WIDTH* yi + xi) * 3 + 2] = color.rgbRed;
				}
				else
				{
					input[(myKinect.DEPTH_FRAME_WIDTH* yi + xi) * 3] = 0;
					input[(myKinect.DEPTH_FRAME_WIDTH* yi + xi) * 3 + 1] = 0;
					input[(myKinect.DEPTH_FRAME_WIDTH* yi + xi) * 3 + 2] = 0;
				}

				pointsVec[jumpSize*yi + xi].x = myKinect.pointCloudptr[myKinect.DEPTH_FRAME_WIDTH*yi + xi].X;
				pointsVec[jumpSize*yi + xi].y = myKinect.pointCloudptr[myKinect.DEPTH_FRAME_WIDTH*yi + xi].Y;
				pointsVec[jumpSize*yi + xi].z = myKinect.pointCloudptr[myKinect.DEPTH_FRAME_WIDTH*yi + xi].Z;
			}
		}

		viz::WMesh W(pointsVec, mesh,M);
		//viz::WCloud W(pointsVec, M);

		myWindow.showWidget("test", W);
		myWindow.spinOnce(1, true);
	}

	//for (;;)
	//{
	//	myKinect.UpdateData();
	//	cv::Mat colorFrame = myKinect.colorRAWFrameMat;
	//	resize(colorFrame, colorFrame, cv::Size(0, 0), 0.25, 0.25);
	//	imshow("Display", colorFrame);

	//	if (waitKey(30) >= 0)
	//	{
	//		break;
	//	}
	//}

	return EXIT_SUCCESS;
}