#pragma once
#include <mrpt/base.h>
#include "SharedResource.h"
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/system/threads.h>

#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv/highgui.h>

#define minObstacleDistance 600 //60 c.m in m.m

//stop='s',forward='f', right rotation='a' (anti-clockwise), left rotation='c' (clockwise)
#define stopNumber 's'
#define forwardNumber 'f'
#define rightRotation 'a'
#define leftRotation 'c'
#define Jcentral 320

using namespace cv; 

class Explorer
{
private:
	
	int getMinDepthValueExceptZeros(Mat&I);
	unsigned char ObstacleAvoidance(Mat&I);
	int getMaxIndexJfromDepthImage(Mat&I);
	Mat * RangeMatrixToOpenCVMat(mrpt::math::CMatrixPtr RangeImage);
public:
	Explorer();
	~Explorer(void);

	void UpdateDirection(SharedResource*Resource);
};

