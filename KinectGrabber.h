#pragma once
#include "SharedResource.h"

#include <mrpt/base.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/system/threads.h>
#include <mrpt/slam/CObservationIMU.h>

#include <opencv2/opencv.hpp>

using namespace cv;

class KinectGrabber
{
private:
	VideoCapture capture;
	//mrpt::hwdrivers::CKinect Kinect;
public:
	KinectGrabber();
	~KinectGrabber(void);
	void GrabKinectObservation(SharedResource* Resource);
};

