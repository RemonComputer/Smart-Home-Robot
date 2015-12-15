#pragma once
#include <mrpt/slam.h>         // Include all classes in mrpt-slam and its dependencies
#include <mrpt/base.h>         // Include all classes in mrpt-base and its dependencies
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/slam/CActionCollection.h>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/slam/CSimpleMap.h>
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/slam/CMultiMetricMap.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/TKLDParams.h>
#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/slam/CObservation.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/gui/CDisplayWindow.h>

#include <opencv2/opencv.hpp>

#include "SharedResource.h"

using namespace mrpt;          // Global methods, and data types.
using namespace mrpt::utils;   // Select namespace for serialization, utilities, etc...
using namespace mrpt::poses;   // Select namespace for 2D & 3D geometry classes.
using namespace mrpt::slam;
using namespace std;
using namespace cv;

class SLAM
{
private:
	int getMinDepthValueExceptZerosOrZeroIfOnlyZeros(Mat I);
	mrpt::slam::CObservation2DRangeScan DepthImageTO2DScan(Mat  Depth);

public:
	SLAM();
	~SLAM(void);

	void MakeSlam(SharedResource*Resource);
};

