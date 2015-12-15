#pragma once
#include <mrpt/base.h>
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/system/threads.h>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>


#include "SharedResource.h"
#include "Segmentor.h"
#include "MyObject.h"


#define DetectorThresh 400
#define DescriptorDistanceThresh 0.25


class Learner
{
private:
	list<MyObject> Objects;
	void AddImage(Mat&Image);
	void Save();
	void AddObjectFromImage(Mat & Image);
	MyObject*Classify(Mat&Image,Mat&descriptors);
	Mat * RangeMatrixToOpenCVMat(mrpt::math::CMatrixPtr RangeImage);
public:
	Learner();
	~Learner(void);

	void Learn(SharedResource*Resource);
};

