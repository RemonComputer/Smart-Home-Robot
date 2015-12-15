#pragma once
#include <iostream>
#include <list>

#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>

#include <mrpt/system/filesystem.h>

using namespace std;
using namespace cv;

class MyObject
{
	list<Mat> Images;
	FlannBasedMatcher matcher;
	int SavedImageCounter;
public:
	MyObject(void);
	~MyObject(void);
	void AddImage(Mat&I);
	void SaveImages(string ParentDirPath,int ObjectNumber);
	void SaveMatcher(string MatcherDirPath,int ObjectNumber);
	void AddDescriptor(Mat&D);
	float GetAvgDescriptorDistance(Mat&QueryDescriptor);
}; 

