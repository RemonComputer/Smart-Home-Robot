#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>

typedef signed short ComputationTypeT;//for changing primary computation type easily
typedef unsigned char SecondaryComputationTypeT;//for changing primary computation type easily

#define GradientThresh 100 //Corrosponds to distance separation
#define	FilterAreaMin 6000 // filter length -> min length of valid object
#define FilterAreaMax  0.5*640*480
#define	RejetedLabel 0 // label of rejected objects
#define	KernalSize 3// for the kernal that is used in the computation of gradient using sobel method, must be odd kernal size
#define	ComputationType CV_16S //used for computation
#define	SecondaryComputationType CV_8U
#define	ImageRows 480 // input image rows 
#define	ImageCols 640 // input image coloums
#define	Cx 322//Cx for depth Camera
#define	Cy 240//Cy for depth camera
#define	FocalLength 575//Focal length for depth camera

using namespace std;
using namespace cv;

class Segmentor
{
private:
	vector<vector<Point>> DepthBasedSegmentation(Mat&Computation,Mat&DerivativeXFilterDepth,Mat&DerivativeYFilterDepth);
	vector<Mat> ContoursToRecognizedObjects(vector<vector<Point>>&ContoursDepth,Mat&InvalidDepthMask);
	vector<Mat> RemoveVeryNarrowObjectsAndRefilter(vector<Mat> &Input);
public:
	Segmentor(void);
	~Segmentor(void);

	void SegmentImage(Mat&DepthImage,Mat&GrayImage,vector<Mat>&GraySegments);
};

