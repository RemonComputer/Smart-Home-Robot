#include "Segmentor.h"


Segmentor::Segmentor(void)
{
}


Segmentor::~Segmentor(void)
{
}

vector<Mat> Segmentor::RemoveVeryNarrowObjectsAndRefilter(vector<Mat> & Input)
{
	//Vector<Mat> Output;
	//Output.reserve(Input.size()*5);
	list<vector<Point>> TotalContours;

	Mat StructuringElement=getStructuringElement(MORPH_RECT,Size(9,9));

	//opening and getting contours after opening
	for(vector<Mat>::iterator i=Input.begin();i!=Input.end();i++)
	{
		//opening of image to remove excess boundaries in place
		erode(*i,*i,StructuringElement);
		dilate(*i,*i,StructuringElement);
		
		Canny(*i,*i,1,100);//detecting canny edges in place

		//getting contours
		vector<vector<Point>> Contours;
		findContours(*i,Contours,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
		
		//pushing contours to total contours
		for (vector<vector<Point>>::iterator i  =Contours.begin() ; i!=Contours.end() ; i++)
		{
			TotalContours.push_back(*i);
		}
	}

	//filtering contours
	for (list<vector<Point>>::iterator i = TotalContours.begin(); i != TotalContours.end();   )
	{
		double area=contourArea(*i);
		if ( area>=FilterAreaMin && area<=FilterAreaMax ) //&& (area/i->size())>10
		{			
			i++;
		}
		else
		{
			i=TotalContours.erase(i);
		}
	}

	//converting contours to ImageMasks
	vector<Mat> Output;
	Output.reserve(TotalContours.size());
	list<Mat> O;
	Scalar color(255);
	for(list<vector<Point>>::iterator i = TotalContours.begin();i!=TotalContours.end();i++)
	{
		//Mat*tmp=new Mat(480,640,CV_8U);
		Mat tmp(480,640,CV_8U);
		fillPoly(tmp,*i,color);
		O.push_back(tmp);
	}

	return Output;
}

vector<Mat> Segmentor::ContoursToRecognizedObjects(vector<vector<Point>>&ContoursDepth,Mat&InvalidDepthMask)
{
	vector<Mat> RecognizedObjects;
	int NumberOfGoodContoursDepth=ContoursDepth.size();
	RecognizedObjects.resize(NumberOfGoodContoursDepth);
	Scalar color(255);
	vector<vector<Point>> tmp(1);
	Mat StructuringElement=getStructuringElement(MORPH_RECT,Size(9,9));
	for(int i=0;i<NumberOfGoodContoursDepth;i++)
	{
		RecognizedObjects.at(i)=Mat(ImageRows,ImageCols,CV_8U);
		tmp.at(0)=ContoursDepth.at(i);
		fillPoly(RecognizedObjects.at(i),tmp,color);
		//convexHull(tmp.at(0),tmp.at(0));
		//fillConvexPoly(RecognizedObjects.at(i),tmp.at(0),color);
		RecognizedObjects.at(i)=RecognizedObjects.at(i)&InvalidDepthMask;
		
		//opening of image to remove excess boundaries
		erode(RecognizedObjects.at(i),RecognizedObjects.at(i),StructuringElement);
		dilate(RecognizedObjects.at(i),RecognizedObjects.at(i),StructuringElement);
	}
	return RecognizedObjects;
}

vector<vector<Point>>  Segmentor::DepthBasedSegmentation(Mat&Computation,Mat&DerivativeXFilterDepth,Mat&DerivativeYFilterDepth)//exports the contours of the objects and it will be used for learning
{
	//gradient thresholding of distance
	Mat XgradientDepth=Mat(ImageRows,ImageCols,ComputationType);
	filter2D(Computation,XgradientDepth,ComputationType,DerivativeXFilterDepth);
	Mat YgradientDepth=Mat(ImageRows,ImageCols,ComputationType);
	filter2D(Computation,YgradientDepth,ComputationType,DerivativeYFilterDepth);
	Mat XYgradientDepth=abs(XgradientDepth)+abs(YgradientDepth);
	Mat GradientThreshImage=(XYgradientDepth>GradientThresh);

	//getting the contours of objects
	vector<vector<Point>> InvalidContoursDepth; //the invalid contour points of the shape
	findContours(GradientThreshImage,InvalidContoursDepth,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);

	//filltering the contours
	vector<vector<Point>> ContoursDepth;
	for( int i = 0; i< InvalidContoursDepth.size(); i++ ) //filter recognized objects by area
    {
		double area=contourArea(InvalidContoursDepth.at(i));
		if ( area>=FilterAreaMin && area<=FilterAreaMax  )
		{
			ContoursDepth.push_back(InvalidContoursDepth.at(i));
		}
	}
	return ContoursDepth;
}

void Segmentor::SegmentImage(Mat&DepthImage,Mat&GrayImage,vector<Mat>&GraySegments)
{
	//central diffrence depth filter operator x
	Mat DerivativeXFilterDepth=Mat::zeros(KernalSize,KernalSize,ComputationType);
	DerivativeXFilterDepth.at<ComputationTypeT>(1,0)=-1;
	DerivativeXFilterDepth.at<ComputationTypeT>(1,2)=1;
	//central diffrence depth filter operator y
	Mat DerivativeYFilterDepth=Mat::zeros(KernalSize,KernalSize,ComputationType);
	DerivativeYFilterDepth.at<ComputationTypeT>(0,1)=-1;
	DerivativeYFilterDepth.at<ComputationTypeT>(2,1)=1;
	
	Mat Computation;//for performing computation
	DepthImage.convertTo(Computation,ComputationType);
	Mat InvalidDepthMask=(Computation!=0);//Invalid depth mask
	//blur(Computation,Computation,Size(9,9));//newly added line , delete if any problems happen
	GaussianBlur(Computation,Computation,Size(9,9),0.3,0.3);
	vector<vector<Point>> ContoursDepth=DepthBasedSegmentation(Computation,DerivativeXFilterDepth,DerivativeYFilterDepth);
	vector<Mat>RecognizedObjects=ContoursToRecognizedObjects(ContoursDepth,InvalidDepthMask);
	//RecognizedObjects=RemoveVeryNarrowObjectsAndRefilter(RecognizedObjects);

	GraySegments.resize(RecognizedObjects.size());
	for(int i=0;i<RecognizedObjects.size();i++)//watch for errors inside this loop
	{
		 GraySegments.at(i)=GrayImage.clone();
		 GraySegments.at(i).setTo(0,~RecognizedObjects.at(i));
	}
}