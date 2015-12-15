#include "Explorer.h"


Explorer::Explorer()
{
}


Explorer::~Explorer(void)
{
}

Mat * Explorer::RangeMatrixToOpenCVMat(mrpt::math::CMatrixPtr RangeImage)
{
	int height=RangeImage->rows();
	int width=RangeImage->cols();
	Mat * Result=new Mat(height,width,CV_16U);
	for(int i=0;i<height;i++)
	{
		for(int j=0;j<width;j++)
		{
			Result->at<signed short>(i,j)=mrpt::math::floorl(RangeImage->coeff(i,j)*1000);//conversion from meter to millimeter
		}
	}
	return Result;
}

void Explorer::UpdateDirection(SharedResource*Resource)
{
	char direction='f';//forward
	while (true)
	{
		Mat DepthImage;
		if(Resource->GetSignalsToExit(8)==true)
		{
			Resource->SetSignalsToExit(false,8);
			break;
		}
		else if(Resource->GetManualControl()==true || Resource->GetGoingToDestination()==true)//check that it is in automatic navigation mood
		{
			continue;
		}
		else if(Resource->GetManualControl()==false&&Resource->GetGoingToDestination()==false)//check that it is in automatic navigation mood
		{
			Resource->GetKinectObservationDepthOnly(DepthImage);
			if(DepthImage.empty()==true)//unintialized Observation
			{
				mrpt::utils::sleep(3);
				continue;
			}
			
			direction=ObstacleAvoidance(DepthImage);
			Resource->SetDirection(direction);

			//std::cout<<"Direction: "<<direction<<std::endl;//for debugging only
		}
		mrpt::system::sleep(20);
	}
}

int Explorer::getMaxIndexJfromDepthImage(Mat&I)
{
	Point maxLoc;
	minMaxLoc(I,NULL,NULL,NULL,&maxLoc);
	return maxLoc.x;//take care it might be returning the wrong value
}
unsigned char Explorer::ObstacleAvoidance(Mat&I)//Kinect Depth Image
{
	//stop='s',forward='f', right rotation='a' (anti-clockwise), left rotation='c' (clockwise)
	static unsigned char previousaction=forwardNumber;//will be executed in the first  entrance of the function only
	int min;
	int indexJ;
	if(previousaction==forwardNumber)
	{
		min=getMinDepthValueExceptZeros(I);
		if(min<=minObstacleDistance)
		{
			previousaction=stopNumber;
			return previousaction;
		}
		else
		{
			previousaction=forwardNumber;
			return previousaction;
		}
	}
	else if(previousaction==stopNumber)
	{
		min=getMinDepthValueExceptZeros(I);
		if(min>minObstacleDistance)
		{
			previousaction=forwardNumber;
			return previousaction;
		}
		else
		{
			indexJ=getMaxIndexJfromDepthImage(I);
			if(indexJ>Jcentral)
			{
				previousaction=leftRotation;
				return previousaction;
			}
			else
			{
				previousaction=rightRotation;
				return previousaction;
			}
		}
	}
	else if(previousaction==leftRotation||previousaction==rightRotation)
	{
		min=getMinDepthValueExceptZeros(I);
		if(min<=minObstacleDistance)
		{
			return previousaction;
		}
		else
		{
			previousaction=stopNumber;
			return previousaction;
		}
	}
}

int Explorer::getMinDepthValueExceptZeros(Mat&I)
{
	Mat Mask=(I!=0);
	//imshow("Mask",Mask);//for debugging only
	double minvalue;
	minMaxLoc(I,&minvalue,NULL,NULL,NULL,Mask);
	return (int)minvalue;
}