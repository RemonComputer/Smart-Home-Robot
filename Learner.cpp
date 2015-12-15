#include "Learner.h"


Learner::Learner()
{
}


Learner::~Learner(void)
{
}

void Learner::AddImage(Mat&Image)
{
	//code to add the image,  call classify and add the image and it's descriptor to the object, watch at empty objects list in begining
	if(Objects.size()==0)//test if Objects list is empty
	{
		AddObjectFromImage(Image);
	}
	else
	{
		Mat descriptors;
		MyObject*DesiredObject=Classify(Image,descriptors);
		if(DesiredObject==NULL)
		{
			AddObjectFromImage(Image);
		}
		else
		{
			DesiredObject->AddDescriptor(descriptors);
			DesiredObject->AddImage(Image);
		}
	}
}

void Learner::Save()
{
	string ParentDirPath="SavedThings";
	int counter=0;
	if(!mrpt::system::directoryExists(ParentDirPath))
				mrpt::system::createDirectory(ParentDirPath);
	for(list<MyObject>::iterator i=Objects.begin();i!=Objects.end();i++)
	{
		i->SaveImages(ParentDirPath,counter);
		i->SaveMatcher(ParentDirPath,counter);
		counter++;
	}
}

void Learner::AddObjectFromImage(Mat & Image)
{
	//detecting keypoints
	SurfFeatureDetector detector(DetectorThresh);
	vector<KeyPoint> keypoints;
	detector.detect(Image, keypoints);

	// computing descriptors
    SurfDescriptorExtractor extractor;
    Mat descriptors;
    extractor.compute(Image, keypoints, descriptors);

	MyObject O;//=new MyObject();
	O.AddImage(Image);
	O.AddDescriptor(descriptors);
	Objects.push_back(O);
}

MyObject* Learner::Classify(Mat&Image,Mat&descriptors)//ask the class to classify during learning mode
{
	//detecting keypoints
	SurfFeatureDetector detector(DetectorThresh);
	vector<KeyPoint> keypoints;
	detector.detect(Image, keypoints);

	// computing descriptors
    SurfDescriptorExtractor extractor;
    extractor.compute(Image, keypoints, descriptors);
	
	//comparing between available objects,to get min average distance
	list<MyObject>::iterator i=Objects.begin();
	float mindistance=i->GetAvgDescriptorDistance(descriptors);
	MyObject*DesiredObject=&(*i);
	i++;
	for(;i!=Objects.end();i++)
	{
		float distance=i->GetAvgDescriptorDistance(descriptors);
		if(distance<mindistance)
		{
			mindistance=distance;
			DesiredObject=&(*i);
		}
	}
	if (mindistance>DescriptorDistanceThresh)
		return NULL;
	return DesiredObject;
}

Mat * Learner::RangeMatrixToOpenCVMat(mrpt::math::CMatrixPtr RangeImage)
{
	int height=RangeImage->rows();
	int width=RangeImage->cols();
	Mat * Result=new Mat(height,width,CV_16U);
	for(int i=0;i<height;i++)
	{
		for(int j=0;j<width;j++)
		{
			Result->at<signed short>(i,j)=(unsigned short)mrpt::math::floorl(RangeImage->coeff(i,j)*1000);//conversion from meter to millimeter
		}
	}
	return Result;
}

void Learner::Learn(SharedResource*Resource)
{
	Segmentor S;
	
	////Mat InitializationGrayScaleSegments(480,640,CV_8U);
	//while (true)
	//{
	//	mrpt::slam::CObservation3DRangeScanPtr KinectObs=mrpt::slam::CObservation3DRangeScan::Create();
	//	mrpt::utils::CImagePtr RGB=mrpt::utils::CImage::Create();
	//	mrpt::math::CMatrixPtr DepthMatrix=mrpt::math::CMatrix::Create();

	//	if(Resource->GetSignalsToExit(3)==true)
	//	{
	//		Save();//makes a problem due to not using pointers //saving the descriptors and images before releasing the lock
	//		Resource->SetSignalsToExit(false,3);
	//		break;
	//	}
	//	else
	//	{
	//		KinectObs=Resource->GetKinectObservation();
	//		if(KinectObs->hasRangeImage==false)
	//		{
	//			mrpt::system::sleep(10);
	//			continue;
	//		}
	//		*RGB=KinectObs->intensityImage;
	//		*DepthMatrix=KinectObs->rangeImage;
	//	
	//		//begin conversion from opencv to mrpt
	//
	//		//mrpt::utils::CImagePtr dummy=mrpt::utils::CImage::Create();
	//		//dummy->setFromMatrix(*DepthMatrix,true);
	//		//dummy.setFromMatrix(DepthMatrix,true);//take care that the function considers that the range image is a normalized matrix
	//		Mat * GrayScaleCV=new Mat((IplImage *)RGB->grayscale().getAs<IplImage>(),true); //conversion from mrpt to opencv

	//		/*
	//		Mat * DepthImageCV=new Mat((IplImage *)dummy->getAs<IplImage>(),true); //watch out for the range of values convert from 0->256-1 to 0->2^16-1 ,conversion from mrpt to opencv
	//		Mat * DepthImageCVConverted=new Mat;
	//		DepthImageCV->convertTo(*DepthImageCVConverted,CV_16U,1023/255,0);//convert and scale from 8-bit to 16-bit, because it was initially operating on 16-bit
	//		*/

	//		Mat * DepthImageCV=RangeMatrixToOpenCVMat(DepthMatrix);

	//		//Segmentation using opencv
	//		std::vector<Mat> GrayScaleSegments;//(100,InitializationGrayScaleSegments);
	//		S.SegmentImage(*DepthImageCV,*GrayScaleCV,GrayScaleSegments);
	//		
	//		//delete pointers to avoid memory leaks
	//		delete DepthImageCV;
	//		delete GrayScaleCV;
	//		//delete DepthImageCVConverted;
	//	
	//		//learning using opencv
	//		for (std::vector<Mat>::iterator i=GrayScaleSegments.begin();i!=GrayScaleSegments.end();i++)
	//		{
	//			AddImage(*i);
	//		}
	//		mrpt::system::sleep(500);//not to make it eat memory fast
	//	}
	//}


	while (true)
	{
		//mrpt::slam::CObservation3DRangeScanPtr KinectObs=mrpt::slam::CObservation3DRangeScan::Create();
		//mrpt::utils::CImagePtr RGB=mrpt::utils::CImage::Create();
		//mrpt::math::CMatrixPtr DepthMatrix=mrpt::math::CMatrix::Create();
		Mat DepthImage;
		Mat RGBImage;

		if(Resource->GetSignalsToExit(3)==true)
		{
			Save();//makes a problem due to not using pointers //saving the descriptors and images before releasing the lock
			Resource->SetSignalsToExit(false,3);
			break;
		}
		else
		{
			Resource->GetKinectObservation(DepthImage,RGBImage);
			
			/*
			KinectObs=Resource->GetKinectObservation();
			if(KinectObs->hasRangeImage==false)
			{
				mrpt::system::sleep(10);
				continue;
			}
			*RGB=KinectObs->intensityImage;
			*DepthMatrix=KinectObs->rangeImage;
			*/

			if(DepthImage.empty()||RGBImage.empty())
			{
				mrpt::system::sleep(10);
				continue;
			}
		
			//begin conversion from opencv to mrpt
	
			//mrpt::utils::CImagePtr dummy=mrpt::utils::CImage::Create();
			//dummy->setFromMatrix(*DepthMatrix,true);
			//dummy.setFromMatrix(DepthMatrix,true);//take care that the function considers that the range image is a normalized matrix
			//Mat * GrayScaleCV=new Mat((IplImage *)RGB->grayscale().getAs<IplImage>(),true); //conversion from mrpt to opencv

			/*
			Mat * DepthImageCV=new Mat((IplImage *)dummy->getAs<IplImage>(),true); //watch out for the range of values convert from 0->256-1 to 0->2^16-1 ,conversion from mrpt to opencv
			Mat * DepthImageCVConverted=new Mat;
			DepthImageCV->convertTo(*DepthImageCVConverted,CV_16U,1023/255,0);//convert and scale from 8-bit to 16-bit, because it was initially operating on 16-bit
			*/

			//Mat * DepthImageCV=RangeMatrixToOpenCVMat(DepthMatrix);

			Mat GrayScaleImage;
			cvtColor(RGBImage,GrayScaleImage,CV_RGB2GRAY);

			//Segmentation using opencv
			std::vector<Mat> GrayScaleSegments;
			S.SegmentImage(DepthImage,GrayScaleImage,GrayScaleSegments);

			//delete pointers to avoid memory leaks
			//delete DepthImageCV;
			//delete GrayScaleCV;
			//delete DepthImageCVConverted;
		
			//learning using opencv
			for (std::vector<Mat>::iterator i=GrayScaleSegments.begin();i!=GrayScaleSegments.end();i++)
			{
				AddImage(*i);
			}
			mrpt::system::sleep(500);//not to make it eat memory fast
		}
	}
}