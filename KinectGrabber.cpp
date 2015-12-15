#include "KinectGrabber.h"


KinectGrabber::KinectGrabber()
{	/*
	mrpt::utils::TCamera CameraConfig;
	mrpt::utils::CConfigFile ConfigFile("D:/Program Files (x86)/mrpt-1.0.1/share/mrpt/config_files/rawlog-grabber/kinect.ini");
	
	CameraConfig.loadFromConfigFile("KINECT_LEFT",ConfigFile);
	Kinect.setCameraParamsDepth(CameraConfig);

	CameraConfig.loadFromConfigFile("KINECT_RIGHT",ConfigFile);
	Kinect.setCameraParamsIntensity(CameraConfig);


	Kinect.enableGrabDepth();//is it good to call that before initialization
	Kinect.enableGrabRGB();
	//Kinect.enableGrab3DPoints();
	Kinect.enablePreviewRGB();
	Kinect.enableVerbose(false);
	Kinect.initialize();//if not good call open()
	Kinect.setTiltAngleDegrees(0);
	*/

	capture=VideoCapture(CV_CAP_OPENNI);
	//capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ );
	//capture.set(CV_CAP_OPENNI_DEPTH_GENERATOR,CV_CAP_OPENNI_
}


KinectGrabber::~KinectGrabber(void)
{
}


void KinectGrabber::GrabKinectObservation(SharedResource* Resource)
{
	/*
	bool ThereIsObservation,HardwareError;
	while (true)
	{
		mrpt::slam::CObservation3DRangeScanPtr obs=mrpt::slam::CObservation3DRangeScan::Create();

		do{
		Kinect.getNextObservation(*obs,ThereIsObservation,HardwareError);
		}while(ThereIsObservation==false);

		if(Resource->GetSignalsToExit(0)==true)
		{
			break;
		}
		else if(true)//Resource->GetPersistantSynchronizationMechanism(1)==true&&Resource->GetPersistantSynchronizationMechanism(2)==true
		{
			Resource->SetPersistantSynchronizationMechanism(false,1);//indicate that it is executed
			Resource->SetKinectObservation(obs);
		}
	
		mrpt::system::sleep(10);
	}
	Resource->SetSignalsToExit(false,0);
	*/
	const double MaxDepthOpenNI=capture.get(CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH);
	const double ScaleRatio=255/MaxDepthOpenNI;
	while (true)
	{
		if(Resource->GetSignalsToExit(0)==true)
		{
			break;
		}

		Mat DepthImage;
		Mat RGBImage;

		capture.grab();
		capture.retrieve(DepthImage,CV_CAP_OPENNI_DEPTH_MAP);
		capture.retrieve(RGBImage,CV_CAP_OPENNI_BGR_IMAGE);//may be wrong enum number
		
		if(DepthImage.empty()||RGBImage.empty())
		{
			waitKey(20);
			continue;
		}

		Resource->SetKinectObservation(DepthImage,RGBImage);//set  DepthImage and RGBImage in Shared Resource
		DepthImage.convertTo(DepthImage,CV_8U,ScaleRatio);//convert depth image for showing in window

		imshow("Depth Image",DepthImage);
		waitKey(3);//is that delay enough
		imshow("RGB Image",RGBImage);
		waitKey(3);//is that delay enough

		waitKey(14);
	}
}
