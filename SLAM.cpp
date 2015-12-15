#include "SLAM.h"


SLAM::SLAM()
{
	
}


SLAM::~SLAM(void)
{
}

mrpt::slam::CObservation2DRangeScan SLAM::DepthImageTO2DScan(Mat  Depth)
{
	vector<float> ScanValues(Depth.cols);
	vector<char> ValidMeasurement(Depth.cols);
	for(int i=0;i<Depth.cols;i++)
	{
		Mat Coloum=Depth.col(i);
		float V=getMinDepthValueExceptZerosOrZeroIfOnlyZeros(Coloum);
		if(V==0)
		{
			ScanValues.at(i)=0;
			ValidMeasurement.at(i)=0;
		}
		else
		{
			ScanValues.at(i)=V/1000;
			ValidMeasurement.at(i)=1;
		}
	}
	mrpt::slam::CObservation2DRangeScan Scan;
	Scan.maxRange=10;
	Scan.scan=ScanValues;
	Scan.validRange=ValidMeasurement;
	Scan.aperture=mrpt::utils::DEG2RAD(57);
	Scan.timestamp=mrpt::system::now();//approximate it's time stamp by now

	return Scan;
}

int SLAM::getMinDepthValueExceptZerosOrZeroIfOnlyZeros(Mat I)
{
	Mat Mask=(I!=0);
	if(countNonZero(I)==0)
		return 0;
	double minvalue;
	minMaxLoc(I,&minvalue,NULL,NULL,NULL,Mask);
	return (int)minvalue;
}



void SLAM::MakeSlam(SharedResource*Resource)
{
	//
	//initialization parameters of parameters
	const std::string fileName="SavedThings/Map.simplemap.gz" ; //Good
	bool  	compressGZ = true ; //Good

	
	/*********************************mapBuilder declaration and initialization************************************************/
	mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions TC;
	TC.loadFromConfigFileName("D:/Engineering/Graduation Project/Modified Version/Config.ini","MappingApplication");

	CMetricMapBuilderRBPF mapBuilder(TC); //Good

	mapBuilder.options.verbose=false;

	mrpt::slam::CActionCollection Caction; //Good
	mrpt::slam::CSensoryFrame Cframe;//Good

	mrpt::gui::CDisplayWindow MapShow("Map",640,480);

	//while(true)
	//	{
	//		COccupancyGridMap2DPtr grid=COccupancyGridMap2D::Create();
	//		mrpt::slam::CObservation3DRangeScanPtr kinect;
	//		mrpt::slam::CObservation2DRangeScanPtr   Scan2D=mrpt::slam::CObservation2DRangeScan::Create();
	//		mrpt::utils::CImagePtr MapImage=mrpt::utils::CImage::Create();
	//		mrpt::slam::CActionRobotMovement2DPtr Movement;
	//		mrpt::poses::CPose3DPtr Position3D=mrpt::poses::CPose3D::Create();

	//		if(Resource->GetSignalsToExit(2)==true)//exit signal is on
	//		{
	//			//create folder and save the map in it
	//			if(!mrpt::system::directoryExists("SavedThings"))
	//				mrpt::system::createDirectory("SavedThings");
	//			mapBuilder.saveCurrentMapToFile(fileName,compressGZ);
	//			mapBuilder.saveCurrentEstimationToImage("SavedThings/LastEstimation.bmp",false);//save last map Image and robot position to image
	//			Resource->SetSignalsToExit(false,2);
	//			break;
	//		}
	//		else if(Resource->GetPersistantSynchronizationMechanism(0)==false&&Resource->GetPersistantSynchronizationMechanism(1)==false&&Resource->GetPersistantSynchronizationMechanism(2)==true)// 
	//		{
	//			//getting reading
	//			kinect=Resource->GetKinectObservation();
	//			if(kinect->hasRangeImage==false)
	//			{
	//				mrpt::system::sleep(10);
	//				continue;
	//			}
	//			Movement=Resource->GetIncrementalMotion();//may be that makes error like GetKinectObservation()
	//			/*
	//			if(Movement->)
	//			{
	//				mrpt::system::sleep(10);
	//				continue;
	//			}
	//			*/
	//			kinect->convertTo2DScan (*Scan2D,sensorLabel,angle_sup,angle_inf,oversampling_ratio);
	//			Cframe.insert(Scan2D);
	//			Caction.insert(*Movement);

	//			mapBuilder.processActionObservation(Caction,Cframe); //this line sometimes makes errors and sometimes not, may be it uses a pointer that has been destroyed, maybe the values are not good
	//			
	//			//updating variables
	//			*Position3D=mapBuilder.getCurrentPoseEstimation()->getMeanVal();
	//			Resource->SetAbsolutePosition(Position3D->x(),Position3D->y(),Position3D->yaw());
	//			
	//			if(mapBuilder.getCurrentlyBuiltMetricMap()->m_gridMaps.size()>0)
	//				{
	//					grid=mapBuilder.getCurrentlyBuiltMetricMap()->m_gridMaps[0];
	//					grid->getAsImage(*MapImage);
	//					MapImage->scaleImage(640,480);
	//					MapShow.showImage(*MapImage);
	//				}
	//			//Resource->SetMapImage(MapImage);
	//			Resource->SetMap(grid);
	//			for(int i=0;i<3;i++)//enable a second loop of persistant synch mechanism
	//				Resource->SetPersistantSynchronizationMechanism(true,i);
	//			Caction.clear();
	//			Cframe.clear();
	//			mrpt::system::sleep(33);
	//		}		
	//	}

	while(true)
		{
			COccupancyGridMap2DPtr grid=COccupancyGridMap2D::Create();
			mrpt::slam::CObservation2DRangeScanPtr   Scan2D=mrpt::slam::CObservation2DRangeScan::Create();
			mrpt::utils::CImagePtr MapImage=mrpt::utils::CImage::Create();
			mrpt::slam::CActionRobotMovement2DPtr Movement;
			mrpt::poses::CPose3DPtr Position3D=mrpt::poses::CPose3D::Create();
			Mat DepthImage;

			if(Resource->GetSignalsToExit(2)==true)//exit signal is on
			{
				//create folder and save the map in it
				if(!mrpt::system::directoryExists("SavedThings"))
					mrpt::system::createDirectory("SavedThings");
				mapBuilder.saveCurrentMapToFile(fileName,compressGZ);
				mapBuilder.saveCurrentEstimationToImage("SavedThings/LastEstimation.bmp",false);//save last map Image and robot position to image
				Resource->SetSignalsToExit(false,2);
				break;
			}
			else 
			{
				Resource->GetKinectObservationDepthOnly(DepthImage);
				if(DepthImage.empty())
				{
					mrpt::system::sleep(10);
					continue;
				}
				*Scan2D=DepthImageTO2DScan(DepthImage);
				Movement=Resource->GetIncrementalMotion();//may be that makes error like GetKinectObservation()
		
				Cframe.insert(Scan2D);
				Caction.insert(*Movement);

				mapBuilder.processActionObservation(Caction,Cframe); //this line sometimes makes errors and sometimes not, may be it uses a pointer that has been destroyed, maybe the values are not good
				
				//updating variables
				*Position3D=mapBuilder.getCurrentPoseEstimation()->getMeanVal();
				Resource->SetAbsolutePosition(Position3D->x(),Position3D->y(),Position3D->yaw());
				
				if(mapBuilder.getCurrentlyBuiltMetricMap()->m_gridMaps.size()>0)
					{
						grid=mapBuilder.getCurrentlyBuiltMetricMap()->m_gridMaps[0];
						grid->getAsImage(*MapImage);
						MapImage->scaleImage(640,480);
						MapShow.showImage(*MapImage);
					}
				Resource->SetMap(grid);

				Caction.clear();
				Cframe.clear();
				mrpt::system::sleep(33);
			}		
		}
}
