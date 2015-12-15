#include "Viewer.h"


Viewer::Viewer()
{
	
}


Viewer::~Viewer(void)
{
}

void Viewer::UpdateViews(SharedResource*Resource)
{
	mrpt::gui::CDisplayWindow RGB("RGB Image");
	mrpt::gui::CDisplayWindow Depth("Depth Image");
	mrpt::gui::CDisplayWindow Map("Map");
	while (true)
	{
		mrpt::utils::CImagePtr DepthImage=mrpt::utils::CImage::Create();
		mrpt::slam::CObservation3DRangeScanPtr Obs;
		mrpt::utils::CImagePtr MapP;

		if(Resource->GetSignalsToExit(5)==true)
		{
			Resource->SetSignalsToExit(false,5);
			break;
		}
		else
		{
			Obs=Resource->GetKinectObservation();
		}
		if(Obs->hasRangeImage==false)//uninitalized obs
		{
			mrpt::utils::sleep(33);
			continue;
		}
		RGB.showImage(Obs->intensityImage);
		DepthImage->setFromMatrix(Obs->rangeImage*(255/Obs->rangeImage.maxCoeff()),false);
		Depth.showImage(*DepthImage);

		MapP=Resource->GetMapImage();
		
		if(MapP->getWidth()==0)//unintialized map Image
		{
			mrpt::utils::sleep(33);
			continue;
		}

		MapP->scaleImage(*MapP,640,480);
		Map.showImage(*MapP);
		mrpt::utils::sleep(33);
	}
}