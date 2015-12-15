#include "SharedResource.h"

SharedResource::SharedResource()
{
	for(int i=0;i<3;i++)
		AbsolutePosition[0]=0;
	Direction='f';
	GoingToDestination=false;
	ManualControl=false;
	IncrementalMotion=mrpt::slam::CActionRobotMovement2D::Create();
	IncrementalPosition=mrpt::poses::CPose2D(0,0,0);
	KinectObservation=mrpt::slam::CObservation3DRangeScan::Create();
	MapImage=mrpt::utils::CImage::Create();
	for(int i=0;i<9;i++)
		SignalsToExit[i]=false;
	DestinationLocation[0]=0;
	DestinationLocation[1]=0;
	for(int i=0;i<3;i++)
		PersistantSynchronizationMechanism[i]=true;
	Map=mrpt::slam::COccupancyGridMap2D::Create();
}

SharedResource::~SharedResource()
{
	/*
	KinectObservation->~CObservation3DRangeScan();
	MapImage->~CImage();
	IncrementalMotion->~CActionRobotMovement2D();
	Map->~COccupancyGridMap2D();

	AbsolutePositionMutex.~QMutex();
	DirectionMutex.~QMutex();
	GoingToDestinationMutex.~QMutex();
	ManualControlMutex.~QMutex();
	IncrementalMotionMutex.~QMutex();
	KinectObservationMutex.~QMutex();
	MapImageMutex.~QMutex();
	for(int i=0;i<9;i++)
		SignalsToExitMutex[i].~QMutex();
	DestinationLocationMutex.~QMutex();
	for(int i=0;i<3;i++)
		PersistantSynchronizationMechanismMutex[i].~QMutex();
	MapImageMutex.~QMutex();
	*/

}

mrpt::slam::CObservation3DRangeScanPtr SharedResource::GetKinectObservation()
{
	KinectObservationMutex.lock();
	mrpt::slam::CObservation3DRangeScanPtr V;
	if(KinectObservation->hasRangeImage==true)
	{
		V=(mrpt::slam::CObservation3DRangeScanPtr)KinectObservation->duplicateGetSmartPtr();
	}
	else
	{
		V=mrpt::slam::CObservation3DRangeScan::Create();
	}
	KinectObservationMutex.unlock();
	return V;
}
	void SharedResource::SetKinectObservation(mrpt::slam::CObservation3DRangeScanPtr V)
	{
		KinectObservationMutex.lock();
		KinectObservation=(mrpt::slam::CObservation3DRangeScanPtr)V->duplicateGetSmartPtr();
		KinectObservationMutex.unlock();
	}

	void SharedResource::GetKinectObservationDepthOnly(cv::Mat & DepthImage)
	{
		KinectObservationMutex.lock();
		DepthImage=this->DepthImage.clone();
		KinectObservationMutex.unlock();
	}
	void SharedResource::GetKinectObservation(cv::Mat & DepthImage,cv::Mat & RGBImage)
	{
		KinectObservationMutex.lock();
		DepthImage=this->DepthImage.clone();
		RGBImage=this->RGBImage.clone();
		KinectObservationMutex.unlock();
	}
	void SharedResource::SetKinectObservation(cv::Mat & DepthImage,cv::Mat & RGBImage)
	{
		KinectObservationMutex.lock();
		this->DepthImage=DepthImage.clone();
		this->RGBImage=RGBImage.clone();
		KinectObservationMutex.unlock();
	}

	mrpt::slam::CActionRobotMovement2DPtr SharedResource::GetIncrementalMotion()
	{
		/*
		IncrementalMotionMutex.lock();
		mrpt::slam::CActionRobotMovement2DPtr V=(mrpt::slam::CActionRobotMovement2DPtr)IncrementalMotion->duplicateGetSmartPtr();
		IncrementalMotionMutex.unlock();
		return V;
		*/
		mrpt::slam::CActionRobotMovement2D::TMotionModelOptions opts=mrpt::slam::CActionRobotMovement2D::TMotionModelOptions::TMotionModelOptions();
		mrpt::slam::CActionRobotMovement2DPtr Movement=mrpt::slam::CActionRobotMovement2D::Create();
		Movement->estimationMethod=mrpt::slam::CActionRobotMovement2D::TEstimationMethod::emOdometry;
		IncrementalMotionMutex.lock();
		Movement->computeFromOdometry(IncrementalPosition,opts);
		IncrementalPosition=mrpt::poses::CPose2D(0,0,0);
		IncrementalMotionMutex.unlock();

		return Movement;
	}
	void SharedResource::SetIncrementalMotion(mrpt::slam::CActionRobotMovement2DPtr V)
	{
		IncrementalMotionMutex.lock();
		IncrementalMotion=(mrpt::slam::CActionRobotMovement2DPtr)V->duplicateGetSmartPtr();
		IncrementalMotionMutex.unlock();
	}
	void SharedResource::IncrementIncrementalPosition(double x,double y,double phi)
	{
		IncrementalMotionMutex.lock();

		IncrementalPosition.x_incr(x);
		IncrementalPosition.y_incr(y);
		IncrementalPosition.phi_incr(phi);
		
		IncrementalMotionMutex.unlock();
	}

	void SharedResource::GetAbsolutePosition(double &x,double &y,double &yaw)// x y yaw(or theta)
	{
		AbsolutePositionMutex.lock();
		x=AbsolutePosition[0];
		y=AbsolutePosition[1];
		yaw=AbsolutePosition[2];
		AbsolutePositionMutex.unlock();
	}
	void SharedResource::SetAbsolutePosition(double x,double y,double yaw)
	{
		AbsolutePositionMutex.lock();
		AbsolutePosition[0]=x;
		AbsolutePosition[1]=y;
		AbsolutePosition[2]=yaw;
		AbsolutePositionMutex.unlock();
	}

	mrpt::utils::CImagePtr SharedResource::GetMapImage()
	{
		MapImageMutex.lock();
		mrpt::utils::CImagePtr V=(mrpt::utils::CImagePtr)MapImage->duplicateGetSmartPtr(); 
		MapImageMutex.unlock();
		return V;
	}
	void SharedResource::SetMapImage(mrpt::utils::CImagePtr V)
	{
		MapImageMutex.lock();
		MapImage=(mrpt::utils::CImagePtr)V->duplicateGetSmartPtr();
		MapImageMutex.unlock();
	}

	char SharedResource::GetDirection()
	{
		DirectionMutex.lock();
		char V=Direction;
		DirectionMutex.unlock();
		return V;
	}
	void SharedResource::SetDirection(char V)
	{
		DirectionMutex.lock();
		Direction=V;
		DirectionMutex.unlock();
	}
	
	bool SharedResource::GetManualControl()
	{
		ManualControlMutex.lock();
		bool V=ManualControl;
		ManualControlMutex.unlock();
		return V;
	}
	void SharedResource::SetManualControl(bool V)
	{
		ManualControlMutex.lock();
		ManualControl=V;
		ManualControlMutex.unlock();
	}
	
	bool SharedResource::GetGoingToDestination()
	{
		GoingToDestinationMutex.lock();
		bool V=GoingToDestination;
		GoingToDestinationMutex.unlock();
		return V;
	}
	void SharedResource::SetGoingToDestination(bool V)
	{
		GoingToDestinationMutex.lock();
		GoingToDestination=V;
		GoingToDestinationMutex.unlock();
	}
	
	void SharedResource::GetDestinationLocation(float &x, float &y)// [x,y] , location for destination if the robot is going to certain destination
	{
		DestinationLocationMutex.lock();
		x=DestinationLocation[0];
		y=DestinationLocation[1];
		DestinationLocationMutex.unlock();
	}
	void SharedResource::SetDestinationLocation(float x, float y)
	{
		DestinationLocationMutex.lock();
		DestinationLocation[0]=x;
		DestinationLocation[1]=y;
		DestinationLocationMutex.unlock();
	}
	
	bool SharedResource::GetSignalsToExit(int index)//when any of them is one the corresponding process exit and assign that to zero
	{
		SignalsToExitMutex[index].lock();
		bool V= SignalsToExit[index];
		SignalsToExitMutex[index].unlock();
		return V;
	}
	void SharedResource::SetSignalsToExit(bool V,int index)
	{
		SignalsToExitMutex[index].lock();
		SignalsToExit[index]=V;
		SignalsToExitMutex[index].unlock();
	}
	
	bool SharedResource::GetPersistantSynchronizationMechanism(int index)
	{
		PersistantSynchronizationMechanismMutex[index].lock();
		bool V=PersistantSynchronizationMechanism[index];
		PersistantSynchronizationMechanismMutex[index].unlock();
		return V;
	}
	void SharedResource::SetPersistantSynchronizationMechanism(bool V,int index)
	{
		PersistantSynchronizationMechanismMutex[index].lock();
		PersistantSynchronizationMechanism[index]=V;
		PersistantSynchronizationMechanismMutex[index].unlock();
	}

	mrpt::slam::COccupancyGridMap2DPtr SharedResource::GetMap()
	{
		MapMutex.lock();
		mrpt::slam::COccupancyGridMap2DPtr V=(mrpt::slam::COccupancyGridMap2DPtr)Map->duplicateGetSmartPtr();
		MapMutex.unlock();
		return V;
	}
	void SharedResource::SetMap(mrpt::slam::COccupancyGridMap2DPtr V)
	{
		MapMutex.lock();
		Map=(mrpt::slam::COccupancyGridMap2DPtr)V->duplicateGetSmartPtr();
		MapMutex.unlock();
	}
