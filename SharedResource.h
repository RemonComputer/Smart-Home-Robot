/*
 * SharedResource.h
 *
 *  Created on: Jun 11, 2013
 *      Author: remon
 */
#pragma once
#include <mrpt/base.h>
//#include <mrpt/hwdrivers/CSerialPort.h>  //remove if using one reading thread and one writting thread on same serial port has no race conditions
#include <mrpt/utils/CImage.h>
#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/slam/CActionRobotMovement2D.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/synch/CThreadSafeVariable.h>
#include <qmutex.h>

#include <opencv2/opencv.hpp>

class SharedResource
{
private:
	mrpt::slam::CObservation3DRangeScanPtr KinectObservation;
	cv::Mat DepthImage;
	cv::Mat RGBImage;
	QMutex KinectObservationMutex;

	mrpt::slam::CActionRobotMovement2DPtr IncrementalMotion;
	mrpt::poses::CPose2D IncrementalPosition;
	QMutex IncrementalMotionMutex;

	double AbsolutePosition[3];// x y yaw(or theta)
	QMutex AbsolutePositionMutex;

	mrpt::utils::CImagePtr MapImage;
	QMutex MapImageMutex;

	//mrpt::hwdrivers::CSerialPort SerialPort; //remove if using one reading thread and one writting thread on same serial port has no race conditions
	
	char Direction;
	QMutex DirectionMutex;
	
	bool ManualControl;
	QMutex ManualControlMutex;
	
	bool GoingToDestination;
	QMutex GoingToDestinationMutex;
	
	float DestinationLocation[2];// [x,y] , location for destination if the robot is going to certain destination
	QMutex DestinationLocationMutex;
	
	bool SignalsToExit[9];//when any of them is one the corresponding process exit and assign that to zero
	QMutex SignalsToExitMutex[9];
	
	bool PersistantSynchronizationMechanism[3];
	QMutex PersistantSynchronizationMechanismMutex[3];

	mrpt::slam::COccupancyGridMap2DPtr Map;
	QMutex MapMutex;

public:
	SharedResource();
	~SharedResource();

	mrpt::slam::CObservation3DRangeScanPtr GetKinectObservation();
	void SetKinectObservation(mrpt::slam::CObservation3DRangeScanPtr V);

	void GetKinectObservationDepthOnly(cv::Mat & DepthImage);
	void GetKinectObservation(cv::Mat & DepthImage,cv::Mat & RGBImage);
	void SetKinectObservation(cv::Mat & DepthImage,cv::Mat & RGBImage);

	mrpt::slam::CActionRobotMovement2DPtr GetIncrementalMotion();
	void IncrementIncrementalPosition(double x,double y,double phi);
	void SetIncrementalMotion(mrpt::slam::CActionRobotMovement2DPtr V);

	void GetAbsolutePosition(double &x,double &y,double &yaw);// x y yaw(or theta)
	void SetAbsolutePosition(double x,double y,double yaw);

	mrpt::utils::CImagePtr GetMapImage();
	void SetMapImage(mrpt::utils::CImagePtr V);

	char GetDirection();
	void SetDirection(char V);
	
	bool GetManualControl();
	void SetManualControl(bool V);
	
	bool GetGoingToDestination();
	void SetGoingToDestination(bool V);
	
	void GetDestinationLocation(float &x, float &y);// [x,y] , location for destination if the robot is going to certain destination
	void SetDestinationLocation(float x, float y);
	
	bool GetSignalsToExit(int index);//when any of them is one the corresponding process exit and assign that to zero
	void SetSignalsToExit(bool V,int index);
	
	bool GetPersistantSynchronizationMechanism(int index);
	void SetPersistantSynchronizationMechanism(bool V,int index);

	mrpt::slam::COccupancyGridMap2DPtr GetMap();
	void SetMap(mrpt::slam::COccupancyGridMap2DPtr V);
};

