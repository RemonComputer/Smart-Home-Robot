/*
 * OdometryGrabber.cpp
 *
 *  Created on: Jun 11, 2013
 *      Author: remon
 */

#include "OdometryGrabber.h"

OdometryGrabber::OdometryGrabber() {
	
	//SerialPort=Port;//keep that if using one reading thread and one writting thread on same serial port has no race conditions
}

OdometryGrabber::~OdometryGrabber() {
	// TODO Auto-generated destructor stub
}

void OdometryGrabber::GrabOdometry(SharedResource*Resource){
	// look at http://reference.mrpt.org/svn/classmrpt_1_1poses_1_1_c_pose3_d.html to understand the coordinate system that we are working with

	// Prepare the "options" structure for using in computing the CMovements
	mrpt::slam::CActionRobotMovement2D::TMotionModelOptions opts=mrpt::slam::CActionRobotMovement2D::TMotionModelOptions::TMotionModelOptions();
	//opts.modelSelection=mrpt::slam::CActionRobotMovement2D::mmThrun;
	//opts.thrunModel.alfa3_trans_trans=0.1;//can this make problems ??

	const float Cos45=mrpt::math::cos(mrpt::utils::DEG2RAD(45));
	const float t=10/1000; //10 ms

	const int RotationPerSecondMax=50;//dummy value modify it
	const float Rwheels=5/100;//dummy value modify it, radius of wheels in meter
	const float Vmax=RotationPerSecondMax*Rwheels*M_2PIf; // means mutiply by 2*pi in float
	const float d=45/100;//distance from center of robot wheel to center of robot , look at drawing
	const int EncoderRatioNormalizer=mrpt::utils::pow((float)2,8)-1; //it is just the value that we divide by it to convert Encoder reading to actual measurement,
	//this value corresponds to 8-bit encoder reading so EncoderRatioNormalizer=2^8 -1 

	while (true)
	{
		//char Buffer[5];//buffer for putting data from serial port, 
		// I will assume that Buffer[0]=EncoderReadingofV1 , Buffer[1]=EncoderReadingofV2 , Buffer[2]=EncoderReadingofV3, Buffer[3]=EncoderReadingofV4 
		// were V1 , V2 , V3 , V4 are the wheels in drawing , Buffer[5]=DirectionSignal that the robot executed, in order that we would know the velocities directions 
		// 'f' for forward motion , 's' stop , 'c' clockwise rotation , 'a' anti-clockwise rotation

		 //disabled for debugging purpose
		//read from serial port
		char Buffer;
		int SizeOfDataAcuallyReaded=Serial.Read(&Buffer,1);//endianess dependant watch out
		if (SizeOfDataAcuallyReaded!=1)//for debugging issue remove it in the final version
		{
			throw "\ncouldn't read 1 bytes from serial port\n";
		}
		/**/
				
		mrpt::poses::CPose2DPtr actualOdometryReading=mrpt::poses::CPose2D::Create(); //(1,1,0);//(x, y, phi);
		double x=0;
		double y=0;
		double phi=0;

		switch (Buffer)
		{
		case 's':
			//*actualOdometryReading=mrpt::poses::CPose2D(0,0,0);
			break;
		case 'f':
			//*actualOdometryReading=mrpt::poses::CPose2D(0,0.1,0);
			y=0.65;
			break;
		case 'c':
			//*actualOdometryReading=mrpt::poses::CPose2D(0,0,-mrpt::utils::DEG2RAD(1));
			phi=mrpt::utils::DEG2RAD(135);
			break;
		case 'a':
			//*actualOdometryReading=mrpt::poses::CPose2D(0,0,mrpt::utils::DEG2RAD(1));
			phi=-mrpt::utils::DEG2RAD(135);
			break;
		}

		mrpt::slam::CActionRobotMovement2DPtr Movement=mrpt::slam::CActionRobotMovement2D::Create();
		Movement->estimationMethod=mrpt::slam::CActionRobotMovement2D::TEstimationMethod::emOdometry;
		Movement->computeFromOdometry(*actualOdometryReading,opts);

		//Movement->rawOdometryIncrementReading=*actualOdometryReading;
		Resource->IncrementIncrementalPosition(x,y,phi);
		
		if (Resource->GetSignalsToExit(1)==true)//check on exit signal 
		{
			Resource->SetSignalsToExit(false,1);
			break;
		}
		else if(true)//Resource->GetPersistantSynchronizationMechanism(0)==true&&Resource->GetPersistantSynchronizationMechanism(2)==true//check on persistant condition
		{
			Resource->SetPersistantSynchronizationMechanism(false,0);//indicate that it is executed
			//write the Movement in IncrementalMotion
			//disabled updating of incremental motion for debugging
			//mrpt::slam::CActionRobotMovement2DPtr MovementPtr(&Movement);
			std::cout<<"Diretion: "<<Buffer<<" -Grabbed Odometry"<<std::endl;//for debugging purpose only
			Resource->SetIncrementalMotion(Movement);//i hope no race condition happens here or in any value copying using value type assignment
			mrpt::system::sleep(10);
		}
	}
}