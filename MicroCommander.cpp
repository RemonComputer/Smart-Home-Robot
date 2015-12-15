#include "MicroCommander.h"


MicroCommander::MicroCommander()
{
}


MicroCommander::~MicroCommander(void)
{
}

void MicroCommander::UpdateOdometry(char Buffer,double & x,double & y,double & phi)
{
		x=0;
		y=0;
		phi=0;

		switch (Buffer)
		{
		case 's':
			//*actualOdometryReading=mrpt::poses::CPose2D(0,0,0);
			break;
		case 'f':
			//*actualOdometryReading=mrpt::poses::CPose2D(0,0.1,0);
			y=0.65/100;
			break;
		case 'c':
			//*actualOdometryReading=mrpt::poses::CPose2D(0,0,-mrpt::utils::DEG2RAD(1));
			phi=mrpt::utils::DEG2RAD(135/100);
			break;
		case 'a':
			//*actualOdometryReading=mrpt::poses::CPose2D(0,0,mrpt::utils::DEG2RAD(1));
			phi=-mrpt::utils::DEG2RAD(135/100);
			break;
		}

}

void MicroCommander::SendCommandToMicro(SharedResource*Resource)
{
	while (true)
	{
		char Direction;
		//CriticalSection->enter();
		if(Resource->GetSignalsToExit(4)==true)
		{
			Direction='s';//stop robot before exit
			Serial.WriteBuffer(&Direction,1);
			Resource->SetSignalsToExit(false,4);
			break;
		}
		else
		{
			Direction=Resource->GetDirection();
			//std::cout<<"Direction: "<<Direction<<"-Micro-Commander"<<std::endl;//for debugging only
			Serial.Write(&Direction,1);
			/*
			if(Direction=='s')
			{
				mrpt::slam::CActionRobotMovement2DPtr IncrementalMotion =Resource->GetIncrementalMotion();
				mrpt::poses::CPose2D EM;
				IncrementalMotion->poseChange->getMean(EM);
				std::cout<<"x: "<<EM.x()<<"y: "<<EM.y()<<"phi: "<<mrpt::utils::RAD2DEG(EM.phi())<<std::endl;
				mrpt::system::sleep(1000);
			}
			*/
		}
 		mrpt::system::sleep(10);
		double x,y,phi;
		UpdateOdometry(Direction,x,y,phi);
		Resource->IncrementIncrementalPosition(x,y,phi);
	}
}