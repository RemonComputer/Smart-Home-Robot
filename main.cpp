/**/

   

/*
#include <stdlib.h>
#ifdef _DEBUG
#define new DEBUG_NEW
#endif
*/

//#include <crtdbg.h>


#include <mrpt/base.h>
#include <mrpt/system/threads.h>
#include <mrpt/hwdrivers.h>

#include <omp.h>


#include "KinectGrabber.h"
#include "OdometryGrabber.h"
#include "SLAM.h"
#include "Learner.h"
#include "MicroCommander.h"
#include "Viewer.h"
#include "PathDirector.h"
#include "Explorer.h"
#include "SharedResource.h"
/**/

//deugging dlls and defines
#define _CRTDBG_MAP_ALLOC  
#define _CrtDumpMemoryLeaks
#include <stdlib.h>
#include <crtdbg.h>
#ifdef _DEBUG
   #ifndef DBG_NEW
      #define DBG_NEW new ( _NORMAL_BLOCK , __FILE__ , __LINE__ )
      #define new DBG_NEW
   #endif

#endif  // _DEBUG

SharedResource R;//global Shared Resource
mrpt::hwdrivers::CSerialPort Serial("COM13",true);//open the serial port //shared across micro-commander and odometry grabber //change to COM13 when Arduino Connected

void main()
{
	_CrtSetDbgFlag ( _CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF );//debug function to dump leak output at Program console screen 

	Serial.setConfig(9600,0,8,1);//configure serial port //enable when arduino connected
	KinectGrabber K;//enable after installing Libfreenect and libusb
	
	OdometryGrabber O; //enable when arduino connected
	SLAM S;
	Learner L;
	Viewer V;
	PathDirector P;
	Explorer E;
	MicroCommander M;
		
	omp_set_num_threads(12);
#pragma omp parallel sections 
	{
		
		#pragma omp section
		{
			K.GrabKinectObservation(&R);
		}
		/*
		#pragma omp section
		{
			//O.GrabOdometry(&R);
		}
		*/
		#pragma omp section
		{
			S.MakeSlam(&R);
		}
		#pragma omp section
		{
			M.SendCommandToMicro(&R);
		}
		#pragma omp section
		{
			L.Learn(&R);
		}
		#pragma omp section
		{
			E.UpdateDirection(&R);
		}
		/*
		#pragma omp section
		{
			V.UpdateViews(&R);
		}
		*/
		/*
		#pragma omp section
		{
			//P.UpdateDirection(&R);
		}
		*/
		/*
		#pragma omp section
		{
			while (true)
			{
				mrpt::slam::CActionRobotMovement2DPtr IncrementalMotion =R.GetIncrementalMotion();
				mrpt::poses::CPose2D EM;
				IncrementalMotion->poseChange->getMean(EM);
				std::cout<<"x: "<<EM.x()<<"y: "<<EM.y()<<"phi: "<<mrpt::utils::RAD2DEG(EM.phi())<<std::endl;
				mrpt::system::sleep(1000);

			}
		}
		*/
		#pragma omp section
		{
			while (true)
			{
				//std::cout<<"Main Thread "<<std::endl;//dummy line for debugging remove it in the final version
				std::cout<<"choose one of the options "<<std::endl;
				std::cout<<"---------------------------"<<std::endl;
				std::cout<<"w -> move forward (manual control)"<<std::endl;
				std::cout<<"s -> stop (manual control)"<<std::endl;
				std::cout<<"a -> clockwise rotation (manual control)"<<std::endl;
				std::cout<<"d -> anti-clockwise rotation (manual control)"<<std::endl;
				std::cout<<"e -> enable automatic navigation"<<std::endl;
				std::cout<<"g -> go to certain location "<<std::endl;
				std::cout<<"x -> exit "<<std::endl;
				char Choice;
				std::cin>>Choice;
				switch(Choice)
				{
				case 'w':
					R.SetManualControl(true);
					R.SetGoingToDestination(false);
					R.SetDirection('f');
					break;
				case 's':
					R.SetManualControl(true);
					R.SetGoingToDestination(false);
					R.SetDirection('s');
					break;
				case 'a':
					R.SetManualControl(true);
					R.SetGoingToDestination(false);
					R.SetDirection('c');
					break;
				case 'd':
					R.SetManualControl(true);
					R.SetGoingToDestination(false);
					R.SetDirection('a');
					break;
				case 'e':
					R.SetManualControl(false);
					R.SetGoingToDestination(false);
					R.SetDirection('f');
					break;
				case 'g':
					std::cout<<"Enter x location: ";
					float x,y;
					std::cin>>x;
					std::cout<<"Enter y location: ";
					std::cin>>y;
					std::cout<<"Going to Destination ...."<<std::endl;
					R.SetManualControl(false);
					R.SetDestinationLocation(x,y);
					R.SetGoingToDestination(true);
					break;
				case 'x':			
					for(int i=0;i<9;i++)
						R.SetSignalsToExit(true,i);
					R.SetSignalsToExit(false,7);//signal for main thread
					goto EXIT_FOR_AND_WHILE;
					break;
				}
				
				mrpt::system::sleep(20);//default 20
			}
			EXIT_FOR_AND_WHILE:;
			
		}
	}
	/**/
}