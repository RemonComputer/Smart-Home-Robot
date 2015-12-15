#pragma once
#include <mrpt/base.h>
#include "SharedResource.h"
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/system/threads.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <iostream>

extern  mrpt::hwdrivers::CSerialPort Serial;

class MicroCommander
{
private:
	
	//mrpt::hwdrivers::CSerialPort*SerialPort;
void UpdateOdometry(char Buffer,double & x,double & y,double & phi);
public:
	MicroCommander();
	~MicroCommander(void);

	void SendCommandToMicro(SharedResource*Resource);
};

