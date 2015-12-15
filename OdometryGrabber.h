/*
 * OdometryGrabber.h
 *
 *  Created on: Jun 11, 2013
 *      Author: remon
 */
#pragma once
#include <mrpt/base.h>
#include "SharedResource.h"
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/system/threads.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/utils/utils_defs.h>

extern  mrpt::hwdrivers::CSerialPort Serial;
class OdometryGrabber {
private:

public:
	OdometryGrabber();
	~OdometryGrabber();

	void GrabOdometry(SharedResource*Resource);
};


