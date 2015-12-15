#pragma once
#include <mrpt/base.h>
#include "SharedResource.h"
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/system/threads.h>
#include <mrpt/gui/CDisplayWindow.h>


class Viewer
{
private:
	

public:
	Viewer();
	~Viewer(void);

	void UpdateViews(SharedResource*Resource);
};

