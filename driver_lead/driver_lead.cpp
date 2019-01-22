/***************************************************************************

                 file : driver_cruise.cpp
             author : Jiayue Bao
             email : lily.jybao@sjtu.edu.cn
             created: 2015/03/05
    description : user module for CyberFollow

 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_lead.h"

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm, float DistanceFromStart, int laps);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_lead(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_lead";	// name of the module (short).
	modInfo[0].desc    =  "leader module for CyberFollow" ;	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId    = 0;
	modInfo[0].index   = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tLeaderItf *itf = (tLeaderItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}

// Define the parameters
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm,  _DistanceFromStart;
static int _gearbox, _laps;

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm,  float DistanceFromStart, int laps){
	
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	_DistanceFromStart = DistanceFromStart;
	_laps = laps;

	printf("speed %f DFS %f lap %d 10m far target(%f, %f)\n", _speed, _DistanceFromStart, _laps, _midline[10][0], _midline[10][1]);
	
}

static float ki,k; // define variables

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear){
	/*******************************Path Design********************************/
	switch(_laps)
	{
	    case 1: ki = 0;break;           // normal driving for the first round
	    case 2: ki += 0.01 * PI; break; // curve travel for the second round
	    default:break;
	}
	k = sin(ki)/2.0;

	/*******************************Vehicle Control********************************/
	
	*cmdAcc = 0.8;// accelerator set to 80%
	*cmdBrake = 0;// no brake
	*cmdSteer = (_yaw -8*atan2( _midline[30][0]+k*_width ,_midline[30][1]))/3.14 ;// set the steer direction
	*cmdGear = 1;// gear set to 1
	
}
