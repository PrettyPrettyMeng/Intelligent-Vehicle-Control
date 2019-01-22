/***************************************************************************

    file           : driver_parking.cpp
    author         : Jiayue Bao, Xinyun Gao
 created           : 2016/02/23
    description    : user module for CyberParking

 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <math.h>
#include "driver_parking.h"

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam (bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_parking(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_parking";	// name of the module (short).
	modInfo[0].desc    =  "user module for CyberParking" ;	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId    = 0;
	modInfo[0].index   = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tUserItf *itf = (tUserItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	printf("OK!\n");
	return 0;
}

// define the parameters
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _lotX, _lotY, _lotAngle, _carX, _carY, _caryaw;
static int _gearbox;
static bool _bFrontIn;

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm){
	/* write your own code here */
	
	_lotX = lotX;
	_lotY = lotY;
	_lotAngle = lotAngle;
	_bFrontIn = bFrontIn;
	_carX = carX;
	_carY = carY;
	_caryaw = caryaw;
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;

	//printf("speed %.3f yaw %.2f distance^2 %.3f\n", _speed, _caryaw, (_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) );
}

static int flag = 0;
static float k,b,dist;
static int hehe1=1;
static int hehe2=1;
static int flagt = 0;

static void userDriverSetParam (bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear){
    // calculate the distance between car center and parking space
	if(abs(_lotAngle)>(PI/2 - 0.05) && abs(_lotAngle)< (PI/2 + 0.05))
		dist = abs(_carX - _lotX);
	else
	{
		k = tan(_lotAngle);
		b = (_lotY - k * _lotX);
		dist = abs(k*_carX - _carY +b)/sqrt(k*k + 1);
	}

	
	if ( flagt == 1 ){
		*cmdAcc = 1;
		*cmdBrake = 0;
		*cmdGear = -1;
		*cmdSteer = (_yaw -atan2( _midline[10][0]+_width/3,_midline[10][1]))/3.14;
	}else
	{
		if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 0.5 ) {
            // using the speed to judge whether or not stop parking
			*cmdSteer = 20*(_lotAngle -_caryaw)/3.14 ;
			if(*cmdSteer > 1)
			    *cmdSteer = 1;
			if( _speed < 0.01){
			    *bFinished = true;
				flagt = 1;}
			else
			{*cmdBrake = 0.1;*cmdGear = 1;*cmdAcc = 0;}
			flag = 1; 
		}else if( ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 10)&& ( flagt ==2 )) {
            // very near the parking space, control the direction of the car to be the same as parking space, speed set to 2
			//*cmdSteer = 0;
			*cmdSteer = 20*(_lotAngle -_caryaw)/3.14 ;
			if( _speed > 2 ){*cmdBrake = 0.1;*cmdGear = 1;*cmdAcc = 0;}
			else if(_speed >0.07){*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			else{*bFinished = true;flagt=1; }
			flag = 2;
		} else if (((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 500 && dist <7.2)&&(flagt!=2)){
            // near the parking space, give a great direction change, speed set to 10
		    *cmdSteer = 1;
			if( _speed > 10 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			if (fabs(fabs(_caryaw)-fabs(_lotAngle))<0.015){
				flagt = 2;
			}
			flag = 3;
		} else if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 5000) {
            // adjust the car to the right side of the road, increase turning radius, speed set to 15
			*cmdSteer = (_yaw -atan2( _midline[10][0]+_width/3,_midline[10][1]))/3.14;
			if( _speed > 15 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			flag = 4;
		}
		else {
            // other cases
			*cmdAcc = 1;// accelarator 100%
		    *cmdBrake = 0;// brake set to 0
		    *cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14 ;// steer direction
		    *cmdGear = 1;// gear set to 1
			flag = 5;
	    }
	}

	
	if(*bFinished)
	{
		if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 10 && hehe1) {
            // very near the parking space, control the direction of the car to be the same as parking space
			*cmdSteer = 20*(_lotAngle -_caryaw)/3.14 ;
			if(*cmdSteer > 1)
			    *cmdSteer = 1;
			if( _speed > 2 ){*cmdBrake = 0.1;*cmdGear = -1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = -1;*cmdAcc = 0.1;}
		}
		else if ((_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) < 500 && dist <7.2 &&hehe2){
            //  near the parking space, give a great direction change
		    *cmdSteer = 1;
			if( _speed > 10 ){*cmdBrake = 0.2;*cmdGear = -1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = -1;*cmdAcc = 0.1;}
			hehe1=0;
		}
		else {
            // other cases
			*cmdAcc = 1;
		    *cmdBrake = 0;
		    *cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14 ;
		    *cmdGear = 1;
			hehe2=0;
	    }
	}
	


	printf("Steer:%.2f\tflag:%d\tspeed:%.2f\tdist:%.2f\tlotAngle:%.2f\tcaryaw:%.2f\tflagt:%d\n",*cmdSteer,flag,_speed,dist,_lotAngle,_caryaw,flagt);
	printf("Steer:%.2f\tflag:%d\tflagt:%d\n",*cmdSteer,flag,flagt);
}
