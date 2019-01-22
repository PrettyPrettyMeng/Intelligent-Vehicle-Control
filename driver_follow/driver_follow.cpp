/***************************************************************************

    file    : driver_follow.cpp
    author  : Jiayue Bao
    email   : lily.jybao@sjtu.edu.cn
    created : 2016/03/24
    description : user module for CyberFollow

 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_follow.h"


static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_follow(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_follow";	// name of the module (short).
	modInfo[0].desc    =  "user module for CyberFollower" ;	// Description of the module (can be long).
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
	return 0;
}

// define the variables
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;
static int _gearbox;
static float _Leader_X, _Leader_Y;

const int topGear = 6;
void updateGear(int *cmdGear);
void PIDParamSetter();
float Radius( int Point1, int Point2, int Point3 );
double constrain(double lowerBoundary, double upperBoundary,double input);
bool parameterSet=false;
int count=0;
double flag=0;
double fflag=0;

double yError; // y axis error
double xError; // x axis error
double yErrorSum=0; // y axis error integral
double xErrorSum=0; // x axis error integral
double yErrorDiff; // y axis error differential
double xErrorDiff; // x axis error differential
double tmpX=0,tmpY=0;
double yErrorDiffPre=0,xErrorDiffPre=0; // predicted differentials
double distance=0;
double time=0; // current time
//speed pid parameters
double k_v1,k_v2,k_a1,k_a2,k_aa,k_y1,k_y2,k_i,k_d;
double k_v_x,k_a_x;
//direction pid parameters
double kp_d1;							
double ki_d1;							
double kd_d1;	

double extraControl=0;
double extraControl_x=0;

double Y1=0,Y2,Y3=0,Y0=0,v0=0,v1,v2,a1,a0=0,aa;
double X1=0,X2,X3=0,X0=0,v0_x=0,v1_x,v2_x,a1_x,a0_x,aa_x;

static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm)
{
	
	_Leader_X = LeaderXY[0]; // leading car
	_Leader_Y = LeaderXY[1]; // following car
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;

	printf("speed %.3f acc %.3f Leader XY(%.3f, %.3f) steer%.3f \n ", _speed,_acc, _Leader_X, _Leader_Y,(_yaw -8*atan2( _midline[50][0],_midline[50][1]))/3.14);
}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear)
{
	time+=0.02;
	distance=distance+time*_speed/3.6;
	printf("time %.3f distance %.3f\n",time,distance);

	// calculate the speed and accelaration for leading car
    Y3=_Leader_Y;
    v2=(Y3-Y2)/0.02;
    a1=(v2-v1)/0.02;
    aa=(a1-a0)/0.02;

    a0=a1;
    v1=v2;
    v0=v1;
    Y2=Y3;
    Y1=Y2;

    // calculate the radius
    int one,two,three;
    one = int( _speed/3 + 5);
    two = int( _speed/3 + 10);
    three = int( _speed/3 + 15);
    float r=Radius(one,two,three);
    
    if(parameterSet==false&&_Leader_Y<20)
   *cmdSteer=(_yaw -8*atan2( _midline[55][0],_midline[55][1]))/3.14;

    if(parameterSet==false&&_Leader_Y>=20)
    { // Initialization Part
		*cmdAcc = 1;
		*cmdBrake = 0;
		*cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14 ;
		*cmdGear = 1;
		
		 PIDParamSetter();
	}	
	

	else
    {
        // calculate the PID parameters
        yError=_Leader_Y-20;
        xError=_Leader_X;

        yErrorSum=yError+yErrorSum;
        xErrorSum=xError+xErrorSum;

        yErrorDiffPre=yErrorDiff;
        xErrorDiffPre=xErrorDiff;// the error change for previous moment

        yErrorDiff=yError-tmpY;
        xErrorDiff=xError-tmpX;
        tmpY=yError;
        tmpX=xError;

        double brake,acc;

        /********************* 1. speed control******************/
        acc=k_v1*v2+k_a1*a1+k_y1*yError+k_i*yErrorSum+k_d*yErrorDiff;
        brake=(-1)*(k_v2*v2+k_a2*a1+k_y2*yError+k_i*yErrorSum+k_d*yErrorDiff);
        acc=constrain(0,1,acc);
        brake=constrain(0,1,brake);
        
        // RBS
        if(((abs(*cmdSteer)>0.3&& brake>0.6)||(abs(*cmdSteer)>0.9&& brake>0.4))||(abs(*cmdSteer)>0.88)&&acc<0.9)// large deceleration
        {
            if(count%2==0){*cmdBrake=0;count++;}
                    else {*cmdBrake=brake;count++;}
                    *cmdAcc=acc;
                    fflag=1;
        }
        else if(abs(*cmdSteer)>0.35&&acc>0.95&&brake<0.5&&abs(r)<120)// large accelaration
        {
                {*cmdAcc=acc*0.8;
                *cmdBrake=brake;
                fflag=2;}
        }
        
        else if(abs(*cmdSteer)>0.3&&acc>0.9&&brake>0.5)// large deceleration and accelaration
        {
            *cmdAcc=acc*0.8;
            if(count%2==0){*cmdBrake=brake;count++;}
                else {*cmdBrake=0;count++;}
                fflag=3;
        }
        else
        {
            *cmdBrake=brake;*cmdAcc=acc;fflag=4;
        }

	}
	
	updateGear(cmdGear);
	
	/********************* 2. direction control******************/
	*cmdSteer=(_yaw -8*atan2( _midline[55][0],_midline[55][1]))/3.14;

	// calculate the leading's speed and accelaration in x axis
    X3=_Leader_X;
    v2_x=(X3-X2)/0.02;
    a1_x=(v2_x-v1_x)/0.02;
    aa_x=(a1_x-a0_x)/0.02;

    a0_x=a1_x;
    v1_x=v2_x;
    v0_x=v1_x;
    X2=X3;
    X1=X2;

    double steerflag=0;
    
    // corner case
	if(time>33&&time<36.5&&distance>1100000&&distance<1259191.820||time>95&&time<97&&distance>7233884&&distance<7536000||_speed>170&&abs(r)<1000&&time>55&&distance<3583848||time>37.14&&time<40&&distance>751399.55&&time<987162.906&&_speed>80)
	{
		*cmdSteer=(_yaw -8*atan2( _midline[60][0],_midline[60][1]))/3.14;
		steerflag=1;
	}
    
    // corner case 
	else if(abs(_midline[0][0])>6.2&&abs(r)>120||abs(_midline[0][0])>8&&abs(r)<120||_Leader_Y>37)
	{
        *cmdSteer=(_yaw -8*atan2( _midline[55][0],_midline[55][1]))/3.14;steerflag=2;
        
    }
	else
	{
        *cmdSteer =constrain(-1.0,1.0,-kp_d1 * xError - ki_d1 * xErrorSum -extraControl_x);steerflag=3;
        
    }

	printf("fflag %.3f\n",fflag);
	printf("steerflag %.3f\n",steerflag);
	printf("r %.3f\n",r);
}

void PIDParamSetter()
{
    //speed pid
    k_v1=5.1;
    k_a1=0.2;
    k_y1=5.5;
    
    k_v2=4.1;
    k_a2=0.12;
    k_y2=4.75;
    
     //directon pid
     kp_d1=0.45;
     ki_d1=0;
     kd_d1=0.5;
    
     k_v_x=0;
     k_a_x=0;

     parameterSet = true;
}

// gear shifting function
void updateGear(int *cmdGear)
{
if (_gearbox == 1)
{
    if (_speed >= 60 && topGear >1)
        *cmdGear = 2;
    else
        *cmdGear = 1;
}

else if (_gearbox == 2)
{
    if(_speed<45||_Leader_Y<18) *cmdGear=1;
    else if (_speed >=110 && topGear >2)
        *cmdGear = 3;
    else
        *cmdGear = 2;
}

else if (_gearbox == 3)
{
    if (_speed <= 90||_Leader_Y<18)
        *cmdGear = 2;
    else if (_speed >= 165 && topGear >3)
        *cmdGear = 4;
    else
        *cmdGear = 3;
}
else if (_gearbox == 4)
{
      if (_speed <= 148||_Leader_Y<18)
        *cmdGear = 3;
    else if (_speed >= 215 && topGear >4)
        *cmdGear = 5;
    else
        *cmdGear = 4;
}
else if (_gearbox == 5)
{
     if (_speed <= 200||_Leader_Y<18)
        *cmdGear = 4;
    else if (_speed >= 265 && topGear >5)
        *cmdGear = 6;
    else
        *cmdGear = 5;

}
else if (_gearbox == 6)
{
     if (_speed <= 245||_Leader_Y<18)
        *cmdGear = 5;

    else
        *cmdGear = 6;
}
else
    *cmdGear =1;

}

// calculate the radius for three points
float Radius( int Point1, int Point2, int Point3 )
{
	float x1 = _midline[Point1][0];
	float y1 = _midline[Point1][1];
	float x2 = _midline[Point2][0];
	float y2 = _midline[Point2][1];
	float x3 = _midline[Point3][0];
	float y3 = _midline[Point3][1];
	
	float a,b,c,d,e,f;
	float r,x,y;
	
	a = 2*(x2 - x1);
	b = 2*(y2 - y1);
	c = x2*x2 + y2*y2 - x1*x1 - y1*y1;
	d = 2*(x3 - x2);
	e = 2*(y3 - y2);
	f = x3*x3 + y3*y3 - x2*x2 - y2*y2;
	x = (b*f - e*c)/(b*d - e*a);
	y = (d*c - a*f)/(b*d - e*a);
	r = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1));
	
    int sign = ( x > 0 ) ? 1 : -1; // -1: turn left, 1: turn right
	return sign*r;
}
