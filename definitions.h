#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include "mbed.h"
#include <BNO055.h>
#include <Mx28.h>

// Generic definitions
#define     TRUE            1
#define     FALSE           0

// ROS definitions
#define     ROSDATALENGTH   5
#define     ROSCONTROL      0
#define     ROSYAW          1
#define     ROSPITCH        2
#define     ROSROLL         3
#define     ROSHEIGHT       4

// Gimbal definitions
#define     DYNASCALE       11.377777777777
#define     DYNASCALE2      13
#define     DYNASCALE3      17
#define     DYNAPROPCONST   0.1
#define     DYNASTEPSIZE    3
#define     IMUBUFFER       10
#define     YAWID           1
#define     PITCHID         2
#define     ROLLID          3
#define     YAWZERO         2015
#define     PITCHZERO       2350
#define     ROLLZERO        2302
#define     YAWMIN          0       // -177 degrees.
#define     YAWMAX          4063    // +177 degrees.
#define     CW              1
#define     CCW             0
#define     PITCHMIN        1326    // -90 degrees.
#define     PITCHMAX        3374    // +90 degrees.
#define     ROLLMIN         1278    // -90 degrees.
#define     ROLLMAX         3326    // +90 degrees.

// Lift definitions
#define     LIFTUP          0
#define     LIFTDOWN        1
#define     ROSCHECKTIME    0.01
#define     HALLCHECKTIME   100
#define     STALLTIME       200 //Was 250
#define     LIFTSCALE       4400
#define     LIFTPROPCONST   0.01
#define     LIFTRAMPCONST   0.002f
#define     LIFTHEIGHTMAX   2640    // 2640/4400 = 0.6m
#define     LIFTHEIGHTMIN   132     // 132/4400 = 0.05m


// Classes
struct  fields{
int     yaw;
int     yawSpeed;
int     pitch;
int     pitchSpeed;
int     roll;
int     rollSpeed;
int     height;
bool    stabilize;
bool    masterOn;
bool    liftRun;
bool    gimbalRun;
bool    shutdown;
bool    initialize;
};
struct  fields_float{
float   yaw;
float   yawSpeed;
float   pitch;
float   pitchSpeed;
float   roll;
float   rollSpeed;
float   height;
bool    stabilize;
bool    masterOn;
bool    liftRun;
bool    gimbalRun;
bool    shutdown;
bool    initialize;
};

extern struct           fields          control;
extern struct           fields_float    rosInput;
extern int              currentPosition; 
extern bool             liftFlag;
extern bool             motorFlag;
extern bool             imuFlag;
extern bool             rosFlag;
extern float            tempyaw;
extern float            temppitch;
extern float            temproll;
extern float            tempheight;
extern Ticker           hallInt;
extern Ticker           dynaInt;
extern Ticker           imuInt;
extern Ticker           rosInt;
extern PwmOut           liftSpeed;
extern DigitalOut       liftDirection;
extern DigitalIn        hallSensor1;
extern DigitalIn        hallSensor2;
extern DigitalOut       LED;
extern BNO055           IMU;
extern DynamixelClass   gimbal;

#endif