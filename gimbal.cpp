
#include <stdint.h>
#include "mbed.h"
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <BNO055.h>
#include <initializations.h>
#include <definitions.h>
#include <prototypes.h>
#include <Mx28.h>

void runGimbal(void){

    // At 1ms update the gimbal.
    if (motorFlag && control.gimbalRun){
        motorFlag = 0;
        gimbal.servo(YAWID, control.yaw, 30); 
        wait(0.01); 
        gimbal.servo(PITCHID, control.pitch, 30);
        wait(0.01);
        gimbal.servo(ROLLID, control.roll, 30);
        control.gimbalRun = FALSE;
    }                               
}