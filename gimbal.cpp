
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

static int i = 0;

void runGimbal(void){

    // At 1ms update the gimbal.
    if (motorFlag && control.gimbalRun){
        motorFlag = 0;
        switch(i){
            case(0):
            gimbal.servo(YAWID, control.yaw, 30);
            i++;
            break;
            case(1): 
            gimbal.servo(PITCHID, control.pitch, 30);
            i++;
            break;
            case(2):
            gimbal.servo(ROLLID, control.roll, 30);
            i = 0;
            break;
        }
        control.gimbalRun = FALSE;
    }                               
}