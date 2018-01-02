
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

void runLift(void){
    
    //static int      liftPosition = 0;
    bool            stall = FALSE;
    static bool     ramp = FALSE;
    static float    rampSpeed = 0.0;
    static float    propSpeed = 0.0;
    static int      speedCount = 20;
    
    if (liftFlag && control.liftRun){
        liftFlag = 0;
    // If the lift has not started moving, start ramp up and direction.
        if (liftSpeed.read() == 0.0f && ramp == FALSE){
            ramp = TRUE;
            if (control.height > currentPosition){
                liftDirection.write(LIFTUP);
            } else {
                liftDirection.write(LIFTDOWN);
            }
        }
        
        // Update the speed
        if (speedCount == 20){
            speedCount = 0;
            propSpeed = 0.2 + (abs(control.height - currentPosition) * LIFTPROPCONST);
            if (ramp){
                rampSpeed = rampSpeed + LIFTRAMPCONST;
                if ((rampSpeed < propSpeed) && (rampSpeed < 1.0f)){
                    liftSpeed.write(rampSpeed);
                } else {
                    liftSpeed.write(propSpeed);
                    ramp = FALSE;
                    rampSpeed = 0.0;
                }
            } else{
                liftSpeed.write(propSpeed);
            }
        } else {
            speedCount++;
        }
        checkLift(currentPosition, stall);
     
        // Check if arrived at destination height.
        if (abs(control.height - currentPosition) < 5){
            control.liftRun = FALSE;
            liftSpeed.write(0);
        }
    }    
}

// Keeps track of the rotation of the lift motor.  After one full rotation up,
// increments *position. If the motor doesn't rotate within 
// ((STALLTIME * HALLCHECKTIME * 4)/10^6) seconds, will set *stall TRUE. 
void checkLift(int& position, bool& stall) {
    
    static char state = 0;
    static char hallState1 = 0;
    static char hallState2 = 0;
    static char hallState1count = 0;
    static char hallState2count = 0;
    static char debounce = 0;
    static int  stallcount = 0;
    int movementStart = position;
    
    // Debounce the hall sensors. Check them 3 times, if they are
    // high for 2 or more, set them as high, else low.
    if (debounce< 3){
        if (hallSensor1){
            hallState1count++;
        }
        if (hallSensor2){
            hallState2count++;
        }
        debounce++;
    }else{
        if (hallState1count > 1){
            hallState1 = 1;
        }else{
            hallState1 = 0;
        }
        if (hallState2count > 1){
            hallState2 = 1;
        }else{
            hallState2 = 0;
        }
        hallState1count = 0;
        hallState2count = 0;       
        
        switch (state){
            
            case 0: // Hall 1 - low, Hall 2 - low
                    if (hallState2 == 1){
                        state = 1;
                        position++;
                    } else if (hallState1 == 1){
                        state = 3;
                    }
                    break;
                    
            case 1: // Hall 1 - low, Hall 2 - high
                    if (hallState1 == 1){
                        state = 2;
                    } else if (hallState2 == 0){
                        state = 0;
                        position--;
                    }
                    break;
                    
            case 2: // Hall 1 - high, Hall 2 - high
                    if (hallState2 == 0){
                        state = 3;
                    } else if (hallState1 == 0){
                        state = 1;
                    }
                    break;
                    
            case 3: // Hall 1 - high, Hall 2 - low
                    if (hallState1 == 0){
                        state = 0;
                    } else if (hallState2 == 1){
                        state = 2;
                    }
                    break;
        }
        debounce = 0;
        
        // Check for stalling
        if (movementStart == position){
            stallcount++;
            if (stallcount >= STALLTIME){
                stall = 1;
            }
        } else{
            stallcount = 0;
        }

    }
}