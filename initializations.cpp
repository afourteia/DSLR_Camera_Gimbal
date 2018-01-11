 #include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <initializations.h>
#include <ros.h>
#include <definitions.h>
#include <prototypes.h>
#include "mbed.h"

#define BNO055_ID        (0xA0)

// Initializations

void setupGimbal(){
    gimbal.servo(YAWID, YAWZERO, 50); // Set the Gimbal at the zero position.
    control.yaw = YAWZERO;
    wait(0.1);
    gimbal.servo(PITCHID, PITCHZERO, 50);
    control.pitch = PITCHZERO;
    wait(0.1);
    gimbal.servo(ROLLID, ROLLZERO, 50);
    control.roll = ROLLZERO;
    dynaInt.attach(&motorInterrupt, 0.0001); // Update Dynamixel at 1kHz.
    
    rosInt.attach(&rosInterrupt, 0.1);
}

void setupLift(){
    
    int position = 0;
    
    hallInt.attach(&hallInterrupt, 0.0001);
    liftSpeed.period_us(50);
    
    // Find the bottom position. Go down at lowest speed and set zero point.
    liftDirection.write(LIFTDOWN);
    liftSpeed.write(0.3);   
    
    // Wait until the motor stalls, know the motor is at bottom.
    while(!stall){
        if (liftFlag){
            liftFlag = 0;
            checkLift(position, stall);
        }
    }

    // Go up 0.3 cm from the bottom.
    liftDirection.write(LIFTUP);
    liftSpeed.write(0.3);
    position = 0;
    stall = FALSE;
    while(position <  LIFTHEIGHTMIN){
        if (liftFlag){
            liftFlag = 0;
            checkLift(position, stall);
        }
    }
    liftSpeed.write(0);
    control.height = currentPosition;
    control.liftRun = FALSE;
}

void setupROSNode(ros::NodeHandle& nh, ros::Subscriber<std_msgs::Float32MultiArray>& sub){
       
    nh.getHardware()->setBaud(57600);
    nh.initNode();
    nh.subscribe(sub);
}
    

