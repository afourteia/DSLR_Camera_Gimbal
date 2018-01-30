/*
 * Code for the Module Microcontroller as part of the 
 * Autonomous Camera Kart project.
 * Controls the Gimbal motors, raise and lower mechanism,
 * IMU sensor input, communication with the NUC onboard computer.
 *
 */

// Library Includes
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




// Micro Module Declarations
DigitalOut          LED(PA_5);
PwmOut              liftSpeed(D10); // Normally D10
DigitalOut          liftDirection(D11);
DigitalIn           hallSensor1(D9);
DigitalIn           hallSensor2(D12);
DigitalIn           button(USER_BUTTON);
Ticker              hallInt;
Ticker              dynaInt;
Ticker              imuInt;
Ticker              rosInt;
BNO055              IMU(D14, D15);
DynamixelClass      gimbal(57600, D7, D8, D2);

ros::NodeHandle     nh;
ros::Subscriber<std_msgs::Float32MultiArray> sub("module_command", &module_commandCB);

// Global Variable
// Variables
struct          fields control;
struct          fields rosInput;
bool            liftFlag = 0;
bool            motorFlag = 0;
bool            homeFlag = 0;
bool            imuFlag = 0;
bool            rosFlag = 0;
int             currentPosition = LIFTHEIGHTMIN;
bool            stall = FALSE;

// Main Program
int main() {
    // Initializations
    setupGimbal();
    setupLift();                                // Lift
    setupROSNode(nh, sub);                      // ROS Node Handle
    // Main Loop
    while (1) {     
        // Lift Operation
        runLift(); 
        //Run Gimbal 
        runGimbal();
        //ROS Functions 
        nh.spinOnce();  
        rosCheck();              
    }
}
