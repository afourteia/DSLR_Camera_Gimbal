
#include <ros_functions.h>
#include <definitions.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <prototypes.h>


void module_commandCB(const std_msgs::Float32MultiArray& command_data){
    
    rosInput.yaw = command_data.data[ROSYAW];
    rosInput.pitch = command_data.data[ROSPITCH];
    rosInput.roll = command_data.data[ROSROLL];
    rosInput.height = command_data.data[ROSHEIGHT];
    rosInput.mode = command_data.data[ROSMODE];
    
}

void rosCheck(){ 
    if (rosFlag){ 
        rosFlag = 0;
        if(rosInput.mode == 0){ //Transport Mode
                control.yaw = YAWZERO;
                control.pitch = PITCHZERO;
                control.roll = ROLLZERO;
                control.gimbalRun = TRUE;
            if(control.height != LIFTHEIGHTMIN){
                control.height = LIFTHEIGHTMIN;
                control.liftRun = TRUE;
            }      
        }
        else if(rosInput.mode == 1){ //Photo Mode         
            switch(rosInput.height){
                case(1):
                control.height = control.height + 50; 
                control.liftRun = TRUE;   
                if(control.height > LIFTHEIGHTMAX){
                    control.height = LIFTHEIGHTMAX;}
                break;
                case(2):
                control.height = control.height - 50;
                control.liftRun = TRUE;   
                if(control.height < LIFTHEIGHTMIN){
                    control.height = LIFTHEIGHTMIN;}
                break;
                case(0):
                control.height = currentPosition;
                break; 
            } 
            switch(rosInput.yaw){
                case(1):
                control.yaw = control.yaw + 50;
                control.gimbalRun = TRUE;
                if(control.yaw > YAWMAX){
                    control.yaw = YAWMAX;}
                break;
                case(2):
                control.yaw = control.yaw - 50;
                control.gimbalRun = TRUE;
                if(control.yaw < YAWMIN){
                    control.yaw = YAWMIN;}
                break;
                case(0):
                break;
            }
            switch(rosInput.pitch){
                case(1):
                control.pitch = control.pitch + 50;
                control.gimbalRun = TRUE;
                if(control.pitch > PITCHMAX){
                    control.pitch = PITCHMAX;}
                break;
                case(2):
                control.pitch = control.pitch - 50;
                control.gimbalRun = TRUE;
                if(control.pitch < PITCHMIN){
                    control.pitch = PITCHMIN;}
                break;
                case(0):
                break; 
            }
            switch(rosInput.roll){
                case(1):
                control.roll = control.roll + 50;
                control.gimbalRun = TRUE;
                if(control.roll > ROLLMAX){
                    control.roll = ROLLMAX;}
                break;
                case(2):
                control.roll = control.roll - 50;
                control.gimbalRun = TRUE;
                if(control.roll < ROLLMIN){
                    control.roll = ROLLMIN;}
                break;
                case(0):
                break; 
            }
        } 
    }   
}