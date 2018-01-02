
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
    
}

void rosCheck(){
    int rosyaw = 0;
    int rospitch = 0;
    int rosroll = 0;
    int rosheight = 0;
    if (imuFlag){ 
        imuFlag = 0;     
        rosyaw = (YAWZERO + (int)(rosInput.yaw * DYNASCALE));
        rospitch = (PITCHZERO + (int)(rosInput.pitch * DYNASCALE));
        rosroll = (ROLLZERO + (int)(rosInput.roll * DYNASCALE));
        
        // Error checking for out of bounds values.
        if (rosyaw > YAWMAX){
            rosyaw = YAWMAX;
        }else if (rosyaw < YAWMIN){
            rosyaw = YAWMIN;
        }
        if (rospitch > PITCHMAX){
            rospitch = PITCHMAX;
        }else if (rospitch < PITCHMIN){
            rospitch = PITCHMIN;
        }
        if (rosroll > ROLLMAX){
           rosroll = ROLLMAX;
        }else if (rosroll < ROLLMIN){
            rosroll = ROLLMIN;
        }    
        
        // Enter the new values.
        if (control.yaw != rosyaw || control.pitch != rospitch || control.roll != rosroll){     
                control.yaw = rosyaw;
                control.pitch = rospitch;
                control.roll = rosroll;
                control.gimbalRun = TRUE;
        }
        
        rosheight = (int)(rosInput.height * LIFTSCALE);            // Retrieve the height data.
        
        // Error checking for out of bounds values.
        if (rosheight > LIFTHEIGHTMAX){
            rosheight = LIFTHEIGHTMAX;
        } else if(rosheight < LIFTHEIGHTMIN) {
            rosheight = LIFTHEIGHTMIN;
        }
        
        // Check if it is new data.
        if (control.height != rosheight){
            control.height = rosheight;
            control.liftRun = TRUE;
        }          
    }
}