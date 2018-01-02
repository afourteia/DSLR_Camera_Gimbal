
#include <definitions.h>

void hallInterrupt(){

    liftFlag = 1;
}

void motorInterrupt(){
    
    motorFlag = 1;
}

void imuInterrupt(){
    
    imuFlag = 1;
}

void rosInterrupt(){
    
    rosFlag = 1;
}
