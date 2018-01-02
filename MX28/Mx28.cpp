#include <stdlib.h>
#include <stdio.h>
#include "mbed.h"
#include "Mx28.h"


    unsigned char   Instruction_Packet_Array[15];   // Array to hold instruction packet data 
    unsigned char   Status_Packet_Array[8];         // Array to hold returned status packet data
    unsigned int    Time_Counter;                   // Timer for time out watchers
    unsigned char   Direction_Pin;                  // Pin to control TX/RX buffer chip
    unsigned char   Status_Return_Value = READ;     // Status packet return states ( NON , READ , ALL )

//-------------------------------------------------------------------------------------------------------------------------------
//            Private Methods 
//-------------------------------------------------------------------------------------------------------------------------------
void DynamixelClass::debugframe(void) {
 for (int i=0; i<10; i++)
  printf("%x,",Instruction_Packet_Array[i]);
 printf("\r\n"); 
}

void DynamixelClass::debugStatusframe(void) {
 for (int i=0; i<10; i++)
  printf("%x,",Status_Packet_Array[i]);
 printf("\r\n"); 
}


void DynamixelClass::transmitInstructionPacket(void){                                   // Transmit instruction packet to Dynamixel

    unsigned char Counter;
    Counter = 0;    
    servoSerialDir->write(1);                                                          // Set TX Buffer pin to HIGH    

    servoSerial->putc(HEADER);                                                              // Write Header (0xFF) data 1 to serial                     
    servoSerial->putc(HEADER);                                                              // Write Header (0xFF) data 2 to serial
    servoSerial->putc(Instruction_Packet_Array[0]);                                         // Write Dynamixal ID to serial 
    servoSerial->putc(Instruction_Packet_Array[1]);                                         // Write packet length to serial    
    
    do{                                                                                 
        servoSerial->putc(Instruction_Packet_Array[Counter + 2]);                           // Write Instuction & Parameters (if there is any) to serial
        Counter++;
    }while((Instruction_Packet_Array[1] - 2) >= Counter);
    
    servoSerial->putc(Instruction_Packet_Array[Counter + 2]);                               // Write check sum to serial
    wait_us((Counter + 4)*190);
    servoSerialDir->write(0);                                                          // Set TX Buffer pin to LOW after data has been sent

}


unsigned int DynamixelClass::readStatusPacket(void){

    unsigned char Counter = 0x00;

    static unsigned char InBuff[20];
    unsigned char i, j, RxState;

    
    Status_Packet_Array[0] = 0x00;
    Status_Packet_Array[1] = 0x00;
    Status_Packet_Array[2] = 0x00;                                                      
    Status_Packet_Array[3] = 0x00;
    i=0; RxState=0; j=0; InBuff[0]=0;
    Timer timer;
    timer.start();


while (RxState<3){                              // Wait for " header + header + frame length + error " RX data
 if (timer.read_ms() >= STATUS_PACKET_TIMEOUT){
        return Status_Packet_Array[2] = 0x80;                                      // Return with Error if Serial data not received with in time limit

 }

 if (servoSerial->readable()) {
   InBuff[i]=servoSerial->getc();
   if (InBuff[0]==0xff) i++;                                                            // When we have the first header we starts to inc data to inbuffer

   
   if ((i>=(STATUS_FRAME_BUFFER-1)) &&(RxState==0)) RxState++; //read header
   
   switch (RxState) {
    case 1: {//Read header 
            if ((InBuff[j] == 0xFF) &&  (InBuff[j+1]== 0xFF)){                                    // checkes that we have got the buffer
                    j=2;
                    Status_Packet_Array[0] = InBuff[j++];                                    // ID sent from Dynamixel
                    Status_Packet_Array[1] = InBuff[j++];                                    // Frame Length of status packet
                    Status_Packet_Array[2] = InBuff[j++];                                    // Error (char) 
                    RxState++; led2->write(0);
            }
    } break;
    case 2: {//Read data
      if (i>Status_Packet_Array[1]+3) { //We have recieved all data
            do{
                Status_Packet_Array[3 + Counter] = InBuff[j++];
                Counter++;              
            }while(Status_Packet_Array[1] > Counter);                           // Read Parameter(s) into array
            
            Status_Packet_Array[Counter + 4] = InBuff[j++];                          // Read Check sum   
           RxState++; 
        }    
      } break;      
   } //switch 
   }
  }//while..
  timer.stop();
//  debugStatusframe();

  return 0x00;
}



//-------------------------------------------------------------------------------------------------------------------------------
// Public Methods 
//-------------------------------------------------------------------------------------------------------------------------------

 DynamixelClass::DynamixelClass(int baud, PinName D_Pin, PinName tx, PinName rx){ 
    servoSerial=new Serial(tx, rx);
    servoSerial->baud(baud);
    servoSerialDir= new DigitalOut(D_Pin);     
    servoSerialDir->write(0);
    led2=new DigitalOut(LED2); 

}   

 DynamixelClass::~DynamixelClass(){
    if(servoSerial != NULL)
        delete servoSerial;
}

unsigned int DynamixelClass::reset(unsigned char ID){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = RESET_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_RESET;
    Instruction_Packet_Array[3] = ~(ID + RESET_LENGTH + COMMAND_RESET); //Checksum;
 
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }   
}

void DynamixelClass::NewBaudRate(int baud) {
//Change the baudrate on then MBED
    int Baudrate_BPS = 0;
    Baudrate_BPS  =(int) 2000000 / (baud + 1);                        // Calculate Baudrate as ber "Robotis e-manual"
    servoSerial->baud(Baudrate_BPS);
}

unsigned int DynamixelClass::ping(unsigned char ID){
    
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = PING_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_PING;
    Instruction_Packet_Array[3] = ~(ID + PING_LENGTH + COMMAND_PING);
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    readStatusPacket();
    
    if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
        return (Status_Packet_Array[0]);            // Return SERVO ID
    }else{
        return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
    }            
}

unsigned int DynamixelClass::setStatusPaketReturnDelay(unsigned char ID,unsigned char ReturnDelay){
    
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_RETURN_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = EEPROM_RETURN_DELAY_TIME;
    Instruction_Packet_Array[4] = (char) (ReturnDelay/2);
    Instruction_Packet_Array[5] = ~(ID + SET_RETURN_LENGTH + COMMAND_WRITE_DATA + EEPROM_RETURN_DELAY_TIME + (char)(ReturnDelay/2));  
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }       

}

unsigned int DynamixelClass::setID(unsigned char ID, unsigned char New_ID){    

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_ID_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = EEPROM_ID;
    Instruction_Packet_Array[4] = New_ID;
    Instruction_Packet_Array[5] = ~(ID + SET_ID_LENGTH + COMMAND_WRITE_DATA+ EEPROM_ID + New_ID);  
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }       
}

unsigned int DynamixelClass::setBaudRate(unsigned char ID, long Baud){                      
    
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_BD_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = EEPROM_BAUD_RATE;
//  if (Baud > 2250000){
if (Baud >= 1000000){
    switch (Baud){
        case 2250000:
        Instruction_Packet_Array[4] = 0xFA;
        break;
        case 2500000:
        Instruction_Packet_Array[4] = 0xFB;
        break;
        case 3000000:
        Instruction_Packet_Array[4] = 0xFC;
        break;
        case 1000000:
        Instruction_Packet_Array[4] = 0x01;
        }
    }else{
    Instruction_Packet_Array[4] = (char)((2000000/Baud) - 1);
    }
    Instruction_Packet_Array[5] = ~(ID + SET_BD_LENGTH + COMMAND_WRITE_DATA + EEPROM_BAUD_RATE + (char)((2000000/Baud) - 1) ); 
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }   
}

unsigned int DynamixelClass::setMaxTorque( unsigned char ID, int Torque){
    
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_MAX_TORQUE_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = EEPROM_MAX_TORQUE_L ;
    Instruction_Packet_Array[4] = (char)(Torque);
    Instruction_Packet_Array[5] = (char)(Torque >> 8);
    Instruction_Packet_Array[6] = ~(ID + SET_MAX_TORQUE_LENGTH + COMMAND_WRITE_DATA + EEPROM_MAX_TORQUE_L + (char)(Torque) + (char)(Torque >> 8));
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }       
}

unsigned int DynamixelClass::setHoldingTorque(unsigned char ID, bool Set){
  
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_HOLDING_TORQUE_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = RAM_TORQUE_ENABLE;
    Instruction_Packet_Array[4] = Set;
    Instruction_Packet_Array[5] = ~(ID + SET_HOLDING_TORQUE_LENGTH + COMMAND_WRITE_DATA + RAM_TORQUE_ENABLE + Set); 
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }       
}

unsigned int DynamixelClass::setAlarmShutdown(unsigned char  ID,unsigned char Set){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_ALARM_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = EEPROM_ALARM_SHUTDOWN;
    Instruction_Packet_Array[4] = Set;
    Instruction_Packet_Array[5] = ~(ID + SET_ALARM_LENGTH + COMMAND_WRITE_DATA + EEPROM_ALARM_SHUTDOWN + Set);  
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }           

}

unsigned int DynamixelClass::setStatusPaket(unsigned char  ID,unsigned char Set){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_RETURN_LEVEL_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = EEPROM_RETURN_LEVEL;
    Instruction_Packet_Array[4] = Set;
    Instruction_Packet_Array[5] = ~(ID + SET_RETURN_LEVEL_LENGTH + COMMAND_WRITE_DATA + EEPROM_RETURN_LEVEL + Set);
    
    Status_Return_Value = Set;
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }       

}


unsigned int DynamixelClass::setMode(unsigned char ID, bool Dynamixel_Mode, unsigned int Dynamixel_CW_Limit,unsigned int Dynamixel_CCW_Limit){

    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_MODE_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = EEPROM_CW_ANGLE_LIMIT_L;
     if ( Dynamixel_Mode == WHEEL) {                                    // Set WHEEL mode, this is done by setting both the clockwise and anti-clockwise angle limits to ZERO
        Instruction_Packet_Array[4] = 0x00;
        Instruction_Packet_Array[5] = 0x00;
        Instruction_Packet_Array[6] = 0x00;
        Instruction_Packet_Array[7] = 0x00;
        Instruction_Packet_Array[8] = ~(ID + SET_MODE_LENGTH + COMMAND_WRITE_DATA + EEPROM_CW_ANGLE_LIMIT_L);
    }else {                                                             // Else set SERVO mode
        Instruction_Packet_Array[4] = (char)(Dynamixel_CW_Limit);
        Instruction_Packet_Array[5] = (char)((Dynamixel_CW_Limit & 0x0F00) >> 8);
        Instruction_Packet_Array[6] = (char)(Dynamixel_CCW_Limit);
        Instruction_Packet_Array[7] = (char)((Dynamixel_CCW_Limit & 0x0F00) >> 8);
        Instruction_Packet_Array[8] = ~(ID + SET_MODE_LENGTH + COMMAND_WRITE_DATA + EEPROM_CW_ANGLE_LIMIT_L + (char)(Dynamixel_CW_Limit) + (char)((Dynamixel_CW_Limit & 0x0F00) >> 8) + (char)(Dynamixel_CCW_Limit) + (char)((Dynamixel_CCW_Limit & 0x0F00) >> 8));
    }   
        
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();
    
        if (Status_Return_Value == ALL){
        readStatusPacket();
            if (Status_Packet_Array[2] != 0){
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
            }
            
        }
   return 0x00; //if no errors
 } 
 
 unsigned int DynamixelClass::setPunch(unsigned char ID,unsigned int Punch){
 
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_PUNCH_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = RAM_PUNCH_L;
    Instruction_Packet_Array[4] = (char)(Punch);
    Instruction_Packet_Array[5] = (char)(Punch >> 8);
    Instruction_Packet_Array[6] = ~(ID + SET_PUNCH_LENGTH + COMMAND_WRITE_DATA + RAM_PUNCH_L + (char)(Punch) + (char)(Punch >> 8) );
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }   
 
 }
 
 unsigned int DynamixelClass::setPID(unsigned char ID ,unsigned char P,unsigned char I,unsigned char D){
 
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_PID_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = RAM_PROPORTIONAL_GAIN;
    Instruction_Packet_Array[4] = P;
    Instruction_Packet_Array[5] = I;
    Instruction_Packet_Array[6] = D;
    Instruction_Packet_Array[7] = ~(ID + SET_PID_LENGTH + COMMAND_WRITE_DATA + RAM_PROPORTIONAL_GAIN + P + I + D );
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
 }

unsigned int DynamixelClass::setTemp(unsigned char ID,unsigned char temp){
    
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_TEMP_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = EEPROM_LIMIT_TEMPERATURE;
    Instruction_Packet_Array[4] = temp;
    Instruction_Packet_Array[5] = ~(ID + SET_TEMP_LENGTH + COMMAND_WRITE_DATA + EEPROM_LIMIT_TEMPERATURE + temp);   
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }   
}

unsigned int DynamixelClass::setVoltage(unsigned char ID,unsigned char Volt_L, unsigned char Volt_H){
    
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SET_VOLT_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = RAM_GOAL_POSITION_L;
    Instruction_Packet_Array[4] = Volt_L;
    Instruction_Packet_Array[5] = Volt_H;
    Instruction_Packet_Array[6] = ~(ID + SET_VOLT_LENGTH + COMMAND_WRITE_DATA + RAM_GOAL_POSITION_L + Volt_L + Volt_H);    
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}
 
unsigned int DynamixelClass::writeReg(unsigned char ID,unsigned char reg,unsigned int data){
    
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = 0x05;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = reg;
    Instruction_Packet_Array[4] = (char)data;
    Instruction_Packet_Array[5] = (char)((data & 0xFF00) >> 8);
    Instruction_Packet_Array[6] = ~(ID + 0x05 + COMMAND_WRITE_DATA + reg + (char)data + (char)((data & 0xFF00) >> 8));   
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
} 
 
unsigned int DynamixelClass::servo(unsigned char ID,unsigned int Position,unsigned int Speed){
    
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SERVO_GOAL_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = RAM_GOAL_POSITION_L;
    Instruction_Packet_Array[4] = (char)(Position);
    Instruction_Packet_Array[5] = (char)((Position & 0x0F00) >> 8);
    Instruction_Packet_Array[6] = (char)(Speed);
    Instruction_Packet_Array[7] = (char)((Speed & 0x0F00) >> 8);
    Instruction_Packet_Array[8] = ~(ID + SERVO_GOAL_LENGTH + COMMAND_WRITE_DATA + RAM_GOAL_POSITION_L + Position + (char)((Position & 0x0F00) >> 8) + Speed + (char)((Speed & 0x0F00) >> 8));   
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}

unsigned int DynamixelClass::servoPreload(unsigned char ID,unsigned int Position,unsigned int Speed){
    
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = SERVO_GOAL_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_REG_WRITE_DATA;
    Instruction_Packet_Array[3] = RAM_GOAL_POSITION_L;
    Instruction_Packet_Array[4] = (char)(Position);
    Instruction_Packet_Array[5] = (char)(Position >> 8);
    Instruction_Packet_Array[6] = (char)(Speed);
    Instruction_Packet_Array[7] = (char)(Speed >> 8);
    Instruction_Packet_Array[8] = ~(ID + SERVO_GOAL_LENGTH + COMMAND_REG_WRITE_DATA + RAM_GOAL_POSITION_L + (char)(Position) + (char)(Position >> 8) + (char)(Speed) + (char)(Speed >> 8)); 
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }
}
 
unsigned int DynamixelClass::wheel(unsigned char ID, bool Rotation,unsigned int Speed){ 

    char Speed_H,Speed_L;
    Speed_L = Speed;    
        if (Rotation == 0){                         // Move Left                     
            Speed_H = Speed >> 8;
            }
        else if (Rotation == 1){                    // Move Right
            Speed_H = (Speed >> 8)+4;   
            }   
            
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = WHEEL_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = RAM_GOAL_SPEED_L;
    Instruction_Packet_Array[4] = Speed_L;
    Instruction_Packet_Array[5] = Speed_H;
    Instruction_Packet_Array[6] = ~(ID + WHEEL_LENGTH + COMMAND_WRITE_DATA + RAM_GOAL_SPEED_L  + Speed_L + Speed_H);            
            
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }           

}

void DynamixelClass::wheelSync(unsigned char ID1, bool Dir1, unsigned int Speed1, unsigned char ID2, bool Dir2, unsigned int Speed2, unsigned char ID3, bool Dir3, unsigned int Speed3){
    
    char Speed1_H,Speed1_L;
    Speed1_L = Speed1; 
        if (Dir1 == 0){                          // Move Left
            Speed1_H = Speed1 >> 8;
        }
        else if (Dir1 == 1)                     // Move Right
        {   
            Speed1_H = (Speed1 >> 8)+4;
        }   

    char Speed2_H,Speed2_L;
    Speed2_L = Speed2; 
        if (Dir2 == 0){                          // Move Left
            Speed2_H = Speed2 >> 8;
        }
        else if (Dir2 == 1)                     // Move Right
        {   
            Speed2_H = (Speed2 >> 8)+4;
        }
  
    char Speed3_H,Speed3_L;
    Speed3_L = Speed3; 
        if (Dir3 == 0){                          // Move Left
            Speed3_H = Speed3 >> 8;
        }
        else if (Dir3 == 1)                     // Move Right
        {   
            Speed3_H = (Speed3 >> 8)+4;
        }       
        
    Instruction_Packet_Array[0] = 0xFE;         // When Writing a Sync comman you must address all(0xFE) servos
    Instruction_Packet_Array[1] = SYNC_LOAD_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_SYNC_WRITE;
    Instruction_Packet_Array[3] = RAM_GOAL_SPEED_L;
    Instruction_Packet_Array[4] = SYNC_DATA_LENGTH;
    Instruction_Packet_Array[5] = ID1;
    Instruction_Packet_Array[6] = Speed1_L;
    Instruction_Packet_Array[7] = Speed1_H;
    Instruction_Packet_Array[8] = ID2;
    Instruction_Packet_Array[9] = Speed2_L;
    Instruction_Packet_Array[10] = Speed2_H;
    Instruction_Packet_Array[11] = ID3;
    Instruction_Packet_Array[12] = Speed3_L;
    Instruction_Packet_Array[13] = Speed3_H;    
    Instruction_Packet_Array[14] = (char)(~(0xFE + SYNC_LOAD_LENGTH + COMMAND_SYNC_WRITE + RAM_GOAL_SPEED_L + SYNC_DATA_LENGTH + ID1 + Speed1_L + Speed1_H + ID2 + Speed2_L + Speed2_H + ID3 + Speed3_L + Speed3_H));             
    
    transmitInstructionPacket();
 
}

unsigned int DynamixelClass::wheelPreload(unsigned char ID, bool Dir,unsigned int Speed){
    
    char Speed_H,Speed_L;
    Speed_L = Speed; 
        if (Dir == 0){                          // Move Left
            Speed_H = Speed >> 8;
        }
        else if (Dir == 1)                      // Move Right
        {   
            Speed_H = (Speed >> 8)+4;
        }   
        
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = WHEEL_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_REG_WRITE_DATA;
    Instruction_Packet_Array[3] = RAM_GOAL_SPEED_L;
    Instruction_Packet_Array[4] = Speed_L;
    Instruction_Packet_Array[5] = Speed_H;
    Instruction_Packet_Array[6] = ~(ID + WHEEL_LENGTH + COMMAND_REG_WRITE_DATA + RAM_GOAL_SPEED_L + Speed_L + Speed_H);             
            
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }       

}

unsigned int DynamixelClass::action(unsigned char ID){
    
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = RESET_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_ACTION;
    Instruction_Packet_Array[3] = ~(ID + ACTION_LENGTH + COMMAND_ACTION);
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }       
}

unsigned int DynamixelClass::ledState(unsigned char ID, bool Status){  
  
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = LED_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_WRITE_DATA;
    Instruction_Packet_Array[3] = RAM_LED;
    Instruction_Packet_Array[4] = Status;
    Instruction_Packet_Array[5] = ~(ID + LED_LENGTH + COMMAND_WRITE_DATA + RAM_LED + Status);   
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    
    if (ID == 0XFE || Status_Return_Value != ALL ){     // If ID of FE is used no status packets are returned so we do not need to check it
        return (0x00);
    }else{
        readStatusPacket();
        if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
            return (Status_Packet_Array[0]);            // Return SERVO ID
        }else{
            return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
        }
    }   
}

unsigned int DynamixelClass::readTemperature(unsigned char ID){ 
        
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = READ_TEMP_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
    Instruction_Packet_Array[3] = RAM_PRESENT_TEMPERATURE;
    Instruction_Packet_Array[4] = READ_ONE_BYTE_LENGTH;
    Instruction_Packet_Array[5] = ~(ID + READ_TEMP_LENGTH  + COMMAND_READ_DATA + RAM_PRESENT_TEMPERATURE + READ_ONE_BYTE_LENGTH);
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();
    readStatusPacket(); 

    if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
        return Status_Packet_Array[3];
    }else{
        return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
    }
}

unsigned int DynamixelClass::readGoalPosition(unsigned char ID){    
        
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = READ_POS_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
    Instruction_Packet_Array[3] = RAM_GOAL_POSITION_L;
    Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;
    Instruction_Packet_Array[5] = ~(ID + READ_POS_LENGTH + COMMAND_READ_DATA + RAM_GOAL_POSITION_L + READ_TWO_BYTE_LENGTH);  
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    readStatusPacket();
    
    if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value                                          // If there is no status packet error return value
    return Status_Packet_Array[4] << 8 | Status_Packet_Array[3];    // Return present position value
    }else{
        return (Status_Packet_Array[2] | 0xF000);                           // If there is a error Returns error value
    }
}

unsigned int DynamixelClass::readPosition(unsigned char ID){    
        
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = READ_POS_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
    Instruction_Packet_Array[3] = RAM_PRESENT_POSITION_L;
    Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;
    Instruction_Packet_Array[5] = ~(ID + READ_POS_LENGTH + COMMAND_READ_DATA + RAM_PRESENT_POSITION_L + READ_TWO_BYTE_LENGTH);  
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();    
    readStatusPacket();
    
    if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value                                          // If there is no status packet error return value
        return Status_Packet_Array[4] << 8 | Status_Packet_Array[3];    // Return present position value
    }else{
        return (Status_Packet_Array[2] | 0xF000);                           // If there is a error Returns error value
    }
}

unsigned int DynamixelClass::readLoad(unsigned char ID){    
    
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = READ_LOAD_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
    Instruction_Packet_Array[3] = RAM_PRESENT_LOAD_L;
    Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;
    Instruction_Packet_Array[5] = ~(ID + READ_LOAD_LENGTH + COMMAND_READ_DATA + RAM_PRESENT_LOAD_L  + READ_TWO_BYTE_LENGTH);
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();
    readStatusPacket();
    
    if (Status_Packet_Array[2] == 0){                                           // If there is no status packet error return value
        return ((Status_Packet_Array[4] << 8) | Status_Packet_Array[3]);    // Return present load value
    }else{
        return (Status_Packet_Array[2] | 0xF000);                                   // If there is a error Returns error value
    }
}

unsigned int DynamixelClass::readSpeed(unsigned char ID){   
    
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = READ_SPEED_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
    Instruction_Packet_Array[3] = RAM_PRESENT_SPEED_L;
    Instruction_Packet_Array[4] = READ_TWO_BYTE_LENGTH;
    Instruction_Packet_Array[5] = ~(ID + READ_SPEED_LENGTH + COMMAND_READ_DATA + RAM_PRESENT_SPEED_L + READ_TWO_BYTE_LENGTH);
    
    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();
    readStatusPacket();
    
    if (Status_Packet_Array[2] == 0){                                           // If there is no status packet error return value
        return (Status_Packet_Array[4] << 8) | Status_Packet_Array[3];  // Return present position value
    }else{
        return (Status_Packet_Array[2] | 0xF000);                           // If there is a error Returns error value
    }
}


unsigned int DynamixelClass::readVoltage(unsigned char ID){    
        
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = READ_VOLT_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
    Instruction_Packet_Array[3] = RAM_PRESENT_VOLTAGE;
    Instruction_Packet_Array[4] = READ_ONE_BYTE_LENGTH;
    Instruction_Packet_Array[5] = ~(ID + READ_VOLT_LENGTH + COMMAND_READ_DATA + RAM_PRESENT_VOLTAGE + READ_ONE_BYTE_LENGTH);    

    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();
    readStatusPacket();
    
    if (Status_Packet_Array[2] == 0){                   // If there is no status packet error return value
        return Status_Packet_Array[3];                  // Return voltage value (value retured by Dynamixel is 10 times actual voltage)
    }else{
        return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
    }
}

unsigned int DynamixelClass::checkRegister(unsigned char ID){    
        
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = READ_REGISTER_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
    Instruction_Packet_Array[3] = RAM_REGISTER;
    Instruction_Packet_Array[4] = READ_ONE_BYTE_LENGTH;
    Instruction_Packet_Array[5] = ~(ID + READ_REGISTER_LENGTH + COMMAND_READ_DATA + RAM_REGISTER + READ_ONE_BYTE_LENGTH);

    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();
    readStatusPacket();
    
    if (Status_Packet_Array[2] == 0){                   // If there is no status packet error return value
        return (Status_Packet_Array[3]);            // Return register value
    }else{
        return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
    }
}

unsigned int DynamixelClass::checkMovement(unsigned char ID){    
        
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = READ_MOVING_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
    Instruction_Packet_Array[3] = RAM_MOVING;
    Instruction_Packet_Array[4] = READ_ONE_BYTE_LENGTH;
    Instruction_Packet_Array[5] = ~(ID + READ_MOVING_LENGTH + COMMAND_READ_DATA + RAM_MOVING + READ_ONE_BYTE_LENGTH);

    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();
    readStatusPacket();
    
    if (Status_Packet_Array[2] == 0){                   // If there is no status packet error return value
        return (Status_Packet_Array[3]);            // Return movement value
    }else{
        return (Status_Packet_Array[2] | 0xF000);            // If there is a error Returns error value
    }
}

unsigned int DynamixelClass::checkLock(unsigned char ID){    
    
    Instruction_Packet_Array[0] = ID;
    Instruction_Packet_Array[1] = READ_LOCK_LENGTH;
    Instruction_Packet_Array[2] = COMMAND_READ_DATA;
    Instruction_Packet_Array[3] = RAM_LOCK;
    Instruction_Packet_Array[4] = READ_ONE_BYTE_LENGTH;
    Instruction_Packet_Array[5] = ~(ID + READ_LOCK_LENGTH + COMMAND_READ_DATA + RAM_LOCK + READ_ONE_BYTE_LENGTH);   

    if (servoSerial->readable()) 
      while (servoSerial->readable()) servoSerial->getc(); //emthy buffer

    transmitInstructionPacket();
    readStatusPacket();
    
    if (Status_Packet_Array[2] == 0){               // If there is no status packet error return value
        return (Status_Packet_Array[3]);            // Return Lock value
    }else{
        return (Status_Packet_Array[2] | 0xF000);   // If there is a error Returns error value
    }
}


