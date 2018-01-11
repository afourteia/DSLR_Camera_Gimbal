 #ifndef __PROTOTYPES_H__
#define __PROTOTYPES_H__

void setupIMU();
void setupLift();
void setupGimbal();
void setupROSNode(ros::NodeHandle& nh, ros::Subscriber<std_msgs::Float32MultiArray>& sub);

void runGimbal();
void runLift();
void checkLift(int& position, bool& stall);

void module_commandCB(const std_msgs::Float32MultiArray& command_data);

void hallInterrupt(void);
void motorInterrupt(void);
void imuInterrupt(void);
void rosInterrupt(void);

void rosCheck(void);

#endif