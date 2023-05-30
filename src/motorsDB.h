#ifndef SRC_MOTORSCHARACTERSDB_H_
#define SRC_MOTORSCHARACTERSDB_H_

#include "generalDefines.h"


#define MOTOR_SPEED_CONSTANT (5460)  // [RPM/V]
#define SPEED_TO_TORQUE_GRAD (38500) // [RPM/mNm]
#define TORQUE_CONST (1.75) // [mNm/A]
#define MOTOR_GEAR_EFFICIENCY (0.76)
#define MOTOR_EFFICIENCY (0.63 * MOTOR_GEAR_EFFICIENCY)
#define GEAR_RATIO (15.0)
#define INV_GEAR_RATIO (1.0/GEAR_RATIO)
#define POWER_SUPPLY_VOLTAGE (12.0)
#define STARTING_VOLTAGE (0.05) //percent from power supply


// control gains
// TODO - need to finetune
#define KP_SPEED (1.0) 
#define KI_SPEED (0.1) 

#define KP_POS (400.0)   
#define KI_POS (0.15) 

#define MOTOR_CONTROLLER_HZ (1000) // TODO - change to the correct Hz (might not work for very slow velocities)

#define SEND_METRY_HZ (300.0f) // 


#define CHANGE_DIR_HZ (40.0f)
#define NUM_OF_STEPS (6.0)
#define UPDATE_POSITION_HZ (NUM_OF_STEPS*CHANGE_DIR_HZ)
#define UPDATE_VELOCITY_HZ (1)


#define MAX_SPEED (63.0f) //(DEG2RAD(180.0f)/CHANGE_DIR_HZ) * 10
#define MIN_SPEED (-1.0f * MAX_SPEED)

#endif /* SRC_MOTORSCHARACTERSDB_H_ */
