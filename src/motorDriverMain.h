#ifndef INC_MOTORDRIVERMAIN_H_
#define INC_MOTORDRIVERMAIN_H_

#include "generalDefines.h"
#include "stdbool.h"
#include "stdint.h"
#include "generalPurposeFunctions.h"
#include "motorsDB.h"
#include "motorDriverMain.h"
#include <math.h>
#include "counter.h"

#define MOTOR_PHASE_CONFIG_SIZE (8)


typedef enum{
  L = 0,
  H,
  D,
  NA = 0
}EPolarity;

typedef struct
{
  uint8_t Upolarity;
  uint8_t Vpolarity;
  uint8_t Wpolarity;
}S_motorPhaseConfiguration;

typedef struct
{
  S_motorPhaseConfiguration forward[MOTOR_PHASE_CONFIG_SIZE];
  S_motorPhaseConfiguration backward[MOTOR_PHASE_CONFIG_SIZE];
}S_fullMotorPhaseConfiguration;

typedef struct
{
  uint8_t   currentSequence;
  uint8_t   prevSequence;
  uint8_t   HullU;
  uint8_t   HullV;
  uint8_t   HullW;
  int32_t   cnt;
  int8_t    prevHullAdded;
  uint32_t  cnt_last_time_uSec;
  int8_t    hullAdd;
}SGDComutation;


typedef struct
{
  int32_t   cnt;
  uint64_t  cnt_last_time_uSec;
}SGDEncoder;


typedef enum{
  DS_STOP = 0,
  DS_CW,
  DS_CCW
}e_driveState;


typedef struct {
  float               I_correction;
  float               corrected;     // [Rad/sec]
  float               FromHull;      // [Rad/sec]
  float               FromEncoder;   // [rad/sec]
  float               current;       // [rad/sec]
  uint64_t            lastHullCalcTimeuSec;
  uint64_t            lastEncoderCalcTimeuSec;
  int32_t             lastHullCnt;
  int32_t             lastEncoderCnt;
  float               ref;
  SMovingAverage      Average;
}SPIContorl;

typedef struct{
  SGDComutation                 hull;
  SGDEncoder                    encoder;
  S_motorPhaseConfiguration     commutation;
  float                         payloadAngle;
  e_driveState                  motorDriveState;
  uint8_t                       PWMCommand;
  SPIContorl                    speedControler;
  SPIContorl                    positionControler;
  bool                          isRunning;
  bool                          isActive;
}SMotorsData;



// ================= function prototypes ===========================
void sendCommandToDriver(int motor);
void getAllMotorsCommutation(void);
void getMotorHulls(int motor);
float PIControl(SPIContorl * data, float kp, float ki);
void GPIO_motorPinPWMoutDisable(uint8_t motor_ch, int motor);
void GPIO_motorPinPWMoutHigh(uint8_t motor_ch, int motor);
void GPIO_motorPinPWMoutLow(uint8_t motor_ch, int motor);
void setMotorDriveState(int motor);
void setAllMotorsDriveState(void);
void motorDriverPhaseConfigurationInit(void);
void getHullSequence(int motor);
void calcSpeedFromHulls(int motor);
void calcSpeedFromEncoder(int motor);
void calcAngleFromEncoder(int motor);
void speedControlHandle(int motor);
void positionControlHandle(int motor);
void resetMotorData(int motor);
void resetAllDriveMotorsData(void);
void calcPWMpercent(int motor);
void sendPWMCommadToAllMotors(void);
void motorPhaseConfigurationHandle(int motor);
void calcHullAdd(int motor);

// ============================= Type def =======================
typedef void(*fctPtr)(uint8_t , int);

#ifdef DEBUG_SPEED_CONTROL
  #include "debugFunctions.h"
#endif

#endif /* INC_MOTORDRIVERMAIN_H_ */
