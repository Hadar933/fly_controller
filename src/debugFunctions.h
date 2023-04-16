#ifndef SRC_DEBUGFUNCTIONS_H_
#define SRC_DEBUGFUNCTIONS_H_

#include "generalDefines.h"
#include "motorDriverMain.h"
#include "stdint.h"

#define speedControlDbgArraySize 70

typedef struct
{
  float                         speedError[speedControlDbgArraySize];
  float                         Pcorrection[speedControlDbgArraySize];
  float                         speed_I_correction[speedControlDbgArraySize];
  float                         speedFromHull[speedControlDbgArraySize];
  float                         speedAverage[speedControlDbgArraySize];
  float                         motorSpeedCommandRPM[speedControlDbgArraySize];
  float                         commandVrms[speedControlDbgArraySize];
  uint8_t                       PWMpercent[speedControlDbgArraySize];
  int                           counter;
  float                         PI_correction[speedControlDbgArraySize];
  float                         correcteddSpeed[speedControlDbgArraySize];
  S_motorPhaseConfiguration     commutation[speedControlDbgArraySize];
  uint8_t                       gCommotationState[speedControlDbgArraySize];
}SDBGSpeedControl;

#define hullCounterDbgArraySize 100

typedef struct
{
  uint8_t   HullU;
  uint8_t   HullV;
  uint8_t   HullW;
  uint8_t   currentSequence;
}ShullsOnly;

typedef struct
{
  ShullsOnly motorsRealHulls [hullCounterDbgArraySize];
  SGDComutation hull[hullCounterDbgArraySize];
  int counter;
}SDBGHallCounter;

typedef struct
{
  SDBGSpeedControl speedControl[NUM_OF_MOTORS];
  SDBGHallCounter  hullCounter[NUM_OF_MOTORS];
}SDBGMainDBG;


// function proto
void record_motor_data(int motor);
void record_hull(int motor);
void record_SpeedError(int motor, float SpeedError);
void record_Pcorrection(int motor, float Pcorrection);
void record_speed_I_correction(int motor, float speed_I_correction);
void record_motorSpeedCommandRPM(int motor, float refMotorSpeedRPM);
void record_commandVrms(int motor, float commandVrms);
void record_gCommotationState(int motor, uint8_t gCommotationState);
void getHullsDBG(int motor);
#endif /* SRC_DEBUGFUNCTIONS_H_ */
