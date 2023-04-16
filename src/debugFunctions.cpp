#include <Arduino.h>
#include "debugFunctions.h"
#include "motorDriverMain.h"

extern SMotorsData motors[NUM_OF_MOTORS];
//extern uint32_t pinCounter[16];

SDBGMainDBG mainDBG;


void record_motor_data(int motor)
{
  // ========================================================== DEBUG - START ========================================================================
  mainDBG.speedControl[motor].counter++;
  uint32_t ind = mainDBG.speedControl[motor].counter;
  mainDBG.speedControl[motor].commutation[ind] = motors[motor].commutation;
  mainDBG.speedControl[motor].PI_correction[ind] = motors[motor].speedControler.I_correction;
  mainDBG.speedControl[motor].PWMpercent[ind] =  motors[motor].PWMCommand;
  mainDBG.speedControl[motor].speedFromHull[ind] = motors[motor].speedControler.FromHull;
  mainDBG.speedControl[motor].speedAverage[ind] = motors[motor].speedControler.Average.AverageData;
  mainDBG.speedControl[motor].correcteddSpeed[ind] = motors[motor].speedControler.corrected;
  if (mainDBG.speedControl[motor].counter > (speedControlDbgArraySize - 1))
  {
    while (1);
//    mainDBG.speedControl[motor].counter = 0;
  }
  // ========================================================== DEBUG - END ==========================================================================
}

void record_gCommotationState(int motor, uint8_t gCommotationState)
{
  mainDBG.speedControl[motor].gCommotationState[mainDBG.speedControl[motor].counter] = gCommotationState;
}

void record_SpeedError(int motor, float SpeedError)
{
  mainDBG.speedControl[motor].speedError[mainDBG.speedControl[motor].counter] = SpeedError;
}

void record_Pcorrection(int motor, float Pcorrection)
{
  mainDBG.speedControl[motor].Pcorrection[mainDBG.speedControl[motor].counter] = Pcorrection;
}

void record_speed_I_correction(int motor, float speed_I_correction)
{
  mainDBG.speedControl[motor].speed_I_correction[mainDBG.speedControl[motor].counter] = speed_I_correction;
}

void record_motorSpeedCommandRPM(int motor, float refMotorSpeedRPM)
{
  mainDBG.speedControl[motor].motorSpeedCommandRPM[mainDBG.speedControl[motor].counter] = refMotorSpeedRPM;
}

void record_commandVrms(int motor, float commandVrms)
{
  mainDBG.speedControl[motor].commandVrms[mainDBG.speedControl[motor].counter] = commandVrms;
}


void record_hull(int motor)
{
  mainDBG.hullCounter[motor].hull[mainDBG.hullCounter[motor].counter] = motors[motor].hull;
  mainDBG.hullCounter[motor].counter++;
  if (mainDBG.hullCounter[motor].counter > (hullCounterDbgArraySize - 1))
  {
    while (1);
  }
}

void getHullsDBG(int motor)
{
  uint8_t ind = mainDBG.hullCounter[motor].counter;
  switch(motor)
  {
    case motor1:
      mainDBG.hullCounter[motor].motorsRealHulls[ind].HullU = digitalRead(MOTOR1_HALL_U_PIN);
      mainDBG.hullCounter[motor].motorsRealHulls[ind].HullV = digitalRead(MOTOR1_HALL_V_PIN);
      mainDBG.hullCounter[motor].motorsRealHulls[ind].HullW = digitalRead(MOTOR1_HALL_W_PIN);
      break;

    case motor2:
      mainDBG.hullCounter[motor].motorsRealHulls[ind].HullU = digitalRead(MOTOR2_HALL_U_PIN);
      mainDBG.hullCounter[motor].motorsRealHulls[ind].HullV = digitalRead(MOTOR2_HALL_V_PIN);
      mainDBG.hullCounter[motor].motorsRealHulls[ind].HullW = digitalRead(MOTOR2_HALL_W_PIN);
      break;
  }
  mainDBG.hullCounter[motor].motorsRealHulls[ind].currentSequence = ((mainDBG.hullCounter[motor].motorsRealHulls[ind].HullU << 2) | (mainDBG.hullCounter[motor].motorsRealHulls[ind].HullV << 1) | (mainDBG.hullCounter[motor].motorsRealHulls[ind].HullW)) & 0x7;
  mainDBG.hullCounter[motor].motorsRealHulls[ind].currentSequence -= 1;
  mainDBG.hullCounter[motor].counter++;
  if (mainDBG.hullCounter[motor].counter > (hullCounterDbgArraySize - 1))
  {
    while (1);
  }
  return;
}
