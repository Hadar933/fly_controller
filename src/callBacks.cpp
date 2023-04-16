#include "callBacks.h"
#define _USE_MATH_DEFINES
#include <math.h>

#ifdef DEBUG_HULLS
  #include "debugFunctions.h"
#endif
extern int FIRST_RUN_FLAG;
//uint8_t prevHullSequence[NUM_OF_MOTORS];

// the correct hull sequence Hall State (Hall a, Hall b, Hall c)
//4 (100) 00  10  01
//6 (110) 01  10  00
//2 (010) 01  00  10
//3 (011) 00  01  10
//1 (001) 10  01  00
//5 (101) 10  00  01
// using this sequence we create a look up table:
int8_t hallIrqCntAdevanceMatrix[6][6] = { {0,0,-1,0,1,0},\
                                          {0,0,1,0,0,-1},\
                                          {1,-1,0,0,0,0},\
                                          {0,0,0,0,-1,1},\
                                          {-1,0,0,1,0,0},\
                                          {0,1,0,-1,0,0}  };

static STimedCallBacks timedCallBacksDB[endOfTimedCallbacksFuncList];
static SIntCallBacks intCallBacksDB[endOfIntCallbacksFuncList];

extern SMotorsData motors[NUM_OF_MOTORS];

//uint32_t pinCounter[16];

void setTimedCallBacksDB(void)
{
  timedCallBacksDB[motorHandle].func = handleMotors;
  timedCallBacksDB[motorHandle].us = CALLBACK_uS(MOTOR_CONTROLLER_HZ); // 1000 hz
  timedCallBacksDB[motorHandle].prevTimeCall = micros();

  timedCallBacksDB[sendmetry].func = callcabk_send_metry;
  timedCallBacksDB[sendmetry].us = CALLBACK_uS(SEND_METRY_HZ); // 300hz
  timedCallBacksDB[sendmetry].prevTimeCall = micros();

  // TODO: for position, change the callback and the hz 
  timedCallBacksDB[updateposition].func = callcabk_update_position;
  timedCallBacksDB[updateposition].us = CALLBACK_uS(UPDATE_POSITION_HZ); // 240 hz
  timedCallBacksDB[updateposition].prevTimeCall = micros();
}

void setIntCallBacksDB(void)
{
  // ====================== Hull motor 1 =================================
  intCallBacksDB[motor1U].func = callback_u_interrupt_m1;
  intCallBacksDB[motor1U].pin = MOTOR1_HALL_U_PIN;

  intCallBacksDB[motor1V].func = callback_v_interrupt_m1;
  intCallBacksDB[motor1V].pin = MOTOR1_HALL_V_PIN;

  intCallBacksDB[motor1W].func = callback_w_interrupt_m1;
  intCallBacksDB[motor1W].pin = MOTOR1_HALL_W_PIN;

  // ====================== Hull motor 2 =================================
  intCallBacksDB[motor2U].func = callback_u_interrupt_m2;
  intCallBacksDB[motor2U].pin = MOTOR2_HALL_U_PIN;

  intCallBacksDB[motor2V].func = callback_v_interrupt_m2;
  intCallBacksDB[motor2V].pin = MOTOR2_HALL_V_PIN;

  intCallBacksDB[motor2W].func = callback_w_interrupt_m2;
  intCallBacksDB[motor2W].pin = MOTOR2_HALL_W_PIN;

  // // ====================== Encoder motor 1 =================================
  // intCallBacksDB[motor1EncA].func = callback_pin1;
  // intCallBacksDB[motor1EncA].pin = SL_EMLIB_GPIO_INIT_I1_PIN;
  // intCallBacksDB[motor1EncA].port = SL_EMLIB_GPIO_INIT_I1_PORT;

  // intCallBacksDB[motor1EncB].func = callback_pin2;
  // intCallBacksDB[motor1EncB].pin = SL_EMLIB_GPIO_INIT_I2_PIN;
  // intCallBacksDB[motor1EncB].port = SL_EMLIB_GPIO_INIT_I2_PORT;

  // intCallBacksDB[motor1EncI].func = callback_pin3;
  // intCallBacksDB[motor1EncI].pin = SL_EMLIB_GPIO_INIT_I3_PIN;
  // intCallBacksDB[motor1EncI].port = SL_EMLIB_GPIO_INIT_I3_PORT;

  // // ====================== Encoder motor 2 =================================
  // intCallBacksDB[motor2EncA].func = callback_pin9;
  // intCallBacksDB[motor2EncA].pin = SL_EMLIB_GPIO_INIT_PB9_PIN;
  // intCallBacksDB[motor2EncA].port = SL_EMLIB_GPIO_INIT_PB9_PORT;

  // intCallBacksDB[motor2EncB].func = callback_pin4;
  // intCallBacksDB[motor2EncB].pin = SL_EMLIB_GPIO_INIT_PC4_PIN;
  // intCallBacksDB[motor2EncB].port = SL_EMLIB_GPIO_INIT_PC4_PORT;

  // intCallBacksDB[motor2EncI].func = callback_pin5;
  // intCallBacksDB[motor2EncI].pin = SL_EMLIB_GPIO_INIT_PC5_PIN;
  // intCallBacksDB[motor2EncI].port = SL_EMLIB_GPIO_INIT_PC5_PORT;
}


void executeTimedFunctions(void)
{
  // static uint32_t prev_time = micros();
  for(int ind = motorHandle; ind < endOfTimedCallbacksFuncList; ind++)
    {
      uint64_t currentTime = micros();
      if(currentTime - timedCallBacksDB[ind].prevTimeCall >= timedCallBacksDB[ind].us)
      {
          timedCallBacksDB[ind].prevTimeCall = currentTime;
          (*timedCallBacksDB[ind].func)();
      }
    }
}


void init_callbacks_GPIO(void)
{
  for(uint8_t ind = 0; ind < endOfIntCallbacksFuncList; ind++)
  {
    Serial.print("Setting interrupt pin : ");
    Serial.println(intCallBacksDB[ind].pin);
    attachInterrupt(digitalPinToInterrupt(intCallBacksDB[ind].pin), intCallBacksDB[ind].func, CHANGE);
  }
}


void callback_u_interrupt_m1()
{
  // Serial.println("Got interupt U M1");
  hullHandle(motor1);
}

void callback_v_interrupt_m1()
{
  // Serial.println("Got interupt V M1");
  hullHandle(motor1);
}

void callback_w_interrupt_m1()
{
  // Serial.println("Got interupt W M1");
  hullHandle(motor1);
}

void callback_u_interrupt_m2()
{
  hullHandle(motor2);
}

void callback_v_interrupt_m2()
{
  hullHandle(motor2);
}

void callback_w_interrupt_m2()
{
  hullHandle(motor2);
}

void hullHandle(int motor)
{
  getMotorHulls(motor);
  motorPhaseConfigurationHandle(motor);
  getHullSequence(motor);
  calcHullAdd(motor);
  motors[motor].hull.cnt_last_time_uSec = micros();
  motors[motor].hull.cnt += motors[motor].hull.hullAdd;
  sendCommandToDriver(motor);
#ifdef DEBUG_HULLS
//  getHullsDBG(motor);
  record_hull(motor);
#endif
}

void callback_change_dir()
{
//  motors[motor2].speedControler.ref *= -1;
//  motors[motor2].positionControler.ref *= -1;
}

void callcabk_send_metry()
{
  Serial.print(micros());
  Serial.print(", 1, ");
  Serial.println(motors[motor1].positionControler.current);
  Serial.print(micros());
  Serial.print(", 2, ");
  Serial.println(motors[motor1].positionControler.ref);
}

static 
void tst_callcabk_update_position()
{
  static int16_t step_i = 0;
  static int8_t dir = 1;
  static float angle = 0.0f;
  static float step_size = PI/NUM_OF_STEPS;
  if((step_i < NUM_OF_STEPS) && (dir == 1))
  {
    angle += step_size;
    motors[motor1].positionControler.ref = angle;
    step_i++;
  }
  else if((step_i >= 0) && (dir == -1))
  {
      angle -= step_size;
      motors[motor1].positionControler.ref = angle;
      step_i--;
  }
  else 
  {
     dir *= -1;
     if (step_i >= NUM_OF_STEPS)
        step_i= NUM_OF_STEPS-1;
     if (step_i < 0)
        step_i = 0;
  }
}

static void callcabk_update_position()
{
  // Sine wave function:
  static float freq = 5.0f;
  static float amp = M_PI / 2 ; 
  static uint32_t t0;
  if (FIRST_RUN_FLAG ==  1) 
     t0 = micros();
  FIRST_RUN_FLAG = 0; 
  
  float t = (micros()-t0) / 1000000.0; // seconds since experiment began
  float sine = amp * sin(2.0f * M_PI * freq * t);
  motors[motor1].positionControler.ref = sine;
}

void callcabk_update_velocity()
{
  motors[motor1].speedControler.ref *= -1;
  // TODO: convert to speed. can also provide a static vector of values and increment index
  // static int16_t i = 0;
  // static int8_t dir = 1;
  // static float step = PI/NUM_OF_STEPS;
  // static float angle = 0.0f;

  // if((i < NUM_OF_STEPS) && (dir == 1))
  // {
  //   angle += step;
  //   motors[motor1].speedControler.ref = 40.0; 
  //   i++;
  // }
  // else if((i >= 0) && (dir == -1))
  // {
  //     angle -= step;
  //     motors[motor1].positionControler.ref = angle;
  //     i--;
  // }
  // else
  // {
  //    dir *= -1;
  //    if (i >= NUM_OF_STEPS)
  //       i= NUM_OF_STEPS-1;
  //    if (i < 0)
  //       i = 0;
  // }
//  motors[motor2].positionControler.ref = PI;
}