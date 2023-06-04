#ifndef SRC_CALLBACKS_H_
#define SRC_CALLBACKS_H_

#include "generalDefines.h"
#include <stddef.h>
#include "motorControlStateMachine.h"
#include "debugFunctions.h"
#include "metryHandle.h"

typedef void (*timedCallbackFctPtr)(void);

typedef struct{
  timedCallbackFctPtr func;
  uint32_t us;
  uint64_t prevTimeCall;
}STimedCallBacks;


typedef enum{
  motorHandle = 0,
  updateposition,
  sendmetry,
  endOfTimedCallbacksFuncList,
}ETimedCallBacksdFunctions;


typedef enum{
  motor1U = 0,
  motor1V,
  motor1W,
  motor2U,
  motor2V,
  motor2W,
  // motor1EncA,
  // motor1EncB,
  // motor1EncI,
  // motor2EncA,
  // motor2EncB,
  // motor2EncI,
  endOfIntCallbacksFuncList
}EIntCallBacksdFunctions;


typedef struct{
  timedCallbackFctPtr func;
  uint8_t pin;
}SIntCallBacks;

// function protos
void set_controller_freqs(float profile_freq, float n_samples);
void setTimedCallBacksDB();
//void setChangeDir(float);
void executeTimedFunctions(void);

void setIntCallBacksDB(void);
void init_callbacks_GPIO(void);

void encoderHandle(int motor);
void hullHandle(int motor);

void callback_change_dir();
void callcabk_send_metry();
void callcabk_update_position();
void callcabk_update_velocity();

float sin_wave(float t0, float A, float f, float t);
float fly_stroke(float t0, float A, float f, float t);

void callback_u_interrupt_m1();
void callback_v_interrupt_m1();
void callback_w_interrupt_m1();
void callback_u_interrupt_m2();
void callback_v_interrupt_m2();
void callback_w_interrupt_m2();

#endif /* SRC_CALLBACKS_H_ */
