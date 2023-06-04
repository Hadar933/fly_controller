#include "generalDefines.h"
#include "generalPurposeFunctions.h"
#include "motorControlStateMachine.h"
#include "callBacks.h"
#include "motorDriverMain.h"
#include "counter.h"
#include <iostream>
using namespace std;

extern SMotorsData motors[NUM_OF_MOTORS];

float freq = 20;
float amp = M_PI / 6;

void setup() 
{
  Serial.begin(115200);
  // delay(5000);

  Serial.println("At Setup");
  pinMode(MOTOR1_PWM_U_PIN, OUTPUT);
  analogWriteFrequency(MOTOR1_PWM_U_PIN, 1000000);

  pinMode(MOTOR1_PWM_V_PIN, OUTPUT);
  analogWriteFrequency(MOTOR1_PWM_V_PIN, 1000000);

  pinMode(MOTOR1_PWM_W_PIN, OUTPUT);
  analogWriteFrequency(MOTOR1_PWM_W_PIN, 1000000);

  pinMode(MOTOR2_PWM_U_PIN, OUTPUT);
  analogWriteFrequency(MOTOR2_PWM_U_PIN, 1000000);

  pinMode(MOTOR2_PWM_V_PIN, OUTPUT);
  analogWriteFrequency(MOTOR2_PWM_V_PIN, 1000000);

  pinMode(MOTOR2_PWM_W_PIN, OUTPUT);
  analogWriteFrequency(MOTOR2_PWM_W_PIN, 1000000);

  pinMode(SLAVE_CS, OUTPUT);
  pinMode(SLAVE_EN, OUTPUT);

  initSPI();

  motors[motor1].isActive = true;
  motors[motor2].isActive = false;

  motorDriverPhaseConfigurationInit();
  Serial.println("motorDriverPhaseConfigurationInit done");
  set_controller_freqs(PROFILE_FREQ, N_SAMPLES_PER_CYCLE);
  setTimedCallBacksDB();
  Serial.println("setTimedCallBacksDB done");

  setIntCallBacksDB();
  init_callbacks_GPIO();
  Serial.println("init_callbacks_GPIO done");

  counter_default_cfg( );
  Serial.println("counter_default_cfg done");

  delay(300); 

  Serial.println("Setup done");

  // motors[motor1].speedControler.ref = 40.0; // rad/sec.
}

int FIRST_RUN_FLAG = 1;

void loop() 
{
  executeTimedFunctions();
}

