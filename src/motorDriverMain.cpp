#include "motorDriverMain.h"

fctPtr pwmSetDutyCycle[] = {GPIO_motorPinPWMoutLow , GPIO_motorPinPWMoutHigh , GPIO_motorPinPWMoutDisable};
S_fullMotorPhaseConfiguration motorPhaseConfiguration;
uint8_t gCommotationState[NUM_OF_MOTORS];
SMotorsData motors[NUM_OF_MOTORS];

extern int8_t hallIrqCntAdevanceMatrix[6][6];

void sendCommandToDriver(int motor){

  static uint8_t motor_pwm_ch0;
  static uint8_t motor_pwm_ch1;
  static uint8_t motor_pwm_ch2;

  switch(motor){
    case motor1:
      motor_pwm_ch0 = MOTOR1_PWM_U_PIN;
      motor_pwm_ch1 = MOTOR1_PWM_V_PIN;
      motor_pwm_ch2 = MOTOR1_PWM_W_PIN;
      break;

    case motor2:
      motor_pwm_ch0 = MOTOR2_PWM_U_PIN;
      motor_pwm_ch1 = MOTOR2_PWM_V_PIN;
      motor_pwm_ch2 = MOTOR2_PWM_W_PIN;
      break;

    default:
      return;
  }
  
	(*pwmSetDutyCycle[motors[motor].commutation.Upolarity])(motor_pwm_ch0, motor);
	(*pwmSetDutyCycle[motors[motor].commutation.Vpolarity])(motor_pwm_ch1, motor);
	(*pwmSetDutyCycle[motors[motor].commutation.Wpolarity])(motor_pwm_ch2, motor);
}


void getAllMotorsCommutation(void)
{
  for(int motor = motor1; motor < endOfMotors; motor++)
  {
      getMotorHulls(motor);
  }
}


void getMotorHulls(int motor){
  switch(motor)
  {
    case motor1:
      motors[motor].hull.HullU = digitalRead(MOTOR1_HALL_U_PIN);
      motors[motor].hull.HullV = digitalRead(MOTOR1_HALL_V_PIN);
      motors[motor].hull.HullW = digitalRead(MOTOR1_HALL_W_PIN);
      break;

    case motor2:
      motors[motor].hull.HullU = digitalRead(MOTOR2_HALL_U_PIN);
      motors[motor].hull.HullV = digitalRead(MOTOR2_HALL_V_PIN);
      motors[motor].hull.HullW = digitalRead(MOTOR2_HALL_W_PIN);
      break;
  }
  return;
}

void motorPhaseConfigurationHandle(int motor)
{
  gCommotationState[motor] = (motors[motor].hull.HullU << 2 | motors[motor].hull.HullV << 1 | motors[motor].hull.HullW) & 0x7;

#ifdef DEBUG_SPEED_CONTROL
  record_gCommotationState(motor, gCommotationState[motor]);
#endif


  if ((motors[motor].motorDriveState == DS_CW) || (motors[motor].motorDriveState == DS_STOP))
  {
    motors[motor].commutation = motorPhaseConfiguration.forward[gCommotationState[motor]];
  }
  else if (motors[motor].motorDriveState == DS_CCW)
  {
    motors[motor].commutation = motorPhaseConfiguration.backward[gCommotationState[motor]];
  }
  return;
}

// ==================================== PI speed control algorithm - START ===================================
float PIControl(SPIContorl * data, float kp, float ki)
{
// TODO - enable if each motor has different parrams
//  static float ki;
//  static float kp;
//  switch(motor){
//    case motor2:
//      ki = LEFT_KI;
//      kp = LEFT_KP;
//      break;
//
//    case motor1:
//      ki = RIGHT_KI;
//      kp = RIGHT_KP;
//      break;
//  }
	float Error = data->ref - data->current; // TODO - speedAverage or speedFromHall

	float P_correction = kp * Error;
	float I_correction = ki * Error + data->I_correction; 


	if (I_correction > PI_ANTIWINDUP) // unti-windup
	{
		I_correction = PI_ANTIWINDUP;
	}

	if (I_correction < -1 * PI_ANTIWINDUP) // unti-windup
	{
		I_correction = -1 * PI_ANTIWINDUP;
	}
	data->I_correction = I_correction;

	return P_correction + I_correction;
}
// ==================================== PI linear speed control algorithm - START ===================================


// ==================================== GPIO motor Pin PWM out Disable - START ===================================
void GPIO_motorPinPWMoutDisable(uint8_t motor_pwm_ch, int motor)
{
  (void) motor; // unused argument
  analogWrite(motor_pwm_ch, PWM_VAL(50));
}
// ==================================== GPIO motor Pin PWM out Disable - END ===================================


// ==================================== GPIO motor Pin PWM out High - START ===================================
void GPIO_motorPinPWMoutHigh(uint8_t motor_pwm_ch, int motor){
  analogWrite(motor_pwm_ch, PWM_VAL(motors[motor].PWMCommand));
}
// ==================================== GPIO motor Pin PWM out High - END ===================================


// ==================================== GPIO motor Pin PWM out Low - START ===================================
void GPIO_motorPinPWMoutLow(uint8_t motor_pwm_ch, int motor){
  (void) motor; // unused argument
  analogWrite(motor_pwm_ch, PWM_VAL(0));
}
// ==================================== GPIO motor Pin PWM out Low - END ===================================


// ==================================== set Motor Drive State - START ===================================
void setMotorDriveState(int motor)
{

  if(motors[motor].motorDriveState == DS_STOP)
  {
    if(motors[motor].speedControler.ref < 0)
    {
        motors[motor].motorDriveState = DS_CCW;
    }
    else
    {
        motors[motor].motorDriveState = DS_CW;
    }
  }
  else // motor at drive mode
  {
      if(IS_ZERO_FLOAT(motors[motor].speedControler.ref))
      {
          motors[motor].motorDriveState = DS_STOP;
      }
      else
      {
        if (motors[motor].speedControler.corrected < 0)
        {
            motors[motor].motorDriveState = DS_CCW;
        }
        else
        {
            motors[motor].motorDriveState = DS_CW;
        }
      }
  }
}
// ==================================== set Motor Drive State - END ===================================


// ==================================== set All Motors Drive State - START ===================================
void setAllMotorsDriveState(void)
{
  for(int motor = motor1; motor < endOfMotors; motor++)
  {
      setMotorDriveState(motor);
  }
}
// ==================================== set all Motors Drive State - START ===================================


// ==================================== motor Driver Phase Configuration Initialize - START ===================================
void motorDriverPhaseConfigurationInit(void){

	// Xpolarity = 0 - LOW
	// Xpolarity = 1 - HIGH
	// Xpolarity = 2 - 50% duty cycle

	// ======================= BACKWARD CONFIGURATION - START =============================
	motorPhaseConfiguration.backward[0].Upolarity = NA;
	motorPhaseConfiguration.backward[0].Vpolarity = NA;
	motorPhaseConfiguration.backward[0].Wpolarity = NA;


	motorPhaseConfiguration.backward[1].Upolarity = D;
	motorPhaseConfiguration.backward[1].Vpolarity = H;
	motorPhaseConfiguration.backward[1].Wpolarity = L;


	motorPhaseConfiguration.backward[2].Upolarity = H;
	motorPhaseConfiguration.backward[2].Vpolarity = L;
	motorPhaseConfiguration.backward[2].Wpolarity = D;


	motorPhaseConfiguration.backward[3].Upolarity = H;
	motorPhaseConfiguration.backward[3].Vpolarity = D;
	motorPhaseConfiguration.backward[3].Wpolarity = L;


	motorPhaseConfiguration.backward[4].Upolarity = L;
	motorPhaseConfiguration.backward[4].Vpolarity = D;
	motorPhaseConfiguration.backward[4].Wpolarity = H;


	motorPhaseConfiguration.backward[5].Upolarity = L;
	motorPhaseConfiguration.backward[5].Vpolarity = H;
	motorPhaseConfiguration.backward[5].Wpolarity = D;


	motorPhaseConfiguration.backward[6].Upolarity = D;
	motorPhaseConfiguration.backward[6].Vpolarity = L;
	motorPhaseConfiguration.backward[6].Wpolarity = H;


	motorPhaseConfiguration.backward[7].Upolarity = NA;
	motorPhaseConfiguration.backward[7].Vpolarity = NA;
	motorPhaseConfiguration.backward[7].Wpolarity = NA;
	// ======================= BACKWARD CONFIGURATION - END =============================


	// ======================= FORWARD CONFIGURATION - START =============================
	motorPhaseConfiguration.forward[0].Upolarity = NA;
	motorPhaseConfiguration.forward[0].Vpolarity = NA;
	motorPhaseConfiguration.forward[0].Wpolarity = NA;


	motorPhaseConfiguration.forward[1].Upolarity = D;
	motorPhaseConfiguration.forward[1].Vpolarity = L;
	motorPhaseConfiguration.forward[1].Wpolarity = H;


	motorPhaseConfiguration.forward[2].Upolarity = L;
	motorPhaseConfiguration.forward[2].Vpolarity = H;
	motorPhaseConfiguration.forward[2].Wpolarity = D;


	motorPhaseConfiguration.forward[3].Upolarity = L;
	motorPhaseConfiguration.forward[3].Vpolarity = D;
	motorPhaseConfiguration.forward[3].Wpolarity = H;


	motorPhaseConfiguration.forward[4].Upolarity = H;
	motorPhaseConfiguration.forward[4].Vpolarity = D;
	motorPhaseConfiguration.forward[4].Wpolarity = L;


	motorPhaseConfiguration.forward[5].Upolarity = H;
	motorPhaseConfiguration.forward[5].Vpolarity = L;
	motorPhaseConfiguration.forward[5].Wpolarity = D;


	motorPhaseConfiguration.forward[6].Upolarity = D;
	motorPhaseConfiguration.forward[6].Vpolarity = H;
	motorPhaseConfiguration.forward[6].Wpolarity = L;


	motorPhaseConfiguration.forward[7].Upolarity = NA;
	motorPhaseConfiguration.forward[7].Vpolarity = NA;
	motorPhaseConfiguration.forward[7].Wpolarity = NA;
	// ======================= FORWARD CONFIGURATION - END =============================
}
// ==================================== motor Driver Phase Configuration Initialize - END ===================================


void getHullSequence(int motor)
{
	uint8_t sequence = ((motors[motor].hull.HullU << 2) | (motors[motor].hull.HullV << 1) | (motors[motor].hull.HullW)) & 0x7;
	motors[motor].hull.currentSequence = sequence-1;
}


void calcHullAdd(int motor)
{
  motors[motor].hull.hullAdd =  hallIrqCntAdevanceMatrix[motors[motor].hull.prevSequence][motors[motor].hull.currentSequence];
  motors[motor].hull.prevSequence = motors[motor].hull.currentSequence;
}

void calcSpeedFromHulls(int motor)
{
  // to prevent changes in current time and hall counts during calculation we copy them to local variables
  uint32_t curentTimeuSec = motors[motor].hull.cnt_last_time_uSec;
  uint32_t currentHallCnt = motors[motor].hull.cnt;

	int currentDeltaCount = currentHallCnt - motors[motor].speedControler.lastHullCnt;
	uint32_t currentDt = curentTimeuSec - motors[motor].speedControler.lastHullCalcTimeuSec;

	if (currentDt == 0)
	{
	  motors[motor].speedControler.FromHull = 0;
	  return;
	}

  float motorAngleRotated = currentDeltaCount * RAD_PER_HULL_INT;
  float dtSec = currentDt/1000000.0;
  float motorSpeedBeforeGearRadSec = motorAngleRotated/dtSec;
  motors[motor].speedControler.FromHull = motorSpeedBeforeGearRadSec * INV_GEAR_RATIO;

	motors[motor].speedControler.lastHullCalcTimeuSec = curentTimeuSec;
	motors[motor].speedControler.lastHullCnt = currentHallCnt;

	motors[motor].speedControler.current = motors[motor].speedControler.FromHull;
	motors[motor].speedControler.Average.currentData = motors[motor].speedControler.current;
  movingAverage(&motors[motor].speedControler.Average);
	return;
}


void calcAngleFromEncoder(int motor)
{
  // to prevent changes in current time and hall counts during calculation we copy them to local variables
  uint32_t currentEncoderCnt = motors[motor].encoder.cnt;
  int32_t deltaAngle = currentEncoderCnt - motors[motor].positionControler.lastEncoderCnt;

  // Serial.print("The Encoder current cnt : ");
  // Serial.println(currentEncoderCnt);

  float DeltaAngleRotated = COUNTER_TO_RAD(deltaAngle)*INV_GEAR_RATIO; // the payload angle
  float prevAngle = motors[motor].positionControler.current;
  float currentAngle = DeltaAngleRotated + prevAngle;

  if (currentAngle > 2*PI)
  {
      currentAngle = fmodf(currentAngle, 2*PI);
  }

  motors[motor].positionControler.current = currentAngle;
//  motors[motor].positionControler.Average.currentData = motors[motor].positionControler.current;
  movingAverage(&motors[motor].positionControler.Average);
  motors[motor].positionControler.lastEncoderCnt = currentEncoderCnt;
}

void calcSpeedFromEncoder(int motor)
{
  // to prevent changes in current time and hall counts during calculation we copy them to local variables
  uint32_t currentEncoderCnt = motors[motor].encoder.cnt;
  uint32_t curentTimeuSec = motors[motor].encoder.cnt_last_time_uSec;

  int currentDeltaCount = currentEncoderCnt - motors[motor].speedControler.lastEncoderCnt;
  uint32_t currentDt = curentTimeuSec - motors[motor].speedControler.lastEncoderCalcTimeuSec;

  if (currentDt == 0)
  {
    motors[motor].speedControler.FromEncoder = 0; //TODO - zero order hold the last speed
    return;
  }

  float motorAngleRotated = COUNTER_TO_RAD(currentDeltaCount);
  float dtSec = currentDt/1000000.0;
  float motorSpeedBeforeGearRadSec = motorAngleRotated/dtSec;

  motors[motor].speedControler.FromEncoder = motorSpeedBeforeGearRadSec * INV_GEAR_RATIO;

  motors[motor].speedControler.lastEncoderCalcTimeuSec = curentTimeuSec;
  motors[motor].speedControler.lastEncoderCnt = currentEncoderCnt;

  motors[motor].speedControler.current = motors[motor].speedControler.FromEncoder;
  motors[motor].speedControler.Average.currentData = motors[motor].speedControler.current;
  movingAverage(&motors[motor].speedControler.Average);
  return;
}


void speedControlHandle(int motor)
{
  float speedCorrection = PIControl(&motors[motor].speedControler, KP_SPEED, KI_SPEED);
  motors[motor].speedControler.corrected = motors[motor].speedControler.ref + speedCorrection;
}

void positionControlHandle(int motor)
{
  float speedRef = PIControl(&motors[motor].positionControler, KP_POS, KI_POS);
  Serial.print(micros());
  Serial.print(", 3, ");
  Serial.println(speedRef);

  if(speedRef > MAX_SPEED)
  {
      motors[motor].speedControler.ref = MAX_SPEED;
  }
  else if (speedRef < MIN_SPEED)
  {
      motors[motor].speedControler.ref = MIN_SPEED;
  }
  else
  {
      motors[motor].speedControler.ref = speedRef;
  }
}

void resetMotorData(int motor)
{
	motors[motor].speedControler.I_correction = 0;
	motors[motor].speedControler.corrected = 0;
	motors[motor].speedControler.FromHull = 0;
	motors[motor].speedControler.FromEncoder = 0;
	motors[motor].speedControler.current = 0;
	motors[motor].speedControler.ref = 0;
	motors[motor].speedControler.corrected = 0;
	motors[motor].speedControler.Average.reset = true;
	setMotorDriveState(motor);
}


void resetAllDriveMotorsData(void)
{
  for(int motor = motor1; motor < endOfMotors; motor++)
  {
    resetMotorData(motor);
  }
}


// ==================================== calc PWM command - START ===================================
void calcPWMpercent(int motor)
{
  // calculate the motor needed voltage after PI controller
  float motorSpeedCommandRPM = fabs(motors[motor].speedControler.corrected) * GEAR_RATIO * RadPS_TO_RPM;
  float commandVrms = (motorSpeedCommandRPM / MOTOR_SPEED_CONSTANT) / MOTOR_EFFICIENCY + POWER_SUPPLY_VOLTAGE*STARTING_VOLTAGE;
  uint32_t tmp = (commandVrms/POWER_SUPPLY_VOLTAGE)*100.0;
  if(tmp > 100.0)
  {
      motors[motor].PWMCommand = 100.0;
  }
  else
  {
      motors[motor].PWMCommand = tmp;
  }

#ifdef DEBUG_SPEED_CONTROL
  record_motorSpeedCommandRPM(motor, motorSpeedCommandRPM);
  record_commandVrms(motor, commandVrms);
#endif
  return;
}
// ==================================== calc PWM command - END ===================================


void sendPWMCommadToAllMotors(void)
{
  for(int motor = motor1; motor < endOfMotors; motor++)
  {
      sendCommandToDriver(motor);
  }
}
