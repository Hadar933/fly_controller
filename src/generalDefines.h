#ifndef SRC_GENERALDEFINES_H_
#define SRC_GENERALDEFINES_H_


#include <Arduino.h>

//Debug
//#define DEBUG_SPEED_CONTROL
//#define DEBUG_HULLS

// #define SPEED_CONTROL_ON 
#define POSITION_CONTROL_ON

//============== MOTOR 1 DB =================
#define MOTOR1_PWM_U_PIN 2
#define MOTOR1_PWM_V_PIN 3
#define MOTOR1_PWM_W_PIN 4

#define MOTOR1_HALL_U_PIN 5
#define MOTOR1_HALL_V_PIN 6
#define MOTOR1_HALL_W_PIN 7

//============== MOTOR 2 DB =================
#define MOTOR2_PWM_U_PIN 16
#define MOTOR2_PWM_V_PIN 17
#define MOTOR2_PWM_W_PIN 20

#define MOTOR2_HALL_U_PIN 14
#define MOTOR2_HALL_V_PIN 15
#define MOTOR2_HALL_W_PIN 18

#define SPI_CS    10
#define SPI_MOSI  11
#define SPI_MISO  12
#define SPI_CLK   13
#define SLAVE_CS  10
#define SLAVE_EN  8


#define PWM_VAL(x) ((uint8_t)255*((float)x/100.0f))


// Units conversion
#define RAD_PER_HULL_INT (PI/3.0)
#define RAD_PER_ENCODER_INT (PI/512.0) // TODO - need to check that the encoder gives 1024 interapt per rev
#define PI_ANTIWINDUP (1000.0)
#define IS_ZERO_FLOAT(dataIn) ((dataIn < 0.00001 && dataIn > -0.00001)?true:false)
#define RadPS_TO_RPM (60.0/(2.0*PI))
#define COUNTER_TO_RAD(x) (((float)x)*2*PI/(1024.0f))
#define DEG2RAD(x) (x*PI/180.0)

#define WINDOW_SIZE  (5) // moving average window size

#define NUM_OF_MOTORS (2)

typedef enum{
  motor1 = 0,
  motor2,
  endOfMotors
}EMotor;

#endif /* SRC_GENERALDEFINES_H_ */
