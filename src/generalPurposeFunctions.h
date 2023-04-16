#ifndef INC_GENERALPURPOSEFUNCTIONS_H_
#define INC_GENERALPURPOSEFUNCTIONS_H_

#include "stdbool.h"
#include <stddef.h>
#include "generalDefines.h"

typedef struct SContinuousAverage
{
  float AverageData;
  float currentData;
  float N;
  bool reset;
}SContinuousAverage;

typedef struct SMovingAverage
{
  float AverageData;
  float bufferData[WINDOW_SIZE];
  float popData;
  float currentData;
  uint8_t counter;
  bool arrayFull;
  bool reset;
}SMovingAverage;

// functions protos
void continuousAverage(SContinuousAverage * dataIn);
void movingAverage(SMovingAverage * dataIn);

#define CALLBACK_uS(hz) (1.0/((float)hz)*1000000.0)

#endif /* INC_GENERALPURPOSEFUNCTIONS_H_ */
