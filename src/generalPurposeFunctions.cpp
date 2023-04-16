#include "generalPurposeFunctions.h"

void continuousAverage(SContinuousAverage * dataIn)
{
	if (dataIn->reset)
	{
		dataIn->N = 0;
		dataIn->AverageData = 0;
		dataIn->currentData = 0;
		dataIn->reset = false;
	}
	else
	{
		++dataIn->N;
		dataIn->AverageData = (1.0/dataIn->N) * dataIn->currentData + ((dataIn->N - 1)/ dataIn->N)*dataIn->AverageData;
	}
}


void movingAverage(SMovingAverage * dataIn)
{
  if (dataIn->reset)
  {
    dataIn->AverageData = 0;
    dataIn->counter = 0;
    dataIn->reset = false;
  }
  else
  {
      if(dataIn->counter < WINDOW_SIZE && !dataIn->arrayFull)
      {
          dataIn->AverageData = 0;
          dataIn->bufferData[dataIn->counter] = dataIn->currentData;
          dataIn->counter++;
          for(uint8_t i=0; i<dataIn->counter; i++)
          {
              dataIn->AverageData += dataIn->bufferData[i];
          }
          dataIn->AverageData = dataIn->AverageData  / ((float)dataIn->counter);
      }
      else
      {
          dataIn->arrayFull = true;
          dataIn->counter = dataIn->counter % WINDOW_SIZE;
          dataIn->popData = dataIn->bufferData[dataIn->counter];
          dataIn->bufferData[dataIn->counter] = dataIn->currentData;
          dataIn->counter++;
          dataIn->AverageData = dataIn->AverageData + (1.0/((float)WINDOW_SIZE))*(dataIn->currentData - dataIn->popData);
      }
  }
  return;
}


