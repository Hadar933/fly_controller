#include "metryHandle.h"

void sendMetry(MetryDB* msg) // TODO - need to fix
{
  uint8_t data[sizeof(MetryDB)] = {};
  memcpy(data, msg, sizeof(MetryDB));
  // UARTDRV_Count_t count = sizeof(data);
  // UARTDRV_TransmitB(sl_uartdrv_usart_inst0_handle, data, count);
}


