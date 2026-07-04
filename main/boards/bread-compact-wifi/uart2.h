#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"


#define UART2_GPIO_TXD           (GPIO_NUM_17)
#define UART2_GPIO_RXD           (GPIO_NUM_18)

#define RX2_BUF_SIZE             (1024)



void Uart2_Init(void);
int Uart2_Send_Byte(uint8_t data);
int Uart2_Send_Data(uint8_t* data, uint16_t len);


uint16_t Uart2_Available(void);
uint8_t Uart2_Read(void);
void Uart2_Clean_Buffer(void);


#ifdef __cplusplus
}
#endif
