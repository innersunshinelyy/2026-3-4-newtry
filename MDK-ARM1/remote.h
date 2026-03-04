#ifndef REMOTE_H
#define REMOTE_H
#include "stm32g4xx_hal.h"

typedef struct
{
  
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;

  uint8_t sw1;
  uint8_t sw2;
	uint8_t sw3;
	uint8_t sw4;
	uint8_t sw5;
	
	uint8_t button1;
	uint8_t button2;
	uint8_t button3;
	uint8_t button4;
	uint8_t button5;
	uint8_t button6;
	
	int16_t cir;
} rc_info_t;

extern rc_info_t rc;
void Remote_DataUpdate(uint8_t *Remote_RawData);
extern double remote_spedx, remote_spedy, remote_cir;
extern double remote_spedx_zuo, remote_spedy_zuo;
void rc_callback_handler(rc_info_t *rc, uint8_t *buff);


#endif