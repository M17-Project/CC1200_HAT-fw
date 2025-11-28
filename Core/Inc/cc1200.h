#ifndef INC_CC1200_H_
#define INC_CC1200_H_

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "stm32f4xx_hal.h"
#include "enums.h"

#define CC1200_REG_NUM		51	//number of regs used to initialize CC1200

extern SPI_HandleTypeDef hspi1;

typedef struct
{
	char name[20];			//chip's name (CC1200, CC1201, unknown ID)
	uint32_t rx_frequency;	//frequency in hertz
	uint32_t tx_frequency;	//frequency in hertz
	uint8_t pwr;			//power setting (3..63)
	int16_t fcorr;			//frequency correction
	uint8_t pll_locked;		//PLL locked flag
	uint8_t afc;			//AFC on/off
} trx_data_t;

void set_nRST(uint8_t state);
void set_CS(uint8_t state);
uint8_t trx_readreg(uint16_t addr);
void trx_writereg(uint16_t addr, uint8_t val);
void trx_writecmd(uint8_t addr);
uint8_t read_pn(void);
uint8_t read_status(void);
void detect_rf_ic(char* out);
void config_ic(uint8_t* settings);
void config_rf(enum mode_t mode, trx_data_t trx_data);
void trx_reset(void);

#endif /* INC_CC1200_H_ */
