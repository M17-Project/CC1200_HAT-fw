/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "interface_cmds.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IDENT_STR		"CC1200-HAT 420-450 MHz\nFW v1.0.0 by Wojciech SP5WWP"
#define CC1200_REG_NUM	51						//number of regs used to initialize CC1200s
#define BSB_BUFLEN		5760ULL					//tx/rx buffer size in samples (240ms at fs=24kHz)
#define BSB_RUNUP		960ULL					//40ms worth of baseband data (at 24kHz)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FIX_TIMER_TRIGGER(handle_ptr) (__HAL_TIM_CLEAR_FLAG(handle_ptr, TIM_SR_UIF))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
#include "enums.h"

uint32_t dev_err=(uint32_t)ERR_OK;	//default state - no error

struct trx_data_t
{
	uint8_t name[20];		//chip's name (CC1200, CC1201, unknown ID)
	uint32_t rx_frequency;	//frequency in hertz
	uint32_t tx_frequency;	//frequency in hertz
	uint8_t pwr;			//power setting (3..63)
	int16_t fcorr;			//frequency correction
	uint8_t pll_locked;		//PLL locked flag
}trx_data;

//PA
float tx_dbm=0.0f;											//RF power setpoint, dBm

//buffers and interface stuff
volatile uint8_t rxb[100]={0};								//rx buffer for interface data
volatile uint8_t rx_bc=0;									//UART1 rx byte counter
volatile int8_t tx_bsb_buff[BSB_BUFLEN]={0};				//buffer for transmission
volatile uint32_t tx_bsb_total_cnt=0;						//how many samples were received
uint32_t tx_bsb_cnt=0;										//how many samples were transmitted
int8_t tx_bsb_sample=0;										//current tx sample
int8_t rx_bsb_sample=0;										//current rx sample

enum trx_state_t trx_state=TRX_IDLE;						//transmitter state
volatile uint8_t bsb_tx_pend=0;								//do we need to transmit another baseband sample?
volatile uint8_t bsb_rx_pend=0;								//do we need to read another baseband sample?
volatile enum interface_comm_t interface_comm=COMM_IDLE;	//interface comm status
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_nRST(uint8_t state)
{
	if(state)
		TRX_nRST_GPIO_Port->BSRR=(uint32_t)TRX_nRST_Pin;
	else
		TRX_nRST_GPIO_Port->BSRR=(uint32_t)TRX_nRST_Pin<<16;
}

void set_CS(uint8_t state)
{
	if(state)
		TRX_nCS_GPIO_Port->BSRR=(uint32_t)TRX_nCS_Pin;
	else
		TRX_nCS_GPIO_Port->BSRR=(uint32_t)TRX_nCS_Pin<<16;
}

uint8_t trx_readreg(uint16_t addr)
{
	uint8_t txd[3]={0, 0, 0};
	uint8_t rxd[3]={0, 0, 0};

	set_CS(0);
	if((addr>>8)==0)
	{
		txd[0]=(addr&0xFF)|0x80;
		txd[1]=0;
		HAL_SPI_TransmitReceive(&hspi1, txd, rxd, 2, 10);
		set_CS(1);
		return rxd[1];
	}
	else
	{
		txd[0]=((addr>>8)&0xFF)|0x80;
		txd[1]=addr&0xFF;
		txd[2]=0;
		HAL_SPI_TransmitReceive(&hspi1, txd, rxd, 3, 10);
		set_CS(1);
		return rxd[2];
	}
}

void trx_writereg(uint16_t addr, uint8_t val)
{
	uint8_t txd[3]={addr>>8, addr&0xFF, val};

	set_CS(0);
	if((addr>>8)==0)
	{
		txd[0]=addr&0xFF;
		txd[1]=val;
		HAL_SPI_Transmit(&hspi1, txd, 2, 10);
	}
	else
	{
		txd[0]=(addr>>8)&0xFF;
		txd[1]=addr&0xFF;
		txd[2]=val;
		HAL_SPI_Transmit(&hspi1, txd, 3, 10);
	}
	set_CS(1);
}

void trx_writecmd(uint8_t addr)
{
	uint8_t txd=addr;

	set_CS(0);
	HAL_SPI_Transmit(&hspi1, &txd, 1, 10);
	set_CS(1);
}

uint8_t read_pn(void)
{
	return trx_readreg(0x2F8F);
}

uint8_t read_status(void)
{
	uint8_t txd=STR_SNOP; //no operation strobe
	uint8_t rxd=0;

	set_CS(0);
	HAL_SPI_TransmitReceive(&hspi1, &txd, &rxd, 1, 10);
	set_CS(1);

	return rxd;
}

void detect_ic(uint8_t* out)
{
	uint8_t trxid=read_pn();

	if(trxid==0x20)
		sprintf((char*)out, "CC1200");
	else if(trxid==0x21)
		sprintf((char*)out, "CC1201");
	else
		sprintf((char*)out, "unknown ID (0x%02X)", trxid);
}

void config_ic(uint8_t* settings)
{
	for(uint8_t i=0; i<CC1200_REG_NUM; i++)
	{
		set_CS(0);
		if(settings[i*3])
			HAL_SPI_Transmit(&hspi1, &settings[i*3], 3, 10);
		else
			HAL_SPI_Transmit(&hspi1, &settings[i*3+1], 2, 10);
		set_CS(1);
		//HAL_Delay(10);
	}
}

void config_rf(enum mode_t mode, struct trx_data_t trx_data)
{
	static uint8_t cc1200_rx_settings[CC1200_REG_NUM*3] =
	{
		0x00, 0x01, 0x08,
		0x00, 0x03, 0x09,
		0x00, 0x08, 0x1F,
		0x00, 0x0A, 0xAD, //deviation - a little bit under 3.3kHz full scale
		0x00, 0x0B, 0x00, //deviation
		0x00, 0x0C, 0x5D,
		0x00, 0x0D, 0x00,
		0x00, 0x0E, 0x8A,
		0x00, 0x0F, 0xCB,
		0x00, 0x10, 0xAC, //RX filter BW - 9.5kHz
		0x00, 0x11, 0x00,
		0x00, 0x12, 0x45,
		0x00, 0x13, 0x43, //symbol rate 2 - 1.5k sym/s
		0x00, 0x14, 0xA9, //symbol rate 1
		0x00, 0x15, 0x2A, //symbol rate 0
		0x00, 0x16, 0x37, //AGC_REF - AGC Reference Level Configuration
		0x00, 0x17, 0xEC,
		0x00, 0x19, 0x11,
		0x00, 0x1B, 0x51,
		0x00, 0x1C, 0x87,
		0x00, 0x1D, 0x00,
		0x00, 0x20, 0x14,
		0x00, 0x26, 0x03,
		0x00, 0x27, 0x00,
		0x00, 0x28, 0x20,
		0x00, 0x2B, 0x03, //output power - 0x03..0x3F (doesn't matter for RX)
		0x00, 0x2E, 0xFF,
		0x2F, 0x00, 0x1C,
		0x2F, 0x01, 0x02, //AFC, 0x22 - on, 0x02 - off
		0x2F, 0x04, 0x0C, //external oscillator's frequency is 40 MHz
		0x2F, 0x05, 0x09, //16x upsampler, CFM enable
		0x2F, 0x0C, 0x57, //frequency - round((float)435000000/5000000*(1<<16))=0x570000
		0x2F, 0x0D, 0x00, //frequency
		0x2F, 0x0E, 0x00, //frequency
		0x2F, 0x10, 0xEE,
		0x2F, 0x11, 0x10,
		0x2F, 0x12, 0x07,
		0x2F, 0x13, 0xAF,
		0x2F, 0x16, 0x40,
		0x2F, 0x17, 0x0E,
		0x2F, 0x19, 0x03,
		0x2F, 0x1B, 0x33,
		0x2F, 0x1D, 0x17,
		0x2F, 0x1F, 0x00,
		0x2F, 0x20, 0x6E,
		0x2F, 0x21, 0x1C,
		0x2F, 0x22, 0xAC,
		0x2F, 0x27, 0xB5,
		0x2F, 0x32, 0x0E,
		0x2F, 0x36, 0x03,
		0x2F, 0x91, 0x08
	};

	static uint8_t cc1200_tx_settings[CC1200_REG_NUM*3] =
	{
		0x00, 0x01, 0x08,
		0x00, 0x03, 0x09,
		0x00, 0x08, 0x1F,
		0x00, 0x0A, 0xAD, //deviation - a little bit under 3.3kHz full scale
		0x00, 0x0B, 0x00, //deviation
		0x00, 0x0C, 0x5D,
		0x00, 0x0D, 0x00,
		0x00, 0x0E, 0x8A,
		0x00, 0x0F, 0xCB,
		0x00, 0x10, 0xAC, //RX filter BW - 9.5kHz (doesn't matter for TX)
		0x00, 0x11, 0x00,
		0x00, 0x12, 0x45,
		0x00, 0x13, 0x43, //symbol rate 2 - 1.5k symb/s
		0x00, 0x14, 0xA9, //symbol rate 1
		0x00, 0x15, 0x2A, //symbol rate 0
		0x00, 0x16, 0x37,
		0x00, 0x17, 0xEC,
		0x00, 0x19, 0x11,
		0x00, 0x1B, 0x51,
		0x00, 0x1C, 0x87,
		0x00, 0x1D, 0x00,
		0x00, 0x20, 0x14,
		0x00, 0x26, 0x03,
		0x00, 0x27, 0x00,
		0x00, 0x28, 0x20,
		0x00, 0x2B, 0x03, //output power - 0x03..0x3F
		0x00, 0x2E, 0xFF,
		0x2F, 0x00, 0x1C,
		0x2F, 0x01, 0x22,
		0x2F, 0x04, 0x0C, //external oscillator's frequency is 40 MHz
		0x2F, 0x05, 0x09, //16x upsampler, CFM enable
		0x2F, 0x0C, 0x57, //frequency - round((float)435000000/5000000*(1<<16))=0x570000
		0x2F, 0x0D, 0x00, //frequency
		0x2F, 0x0E, 0x00, //frequency
		0x2F, 0x10, 0xEE,
		0x2F, 0x11, 0x10,
		0x2F, 0x12, 0x07,
		0x2F, 0x13, 0xAF,
		0x2F, 0x16, 0x40,
		0x2F, 0x17, 0x0E,
		0x2F, 0x19, 0x03,
		0x2F, 0x1B, 0x33,
		0x2F, 0x1D, 0x17,
		0x2F, 0x1F, 0x00,
		0x2F, 0x20, 0x6E,
		0x2F, 0x21, 0x1C,
		0x2F, 0x22, 0xAC,
		0x2F, 0x27, 0xB5,
		0x2F, 0x32, 0x0E,
		0x2F, 0x36, 0x03,
		0x2F, 0x91, 0x08
	};

	uint32_t freq_word;
	if(mode==MODE_RX)
		freq_word=roundf((float)trx_data.rx_frequency/5000000.0*((uint32_t)1<<16));
	else
		freq_word=roundf((float)trx_data.tx_frequency/5000000.0*((uint32_t)1<<16));

	if(mode==MODE_RX)
	{
		cc1200_rx_settings[32*3-1]=(freq_word>>16)&0xFF;
		cc1200_rx_settings[33*3-1]=(freq_word>>8)&0xFF;
		cc1200_rx_settings[34*3-1]=freq_word&0xFF;
		config_ic(cc1200_rx_settings);
		trx_writereg(0x0001, 29);		//IOCFG2, GPIO2 - CLKEN_CFM
		//carrier sense test
		trx_writereg(0x0000, 17);		//IOCFG3, GPIO3 - CARRIER_SENSE
		trx_writereg(0x0018, 256-97);	//AGC_GAIN_ADJUST
		trx_writereg(0x0017, 256-70);	//AGC_CS_THR
	}
	else if(mode==MODE_TX)
	{
		uint8_t tx_pwr=trx_data.pwr;
		if(tx_pwr>0x3F) tx_pwr=0x3F;
		if(tx_pwr<0x03) tx_pwr=0x03;
		cc1200_tx_settings[26*3-1]=tx_pwr;
		cc1200_tx_settings[32*3-1]=(freq_word>>16)&0xFF;
		cc1200_tx_settings[33*3-1]=(freq_word>>8)&0xFF;
		cc1200_tx_settings[34*3-1]=freq_word&0xFF;
		config_ic(cc1200_tx_settings);
		trx_writereg(0x0001, 30);		//IOCFG2, GPIO2 - CFM_TX_DATA_CLK
	}

	//frequency correction
	trx_writereg(0x2F0A, (uint16_t)trx_data.fcorr>>8);
	trx_writereg(0x2F0B, (uint16_t)trx_data.fcorr&0xFF);
	//disable address autoincrement in burst mode (default - enabled)
	trx_writereg(0x2F06, 0);
}

void interface_resp(enum cmd_t cmd, uint8_t resp)
{
	uint8_t tmp[3]={cmd, 3, resp};
	HAL_UART_Transmit_IT(&huart1, tmp, 3);
}

//interrupts
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	TIM_TypeDef* tim = htim->Instance;

	//USART1 timeout timer
	if(tim==TIM10)
	{
		HAL_TIM_Base_Stop_IT(&htim10);

		if(interface_comm==COMM_IDLE)
		{
			interface_comm=COMM_TOT; //set the TOT flag
		}
	}

	//24kHz baseband timer - workaround
	else if(tim==TIM11)
	{
		bsb_rx_pend=1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==TRX_TRIG_Pin)
	{
		bsb_tx_pend=1;
	}
	/*else if(GPIO_Pin==TRX_GPIO3_Pin) //carrier sense test
	{
		if(TRX_GPIO3_GPIO_Port->IDR&(1U<<14))
		{
			SVC_LED_GPIO_Port->BSRR=(uint32_t)SVC_LED_Pin;
		}
		else
		{
			SVC_LED_GPIO_Port->BSRR=(uint32_t)SVC_LED_Pin<<16;
		}
	}*/
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	USART_TypeDef* iface = huart->Instance;

	if(iface==USART1)
	{
		if(trx_state!=TRX_TX) //normal comms if we are not transmitting at the moment
		{
			//check frame's validity
			if(rxb[1]==rx_bc+1)
			{
				HAL_TIM_Base_Stop_IT(&htim10);
				rx_bc=0;
				interface_comm=COMM_RDY;
				return;
			}

			//handle normal comm conditions and overflow
			if(rx_bc<sizeof(rxb)) //all normal - proceed
			{
				rx_bc++;
				HAL_UART_Receive_IT(&huart1, (uint8_t*)&rxb[rx_bc], 1);
				//reset timeout timer
				TIM10->CNT=0;
				FIX_TIMER_TRIGGER(&htim10);
				HAL_TIM_Base_Start_IT(&htim10);
			}
			else //overflow
			{
				interface_comm=COMM_OVF; //set overflow flag, shouldn't normally happen
			}
		}
		else //pass baseband samples to the buffer
		{
			HAL_UART_Receive_IT(&huart1, (uint8_t*)rxb, 1);
			tx_bsb_buff[tx_bsb_total_cnt%BSB_BUFLEN]=rxb[0];
			tx_bsb_total_cnt++;
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  //enable FPU
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif

  set_nRST(1);
  HAL_Delay(50);
  set_CS(1);
  HAL_Delay(100);
  trx_writecmd(STR_SRES);
  HAL_Delay(50);
  detect_ic(trx_data.name);

  /*if(strstr((char*)trx_data.name, "CC1200")!=NULL)
  {
	  while(1)
	  {
		  HAL_GPIO_TogglePin(SVC_LED_GPIO_Port, SVC_LED_Pin);
		  HAL_Delay(100);
	  }
  }*/

  trx_data.rx_frequency=433475000;			//default
  trx_data.tx_frequency=433475000;			//default
  trx_data.fcorr=0;
  trx_data.pwr=3;							//3 to 63
  tx_dbm=10.00f;							//10dBm default
  config_rf(MODE_RX, trx_data);
  HAL_Delay(10);
  trx_writecmd(STR_SRX);
  HAL_Delay(50);

  trx_data.pll_locked = ((trx_readreg(0x2F8D)^0x80)&0x81)==0x81; //FSCAL_CTRL=1 and FSCAL_CTRL_NOT_USED=0

  if(!trx_data.pll_locked)
  {
	  dev_err|=(1UL<<ERR_TRX_PLL);
  }
  if(strstr((char*)trx_data.name, "unknown")!=NULL)
  {
  	  dev_err|=(1UL<<ERR_TRX_SPI);
  }

  //enable interface comms over UART1
  //memset((uint8_t*)rxb, 0, sizeof(rxb));
  HAL_UART_Receive_IT(&huart1, (uint8_t*)rxb, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
	  if(interface_comm==COMM_RDY) //if a valid interface frame is detected
	  {
		  HAL_UART_AbortReceive_IT(&huart1);
		  HAL_UART_Receive_IT(&huart1, (uint8_t*)rxb, 1);
		  interface_comm=COMM_IDLE;

		  uint32_t freq;
		  uint8_t ident[128]={0};
		  uint8_t resp[10]; //response buffer

		  switch(rxb[0])
		  {
		  	  case CMD_PING:
		  		  resp[0]=CMD_PING;
		  		  resp[1]=6;
		  		  memcpy(&resp[2], (uint8_t*)&dev_err, sizeof(uint32_t));
		  		  HAL_UART_Transmit_IT(&huart1, resp, 6); //error code, 0 for OK
		  	  break;

		  	  case CMD_SET_RX_FREQ:
		  		  memcpy((uint8_t*)&freq, (uint8_t*)&rxb[2], sizeof(uint32_t));
		  		  if(freq>=420e6 && freq<=440e6)
		  		  {
		  			  //reconfig RX
		  			  trx_data.rx_frequency=freq;
		  			  config_rf(MODE_RX, trx_data); //optimize this later
		  			  interface_resp(CMD_SET_RX_FREQ, 0); //OK
		  		  }
		  		  else
		  		  {
		  			  interface_resp(CMD_SET_RX_FREQ, 1); //ERR
		  		  }
		  	  break;

		  	  case CMD_SET_TX_FREQ:
		  		  memcpy((uint8_t*)&freq, (uint8_t*)&rxb[2], sizeof(uint32_t)); //no sanity checks
		  		  if(freq>=420e6 && freq<=440e6)
		  		  {
		  			  //reconfig TX
		  			  trx_data.tx_frequency=freq;
		  			  config_rf(MODE_TX, trx_data); //optimize this later
		  			  interface_resp(CMD_SET_TX_FREQ, 0); //OK
		  		  }
		  		  else
		  		  {
		  			  interface_resp(CMD_SET_TX_FREQ, 1); //ERR
		  		  }
			  break;

		  	  case CMD_SET_TX_POWER:
				  if(rxb[2]*0.25f>=-16.0f && rxb[2]*0.25f<=14.0) //-16 to 14 dBm (0x03 to 0x3F)
				  {
					  tx_dbm=rxb[2]*0.25f;
					  trx_data.pwr=floorf(((float)rxb[2]*0.25f+18.0f)*2.0f-1.0f);
					  interface_resp(CMD_SET_TX_POWER, 0); //OK
				  }
				  else
				  {
					  //no change, return error code
					  interface_resp(CMD_SET_TX_POWER, 1); //ERR
				  }
			  break;

		  	  case CMD_SET_FREQ_CORR:
		  		  trx_data.fcorr=*((int16_t*)&rxb[2]);
		  		  interface_resp(CMD_SET_TX_POWER, 0); //OK
			  break;

		  	  case CMD_SET_AFC:
		  		  if(rxb[2])
		  		  {
		  			  trx_writereg(0x2F01, 0x22);
		  		  }
		  		  else
		  		  {
		  			  trx_writereg(0x2F01, 0x02);
		  		  }
		  		  interface_resp(CMD_SET_AFC, 0); //OK
			  break;

		  	  case CMD_SET_TX_START:
		  		  if(trx_state!=TRX_TX && dev_err==ERR_OK)
		  		  {
		  			trx_state=TRX_TX;
		  			config_rf(MODE_TX, trx_data);
		  			HAL_Delay(10);
		  			trx_writecmd(STR_STX);

		  			//stop UART timeout timer
		  			HAL_TIM_Base_Stop_IT(&htim10);
		  			HAL_UART_AbortReceive_IT(&huart1);

		  			//fill the run-up
		  			memset((uint8_t*)tx_bsb_buff, 0, BSB_RUNUP);
		  			tx_bsb_total_cnt=BSB_RUNUP;
		  			//initiate baseband SPI transfer to the transmitter
		  			uint8_t header[2]={0x2F|0x40, 0x7E}; //CFM_TX_DATA_IN, burst access
		  			set_CS(0); //CS low
		  			HAL_SPI_Transmit(&hspi1, header, 2, 10); //send 2-byte header
		  			HAL_UART_Receive_IT(&huart1, (uint8_t*)rxb, 1);
		  			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); //enable external baseband sample trigger signal
		  			HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, 1);
		  			HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 0);
		  		  }
		  		  else
		  		  {
		  			  interface_resp(CMD_SET_TX_START, dev_err);
		  		  }
			  break;

		  	  case CMD_SET_RX:
		  		  if(rxb[2]) //start
		  		  {
		  			  if(trx_state!=TRX_RX && dev_err==ERR_OK)
		  			  {
		  				  config_rf(MODE_RX, trx_data);
		  				  HAL_Delay(10);
		  				  trx_writecmd(STR_SRX);
		  				  trx_state=TRX_RX;
		  				  uint8_t header[2]={0x2F|0xC0, 0x7D}; //CFM_RX_DATA_OUT, burst access
		  				  set_CS(0); //CS low
		  				  HAL_SPI_Transmit(&hspi1, header, 2, 10); //send 2-byte header
		  				  //for some reason, the external signal runs at 75.7582kHz instead of expected 24kHz
		  				  FIX_TIMER_TRIGGER(&htim11);
		  				  TIM11->CNT=0;
		  				  HAL_TIM_Base_Start_IT(&htim11);
		  				  HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 1);
		  				  HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, 0);
		  			  }
		  			  else
		  			  {
		  				  interface_resp(CMD_SET_RX, dev_err);
		  			  }
		  		  }
		  		  else //stop
		  		  {
		  			  //disable read baseband trigger signal
		  			  HAL_TIM_Base_Stop_IT(&htim11);
		  			  set_CS(1);
		  			  trx_state=TRX_IDLE;
		  			  HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 0);
		  			  interface_resp(CMD_SET_RX, 0); //OK
		  		  }
			  break;

		  	  case CMD_GET_IDENT:
				  //reply with RRU's IDENT string
				  sprintf((char*)&ident[2], IDENT_STR);
				  ident[0]=0x80; //a reply to "Get IDENT string" command
				  ident[1]=strlen((char*)IDENT_STR)+2; //total length
				  HAL_UART_Transmit_IT(&huart1, ident, ident[1]);
			  break;

		  	  case CMD_GET_CAPS:
				  //so far the CC1200-HAT can do FM only, half-duplex
				  interface_resp(CMD_GET_CAPS, 0x02);
			  break;

		  	  case CMD_GET_RX_FREQ:
		  		  resp[0]=CMD_GET_RX_FREQ;
		  		  resp[1]=sizeof(uint32_t)+2;
		  		  memcpy(&resp[2], (uint8_t*)&trx_data.rx_frequency, sizeof(uint32_t));
		  		  HAL_UART_Transmit_IT(&huart1, resp, resp[1]);
			  break;

		  	  case CMD_GET_TX_FREQ:
		  		  resp[0]=CMD_GET_TX_FREQ;
		  		  resp[1]=sizeof(uint32_t)+2;
		  		  memcpy(&resp[2], (uint8_t*)&trx_data.tx_frequency, sizeof(uint32_t));
		  		  HAL_UART_Transmit_IT(&huart1, resp, resp[1]);
			  break;

		  	  /*case 0x88:
		  		  resp[0]=0x88;
		  		  resp[1]=3;
		  		  resp[2]=trx_readreg(0x2F71); //RSSI
		  		  HAL_UART_Transmit_IT(&huart1, resp, 3);
			  break;*/

		  	  default:
		  		  ;
		  	  break;
		  }
	  }
	  else if(interface_comm==COMM_TOT || interface_comm==COMM_OVF)
	  {
		  HAL_TIM_Base_Stop_IT(&htim10);
		  HAL_UART_AbortReceive_IT(&huart1);
		  rx_bc=0;
		  HAL_UART_Receive_IT(&huart1, (uint8_t*)rxb, 1);
		  interface_comm=COMM_IDLE;
	  }

	  //TX
	  if(bsb_tx_pend==1)
	  {
		  //send baseband sample ASAP
		  HAL_SPI_Transmit_IT(&hspi1, (uint8_t*)&tx_bsb_sample, 1);
		  //fetch another sample
		  tx_bsb_sample=tx_bsb_buff[tx_bsb_cnt%BSB_BUFLEN];
		  tx_bsb_cnt++;

		  //nothing else to transmit (or buffer underrun ;)
		  if(tx_bsb_cnt>=tx_bsb_total_cnt)
		  {
			  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); //disable external baseband sample trigger signal
			  set_CS(1); //CS high
			  trx_state=TRX_IDLE;
			  config_rf(MODE_RX, trx_data);
			  HAL_Delay(10);
			  trx_writecmd(STR_SRX);
			  tx_bsb_cnt=0;
			  tx_bsb_total_cnt=0;
			  tx_bsb_sample=0;
			  HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, 0);
		  }

		  bsb_tx_pend=0;
	  }

	  //RX
	  if(bsb_rx_pend==1)
	  {
		  //fetch baseband sample
		  uint8_t tmp=0xFF; //whatever
		  HAL_SPI_TransmitReceive(&hspi1, &tmp, (uint8_t*)&rx_bsb_sample, 1, 2);
		  HAL_UART_Transmit_IT(&huart1, (uint8_t*)&rx_bsb_sample, 1);
		  bsb_rx_pend=0;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 9600-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 20-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 400-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 10-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRX_nCS_GPIO_Port, TRX_nCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TRX_nRST_Pin|SVC_LED_Pin|RX_LED_Pin|TX_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TRX_nCS_Pin */
  GPIO_InitStruct.Pin = TRX_nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TRX_nCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRX_nRST_Pin SVC_LED_Pin RX_LED_Pin TX_LED_Pin */
  GPIO_InitStruct.Pin = TRX_nRST_Pin|SVC_LED_Pin|RX_LED_Pin|TX_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TRX_GPIO0_Pin TRX_TRIG_Pin TRX_GPIO3_Pin */
  GPIO_InitStruct.Pin = TRX_GPIO0_Pin|TRX_TRIG_Pin|TRX_GPIO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); //immediately disable it, as we don't need it right away
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
