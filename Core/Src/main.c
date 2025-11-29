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
#include "enums.h"
#include "cc1200.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IDENT_STR		"CC1200-HAT 420-450 MHz\nFW v2.0 by Wojciech SP5WWP"
#define UART_BUF_SIZE	1024					//buffer length for UART
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
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
//device config
trx_data_t trx_data;

//device state
uint32_t dev_err = ERR_OK;	//default state - no error

//buffers and interface stuff
volatile uint8_t uart_rx_buf[UART_BUF_SIZE];				//DMA UART RX buffer
uint8_t rxb[UART_BUF_SIZE];									//packet data UART RX buffer
volatile uint16_t rx_count;
volatile uint16_t rx_expected;
uint16_t uart_rx_tail;

volatile enum trx_state_t trx_state = TRX_IDLE;				//transmitter state

//misc
uint8_t dbg_enabled = 0;									//debug strings enabled?
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void interface_resp(enum cmd_t cmd, const uint8_t *resp, uint16_t pld_len)
{
	//single-byte response (4 bytes total)
	static uint8_t tmp[UART_BUF_SIZE];
	tmp[0] = cmd;
	*(uint16_t*)&tmp[1] = pld_len+3;
	if (resp != NULL)
		memcpy(&tmp[3], resp, pld_len);

	while(HAL_UART_Transmit_DMA(&huart1, tmp, pld_len+3)==HAL_BUSY); //blocking
}

void interface_resp_byte(enum cmd_t cmd, uint8_t resp)
{
	interface_resp(cmd, &resp, 1);
}

void dbg_txt(const char *s)
{
	//debug text
	static uint8_t tmp[128];
	uint16_t len = strlen(s);

	tmp[0] = CMD_DBG_TXT;
	*(uint16_t*)&tmp[1] = len+3;
	strcpy((char*)&tmp[3], s);

	while(HAL_UART_Transmit_DMA(&huart1, tmp, len+3)==HAL_BUSY); //blocking
}

void parse_cmd(const uint8_t *cmd_buff)
{
    char dbg_msg[128];
	uint8_t cid = cmd_buff[0];
	int16_t pld_len = *(uint16_t*)&cmd_buff[1] - 3;
	uint8_t *pld = &cmd_buff[3];

	float f_val;
	int8_t i8_val;
	uint8_t u8_val;
	int16_t i16_val;
	uint16_t u16_val;

	/*sprintf(dbg_msg, "CMD%d LEN%d", cid, pld_len);
	dbg_txt(dbg_msg);*/

	switch (cid)
	{
		case CMD_PING:
			interface_resp(cid, (uint8_t*)&dev_err, sizeof(dev_err));
		break;

		case CMD_SET_RX_FREQ:
			if (pld_len == sizeof(float))
			{
				memcpy((uint8_t*)&f_val, pld, sizeof(float));
			}
			else
			{
				interface_resp_byte(cid, ERR_CMD_MALFORM);
				break;
			}

			if(f_val>=420e6 && f_val<=450e6)
			{
				trx_data.rx_frequency = f_val;
				trx_set_freq(f_val);
				interface_resp_byte(cid, ERR_OK);
			}
			else
			{
				interface_resp_byte(cid, ERR_RANGE);
			}
		break;

		case CMD_SET_TX_FREQ:
			if (pld_len == sizeof(float))
			{
				memcpy((uint8_t*)&f_val, pld, sizeof(float));
			}
			else
			{
				interface_resp_byte(cid, ERR_CMD_MALFORM);
				break;
			}

			if(f_val>=420e6 && f_val<=450e6)
			{
				trx_data.tx_frequency = f_val;
				trx_set_freq(f_val);
				interface_resp_byte(cid, ERR_OK);
			}
			else
			{
				interface_resp_byte(cid, ERR_RANGE);
			}
		break;

		case CMD_SET_TX_POWER:
			if (pld_len == 1)
			{
				i8_val = *(int8_t*)pld;
			}
			else
			{
				interface_resp_byte(cid, ERR_CMD_MALFORM);
				break;
			}

			float f_val = i8_val*0.25f; //convert TX power to dBm
			if(f_val>=-16.0f && f_val<=14.0) //-16 to 14 dBm (raw values of 0x03 to 0x3F)
			{
				trx_data.pwr = floorf((f_val+18.0f)*2.0f-1.0f);
				//power setting is now stored in the struct
				//the change will be applied at next RX->TX switch
				interface_resp_byte(cid, ERR_OK);
			}
			else
			{
				interface_resp_byte(cid, ERR_RANGE);
			}
		break;

	  	case CMD_SET_FREQ_CORR:
	  		if (pld_len == sizeof(int16_t))
	  		{
	  			memcpy((uint8_t*)&i16_val, pld, sizeof(int16_t));
	  		}
	  		else
	  		{
	  			interface_resp_byte(cid, ERR_CMD_MALFORM);
	  			break;
	  		}

			if(i16_val>=-200 && i16_val<=200) //keep it within a sane range
			{
				trx_data.fcorr = i16_val;
				interface_resp_byte(cid, ERR_OK);
			}
			else
			{
				interface_resp_byte(cid, ERR_RANGE);
			}
		break;

	  	case CMD_SET_AFC:
	  		if (pld_len == 1)
	  		{
	  			memcpy((uint8_t*)&u8_val, pld, 1);
	  		}
	  		else
	  		{
	  			interface_resp_byte(cid, ERR_CMD_MALFORM);
	  			break;
	  		}

			trx_data.afc = u8_val==0 ? 0 : 1;
			interface_resp_byte(cid, ERR_OK);
		break;

	  	  case CMD_TX_START:
	  		  /*if(trx_state!=TRX_TX && dev_err==ERR_OK)
	  		  {
	  			trx_state=TRX_TX;
	  			config_rf(MODE_TX, trx_data);
	  			HAL_Delay(10);
	  			trx_writecmd(STR_STX);

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
	  		  }*/
		  break;

	  	  case CMD_RX_START:
	  		  /*if(rxb[2]) //start
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
	  			  interface_resp(CMD_SET_RX, ERR_OK); //OK
	  		  }*/
		  break;

	  	  case CMD_GET_IDENT:
			  //reply with the device's IDENT string
			  interface_resp(cid, (uint8_t*)IDENT_STR, strlen(IDENT_STR));
		  break;
	}
}

uint16_t uart_rx_get_head(void)
{
    return UART_BUF_SIZE - huart1.hdmarx->Instance->NDTR;
}

void push_byte(uint8_t *dst_buff, uint8_t inp_byte)
{
	static uint8_t buff[UART_BUF_SIZE] = {0};

	//1st byte - CID
    if (rx_count == 0)
    {
    	buff[0] = inp_byte;
        rx_count = 1;
        rx_expected = 0;
        return;
    }

    //2nd byte - packet length (LSB)
    if (rx_count == 1)
    {
    	buff[1] = inp_byte;
        rx_count = 2;
        return;
    }

    //3rd byte - packet length (MSB)
    if (rx_count == 2)
    {
    	buff[2] = inp_byte;
    	uint16_t l = *(uint16_t*)&buff[1];

        if (l > sizeof(buff))  //length cannot be larger than the buffer
        {
            //invalid length: reset
            rx_count = 0;
            rx_expected = 0;
            return;
        }

        rx_count = 3;
        rx_expected = l;  //total expected bytes: cid+len+payload

        if (rx_expected != rx_count)
        	return;
    }

    //4th byte and beyond - store payload bytes
    if (rx_count != rx_expected && rx_count < sizeof(buff))
    {
        buff[rx_count++] = inp_byte;
    }

    //finish work if rx complete
    if (rx_expected > 0 && rx_count == rx_expected)
    {
        //copy to your main rxb[] buffer exactly like old code
        memcpy(dst_buff, buff, rx_expected);

        //parse the received packet
        parse_cmd(dst_buff);

        //reset state machine
        rx_count = 0;
        rx_expected = 0;
    }
}

void process_uart_dma(uint16_t *tail)
{
    uint16_t head = uart_rx_get_head();

    while (*tail != head)
    {
        uint8_t b = uart_rx_buf[*tail];

        (*tail)++;
        if (*tail >= UART_BUF_SIZE)
            *tail = 0;

        push_byte(rxb, b);
    }

    if (*tail == head && htim10.State==HAL_TIM_STATE_READY)
    {
    	TIM10->CNT = 0;
    	HAL_TIM_Base_Start_IT(&htim10);
    }
}

//interrupts
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	TIM_TypeDef* tim = htim->Instance;

	//UART data RX timeout (2ms)
	if(tim==TIM10)
	{
		rx_count = 0;
		rx_expected = 0;
		HAL_TIM_Base_Stop_IT(&htim10);
	}

	//24kHz baseband timer - workaround
	else if(tim==TIM11)
	{
		;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==TRX_TRIG_Pin)
	{
		;
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  //enable FPU if present
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
  #endif

  //reset the C1200
  HAL_Delay(100);
  trx_reset();

  trx_detect(trx_data.name);

  /*if(strcmp(trx_data.name, "CC1200")==0)
  {
	  while(1)
	  {
		  HAL_GPIO_WritePin(SVC_LED_GPIO_Port, SVC_LED_Pin, 1);
		  HAL_Delay(20);
		  HAL_GPIO_WritePin(SVC_LED_GPIO_Port, SVC_LED_Pin, 0);
		  HAL_Delay(2000);
	  }
  }*/

  //default settings
  trx_data.rx_frequency=433475000;			//default
  trx_data.tx_frequency=433475000;			//default
  trx_data.fcorr=0;							//frequency correction (arbitrary units)
  trx_data.afc=0;							//automatic frequency control (AFC)
  trx_data.pwr=3;							//3 to 63 (arbitrary units)

  //config the CC1200
  trx_config(MODE_RX, trx_data);
  HAL_Delay(10);

  //switch the CC1200 to RX mode
  trx_write_cmd(STR_SRX);
  HAL_Delay(50);

  //check if CC1200 PLL locked
  trx_data.pll_locked = ((trx_read_reg(0x2F8D)^0x80)&0x81)==0x81; //FSCAL_CTRL=1 and FSCAL_CTRL_NOT_USED=0

  if(!trx_data.pll_locked)
  {
	  dev_err|=(1UL<<ERR_TRX_PLL);
  }

  if(strstr((char*)trx_data.name, "unknown")!=NULL)
  {
  	  dev_err|=(1UL<<ERR_TRX_SPI);
  }

  //enable interface comms over UART1
  HAL_UART_Receive_DMA(&huart1, uart_rx_buf, UART_BUF_SIZE);
  FIX_TIMER_TRIGGER(&htim10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
	  process_uart_dma(&uart_rx_tail);

	  /*if(interface_comm==COMM_RDY) //if a valid interface frame is detected
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
				  if(freq>=420e6 && freq<=450e6)
		  		  {
		  			  trx_writecmd(STR_IDLE);
		  			  HAL_Delay(10);
		  			  //reconfig RX
		  			  trx_data.rx_frequency=freq;
		  			  uint32_t freq_word=roundf((float)freq/5000000.0*((uint32_t)1<<16));
		  			  trx_writereg(0x2F0C, (freq_word>>16)&0xFF);
		  			  trx_writereg(0x2F0D, (freq_word>>8)&0xFF);
		  			  trx_writereg(0x2F0E, freq_word&0xFF);
		  			  interface_resp(CMD_SET_RX_FREQ, ERR_OK);
		  		  }
		  		  else
		  		  {
		  			  interface_resp(CMD_SET_RX_FREQ, ERR_RANGE); //ERR
		  		  }
		  	  break;

		  	  case CMD_SET_TX_FREQ:
		  		  memcpy((uint8_t*)&freq, (uint8_t*)&rxb[2], sizeof(uint32_t)); //no sanity checks
				  if(freq>=420e6 && freq<=450e6)
		  		  {
		  			  trx_writecmd(STR_IDLE);
		  			  HAL_Delay(10);
		  			  //reconfig TX
		  			  trx_data.tx_frequency=freq;
		  			  uint32_t freq_word=roundf((float)freq/5000000.0*((uint32_t)1<<16));
		  			  trx_writereg(0x2F0C, (freq_word>>16)&0xFF);
		  			  trx_writereg(0x2F0D, (freq_word>>8)&0xFF);
		  			  trx_writereg(0x2F0E, freq_word&0xFF);
		  			  interface_resp(CMD_SET_TX_FREQ, ERR_OK);
		  		  }
		  		  else
		  		  {
		  			  interface_resp(CMD_SET_TX_FREQ, ERR_RANGE); //ERR
		  		  }
			  break;

		  	  case CMD_SET_TX_POWER:
				  if(rxb[2]*0.25f>=-16.0f && rxb[2]*0.25f<=14.0) //-16 to 14 dBm (0x03 to 0x3F)
				  {
					  tx_dbm=rxb[2]*0.25f;
					  trx_data.pwr=floorf(((float)rxb[2]*0.25f+18.0f)*2.0f-1.0f);
					  interface_resp(CMD_SET_TX_POWER, ERR_OK); //OK
				  }
				  else
				  {
					  //no change, return error code
					  interface_resp(CMD_SET_TX_POWER, ERR_RANGE); //ERR
				  }
			  break;

		  	  case CMD_SET_FREQ_CORR:
		  		  trx_data.fcorr=*((int16_t*)&rxb[2]);
		  		  interface_resp(CMD_SET_FREQ_CORR, ERR_OK); //OK
			  break;

		  	  case CMD_SET_AFC:
				  trx_data.afc = rxb[2];
		  		  interface_resp(CMD_SET_AFC, ERR_OK); //OK
			  break;

		  	  case CMD_SET_TX_START:
		  		  if(trx_state!=TRX_TX && dev_err==ERR_OK)
		  		  {
		  			trx_state=TRX_TX;
		  			config_rf(MODE_TX, trx_data);
		  			HAL_Delay(10);
		  			trx_writecmd(STR_STX);

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
		  			  interface_resp(CMD_SET_RX, ERR_OK); //OK
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

		  	  case 0x88:
		  		  resp[0]=0x88;
		  		  resp[1]=3;
		  		  resp[2]=trx_readreg(0x2F71); //RSSI
		  		  HAL_UART_Transmit_IT(&huart1, resp, 3);
			  break;

		  	  default:
		  		  ;
		  	  break;
		  }
	  }*/

	  //TX
	  /*if(bsb_tx_pend==1)
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
	  }*/

	  //RX
	  /*if(bsb_rx_pend==1)
	  {
		  //fetch baseband sample
		  uint8_t tmp=0xFF; //whatever
		  HAL_SPI_TransmitReceive(&hspi1, &tmp, (uint8_t*)&rx_bsb_sample, 1, 2);
		  HAL_UART_Transmit_IT(&huart1, (uint8_t*)&rx_bsb_sample, 1);
	  }*/
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
#ifdef USE_FULL_ASSERT
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
