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
#define IDENT_STR				"CC1200-HAT 420-450 MHz\nFW v2.0 by Wojciech SP5WWP"
#define UART_LONG_BUF_SIZE		1024				//buffer length for UART (baseband transfers)
#define UART_SHORT_BUF_SIZE		128					//buffer length for UART
#define SHORT_TXQ_SIZE			8					//TX queue length (short messages)
#define LONG_TXQ_SIZE   		2					//TX queue length (baseband transfers)
#define BSB_SIZE				960					//size of baseband chunks
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
//main loop counter
uint32_t rev_cnt;

//device config
trx_data_t trx_data;										//CC1200 config

//device state
uint32_t dev_err = ERR_OK;									//default state - no error

//buffers and interface stuff
uint8_t uart_rx_buf[UART_LONG_BUF_SIZE];					//DMA UART RX buffer
uint8_t rxb[UART_LONG_BUF_SIZE];							//packet data UART RX buffer
volatile uint16_t rx_count;
volatile uint16_t rx_expected;
uint16_t uart_rx_tail;										//for the UART RX ring buffer

typedef struct												//TX queue - short packets
{
	uint8_t  data[UART_SHORT_BUF_SIZE];
	uint16_t len;
} txq_queue_t;

txq_queue_t txq[SHORT_TXQ_SIZE];							//TX queue - short packets

volatile uint8_t txq_head;
volatile uint8_t txq_tail;
volatile uint8_t txq_busy;   								//ongoing DMA transfer?

typedef struct												//TX queue - long packets
{
    uint8_t data[UART_LONG_BUF_SIZE];
    uint16_t len;
} bsbq_queue_t;

bsbq_queue_t bsbq[LONG_TXQ_SIZE];							//TX queue - long packets

volatile uint8_t bsbq_head;
volatile uint8_t bsbq_tail;
volatile uint8_t bsbq_busy;

uint8_t bsb_rx[2*BSB_SIZE];									//rx samples
volatile uint8_t rx_pend;									//pending rx sample read?
uint16_t rx_num_wr;
volatile uint8_t tx_pend;									//pending tx sample write?

volatile enum trx_state_t trx_state = TRX_IDLE;				//transmitter state

//misc
uint8_t dbg_enabled = 0;									//debug strings enabled?
char dbg_msg[128];
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
void set_uart_timeout(uint8_t ms)
{
	TIM10->ARR = (uint32_t)ms*10 - 1;
}

void interface_resp_short(enum cmd_t cid, const uint8_t *resp, uint16_t pld_len)
{
	uint16_t len = pld_len+3;

	//drop oversized
    if (len > UART_SHORT_BUF_SIZE)
    	return;

    //next queue entry
    uint8_t next = (txq_head+1) % SHORT_TXQ_SIZE;

    //queue full, drop packet
    if (next == txq_tail)
    	return;

    txq[txq_head].data[0] = cid;
    memcpy(&txq[txq_head].data[1], (uint8_t*)&len, sizeof(uint16_t));
    memcpy(&txq[txq_head].data[3], resp, pld_len);
    txq[txq_head].len = len;
    txq_head = next;

    //if UART TX is idle, start transmission
    if (!txq_busy && !bsbq_busy)
    {
        txq_busy = 1;
        HAL_UART_Transmit_DMA(&huart1, txq[txq_tail].data, txq[txq_tail].len);
    }
}

void interface_resp_long(enum cmd_t cid, const uint8_t *resp, uint16_t pld_len)
{
	uint16_t len = pld_len+3;

	//drop oversized
    if (len > UART_LONG_BUF_SIZE)
    	return;

    //next queue entry
    uint8_t next = (bsbq_head+1) % LONG_TXQ_SIZE;

    //queue full, drop packet
    if (next == bsbq_tail)
    	return;

    bsbq[bsbq_head].data[0] = cid;
    memcpy(&bsbq[bsbq_head].data[1], (uint8_t*)&len, sizeof(uint16_t));
    memcpy(&bsbq[bsbq_head].data[3], resp, pld_len);
    bsbq[bsbq_head].len = len;
    bsbq_head = next;

    //if UART TX is idle, start transmission
    if (!txq_busy && !bsbq_busy)
    {
    	bsbq_busy = 1;
        HAL_UART_Transmit_DMA(&huart1, bsbq[bsbq_tail].data, bsbq[bsbq_tail].len);
    }
}

void interface_resp_byte(enum cmd_t cmd, uint8_t resp)
{
	//single-byte response (4 bytes total)
	interface_resp_short(cmd, &resp, 1);
}

void dbg_txt(const char *s)
{
	//send short text messages for debugging
	interface_resp_short(CMD_DBG_TXT, (uint8_t*)s, strlen(s));
}

void parse_cmd(const uint8_t *cmd_buff)
{
	uint8_t cid = cmd_buff[0];
	int16_t pld_len; memcpy((uint8_t*)&pld_len, &cmd_buff[1], sizeof(pld_len)); pld_len-=3;
	const uint8_t *pld = &cmd_buff[3];

	float f_val;
	int8_t i8_val;
	uint8_t u8_val;
	int16_t i16_val;
	//uint16_t u16_val;
	//int32_t i32_val;
	uint32_t u32_val;

	/*sprintf(dbg_msg, "CMD%d LEN%d", cid, pld_len);
	dbg_txt(dbg_msg);*/

	switch (cid)
	{
		case CMD_PING:
			interface_resp_short(cid, (uint8_t*)&dev_err, sizeof(dev_err));
		break;

		case CMD_SET_RX_FREQ:
			if (pld_len == sizeof(uint32_t))
			{
				memcpy((uint8_t*)&u32_val, pld, sizeof(uint32_t));
			}
			else
			{
				interface_resp_byte(cid, ERR_CMD_MALFORM);
				break;
			}

			if(u32_val>=420e6 && u32_val<=450e6)
			{
				trx_data.rx_frequency = u32_val;
				trx_set_freq(u32_val);
				interface_resp_byte(cid, ERR_OK);
			}
			else
			{
				interface_resp_byte(cid, ERR_RANGE);
			}
		break;

		case CMD_SET_TX_FREQ:
			if (pld_len == sizeof(uint32_t))
			{
				memcpy((uint8_t*)&u32_val, pld, sizeof(uint32_t));
			}
			else
			{
				interface_resp_byte(cid, ERR_CMD_MALFORM);
				break;
			}

			if(u32_val>=420e6 && u32_val<=450e6)
			{
				trx_data.tx_frequency = u32_val;
				trx_set_freq(u32_val);
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

			f_val = i8_val*0.25f; //convert TX power to dBm
			if(f_val>=-16.0f && f_val<=14.0) //-16 to 14 dBm (raw values of 0x03 to 0x3F)
			{
				trx_data.pwr = floorf((f_val+18.0f)*2.0f-1.0f);
				//power setting is now stored in the struct
				//the change will be applied at the next RX->TX switch
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

	  		//the change will be applied at the next TX->RX switch
			trx_data.afc = u8_val==0 ? 0 : 1;
			interface_resp_byte(cid, ERR_OK);
		break;

	  	  case CMD_TX_START:
	  		  if (pld_len == 1)
	  		  {
	  			  memcpy((uint8_t*)&u8_val, pld, 1);
	  		  }
	  		  else
	  		  {
	  			  interface_resp_byte(cid, ERR_CMD_MALFORM);
	  			  break;
	  		  }

	  		  if(u8_val) //start
	  		  {
				  if(trx_state==TRX_IDLE && dev_err==ERR_OK)
				  {
					  //config CC1200
					  trx_state = TRX_TX;
					  trx_config(MODE_TX, trx_data);
					  HAL_Delay(10);

					  //switch CC1200 to TX
					  trx_write_cmd(STR_STX);

					  //initiate samples transfer (write)
					  uint8_t header[2]={0x2F|0x40, 0x7E}; //CFM_TX_DATA_IN, burst access
					  trx_set_CS(0); //CS low
					  HAL_SPI_Transmit(&hspi1, header, 2, 2); //send 2-byte header

					  //enable external baseband sample triggering
					  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

					  //reply
					  interface_resp_byte(cid, ERR_OK);

					  //signal TX state with LEDs
					  HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, 1);
					  HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 0);
				  }
				  else
				  {
					  if (dev_err!=ERR_OK)
	  					  interface_resp_byte(cid, ERR_OTHER);
					  else
						  interface_resp_byte(cid, ERR_BUSY);
				  }
	  		  }
	  		  else //stop
	  		  {
				  //disable external baseband sample triggering
				  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

				  //finalize samples transfer
				  trx_set_CS(1); //CS high

			 	  //switch state
				  trx_state = TRX_IDLE;

				  //config CC1200
				  trx_config(MODE_RX, trx_data);
				  HAL_Delay(10);

				  //switch CC1200 to RX state
				  trx_write_cmd(STR_SRX);

				  //turn off the TX LED
				  HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, 0);

	  			  //reply
	  			  interface_resp_byte(cid, ERR_OK);
	  		  }
		  break;

	  	  case CMD_RX_START:
	  		  if (pld_len == 1)
	  		  {
	  			  memcpy((uint8_t*)&u8_val, pld, 1);
	  		  }
	  		  else
	  		  {
	  			  interface_resp_byte(cid, ERR_CMD_MALFORM);
	  			  break;
	  		  }

	  		  if(u8_val) //start
	  		  {
	  			  if(trx_state==TRX_IDLE && dev_err==ERR_OK)
	  			  {
	  				  //reset buffers
	  				  memset(bsb_rx, 0, sizeof(bsb_rx));
	  				  rx_pend = 0;
	  				  rx_num_wr = 0;

	  				  //config CC1200
	  				  trx_config(MODE_RX, trx_data);
	  				  HAL_Delay(10);

	  				  //switch CC1200 to RX
	  				  trx_write_cmd(STR_SRX);
	  				  trx_state = TRX_RX;

	  				  //initiate samples transfer (readout)
	  				  uint8_t header[2] = {0x2F|0xC0, 0x7D};	//CFM_RX_DATA_OUT, burst access
	  				  trx_set_CS(0);							//CS low
	  				  HAL_SPI_Transmit(&hspi1, header, 2, 10);	//send 2-byte header

	  				  //for some reason, the external signal runs at 75.7582kHz instead of expected 24kHz
	  				  FIX_TIMER_TRIGGER(&htim11);
	  				  TIM11->CNT = 0;
	  				  HAL_TIM_Base_Start_IT(&htim11);

	  				  //reply
	  				  interface_resp_byte(cid, ERR_OK);

	  				  //signal RX state with LEDs
	  				  HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 1);
	  				  HAL_GPIO_WritePin(TX_LED_GPIO_Port, TX_LED_Pin, 0);
	  			  }
	  			  else
	  			  {
					  if (dev_err!=ERR_OK)
	  					  interface_resp_byte(cid, ERR_OTHER);
					  else
						  interface_resp_byte(cid, ERR_BUSY);
	  			  }
	  		  }
	  		  else //stop
	  		  {
	  			  //disable read baseband trigger signal
	  			  HAL_TIM_Base_Stop_IT(&htim11);

	  			  //finalize SPI samples transfer
	  			  trx_set_CS(1);

	  			  //switch device state
	  			  trx_state = TRX_IDLE;

	  			  //turn off the RX LED
	  			  HAL_GPIO_WritePin(RX_LED_GPIO_Port, RX_LED_Pin, 0);

	  			  //reply
	  			  interface_resp_byte(cid, ERR_OK); //OK
	  		  }
		  break;

	  	  case CMD_TX_DATA:
	  		  //TODO: add a real data handler here
	  		  if (trx_state==TRX_TX && pld_len==960)
	  			  interface_resp_byte(cid, ERR_OK);
	  	  break;

	  	  case CMD_GET_IDENT:
			  //reply with the device's IDENT string
			  interface_resp_short(cid, (uint8_t*)IDENT_STR, strlen(IDENT_STR));
		  break;

	  	  case CMD_GET_CAPS:
			  //so far, the CC1200-HAT can do FM only, half-duplex
			  interface_resp_byte(cid, 0x02);
		  break;

	  	  case CMD_GET_RX_FREQ:
	  		  interface_resp_short(cid, (uint8_t*)&trx_data.rx_frequency, sizeof(uint32_t));
		  break;

	  	  case CMD_GET_TX_FREQ:
	  		  interface_resp_short(cid, (uint8_t*)&trx_data.tx_frequency, sizeof(uint32_t));
		  break;

	  	  case CMD_GET_BSB_BUFF:
	  		  //TODO: put some working code here
	  		  interface_resp_byte(cid, 0);
	  	  break;

	  	  case CMD_GET_RSSI:
	  		  u8_val = trx_read_reg(0x2F71); //RSSI
	  		  interface_resp_byte(cid, u8_val);
		  break;
	}
}

uint16_t uart_rx_get_head(void)
{
    return UART_LONG_BUF_SIZE - huart1.hdmarx->Instance->NDTR;
}

void push_byte(uint8_t *dst_buff, uint8_t inp_byte)
{
	static uint8_t buff[UART_LONG_BUF_SIZE] = {0};

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
    	uint16_t l;

    	memcpy((uint8_t*)&l, &buff[1], sizeof(l));

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
        if (*tail >= UART_LONG_BUF_SIZE)
            *tail = 0;

        push_byte(rxb, b);

        TIM10->CNT = 0;
        if (htim10.State != HAL_TIM_STATE_BUSY)
            HAL_TIM_Base_Start_IT(&htim10);
    }
}

//interrupts
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	TIM_TypeDef* tim = htim->Instance;

	//UART data RX timeout
	if(tim==TIM10)
	{
		rx_count = 0;
		rx_expected = 0;
		HAL_TIM_Base_Stop_IT(&htim10);
	}

	//24kHz baseband timer - workaround
	else if(tim==TIM11)
	{
		rx_pend = 1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==TRX_TRIG_Pin)
	{
		tx_pend = 1;
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
    	if (bsbq_busy)
    	{
    		bsbq_tail = (bsbq_tail+1) % LONG_TXQ_SIZE;

			//any more messages pending?
			if (bsbq_tail != bsbq_head)
			{
				HAL_UART_Transmit_DMA(huart, bsbq[bsbq_tail].data, bsbq[bsbq_tail].len);
			}
			else //TX queue is empty
			{
				bsbq_busy = 0;
			}
    	}

    	else if (txq_busy)
    	{
			txq_tail = (txq_tail+1) % SHORT_TXQ_SIZE;

			//any more messages pending?
			if (txq_tail != txq_head)
			{
				HAL_UART_Transmit_DMA(huart, txq[txq_tail].data, txq[txq_tail].len);
			}
			else //TX queue is empty
			{
				txq_busy = 0;
			}
    	}

    	//continue the transmission if there's data in the other queue
        if (!bsbq_busy && bsbq_tail != bsbq_head)
        {
            //start next long packet
            bsbq_busy = 1;
            HAL_UART_Transmit_DMA(&huart1, bsbq[bsbq_tail].data, bsbq[bsbq_tail].len);
            return;
        }

        if (!txq_busy && txq_tail != txq_head)
        {
            //start next short packet
            txq_busy = 1;
            HAL_UART_Transmit_DMA(&huart1, txq[txq_tail].data, txq[txq_tail].len);
            return;
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

  //set UART timeout
  set_uart_timeout(2); //2ms

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
  HAL_UART_Receive_DMA(&huart1, uart_rx_buf, UART_LONG_BUF_SIZE);
  FIX_TIMER_TRIGGER(&htim10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
	  //main loop "revolution" counter
	  rev_cnt++;

	  if (rev_cnt%3900000UL == 0)
	  {
		  HAL_GPIO_WritePin(SVC_LED_GPIO_Port, SVC_LED_Pin, 1);
	  }
	  else if (rev_cnt%4000000UL == 0)
	  {
		  HAL_GPIO_WritePin(SVC_LED_GPIO_Port, SVC_LED_Pin, 0);
		  rev_cnt = 0;
	  }

	  //handle incoming data over UART
	  process_uart_dma(&uart_rx_tail);

	  if (rx_pend)
	  {
		  uint8_t tmp=0xFF; //dummy value for SPI readout
		  HAL_SPI_TransmitReceive(&hspi1, &tmp, &bsb_rx[rx_num_wr], 1, 2);
		  rx_num_wr++;

		  //send a chunk of samples when ready
		  if (rx_num_wr == BSB_SIZE)
		  {
			  interface_resp_long(CMD_RX_DATA, &bsb_rx[0], BSB_SIZE);
		  }
		  else if (rx_num_wr == 2*BSB_SIZE)
		  {
			  interface_resp_long(CMD_RX_DATA, &bsb_rx[BSB_SIZE], BSB_SIZE);
		  }

		  rx_num_wr %= 2*BSB_SIZE;
		  rx_pend = 0;
	  }

	  if (tx_pend)
	  {
		  //HAL_SPI_Transmit
		  tx_pend = 0;
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
