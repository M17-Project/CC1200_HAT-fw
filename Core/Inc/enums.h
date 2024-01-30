/*
 * enums.h
 *
 *  Created on: Dec 27, 2023
 *      Author: SP5WWP
 */

#pragma once

enum mode_t
{
	MODE_RX,
	MODE_TX
};

enum strobe_t
{
	STR_SRES = 0x30,		//reset
	STR_SCAL = 0x33,		//calibrate
	STR_SRX,				//set state to "receive"
	STR_STX,				//set state to "transmit"
	STR_IDLE,				//set state to "idle"
	STR_SNOP = 0x3D			//no operation
};

enum trx_state_t
{
	TRX_IDLE,
	TRX_TX,
	TRX_RX
};

enum interface_comm_t
{
	COMM_IDLE,
	COMM_RDY,
	COMM_TOT,
	COMM_OVF
};

enum err_t
{
	ERR_OK,					//all good
	ERR_TRX_PLL,			//RX PLL lock error
	ERR_TRX_SPI,			//RX SPI comms error
};

const char *errstrings[5]=
{
	"OK",
	"PLL did not lock",
	"SPI communication error",
};
