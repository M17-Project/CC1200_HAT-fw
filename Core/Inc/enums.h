#ifndef INC_ENUMS_H_
#define INC_ENUMS_H_

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
	ERR_TRX_PLL,			//TRX PLL lock error
	ERR_TRX_SPI,			//TRX SPI comms error
	ERR_RANGE,				//value out of range
	ERR_CMD_MALFORM			//malformed command
};

//internal TRX state, as per p. 8 of the datasheet
enum int_state_t
{
	STATE_IDLE,
	STATE_RX,
	STATE_TX,
	STATE_FSTXON,
	STATE_CALIBRATE,
	STATE_SETTLING,
	STATE_RX_FIFO_ERR,
	STATE_TX_FIFO_ERR
};

#endif

