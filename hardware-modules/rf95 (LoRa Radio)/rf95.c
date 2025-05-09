#include "rf95.h"
#include "global.h"
#include "motion.h"
#include "RTT/SEGGER_RTT.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

extern SPI_HandleTypeDef hspi1;

//Register Addresses
#define REG_FIFO             0x00
#define REG_OP_MODE          0x01
#define REG_FREQ_MSB         0x06
#define REG_FREQ_MID         0x07
#define REG_FREQ_LSB         0x08
#define REG_POWER_CONFIG     0x09
#define REG_RX_CURRENT       0x10
#define REG_FIFO_PTR         0x0D
#define REG_TX_FIFO_PTR			 0x0E
#define REG_RX_FIFO_PTR      0x0F
#define REG_IRQ_FLAGS        0x12
#define REG_RX_REC_LEN       0x13
#define REG_RSSI             0x1B
#define REG_MODEM_CONFIG     0x1D
#define REG_TX_PAYLOAD_LEN   0x22

//bit masks
#define BM_WRITE              0x80
#define BM_LONG_RANGE_MODE    0x80
#define BM_SLEEP_MODE         0x00
#define BM_STANDBY_MODE       0x01
#define BM_TX_MODE            0x03
#define BM_RX_CONTINUOUS_MODE 0x05
#define BM_RX_SINGLE_MODE     0x06
#define BM_RX_DONE            0x40
#define BM_TX_DONE            0x08
#define BM_PA_SELECT          0x80

//common values
#define FIFO_RX_ADDR         0x00
#define FIFO_TX_ADDR         0x80

static uint8_t out_buffer[20] = {0,0};
static uint8_t in_buffer[20] = {0,0};
static uint8_t rx_buffer[20] = {0,0};
static uint8_t tx_buffer[20] = {0,0};

//holds the current mode of the device
static uint8_t mode;

void write_reg(uint8_t reg, uint8_t data)
{
  HAL_GPIO_WritePin(GPIOA, NSS_Pin, RESET);
  HAL_Delay(1);
  out_buffer[0] = reg | BM_WRITE;
  out_buffer[1] = data;
  HAL_SPI_TransmitReceive(&hspi1, out_buffer, in_buffer, 2, 100);
  HAL_GPIO_WritePin(GPIOA, NSS_Pin, SET);
}

uint8_t read_reg(uint8_t reg)
{
  HAL_GPIO_WritePin(GPIOA, NSS_Pin, RESET);
  HAL_Delay(1);
  out_buffer[0] = reg;
  out_buffer[1] = 0x00;
  HAL_SPI_TransmitReceive(&hspi1, out_buffer, in_buffer, 2, 100);
  HAL_GPIO_WritePin(GPIOA, NSS_Pin, SET);
  return in_buffer[1];
}

void init_rf_95(void)
{
  //set the reset pin high
  HAL_GPIO_WritePin(GPIOA, RESET_RFM_Pin, SET);
  HAL_Delay(10);

  //initialize the
  mode = BM_STANDBY_MODE;

  //put into sleep mode and LoRa mode
  write_reg(REG_OP_MODE, BM_SLEEP_MODE | BM_LONG_RANGE_MODE);
  HAL_Delay(10);

  //put back into standby mode
  set_rf_95_mode_standby();

  //set the RX address to 0
  write_reg(REG_RX_FIFO_PTR, FIFO_RX_ADDR);
  //set the TX address to 0x80
  write_reg(REG_TX_FIFO_PTR, FIFO_TX_ADDR);

  //setup modem configs
  write_reg(0x1d, 0x72);
  write_reg(0x1e, 0x74);
  write_reg(0x26, 0x00);

  //set the tx power lowish
  set_rf_95_tx_power(10);
}

void set_rf_95_mode_rx(void)
{
  if (mode != BM_RX_CONTINUOUS_MODE)
  {
    write_reg(REG_OP_MODE, BM_RX_CONTINUOUS_MODE);
    mode = BM_RX_CONTINUOUS_MODE;
  }
}

void set_rf_95_mode_tx(void)
{
  if (mode != BM_TX_MODE)
  {
    write_reg(REG_OP_MODE, BM_TX_MODE);
    mode = BM_TX_MODE;
  }
}

void set_rf_95_mode_standby(void)
{
  if (mode != BM_STANDBY_MODE)
  {
    write_reg(REG_OP_MODE, BM_STANDBY_MODE);
    mode = BM_STANDBY_MODE;
  }
}

void set_rf_95_sleep(void)
{
  if (mode != BM_SLEEP_MODE)
  {
    write_reg(REG_OP_MODE, BM_SLEEP_MODE);
    mode = BM_SLEEP_MODE;
  }
}

void set_rf_95_tx_power(uint8_t power)
{
  write_reg(REG_POWER_CONFIG, BM_PA_SELECT | (power));
}

void get_rf_95_rx_data(void)
{
  //TODO make this interrupt driven

  //set to rx mode
  set_rf_95_mode_rx();

  //read the interrupt flag register
  uint8_t interrupt_reg = read_reg(REG_IRQ_FLAGS);

  //check for a rx done packet
  if (interrupt_reg & BM_RX_DONE)
  {
    //read the total length on data in the FIFO
    uint8_t len = read_reg(REG_RX_REC_LEN);

    //read in the fifo position of the last packet received
    uint8_t rx_current_address = read_reg(REG_RX_CURRENT);
    //reset the fifo read ptr to start of read block
    write_reg(REG_FIFO_PTR, rx_current_address);

    //read in the number of bytes in the fifo buffer
    HAL_GPIO_WritePin(GPIOA, NSS_Pin, RESET);
    HAL_Delay(1);
    out_buffer[0] = REG_FIFO;
    out_buffer[1] = FIFO_RX_ADDR;
    HAL_SPI_TransmitReceive(&hspi1, out_buffer, rx_buffer, len+1, 100);
    HAL_GPIO_WritePin(GPIOA, NSS_Pin, SET);

    //process the packet
    rf_parser();

    //clear all irq flags
    write_reg(REG_IRQ_FLAGS, 0xFF);

    //reset the device to standby
    set_rf_95_mode_standby();
  }
}

void get_tx_buffer(void)
{
  //reset the fifo read ptr to start of read block
  write_reg(REG_FIFO_PTR, FIFO_RX_ADDR);
  //read in the number of bytes in the fifo buffer
  HAL_GPIO_WritePin(GPIOA, NSS_Pin, RESET);
  HAL_Delay(1);
  out_buffer[0] = REG_FIFO;
  out_buffer[1] = FIFO_TX_ADDR;
  HAL_SPI_TransmitReceive(&hspi1, out_buffer, in_buffer, 10, 100);
  HAL_GPIO_WritePin(GPIOA, NSS_Pin, SET);
}

void send_rf_95_data(uint8_t* data, uint8_t len)
{
  //place the device in standby mode
  set_rf_95_mode_standby();

  //set the fifo position
  write_reg(REG_FIFO_PTR, FIFO_TX_ADDR);
  HAL_Delay(1);

  //write the data in one transaction
  HAL_GPIO_WritePin(GPIOA, NSS_Pin, RESET);
  HAL_Delay(1);
  out_buffer[0] = REG_FIFO | BM_WRITE;
  memcpy(out_buffer+1, data, len);
  HAL_SPI_TransmitReceive(&hspi1, out_buffer, in_buffer, len+1, 100);
  HAL_GPIO_WritePin(GPIOA, NSS_Pin, SET);

  //write the payload length
  write_reg(REG_TX_PAYLOAD_LEN, len);
  HAL_Delay(10);

  //set the mode to tx to send the message
  set_rf_95_mode_tx();
  while(0x08 != read_reg(REG_IRQ_FLAGS)) {}
  write_reg(REG_IRQ_FLAGS, 0xFF);
  set_rf_95_mode_standby();
}

void rf_parser(void)
{
  uint8_t packet_type = rx_buffer[2];

  if (packet_type != 0)
  {
  	//confirm the packet is for this device id
  	//or that it is a stream to all packet (id = 0x00)
  	if (rx_buffer[1] == DEVICE_ID || rx_buffer[1] == 0x00)
  	{
			//sets the jog mode of all axis
			if(packet_type == E_STOP_ID)
			{
				//deactive all motor functionality
			}
			else if(packet_type == PING_ID)
			{
				//respond with the device ID
				tx_buffer[0] = 0x01; 				//this is the id of the central node
				tx_buffer[1] = PING_ID;		  //this is the command type of the central node
				tx_buffer[2] = DEVICE_ID;
				//send_rf_95_data(tx_buffer, 3);
			}
			else if(packet_type == MOVE_AZ_TO_ID)
			{
				//set the target position
				move_az_to(rx_buffer[3]);
			}
			else if(packet_type == MOVE_TILT_TO_ID)
			{
				//set the target position
				move_tilt_to(rx_buffer[3]);
			}
			else if(packet_type == HOME_UNIT_ID)
			{
				// home both axis
				setStateToHome();
			}
			else if(packet_type == GET_POS_ID)
			{
				//send back the index of both az and tilt
				tx_buffer[0] = 0x01; 				//this is the id of the central node
				tx_buffer[1] = GET_POS_ID;		//this is the command type of the central node
				tx_buffer[2] = getAzIndex();
				tx_buffer[3] = getTiltIndex();
				send_rf_95_data(tx_buffer, 4);
			}
			else if(packet_type == TWO_AXIS_SET_POS_AZ_ID)
			{
				//set the current az position to the passed in value
				setAzTo(rx_buffer[3]);
			}
			else if(packet_type == TWO_AXIS_SET_POS_TILT_ID)
			{
				//set the current tilt position to the passed in value
				setTiltTo(rx_buffer[3]);
			}
			else if(packet_type == TWO_AXIS_MOVE_HOME_ID)
			{
				//perform the homing processes
				homeAxis();
			}

  	}
  }
}
