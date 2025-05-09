#include "neo_m9n.h"
#include <stdlib.h>
#include <string.h>

//update to reflect the correct i2c device
//TODO update to have this passed in on initialization
I2C_HandleTypeDef hi2c1;

//device id
static const uint8_t device_id = 0x84; //7-bit value of device-id: 0x42

//holds incoming and outgoing i2c data
static uint8_t incoming_data[50];
static uint8_t outgoing_data[20];

//holds the parsed latitude and longitude information
volatile static float latitude = 0.0;
volatile static float longitude = 0.0;

//processes incoming messages on the i2c line from the gps unit
bool process_gps_messages(void)
{
  //check for number of bytes available
  outgoing_data[0] = 0xFD;                                                  //the registers that contain the number of available bytes to read
  HAL_I2C_Master_Transmit(&hi2c1, device_id, outgoing_data, 1, 250);        //request the msb and lsb bytes of available data
  for (uint8_t i = 0; i < 30; i++)                                          //spin wait for a few clock cycles
  {
    asm("NOP");
  }
  HAL_I2C_Master_Receive(&hi2c1, device_id, incoming_data, 2, 250);         //read in the available data bytes

  //the device sometimes returns this when there are no bytes available, discard this reads attempt
  if(incoming_data[1] == 0xFF)
    return false;

  //calculate the total bytes from the msb and lsb bytes
  uint16_t available_bytes = (uint16_t)incoming_data[0] << 8 | incoming_data[1];

  //if there are no bytes to read then discard this read attempt
  if(available_bytes == 0)
    return false;

  //read in the number of bytes available from the gps
  outgoing_data[0] = 0xFF;                                                  //the register that contains the data to be read
  HAL_I2C_Master_Transmit(&hi2c1, device_id, outgoing_data, 1, 250);        //set the read from register as active
  for (uint8_t i = 0; i < 30; i++)                                          //spin wait for a few clock cycles
  {
    asm("NOP");
  }

  //make sure the bytes dont overflow the array
  if (available_bytes > 50)
    available_bytes = 50;

  HAL_I2C_Master_Receive(&hi2c1, device_id, incoming_data, available_bytes, 250);         //read in the available data bytes

  //check that it is the right/full packet
  if (incoming_data[0] != 'N')
  {
    return false;
  }

  //check for NTXT Packet
  if (incoming_data[1] == 'T')
  {
    return false;
  }

  //get the latitude
  char *lat_start_location = strchr((char*)(incoming_data), 0x2C);   //find the first occurrence of a comma
  char *lat_sign_start_location = strchr(lat_start_location+1, 0x2C);  //find the start of the N/S declaration for longitude
  char *lon_stop_location = strchr(lat_sign_start_location+3, 0x2C); //find the end of longitude location
  //check for a bad packet
  if (lat_start_location == NULL || lat_sign_start_location == NULL || lon_stop_location == NULL)
  {
    return false;
  }
  //check for an empty packet
  if ((lat_sign_start_location - lat_start_location) < 2)
  {
    return false;
  }
  char temp_string[10];
  char temp_string_short[3];
  memcpy(temp_string, lat_start_location+3, lat_sign_start_location - lat_start_location-1);
  latitude = atof(temp_string);
  memcpy(temp_string_short, lat_start_location+1, 2);
  latitude = atof(temp_string_short) + latitude/60.0;
  if (*(lat_sign_start_location+1) == 'S')
  {
    latitude *= -1.0;
  }

  //update the feedback value to be sent to the host computer
  update_latitude(latitude);

  //get the longitude
  memcpy(temp_string, lat_sign_start_location+6, lon_stop_location - (lat_sign_start_location+4));
  longitude = atof(temp_string);
  memcpy(temp_string_short, lat_sign_start_location+3, 3);
  longitude = atof(temp_string_short) + longitude/60.0;
  if (*(lon_stop_location+1) == 'W')
  {
    longitude *= -1.0;
  }

  return true;
}

bool configure_gps(void)
{
  //turn off GGA I2C
  outgoing_data[0] = 0xb5;      //preamble 1
  outgoing_data[1] = 0x62;      //preamble 2
  outgoing_data[2] = 0x06;      //config class byte
  outgoing_data[3] = 0x8a;      //config ID byte
  outgoing_data[4] = 0x09;      //length of the payload lsb
  outgoing_data[5] = 0x00;      //length of the payload msb
  outgoing_data[6] = 0x00;
  outgoing_data[7] = 0x04;
  outgoing_data[8] = 0x00;
  outgoing_data[9] = 0x00;
  outgoing_data[10] = 0xba;
  outgoing_data[11] = 0x00;
  outgoing_data[12] = 0x91;
  outgoing_data[13] = 0x20;
  outgoing_data[14] = 0x00;
  outgoing_data[15] = 0x08;
  outgoing_data[16] = 0xde;
  //issue the command to the gps
  HAL_I2C_Master_Transmit(&hi2c1, device_id, outgoing_data, 17, 250);

  HAL_Delay(1);
  //turn on GLL I2C to 2
  outgoing_data[0] = 0xb5;      //preamble 1
  outgoing_data[1] = 0x62;      //preamble 2
  outgoing_data[2] = 0x06;      //config class byte
  outgoing_data[3] = 0x8a;      //config ID byte
  outgoing_data[4] = 0x09;      //length of the payload lsb
  outgoing_data[5] = 0x00;      //length of the payload msb
  outgoing_data[6] = 0x00;
  outgoing_data[7] = 0x04;
  outgoing_data[8] = 0x00;
  outgoing_data[9] = 0x00;
  outgoing_data[10] = 0xc9;
  outgoing_data[11] = 0x00;
  outgoing_data[12] = 0x91;
  outgoing_data[13] = 0x20;
  outgoing_data[14] = 0x02;
  outgoing_data[15] = 0x19;
  outgoing_data[16] = 0x2b;
  //issue the command to the gps
  HAL_I2C_Master_Transmit(&hi2c1, device_id, outgoing_data, 17, 250);

  HAL_Delay(1);
  //turn off GSA I2C
  outgoing_data[0] = 0xb5;      //preamble 1
  outgoing_data[1] = 0x62;      //preamble 2
  outgoing_data[2] = 0x06;      //config class byte
  outgoing_data[3] = 0x8a;      //config ID byte
  outgoing_data[4] = 0x09;      //length of the payload lsb
  outgoing_data[5] = 0x00;      //length of the payload msb
  outgoing_data[6] = 0x00;
  outgoing_data[7] = 0x04;
  outgoing_data[8] = 0x00;
  outgoing_data[9] = 0x00;
  outgoing_data[10] = 0xbf;
  outgoing_data[11] = 0x00;
  outgoing_data[12] = 0x91;
  outgoing_data[13] = 0x20;
  outgoing_data[14] = 0x00;
  outgoing_data[15] = 0x0d;
  outgoing_data[16] = 0xf7;
  //issue the command to the gps
  HAL_I2C_Master_Transmit(&hi2c1, device_id, outgoing_data, 17, 250);

  HAL_Delay(1);
  //turn off GSV I2C
  outgoing_data[0] = 0xb5;      //preamble 1
  outgoing_data[1] = 0x62;      //preamble 2
  outgoing_data[2] = 0x06;      //config class byte
  outgoing_data[3] = 0x8a;      //config ID byte
  outgoing_data[4] = 0x09;      //length of the payload lsb
  outgoing_data[5] = 0x00;      //length of the payload msb
  outgoing_data[6] = 0x00;
  outgoing_data[7] = 0x04;
  outgoing_data[8] = 0x00;
  outgoing_data[9] = 0x00;
  outgoing_data[10] = 0xc4;
  outgoing_data[11] = 0x00;
  outgoing_data[12] = 0x91;
  outgoing_data[13] = 0x20;
  outgoing_data[14] = 0x00;
  outgoing_data[15] = 0x12;
  outgoing_data[16] = 0x10;
  //issue the command to the gps
  HAL_I2C_Master_Transmit(&hi2c1, device_id, outgoing_data, 17, 250);

  HAL_Delay(1);
  //turn off RMC I2C
  outgoing_data[0] = 0xb5;      //preamble 1
  outgoing_data[1] = 0x62;      //preamble 2
  outgoing_data[2] = 0x06;      //config class byte
  outgoing_data[3] = 0x8a;      //config ID byte
  outgoing_data[4] = 0x09;      //length of the payload lsb
  outgoing_data[5] = 0x00;      //length of the payload msb
  outgoing_data[6] = 0x00;
  outgoing_data[7] = 0x04;
  outgoing_data[8] = 0x00;
  outgoing_data[9] = 0x00;
  outgoing_data[10] = 0xab;
  outgoing_data[11] = 0x00;
  outgoing_data[12] = 0x91;
  outgoing_data[13] = 0x20;
  outgoing_data[14] = 0x00;
  outgoing_data[15] = 0xf9;
  outgoing_data[16] = 0x93;
  //issue the command to the gps
  HAL_I2C_Master_Transmit(&hi2c1, device_id, outgoing_data, 17, 250);

  HAL_Delay(1);
  //turn off VTG I2C
  outgoing_data[0] = 0xb5;      //preamble 1
  outgoing_data[1] = 0x62;      //preamble 2
  outgoing_data[2] = 0x06;      //config class byte
  outgoing_data[3] = 0x8a;      //config ID byte
  outgoing_data[4] = 0x09;      //length of the payload lsb
  outgoing_data[5] = 0x00;      //length of the payload msb
  outgoing_data[6] = 0x00;
  outgoing_data[7] = 0x04;
  outgoing_data[8] = 0x00;
  outgoing_data[9] = 0x00;
  outgoing_data[10] = 0xb0;
  outgoing_data[11] = 0x00;
  outgoing_data[12] = 0x91;
  outgoing_data[13] = 0x20;
  outgoing_data[14] = 0x00;
  outgoing_data[15] = 0xfe;
  outgoing_data[16] = 0xac;
  //issue the command to the gps
  HAL_I2C_Master_Transmit(&hi2c1, device_id, outgoing_data, 17, 250);

  HAL_Delay(1);
  //turn off PVT messages
  outgoing_data[0] = 0xb5;      //preamble 1
  outgoing_data[1] = 0x62;      //preamble 2
  outgoing_data[2] = 0x06;      //config class byte
  outgoing_data[3] = 0x8a;      //config ID byte
  outgoing_data[4] = 0x09;      //length of the payload lsb
  outgoing_data[5] = 0x00;      //length of the payload msb
  outgoing_data[6] = 0x00;
  outgoing_data[7] = 0x04;
  outgoing_data[8] = 0x00;
  outgoing_data[9] = 0x00;
  outgoing_data[10] = 0x06;
  outgoing_data[11] = 0x00;
  outgoing_data[12] = 0x91;
  outgoing_data[13] = 0x20;
  outgoing_data[14] = 0x00;
  outgoing_data[15] = 0x54;
  outgoing_data[16] = 0x5a;
  //issue the command to the gps
  HAL_I2C_Master_Transmit(&hi2c1, device_id, outgoing_data, 17, 250);

  return true;

}

bool set_gps_config_value(uint32_t key, uint8_t value)
{
  //populate the packet to send to the gps unit
  outgoing_data[0] = 0xb5;      //preamble 1
  outgoing_data[1] = 0x62;      //preamble 2
  outgoing_data[2] = 0x06;     //config class byte
  outgoing_data[3] = 0x8a;        //config ID byte
  outgoing_data[4] = 0x05;      //length of the payload lsb
  outgoing_data[5] = 0x00;      //length of the payload msb
  outgoing_data[6] = (key && 0xFF);
  outgoing_data[7] = (key >> 8) && 0xFF;
  outgoing_data[8] = (key >> 16) && 0xFF;
  outgoing_data[9] = (key >> 24) && 0xFF;
  outgoing_data[10] = value;

  //calculate the checksum
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  for (uint8_t i = 6; i <= 10; i++)
  {
    ck_a = ck_a + outgoing_data[i];
    ck_b = ck_b + ck_a;
  }
  outgoing_data[11] = ck_a;
  outgoing_data[12] = ck_b;

  //issue the command to the gps
  HAL_I2C_Master_Transmit(&hi2c1, device_id, outgoing_data, 13, 250);

  return true;
}
