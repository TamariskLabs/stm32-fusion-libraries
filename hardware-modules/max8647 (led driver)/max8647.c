#include "max8647.h"


// devivce i2c address
#define MAX8647_I2C_ADDR 0x9A


// bitmask used to select which led channel is being addressed
#define MAX8647_LED_1 0x20
#define MAX8647_LED_2 0x40
#define MAX8647_LED_3 0x60
#define MAX8647_LED_4 0x80
#define MAX8647_LED_5 0xA0
#define MAX8647_LED_6 0xC0


// array used to map between channel integer and channel bitmask
static uint8_t led_channel_map[6] = {MAX8647_LED_1, MAX8647_LED_2, MAX8647_LED_3, MAX8647_LED_4, MAX8647_LED_5, MAX8647_LED_6};


/**
 * @brief Sets the LED brightness for each channel.  LED brightness range is from 0 to 32.
 * @param *hdev The led driver device handler.
 * @param channel The led channel to set the power level on. There are 6 LED's indexed 1 to 6.
 * @param brightness_level The brightness to set the LED between 0 and 32.
 */
void MAX8647_SetBrightness(MAX8647_HandleTypeDef *hdev, uint8_t channel, uint8_t level) {
    // cap the value to the max brightness level of 32 (0x20)
    if (level > 0x20) level = 0x20;

    // combine the channel bitmask and the brightness level to create the control byte to send
    uint8_t control_byte = led_channel_map[channel-1] & level;

    // send the brightness command
    HAL_I2C_Master_Transmit(hdev->hi2c, MAX8647_I2C_ADDR, &control_byte, 1, 100);
}
