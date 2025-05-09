#include "mosfet_haptic.h"


/**
 * @brief Call in main loop frequently to service haptic related tasks.
 * @param haptic_handler reference to haptic device handler.
 */
void haptic_update(Haptic_TypeDef *haptic_handler)
{
	HAL_Delay(1);
}


/**
 * @brief sets the haptic device power level.
 * @param haptic_handler reference to haptic device handler.
 * @param level The haptic intensity as a percentage from 0 to 100.
 */
void haptic_set_level(Haptic_TypeDef *haptic_handler, uint8_t level)
{
	HAL_GPIO_WritePin(haptic_handler->GPIOPort, haptic_handler->GPIOPin, level);
}


/**
 * @brief Configures settings for the on/off haptic pattern feature.  Call haptic_start_pattern to start the pattern.
 * @param haptic_handler reference to haptic device handler.
 * @param level The haptic intensity as a percentage from 0 to 100.
 * @param on_duration The duration for the on time of the on/off pattern in units of milliseconds.
 * @param off_duration The duration for the off time of the on/off pattern in units of milliseconds.
 */
void haptic_set_pattern(Haptic_TypeDef *haptic_handler, uint8_t level, uint32_t on_duration, uint32_t off_duration)
{
	HAL_Delay(1);
}


/**
 * @brief Starts the haptic device in an on/off pattern.
 * @param haptic_handler reference to haptic device handler.
 * @param starts the
 */
void haptic_start_pattern(Haptic_TypeDef *haptic_handler)
{
	HAL_Delay(1);
}


/**
 * @brief Stops the haptic device from an on/off pattern.
 * @param haptic_handler reference to haptic device handler.
 * @param brightness_level The brightness to set the LED to as a percentage from 0 to 100.
 */
void haptic_stop_pattern(Haptic_TypeDef *haptic_handler)
{
	HAL_Delay(1);
}
