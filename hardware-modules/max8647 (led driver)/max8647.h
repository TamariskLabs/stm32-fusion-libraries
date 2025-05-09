#ifndef __MAX8647_H__
#define __MAX8647_H__

#include "main.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
    GPIO_TypeDef *enablePort;
    uint16_t enablePin;
} MAX8647_HandleTypeDef;


void MAX8647_Init(MAX8647_HandleTypeDef *hdev, I2C_HandleTypeDef *hi2c);
void MAX8647_SetBrightness(MAX8647_HandleTypeDef *hdev, uint8_t channel, uint8_t level);


#endif /* __MAX8647_H__ */
