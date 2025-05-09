#ifndef __FUSION_AXIS_H
#define __FUSION_AXIS_H

#include "main.h"
#include "TMC4361A.h"
#include <stdbool.h>
#include <stdint.h>

struct tmc4361_t* getMotionAxis(uint8_t ax_index);
void updateAxis(struct tmc4361_t * ax);
void setAxisVelocityTarget(struct tmc4361_t *ax, int32_t);
void setAxisPositionTarget(struct tmc4361_t *ax, int32_t);
void setAxisVelocityMax(struct tmc4361_t *ax, int32_t);
void setAxisRampAcceleration(struct tmc4361_t *ax, int32_t);
int32_t getAxisPositionTarget(struct tmc4361_t *ax);
int32_t getAxisVelocityTarget(struct tmc4361_t *ax);
int32_t fetchAxisVelocityActual(struct tmc4361_t *ax);
int32_t fetchAxisPositionActual(struct tmc4361_t *ax);
int32_t getAxisPositionActual(struct tmc4361_t *ax);
int32_t getAxisVelocityActual(struct tmc4361_t *ax);
void setAxisPositionActual(struct tmc4361_t* ax, int32_t p);
bool isAxisFaulted(struct tmc4361_t *ax);
void axisShadowTransfer(struct tmc4361_t *ax);
void setAxisPID(struct tmc4361_t *tmc, uint32_t p, uint32_t i, uint32_t d);

#endif
