/*
 * main.h
 *
 *  Created on: 2016Äê10ÔÂ11ÈÕ
 *      Author: Romeli
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "HardwareSerial.h"
#include "I2C.h"
#include "SPI.h"
#include "Delay.h"
#include "PID.h"
#include "LED.h"
#include "MPU.h"
#include "Kalman.h"
#include "Car.h"

#include "math.h"

#define RAD_TO_DEG 57.295779513082320876798154814105f
#define Roll atan2(MPUData.ACC.Y, MPUData.ACC.Z) * RAD_TO_DEG
#define GYROYRate MPUData.GYRO.X / 131.0

#define PIDPosOutMimLimit -1.0
#define PIDPosOutMaxLimit 1.0

typedef struct {
	double P;
	double I;
	double D;
	double Set;
	double Now;
	double Out;
} PIDParam;

void EnKeyInit();
void GetKalAngle(bool isInit = false);
void SerialDataTran();

Kalman KalmanX;

PIDParam PIDPosParam = { 0.04, 0.12, 20, 0, 0, 0 };
PIDClass PIDPos = PIDClass(PIDPosParam.P, PIDPosParam.I, PIDPosParam.D,
		&PIDPosParam.Set, &PIDPosParam.Now, &PIDPosParam.Out, PIDPosOutMimLimit,
		PIDPosOutMaxLimit, PIDPostionPos, PIDMode_Post);
;

bool EnFlag = false;

#endif /* MAIN_H_ */
