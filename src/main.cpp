#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

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

int main(int argc, char* argv[]) {
	SysTick_Init();
	Serial.begin(115200);
	Delay_ms(100);
	I2C.Init(400000);
	MPU.Init();
	Car.Init();
	EnKeyInit();
	LED_Init();

	LED_Turn(LEDColorRed);

	GetKalAngle(true);

	uint32_t EnKeyDelay = millis();
	uint32_t PosCalcDelay = millis();

	while (true) {
		SerialDataTran();
		if (EnFlag) {
			if (millis() - PosCalcDelay > 4) {
				GetKalAngle();
				PIDPos.Compute();
				Car.SetSpeed(PIDPosParam.Out);
			}
			if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET
					&& millis() - EnKeyDelay > 300) {
				EnKeyDelay = millis();
				EnFlag = !EnFlag;
				LED_Turn(LEDColorRed);
				PIDPos.Clear();
				Car.Stop();
			}
		} else {
			if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET
					&& millis() - EnKeyDelay > 300) {
				EnKeyDelay = millis();
				EnFlag = !EnFlag;
				LED_Turn(LEDColorGreen);
				GetKalAngle(true);
				Car.Run();
			}
		}
	}
}

void GetKalAngle(bool isInit) {
	static uint64_t time = micros();
	MPU.RefreshData();
	if (isInit) {
		KalmanX.setAngle(Roll);
	} else {
		PIDPosParam.Now = KalmanX.getAngle(Roll, GYROYRate,
				(double) (micros() - time) / 1000000.0);
		Serial.println(PIDPosParam.Now);
		time = micros();
	}
}

void SerialDataTran() {
	if (Serial.checkFrame()) {
		while (Serial.available() > 0) {
			char c = Serial.read();
			switch (c) {
			case 'P':
				PIDPosParam.P = Serial.nextFloat();
				PIDPos.SetTuningP(PIDPosParam.P);
				Serial.print("P:");
				Serial.print(PIDPosParam.P);
				Serial.write('\t');
				break;
			case 'I':
				PIDPosParam.I = Serial.nextFloat();
				PIDPos.SetTuningP(PIDPosParam.I);
				Serial.print("I set to:");
				Serial.print(PIDPosParam.I);
				Serial.write('\t');
				break;
			case 'D':
				PIDPosParam.D = Serial.nextFloat();
				PIDPos.SetTuningP(PIDPosParam.D);
				Serial.print("D set to:");
				Serial.print(PIDPosParam.D);
				Serial.write('\t');
				break;
			default:
				break;
			}
		}
		Serial.println();
	}
}

void EnKeyInit() {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
