#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#include "main.h"

int main(int argc, char* argv[]) {
	SysTick_Init();
	Serial.begin(115200);
	I2C.Init(400000);
	MPU.Init();
	HMC.Init();
	Car.Init(CarCountBoth);
	EnKeyInit();
	LED_Init();

	LED_Turn(LEDColorRed);

	GetKalAngle(true);
//	Calibrate();

	uint32_t EnKeyDelay = millis();
	uint32_t PosCalcDelay = millis();
	uint32_t SpeedCalcDelay = millis();
	uint32_t SpeedDelay = millis();
	uint32_t AngleRetrunDelay = millis();
	uint32_t HMCDelay = millis();

	double Last_PIDSpeedOut;
	while (true) {
		SerialDataTran();
		if (millis() - PosCalcDelay > PosCalcDelayInterval) {
			PosCalcDelay = millis();
			GetKalAngle();
			if (EnFlag) {
				PIDPos.Compute();
				Car.SetSpeed(PIDPosParam.Out + PIDSpeedParam.Out);
			}
		}
		if (millis() - SpeedCalcDelay > 200) {
			SpeedCalcDelay = millis();
			PIDSpeedParam.Now = (Car.LeftCount + Car.RightCount) / 2.0;
			Car.LeftCount = 0;
			Car.RightCount = 0;
			Last_PIDSpeedOut = PIDSpeedParam.Out;
			PIDSpeed.Compute();
			PIDSpeedParam.Out = SpeedLowParse * Last_PIDSpeedOut
					+ ((1 - SpeedLowParse) * PIDSpeedParam.Out);
		}
		if (millis() - AngleRetrunDelay > AngleReturnDelayInterval
				&& AngleReturnFlag) {
			AngleRetrunDelay = millis();
			Serial.print("NA");
			Serial.print(PIDPosParam.Now, 1);
			Serial.write('|');
		}
		if (millis() - HMCDelay > 200) {
			HMCDelay = millis();
			HMC.RefreshData();
			Serial.print("NY");
			Serial.print(HMCData.Angle, 2);
			Serial.write('|');
			Serial.print(HMCData.X);
			Serial.write('|');
			Serial.print(HMCData.Y);
			Serial.write('|');
			Serial.print(HMCData.Z);
			Serial.println();
		}
		if (millis() - SpeedDelay > SpeedDelayInterval && SpeedReturnFlag) {
			SpeedDelay = millis();
			Serial.print("NS");
			Serial.print(PIDSpeedParam.Now, 1);
			Serial.write('|');
		}

		if (EnFlag) {
			if ((GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) == Bit_RESET
					&& millis() - EnKeyDelay > EnKeyDelayInterval)
					|| (PIDPosParam.Now > 60 || PIDPosParam.Now < -60)) {
				EnKeyDelay = millis();
				EnFlag = !EnFlag;
				LED_Turn(LEDColorRed);
				PIDPos.Clear();
				Car.Stop();
			}
		} else {
			if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)
					== Bit_RESET&& millis() - EnKeyDelay > EnKeyDelayInterval) {
				EnKeyDelay = millis();
				EnFlag = !EnFlag;
				LED_Turn(LEDColorGreen);
				GetKalAngle(true);
				Car.Run();
			}
		}

	}

}

void Calibrate() {
	int Xmax = 0, Xmin = 0, Ymax = 0, Ymin = 0, Zmax = 0, Zmin = 0;
	for (int i = 0; i < 200; i++) {
		HMC.RefreshData();
		if (Xmin > HMCData.X)
			Xmin = HMCData.X;
		if (Ymin > HMCData.Y)
			Ymin = HMCData.Y;
		if (Zmin > HMCData.Z)
			Zmin = HMCData.Z;
		if (Xmax < HMCData.X)
			Xmax = HMCData.X;
		if (Ymax < HMCData.Y)
			Ymax = HMCData.Y;
		if (Zmax < HMCData.Z)
			Zmax = HMCData.Z;
		Serial.print("Now:");
		Serial.print(HMCData.X);
		Serial.print("   ");
		Serial.print(HMCData.Y);
		Serial.print("   ");
		Serial.print(HMCData.Z);
		Serial.println();
		Delay_ms(100);
	}
	Serial.print("Off:");
	Serial.print(Xmax);
	Serial.print("   ");
	Serial.print(Xmin);
	Serial.print("   ");
	Serial.print(Ymax);
	Serial.print("   ");
	Serial.print(Ymin);
	Serial.print("   ");
	Serial.print(Zmax);
	Serial.print("   ");
	Serial.print(Zmin);
	Serial.println();
	HMC.Xoff = (Xmax + Xmin) / 2;
	HMC.Yoff = (Ymax + Ymin) / 2;
	HMC.Zoff = (Zmax + Zmin) / 2;
	Serial.print("Off:");
	Serial.print(HMC.Xoff);
	Serial.print("   ");
	Serial.print(HMC.Yoff);
	Serial.print("   ");
	Serial.print(HMC.Zoff);
	Serial.println();
}

void GetKalAngle(bool isInit) {
	static uint64_t time = micros();
	MPU.RefreshData();
	if (isInit) {
		KalmanX.setAngle(Roll);
	} else {
		PIDPosParam.Now = KalmanX.getAngle(Roll, GYROYRate,
				(double) (micros() - time) / 1000000.0);
		time = micros();
	}
}

void SerialDataTran() {
	if (Serial.checkFrame()) {
		char c;
		while ((c = Serial.read()) != -1) {
			switch (c) {
			case 'K':
				c = Serial.read();
				switch (c) {
				case 'p':
					PIDPosParam.P = Serial.nextFloat();
					PIDPos.SetTuningP(PIDPosParam.P);
					Serial.print("Kp");
					Serial.print(PIDPosParam.P, 3);
					Serial.write('|');
					break;
				case 'i':
					PIDPosParam.I = Serial.nextFloat();
					PIDPos.SetTuningI(PIDPosParam.I);
					PIDPos.Clear();
					Serial.print("Ki");
					Serial.print(PIDPosParam.I, 3);
					Serial.write('|');
					break;
				case 'd':
					PIDPosParam.D = Serial.nextFloat();
					PIDPos.SetTuningD(PIDPosParam.D);
					Serial.print("Kd");
					Serial.print(PIDPosParam.D, 1);
					Serial.write('|');
					break;
				case 's':
					PIDPosParam.Set = Serial.nextFloat();
					Serial.print("Ks");
					Serial.print(PIDPosParam.Set, 1);
					Serial.write('|');
					break;
				}
				break;
			case 'S':
				c = Serial.read();
				switch (c) {
				case 'p':
					PIDSpeedParam.P = Serial.nextFloat();
					PIDSpeed.SetTuningP(PIDSpeedParam.P);
					Serial.print("Sp");
					Serial.print(PIDSpeedParam.P, 3);
					Serial.write('|');
					break;
				case 'i':
					PIDSpeedParam.I = Serial.nextFloat();
					PIDSpeed.SetTuningI(PIDSpeedParam.I);
					PIDSpeed.Clear();
					Serial.print("Si");
					Serial.print(PIDSpeedParam.I, 3);
					Serial.write('|');
					break;
				case 'd':
					PIDSpeedParam.D = Serial.nextFloat();
					PIDSpeed.SetTuningD(PIDSpeedParam.D);
					Serial.print("Sd");
					Serial.print(PIDSpeedParam.D, 1);
					Serial.write('|');
					break;
				case 's':
					PIDSpeedParam.Set = Serial.nextFloat();
					Serial.print("Ss");
					Serial.print(PIDSpeedParam.Set, 1);
					Serial.write('|');
					break;
				}
				break;

			case 'C':
				c = Serial.read();
				if (c == 'A') {
					c = Serial.read();
					if (c == '0')
						AngleReturnFlag = false;
					else if (c == '1')
						AngleReturnFlag = true;
				} else if (c == 'S') {
					c = Serial.read();
					if (c == '0')
						SpeedReturnFlag = false;
					else if (c == '1')
						SpeedReturnFlag = true;
				}
				break;
			case 'R':
				LED_Turn(LEDColorGreen);
				GetKalAngle(true);
				Car.Run();
				EnFlag = true;
				break;
			default:
				break;
			}
		}
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
