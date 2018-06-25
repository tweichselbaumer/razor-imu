#include <SparkFunMPU9250-DMP.h>
#include "LinkUp\LinkUpRaw.h"
#include "LinkUp\Platform.h"

#define SerialPort SerialUSB
#define BAUTRATE 921600

#define INTERRUPT_PIN 4

#define BUFFER_SIZE 64
#define SAMPLE_RATE 200
#define GYRO_FSR 2000
#define ACCEL_FSR 16
#define LFP 188

int count = 0;

MPU9250_DMP imu;
uint8_t pBuffer[BUFFER_SIZE];
LinkUpRaw linkUpConnector;
uint8_t temperature_update = 1;

PACK(ImuData{
	uint32_t timestamp;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t temperature;
	};)

	void setup()
	{
		pinMode(INTERRUPT_PIN, INPUT_PULLUP);

		SerialPort.begin(BAUTRATE);
		SerialPort.setTimeout(1);

		imu.begin();
		imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
		imu.setSampleRate(SAMPLE_RATE);
		imu.setGyroFSR(GYRO_FSR);
		imu.setAccelFSR(ACCEL_FSR);
		imu.setLPF(LFP);
		imu.enableInterrupt();
		imu.setIntLevel(INT_ACTIVE_LOW);
		imu.setIntLatched(INT_LATCHED);
	}

	void loop()
	{
		if (digitalRead(INTERRUPT_PIN) == LOW)
		{
			if (temperature_update > SAMPLE_RATE) {
				imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_TEMP);
				temperature_update = 1;
			}
			else {
				imu.update(UPDATE_ACCEL | UPDATE_GYRO);
				temperature_update++;
			}
			sendIMUData();
		}
		else
		{
			uint32_t nBytesToSend;
			nBytesToSend = linkUpConnector.getRaw(pBuffer, BUFFER_SIZE);

			if (nBytesToSend > 0)
			{
				SerialPort.write(pBuffer, nBytesToSend);
			}
		}
	}

	void sendIMUData(void)
	{
		LinkUpPacket packet;
		packet.pData = (uint8_t*)calloc(1, sizeof(ImuData));

		ImuData *pData = (ImuData*)packet.pData;
		pData->timestamp = imu.time;
		pData->ax = imu.ax;
		pData->ay = imu.ay;
		pData->az = imu.az;
		pData->gx = imu.gx;
		pData->gy = imu.gy;
		pData->gz = imu.gz;
		pData->temperature = imu.temperature;
		packet.nLength = sizeof(ImuData);
		linkUpConnector.send(packet);
	}