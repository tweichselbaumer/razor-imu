#include <SparkFunMPU9250-DMP.h>
#include "LinkUp\LinkUpRaw.h"
#include "LinkUp\Platform.h"

#define SerialPort SerialUSB
#define BAUTRATE 921600

#define INTERRUPT_PIN 4
#define CAMERA_TRIGGER_PIN 10
#define LED_PIN 13

#define CAMERA_SAMPLE_RATE 6
#define TEMPERATUR_SAMPLE_RATE 40
#define CAMERA_TRIGGER_PULSE 1

#define BUFFER_SIZE 64
#define SAMPLE_RATE 200
#define GYRO_FSR 2000
#define ACCEL_FSR 16
#define LFP 188

MPU9250_DMP imu;
uint8_t pBuffer[BUFFER_SIZE];
LinkUpRaw linkUpConnector;
uint32_t sample_counter = 0;

PACK(ImuData{
	uint32_t timestamp;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t temperature;
	bool cam;
	};)

	void setup()
	{
		pinMode(INTERRUPT_PIN, INPUT_PULLUP);
		pinMode(LED_PIN, OUTPUT);
		pinMode(CAMERA_TRIGGER_PIN, OUTPUT);

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
		bool bCamTrigger = false;
		if (digitalRead(INTERRUPT_PIN) == LOW)
		{
			if (sample_counter % TEMPERATUR_SAMPLE_RATE == 0)
			{
				imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_TEMP);
				digitalWrite(LED_PIN, !digitalRead(LED_PIN));
			}
			else
			{
				imu.update(UPDATE_ACCEL | UPDATE_GYRO);
			}
			if (sample_counter % CAMERA_SAMPLE_RATE == 0)
			{
				bCamTrigger = true;
				digitalWrite(CAMERA_TRIGGER_PIN, HIGH);
			}
			if (sample_counter % (CAMERA_SAMPLE_RATE + CAMERA_TRIGGER_PULSE) == 0)
			{
				digitalWrite(CAMERA_TRIGGER_PIN, LOW);
			}
			sendIMUData(bCamTrigger);
			sample_counter++;
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

	void sendIMUData(bool bCamTrigger)
	{
		LinkUpPacket packet;
		packet.pData = (uint8_t*)calloc(1, sizeof(ImuData));

		ImuData *pData = (ImuData*)packet.pData;
		pData->timestamp = sample_counter;
		pData->ax = imu.ax;
		pData->ay = imu.ay;
		pData->az = imu.az;
		pData->gx = imu.gx;
		pData->gy = imu.gy;
		pData->gz = imu.gz;
		pData->cam = bCamTrigger;
		pData->temperature = imu.temperature;
		packet.nLength = sizeof(ImuData);
		linkUpConnector.send(packet);
	}