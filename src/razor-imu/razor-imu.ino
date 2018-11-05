#include "SparkFunMPU9250-DMP\SparkFunMPU9250-DMP.h"
#include "LinkUp\LinkUpRaw.h"
#include "LinkUp\Platform.h"

#define SerialPort Serial1
#define BAUTRATE 115200

#define INTERRUPT_PIN 4
#define CAMERA_TRIGGER_PIN 10
#define LED_PIN 13

#define CAMERA_SAMPLE_RATE 10
#define TEMPERATUR_SAMPLE_RATE 400
#define CAMERA_TRIGGER_PULSE 2

#define BUFFER_SIZE 64
#define SAMPLE_RATE 200
#define GYRO_FSR 500
#define ACCEL_FSR 4
#define LFP 188

#define SEND_TIMEOUT 200*2

String readString;

MPU9250_DMP imu;
uint8_t pBuffer[BUFFER_SIZE];
LinkUpRaw linkUpConnector;
uint32_t sample_counter = 0;
uint16_t sendTimeout = 0;
uint32_t time_us;

PACK(ImuData{
	uint32_t timestamp_ms;
	uint32_t timestamp_us;
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

		/* constant bias correction */
		imu.setGyroBias(5, 67, -32);
		imu.setAccBias(6, 28, -71);
	}

	void loop()
	{
		bool bCamTrigger = false;
		if (digitalRead(INTERRUPT_PIN) == LOW)
		{
			if (sendTimeout > 0) {
				sendTimeout--;
			}
			time_us = micros();
			if (sample_counter % TEMPERATUR_SAMPLE_RATE == 0)
			{
				imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_TEMP);
				if (sendTimeout != 0)
				{
					digitalWrite(LED_PIN, !digitalRead(LED_PIN));
				}
			}
			else
			{
				imu.update(UPDATE_ACCEL | UPDATE_GYRO);
			}
			if (sample_counter % CAMERA_SAMPLE_RATE == 0)
			{
				if (sendTimeout != 0)
				{
					bCamTrigger = true;
					digitalWrite(CAMERA_TRIGGER_PIN, HIGH);
				}
			}
			if ((sample_counter - CAMERA_TRIGGER_PULSE) % CAMERA_SAMPLE_RATE == 0)
			{
				digitalWrite(CAMERA_TRIGGER_PIN, LOW);
			}
			if (sendTimeout != 0)
			{
				sendIMUData(bCamTrigger);
			}
			sample_counter++;

			if (SerialPort.available())
			{
				uint16_t nBytesRead = SerialPort.readBytes(pBuffer, BUFFER_SIZE);
				linkUpConnector.progress(pBuffer, nBytesRead);
			}
			else
			{
				while (linkUpConnector.hasNext())
				{
					LinkUpPacket packet = linkUpConnector.next();
					if (packet.nLength == 1 && packet.pData[0] == 1) {
						sendTimeout = SEND_TIMEOUT;
						free(packet.pData);
					}
				}
			}
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
		pData->timestamp_ms = imu.time;
		pData->timestamp_us = time_us;
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