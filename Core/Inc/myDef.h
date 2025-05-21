/*
 * myDef.h
 *
 *  Created on: May 1, 2025
 *      Author: mines
 */

#ifndef INC_MYDEF_H_
#define INC_MYDEF_H_

typedef struct {
	float acc_x_g;
	float acc_y_g;
	float acc_z_g;
	float acc_total_g;
	float acc_total_ms;
	float temperature_c;
} Acceleration_t;

typedef struct {
	float gyro_x_dps;
	float gyro_y_dps;
	float gyro_z_dps;
	float voltage_v;
} AngularVelocity_t;

typedef struct {
	float roll_deg;
	float pitch_deg;
	float yaw_deg;
	uint16_t version;
} Angle_t;

#define Gs 	9.80665f

typedef struct {
	float Vref;
	float temp;
	float Vtemp;
	float V5amp;
	float Vamp;
	float amp;
} ADC_val;

uint32_t slowLoop = 0;
uint32_t kHzLoop = 0;

uint32_t val_pwmRED = 100;
uint32_t val_pwmBLACK = 100;
uint32_t val_pwmOffset = 220;
uint32_t val_pwmSERVO = 1440;

const int leftPos = 1440 - 220;  // 1230
const int rightPos = 1440 + 220; // 1670

int targetPos = rightPos; // mulai sweeping ke kanan dulu
int currentPos = 1450; // mulai dari tengah
int stepSize = 2; // besar langkah per update (us)

uint8_t dmaBufSize = 11;

typedef struct __attribute__((packed)) {
	uint8_t index;
	uint8_t identifier;
	uint16_t _stearing :12;
	uint16_t _setPoint :12;
	uint16_t _propotional :12;
	uint16_t _integral :12;
	uint16_t _derivative :12; // 68 bits
	uint16_t checkSum :12;    // 80 bits with checksum 10byte
} sendPID_t;

typedef struct __attribute__((packed)) {
	uint8_t index;
	uint8_t indetifier;
	uint16_t _stearing :12;
	uint16_t _setPoint :12;
	uint8_t dummy :4;
	uint16_t checkSum :12;    // 80 bit with chech sum 10byte
} sendControl_t;

volatile typedef struct {

	float setPoint; // PID set Point

	float Kp;     // Proportional gain
	float Ki;     // Integral gain (used in PI & PID modes)
	float Kd;     // Derivative gain (used in PD & PID modes)

	float prevError;   // Previous error for derivative calculation
	float integral;    // Integral sum for integral calculation
	float lastIntegral;
	uint32_t lastTime; // Last update time (in milliseconds)

	float minOutput;   // Minimum output limit
	float maxOutput;   // Maximum output limit

	float deadband;
} PID_t;

PID_t PID = {
		.minOutput = 0.0f,
		.maxOutput = 2000.0f,
		.Kp = 0.25f };

#define brakeThrshld 1.5f
#define brakeThrshldLow 0.5f

#define rxTimeout 500
volatile int32_t rxTime = 0;
volatile uint8_t rxStatus = 0;
uint8_t captureMode = 0;
uint32_t correctData = 0;
sendPID_t receivedDataPID = { ._stearing = 2048, ._integral = 0, ._derivative =
		0, ._propotional = 0, ._setPoint = 0 };
sendControl_t receivedDataControl = { ._stearing = 2048, ._setPoint = 0,
		.dummy = 0 };
//uint8_t rx_buff_LoRa[11];
uint8_t rx_buff_LoRa[11];
uint8_t rx_DMA_buff_LoRa[6];
uint32_t retrying = 0;
uint8_t capture = 0;
uint8_t bufferFlag = 0;
uint8_t signiture1 = 0x55;
uint8_t signiture2 = 0xFF;
uint8_t currIndex = 0;
uint8_t rx_byte = 0;
uint32_t totalByte = 0;
HAL_StatusTypeDef curreErrorUARTRceive = 0;
uint8_t stopReceive = 0;
uint32_t lastControlTime = 0;
uint32_t controlInterval = 0;

uint32_t reLocation = 0;
uint8_t sendTXbuff = 1;
uint8_t tx_buffIMU[3] = { 0xFF, 0xAA, 0x67 };
uint8_t sync = 0;
uint8_t rx_main_IMU[22];
uint8_t currIndexIMU = 0;
uint8_t captureModeIMU = 0;
uint8_t mainIndexBuf = 0;
uint8_t rx_buff_IMU[15];
uint8_t rx_DMA_buff_IMU[11];
uint32_t haltCallUART3 = 0;

int16_t speedTest = 0;

Acceleration_t acc;
AngularVelocity_t gyro;
Angle_t angle;
uint32_t AccelerationCount = 0;
uint32_t AngularVelocityCount = 0;
uint32_t AngleCount = 0;

#if defined(STM32F1)
#define clrIO(PORT, PIN)   ((PORT)->BRR = (PIN))  // Use BRR for STM32F1
#else
#define clrIO(PORT, PIN)   ((PORT)->BSRR = (PIN) << 16)  // Use BSRR for others
#endif

#define setIO(PORT, PIN)       ((PORT)->BSRR = (PIN))    // Set pin high
#define readIO(PORT, PIN)      (((PORT)->IDR & (PIN)) ? 1 : 0) // Read pin state

// _________Motor Loop __________________________________________
#define mMode_Stop 0
#define mMode_For 1
#define mMode_back 2
#define mMode_breaking 5
#define mMode_deAcsFor2stop 3
#define mMode_deAcsBack2stop 4

#define motorSpeedInittials 150

#define motorSpeedMax 2000
#define motorSpeedMin -2000

int32_t Speed = 0;
uint32_t breakingPower = 600;
uint8_t zeroMode = mMode_Stop;
uint8_t motorMode = 0;

//__________ ADC DMA __________________________________________
ADC_val ADCvalue;

#define totalADC 3

#define AVG_SLOPE (4.3F)
#define V_AT_25C  (1.43F)
#define V_REF_INT (1.2F)
#define V_A (0.066F)

uint32_t valADC[totalADC] = { 0, 0, 0 };

#define ADXL345_ADDR     (0x53 << 1) // Shifted left for STM32 HAL
#define DEVID_REG        0x00
#define POWER_CTL_REG	0x2d
#define DATA_FORMAT_REG 0x31
#define BW_RATE_REG 0x2c
#define INT_SOURCE_REG 0x30
#define FIFO_CTL_REG 0x38
#define INT_ENABLE_REG 0x2E
#define DATA_REG     0x32

#define SCALE_FACTOR 0.038259f   // m/s² per LSB (for ±2g)
#define DT 0.1f                  // Sampling interval in seconds (100ms)x

typedef struct{
	int16_t acc_X;
	int16_t acc_Y;
	int16_t acc_Z;
}ADXL_t;

ADXL_t ADXL_Data;

float velocity_mps = 0.0f;       // Initial speed in m/s
float velocity_kmh = 0.0f;

uint32_t lastCNT = 0;
float rpmMotor = 0;
float rpsWheel = 0;
float mmSVelocity = 0;
float mSVelocity = 0;
float rps = 0;
float dump1 = 0;

#endif /* INC_MYDEF_H_ */
