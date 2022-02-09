#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

/* ----------------
*
*	Register map - address defines
*
 ----------------- */

// I2C Addresses
#define MPU_ADDR_R										0x68 << 1 | 1
#define MPU_ADDR_W										0x68 << 1

// Self-test
#define MPU_SELF_TEST_X									0x0D
#define MPU_SELF_TEST_Y									0x0E
#define MPU_SELF_TEST_Z									0x0F
#define MPU_SELF_TEST_A									0x10

// Sample rate divider
#define MPU_SMPLRT_DIV									0x19

// Configuration
#define MPU_CONFIG										0x1A
#define MPU_GYRO_CONFIG									0x1B
#define MPU_ACCEL_CONFIG								0x1C

// Slave regs. 0x23 - 0x36

// INT config
#define MPU_INT_PIN_CFG									0x37
#define MPU_INT_ENABLE									0x38
#define MPU_INT_STATUS									0x3A

// ACCEL OUT
#define MPU_ACCEL_XOUT_H								0x3B
#define MPU_ACCEL_XOUT_L								0x3C

#define MPU_ACCEL_YOUT_H								0x3D
#define MPU_ACCEL_YOUT_L								0x3E

#define MPU_ACCEL_ZOUT_H								0x3F
#define MPU_ACCEL_ZOUT_L								0x40


// TEMP OUT
#define MPU_TEMP_OUT_H									0x41
#define MPU_TEMP_OUT_L									0x42

// GYRO OUT
#define MPU_GYRO_XOUT_H									0x43
#define MPU_GYRO_XOUT_L									0x44

#define MPU_GYRO_YOUT_H									0x45
#define MPU_GYRO_YOUT_L									0x46

#define MPU_GYRO_ZOUT_H									0x47
#define MPU_GYRO_ZOUT_L									0x48

// SLV DATA 0x49 - 0x67

// MISC
#define MPU_SIGNAL_PATH_RESET							0x68
#define MPU_USER_CTRL									0x6A
#define MPU_PWR_MGMT_1									0x6B	// RES to 0x40
#define MPU_PWR_MGMT_2									0x6C
#define MPU_FIFO_COUNTH									0x72
#define MPU_FIFO_COUNTL									0x73
#define MPU_FIFO_R_W									0x74
#define MPU_WHO_AM_I									0x75	// RES to 0x68
#define MPU_WHO_AM_I_VAL								0x68



/* ----------------
*
*	Settings defines
*
*   Only non-default settings are defined
*
 ----------------- */
 
// REGS 0-2: 		Self-test gyro values
//			 		Test value for user performed self-test


// -----
// REGS 13-15:		Self-test accel values
//			 		Test value for user performed self-test

 
// -----
// REG  25:			Sample rate divider
// 					Has effect only with FCHOICE_BYP default 0b00
//					Divides the internal sample rate Fs
#define MPU_SAMPLE_RATE_MAX								0x00
#define MPU_SAMPLE_RATE_MIN								0xFF


// -----
// REG  26:			Configuration
// 					

// DLPF_CFG - digital low-pass filter
#define MPU_DLPF1										0x01
#define MPU_DLPF2										0x02
#define MPU_DLPF3										0x03
#define MPU_DLPF4										0x04
#define MPU_DLPF5										0x05
#define MPU_DLPF6										0x06
// 1 is 1 kHz sampling and 188 Hz BW
// 2 is 1 kHz sampling and 98 Hz BW

// EXT_SYNC_SET - external sync -> leave


// -----
// REG  27:			GYRO Configuration
// 

// FS_SEL - full-scale range select
// default 0 : 250deg/s
#define MPU_FS_SEL_500									0x01 << 3
#define MPU_FS_SEL_1000									0x02 << 3
#define MPU_FS_SEL_2000									0x03 << 3

// Bits 5:7 make a gyro selftest


// -----
// REG  28:			ACCEL Configuration
// 

// AFS_SEL - full-scale range select
// default 0 : 2g
#define MPU_AFS_SEL_4									0x01 << 3
#define MPU_AFS_SEL_8									0x02 << 3
#define MPU_AFS_SEL_16									0x03 << 3

// Bits 5:7 make a accel selftest


// -----
// REG  35:			FIFO Enable
// 

// Likely not needed



// -----
// REG  36-54:		I2C Master configuration
// 

// Skipped

// -----
// REG  55:			INT Pin / Bypass Enable Configuration
// 

#define MPU_INT_LOW										0x01 << 7
#define MPU_INT_OPEN									0x01 << 6
#define MPU_LATCH_INT_EN								0x01 << 5
#define MPU_INT_RD_CLEAR_ANY							0x01 << 4
#define MPU_FSYNC_INT_LQW								0x01 << 3
#define MPU_FSYNC_INT_EN								0x01 << 2


// -----
// REG  56:			Interrupt Enable
// 

#define MPU_FIFO_OFLOW_EN								0x01 << 4
#define MPU_I2C_MST_INT_EN								0x01 << 3
#define MPU_DATA_RDY_EN									0x01


// -----
// REG  58:			Interrupt Status
// 

#define MPU_FIFO_OFLOW_INT								0x01 << 4
#define MPU_I2C_MST_INT									0x01 << 3
#define MPU_DATA_RDY_INT								0x01


// -----
// REG  59 - 64:	Accelerometer Measurements
// 


// -----
// REG  65 - 66:	Temp Measurements
// 


// -----
// REG  67 - 72:	Gyroscope Measurements
// 


// -----
// REG  73 - 96:	External Sensor Data
// 


// -----
// REG  99 - 103:	I2C Master
// 


// -----
// REG  104:		Signal Path Reset
// 

#define MPU_GYRO_RESET									0x01 << 2
#define MPU_ACCEL_RESET									0x01 << 1
#define MPU_TEMP_RESET									0x01


// -----
// REG  106:		User Control
// 

#define MPU_FIFO_EN										0x01 << 6
#define MPU_I2C_MST_EN									0x01 << 5
#define MPU_FIFO_RESET									0x01 << 2
#define MPU_I2C_MST_RESET								0x01 << 1
#define MPU_SIG_COND_RESET								0x01 << 0


// -----
// REG  107:		Power Management 1
// 

#define MPU_DEVICE_RESET								0x01 << 7
#define MPU_SLEEP										0x01 << 6
#define MPU_CYCLE										0x01 << 5
#define MPU_TEMP_DIS									0x01 << 3
#define MPU_TEMP_DIS									0x01 << 3
#define MPU_CLKSEL_GYROX								0x01


// -----
// REG  108:		Power Management 2
// 

#define MPU_LP_WAKE_CTRL_5								0x01 << 6
#define MPU_LP_WAKE_CTRL_20								0x02 << 6
#define MPU_LP_WAKE_CTRL_40								0x03 << 6
#define MPU_STBY_XA										0x01 << 5
#define MPU_STBY_YA										0x01 << 4
#define MPU_STBY_ZA										0x01 << 3
#define MPU_STBY_XG										0x01 << 2
#define MPU_STBY_YG										0x01 << 1
#define MPU_STBY_ZG										0x01


/* ----------------
*
*	Typedefs
*
 ----------------- */

typedef struct MPU_data{
		// Raw accel data
	int16_t accel_X_raw;
	int16_t accel_Y_raw;
	int16_t accel_Z_raw;
		// Computed accel data
	float accel_X;
	float accel_Y;
	float accel_Z;
		// Raw gyro data
	int16_t gyro_X_raw;
	int16_t gyro_Y_raw;
	int16_t gyro_Z_raw;
		// Computed gyro data
	float gyro_X;
	float gyro_Y;
	float gyro_Z;
		// Internal reg addrs of MPU
	uint8_t reg_address[12];
		// Raw data read
	uint8_t raw[14];
		// Interface data
	I2C_HandleTypeDef *i2c_Handle;
	GPIO_TypeDef *int_PinPort;
	uint16_t int_Pin;
		// I2C flags
	uint8_t reading_data;
	uint8_t data_rdy;
		// Accel / Gyro gains for float vars
	float accel_gain;
	float gyro_gain;
} MPU_data;


typedef struct MPU_PI_cont{
	float Kp;
	float Ki;
	float ref;
	float feedback;
	float err;
	float err_z;
	float output;
	float output_p;
	float output_i;
	float output_iz;
} MPU_PI_cont;





/* ----------------
*
*	Function prototypes
*
 ----------------- */

uint8_t MPU_Read_ID(MPU_data *data);
void MPU_Init_Struct(MPU_data *data, I2C_HandleTypeDef *hi2c, GPIO_TypeDef *MPU_IntPinPort, uint16_t MPU_IntPin);
uint8_t MPU_Init(MPU_data *data, I2C_HandleTypeDef *hi2c, GPIO_TypeDef *MPU_IntPinPort, uint16_t MPU_IntPin, uint8_t gyro_fs, uint8_t accel_fs, uint8_t dlpf, uint8_t smplrt);
uint8_t MPU_Read_Meas(MPU_data *data);
uint8_t MPU_SendAddr_DMA(MPU_data *data);
uint8_t MPU_ReadData_DMA(MPU_data *data);
void MPU_ReadCplt_DMA(MPU_data *data);
void MPU_Raw_Construct(MPU_data *data);
void MPU_Compute_FData(MPU_data *data);



#endif
