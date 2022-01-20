#include "MPU6050.h"
#include "main.h"
// #include "arm_benchmark.h"

// Read WHO_AM_I
uint8_t MPU_Read_ID(MPU_data *data){
	
	/* ----------------
	*
	*	Check MPU ID
	*
	----------------- */

	HAL_StatusTypeDef res;
	uint8_t buf;

	buf = MPU_WHO_AM_I;

  	res = HAL_I2C_Master_Transmit(data->i2c_Handle, (uint16_t)MPU_ADDR_W, &buf, 1, HAL_MAX_DELAY); // write the address of the registers to be read

	if ( res != HAL_OK ){
		return 1;
	}
	else{
		res = HAL_I2C_Master_Receive(data->i2c_Handle, (uint16_t)MPU_ADDR_R, &buf, 1, HAL_MAX_DELAY); // read the value of regs

		if ( res != HAL_OK ){
			return 2;
		}
	}

	if ( buf != MPU_WHO_AM_I_VAL ){	// check whether the id was received correctly
		return 3;
	}
	else{
		return 0;
	}
}


void MPU_Init_Struct(MPU_data *data,
		I2C_HandleTypeDef *hi2c,
		GPIO_TypeDef *MPU_IntPinPort, uint16_t MPU_IntPin){

	/* --------
	*
	*	Reset memory junk
	*	Initialize I2C address container
	*
	-------- */

	// Interface parameters store
	data->i2c_Handle = hi2c;
	data->int_PinPort = MPU_IntPinPort;
	data->int_Pin = MPU_IntPin;

	// DMA flag clear
	data->reading_data = 0;

	// ACCEL data
	data->accel_X_raw = 0;
	data->accel_Y_raw = 0;
	data->accel_Z_raw= 0;

	data->accel_X = 0;
	data->accel_Y = 0;
	data->accel_Z = 0;

	// GYRO data
	data->gyro_X_raw = 0;
	data->gyro_Y_raw = 0;
	data->gyro_Z_raw= 0;

	data->gyro_X = 0;
	data->gyro_Y = 0;
	data->gyro_Z = 0;

	// Output data address container
	data->reg_address[0] 	= 		MPU_ACCEL_XOUT_H;
	data->reg_address[1] 	= 		MPU_ACCEL_XOUT_L;
	data->reg_address[2] 	= 		MPU_ACCEL_YOUT_H;
	data->reg_address[3] 	= 		MPU_ACCEL_YOUT_L;
	data->reg_address[4] 	= 		MPU_ACCEL_ZOUT_H;
	data->reg_address[5] 	= 		MPU_ACCEL_ZOUT_L;
	data->reg_address[6] 	= 		MPU_GYRO_XOUT_H;
	data->reg_address[7] 	= 		MPU_GYRO_XOUT_L;
	data->reg_address[8] 	= 		MPU_GYRO_YOUT_H;
	data->reg_address[9] 	= 		MPU_GYRO_YOUT_L;
	data->reg_address[10] 	= 		MPU_GYRO_ZOUT_H;
	data->reg_address[11] 	= 		MPU_GYRO_ZOUT_L;

}


uint8_t MPU_Init(MPU_data *data,
		I2C_HandleTypeDef *hi2c, GPIO_TypeDef *MPU_IntPinPort, uint16_t MPU_IntPin,
		uint8_t gyro_fs, uint8_t accel_fs, uint8_t dlpf, uint8_t smplrt){
	/* --------

	Initialize MPU settings
	Recommended for start:
		- MPU_FS_SEL_500
		- MPU_AFS_SEL_4
		- MPU_DLPF1
		- 0b1

	-------- */

	// Initialize MPU data struct
	MPU_Init_Struct(data, hi2c, MPU_IntPinPort, MPU_IntPin);

	// Perform soft reset for the chip
	// Check chip ID
	// Configure

	HAL_StatusTypeDef res;
	uint8_t buf[14];

	// Reset sequence for sensor
	// Device reset
	buf[0] = MPU_PWR_MGMT_1;
	buf[1] = MPU_DEVICE_RESET;

	res = HAL_I2C_Master_Transmit(data->i2c_Handle, (uint16_t)MPU_ADDR_W, &buf[0], 2, HAL_MAX_DELAY);

	if( res == HAL_OK ){
		for(uint32_t waste = 0; waste < 8400000; waste++){}	// Delay 0,5s to allow device restart
	}

	// Reset signal path and signal conditioning
	buf[0] = MPU_SIGNAL_PATH_RESET;
	buf[1] = MPU_GYRO_RESET | MPU_ACCEL_RESET | MPU_TEMP_RESET;
	buf[2] = MPU_USER_CTRL;
	buf[3] = MPU_SIG_COND_RESET;

	for( uint8_t i=0; i <= 1; i++ ){

		res = HAL_I2C_Master_Transmit(data->i2c_Handle, (uint16_t)MPU_ADDR_W, &buf[2*i], 2, HAL_MAX_DELAY); // Send reset data

		if( res == HAL_OK ){
			for(uint32_t waste = 0; waste < 840000; waste++){}	// Delay 0,05s to allow device restart
		}

	}

	// Set clk source to gyro reference (supposed to be better)
	// Disable temp sensor

	//Take device out of sleep
	buf[0] = MPU_PWR_MGMT_1;															// 10
	buf[1] = (MPU_CLKSEL_GYROX  | MPU_TEMP_DIS) & ((uint8_t)~(MPU_SLEEP));				// 11

	// Set gyro range
	buf[2] = MPU_GYRO_CONFIG;
	buf[3] = gyro_fs;

	// Set accel range
	buf[4] = MPU_ACCEL_CONFIG;
	buf[5] = accel_fs;

	// Set digital low pass filter
	buf[6] = MPU_CONFIG;
	buf[7] = dlpf;

	// Set Sample-rate
	buf[8] = MPU_SMPLRT_DIV;
	buf[9] = smplrt;

	// Set up the interrupt to be cleared by any read operation
	buf[10] = MPU_INT_PIN_CFG;
	buf[11] = MPU_INT_RD_CLEAR_ANY;

	// Enable the generation of interrupts
	buf[12] = MPU_INT_ENABLE;
	buf[13] = MPU_DATA_RDY_EN;

	// ----------
	// Send all the settings
	// ----------
	for( uint8_t i=0; i <= 6; i++ ){

		res = HAL_I2C_Master_Transmit(data->i2c_Handle, (uint16_t)MPU_ADDR_W, &buf[2*i], 2, HAL_MAX_DELAY); // Write config

		if ( res != HAL_OK ){
			continue;
		}
		// check if the settings are  asserted
		else{
			res = HAL_I2C_Master_Transmit(data->i2c_Handle, (uint16_t)MPU_ADDR_W, &buf[2*i], 1, HAL_MAX_DELAY);
			res = HAL_I2C_Master_Receive(data->i2c_Handle, (uint16_t)MPU_ADDR_R, &buf[2*i], 1, HAL_MAX_DELAY);
		}
	}

	// Compute accel_gain and gyro_gain with the data of FS
	uint16_t gyro_temp, accel_temp;
	gyro_temp = 250;
	accel_temp = 2;

	// Gyro scale
	if(gyro_fs == MPU_FS_SEL_500){gyro_temp = 500;}
	else if(gyro_fs == MPU_FS_SEL_1000){gyro_temp = 1000;}
	else if(gyro_fs == MPU_FS_SEL_2000){gyro_temp = 2000;}

	// Accel scale
	if(accel_fs == MPU_AFS_SEL_4){accel_temp = 4;}
	else if(accel_fs == MPU_AFS_SEL_8){accel_temp = 8;}
	else if(accel_fs == MPU_AFS_SEL_16){accel_temp = 16;}

	// Compute gyro gain
	data->gyro_gain = (float)gyro_temp / (float)32768 * (3.14159265359 / (float)180);
	// Compute accel gain
	data->accel_gain = (float)accel_temp / (float)32768;

	// Check if ID read correctly
	return MPU_Read_ID(data);

}

// Polling read
uint8_t MPU_Read_Meas(MPU_data *data){
	/*
	*
	*	Read MPU data
	*	Put data in MPU struct
	*	Polling mode
	*
	*/

	uint8_t buf[14];
	uint8_t status;

	// read accel
	status = (HAL_I2C_Master_Transmit(data->i2c_Handle, (uint16_t)MPU_ADDR_W, &data->reg_address[0], 1, HAL_MAX_DELAY) == HAL_OK); // write the address of the registers to be read
	status &= (HAL_I2C_Master_Receive(data->i2c_Handle, (uint16_t)MPU_ADDR_R, &buf[0], 14, HAL_MAX_DELAY) == HAL_OK); // read the value of regs

	data->accel_X_raw = (int16_t)(buf[0] << 8 | buf[1]);
	data->accel_Y_raw = (int16_t)(buf[2] << 8 | buf[3]);
	data->accel_Z_raw = (int16_t)(buf[4] << 8 | buf[5]);
	data->gyro_X_raw = (int16_t)(buf[8] << 8 | buf[9]);
	data->gyro_Y_raw = (int16_t)(buf[10] << 8 | buf[11]);
	data->gyro_Z_raw = (int16_t)(buf[12] << 8 | buf[13]);

	// Give return value
	return !status;

}

// SEND i2c dma addr
uint8_t MPU_SendAddr_DMA(MPU_data *data){
	/*
	*
	*	Read MPU data DMA
	*	Starting DMA by sending start ADDR
	*
	*/

	if(HAL_I2C_Master_Transmit_DMA(data->i2c_Handle, (uint16_t)MPU_ADDR_W, &data->reg_address[0], 1) == HAL_OK){
		data->reading_data = 1;
		return 0;
	}
	else{
		return 1;
	}

}

// READ i2c dma msg
uint8_t MPU_ReadData_DMA(MPU_data *data){
	/*
	*
	*	Read MPU data DMA
	*	Read data using DMA
	*	Put data in MPU struct
	*
	*	Put this function into:
	*	HAL_I2C_MasterTxCpltCallback();
	*
	*/

	if( HAL_I2C_Master_Receive_DMA(data->i2c_Handle, (uint16_t)MPU_ADDR_R, &data->raw[0], 14) == HAL_OK ){
		return 0;
	}
	else{
		return 1;
	}

}

// Read dma complete
void MPU_ReadCplt_DMA(MPU_data *data){
	/*
	*
	*	Read MPU data DMA
	*	Clear flag when data is read
	*	Save raw data
	*	Compute flow data
	*
	*	Put this function into:
	*	HAL_I2C_MasterRxCpltCallback();
	*
	*/

	// Reset I2C flag
	data->reading_data = 0;

	// Construct raw 16 bit data
	data->accel_X_raw = (int16_t)(data->raw[0] << 8 | data->raw[1]);
	data->accel_Y_raw = (int16_t)(data->raw[2] << 8 | data->raw[3]);
	data->accel_Z_raw = (int16_t)(data->raw[4] << 8 | data->raw[5]);
	data->gyro_X_raw = (int16_t)(data->raw[8] << 8 | data->raw[9]);
	data->gyro_Y_raw = (int16_t)(data->raw[10] << 8 | data->raw[11]);
	data->gyro_Z_raw = (int16_t)(data->raw[12] << 8 | data->raw[13]);

}

//
void MPU_Compute_FData(MPU_data *data){
	/*
	*
	*	Convert raw data read to float
	*
	*/

	data->accel_X = data->accel_X_raw * data->accel_gain;
	data->accel_Y = data->accel_Y_raw * data->accel_gain;
	data->accel_Z = data->accel_Z_raw * data->accel_gain;

	data->gyro_X = data->gyro_X_raw * data->gyro_gain;
	data->gyro_Y = data->gyro_Y_raw * data->gyro_gain;
	data->gyro_Z = data->gyro_Z_raw * data->gyro_gain;
}


/* --------

Code example

-------- */


//	/* USER CODE BEGIN 0 */
//	MPU_data MPU_meas;
//	uint8_t MPU_out;
//	/* USER CODE END 0 */


//	/* USER CODE BEGIN 2 */
//	__disable_irq();
//	// Initialize MPU
//	// Setup MPU [500 deg/s gyro] [4g/s accel] [dlpf 2, smplrt 2]
//	MPU_out = MPU_Init(&MPU_meas, &hi2c1, MPU_INT_Pin_GPIO_Port, MPU_INT_Pin_Pin, MPU_FS_SEL_500, MPU_AFS_SEL_4, MPU_DLPF6, 1);
//	__enable_irq();
//	/* USER CODE END 2 */


//	/* USER CODE BEGIN 4 */
//	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//		if( GPIO_Pin == MPU_INT_Pin_Pin ){
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // CPU busy pin
//	
//			MPU_SendAddr_DMA(&MPU_meas);
//	
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // CPU busy pin
//		}
//	}
//	
//	void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
//	
//		if( hi2c->Instance == I2C1 ){
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // CPU busy pin
//	
//			MPU_ReadData_DMA(&MPU_meas);
//	
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // CPU busy pin
//		}
//	}
//	
//	void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
//	
//		if( hi2c->Instance == I2C1 ){
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET); // CPU busy pin
//	
//			MPU_ReadCplt_DMA(&MPU_meas);
//	
//			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET); // CPU busy pin
//		}
//	}
//	/* USER CODE END 4 */








