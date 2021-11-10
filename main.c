 /******************************************************************************

* @file           : main.c

* @brief          : Main program body

* (c) EE2028 Teaching Team

******************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "main.h"

#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_accelero.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_psensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_magneto.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_gyro.h"
#include "../../Drivers/BSP/B-L475E-IOT01/stm32l475e_iot01_hsensor.h"

#include "stdio.h"
#include "string.h"
#include "math.h"

#define NORMAL 0
#define INTENSIVE 1
#define GYRO_THRESHOLD 20.0
#define TEMP_THRESHOLD 38
#define MAG_THRESHOLD 0.4
#define HUM_THRESHOLD 30.0
#define PRES_THRESHOLD 110000.0
#define WARNING 1
#define SAFE 0

static void UART1_Init(void);
UART_HandleTypeDef huart1;
static void MX_GPIO_Init(void);
void accelero_interrupt_config(void);
void mode_switch(void);
void reset(void);

int startTime=0, mode = NORMAL, i=0;
int time_one = 0;
int time_ten = 0;
int time_two = 0;
volatile short int accflag, gyroflag, pressureflag, tempflag, magflag, humflag;
char nmode[] = "Entering Normal Mode\n\r";

char message_print[50];

HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BUTTON_EXTI13_Pin)
		{   if(HAL_GPIO_ReadPin(GPIOB, LED2_Pin)==1){
			        i++;
				    if(i==1){
				    	startTime= HAL_GetTick();
				    }
				    if(i==2){
				    	if((HAL_GetTick()-startTime)<1000){
				    		mode_switch();
				    		i=0;
				    	} else{
				    		i=0;
				    	}
				    }
			}
		}

	if (GPIO_Pin == LSM6DSL_INT1_EXTI11_Pin){
		        uint8_t temp;
				temp = SENSOR_IO_Read(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW+1, LSM6DSL_ACC_GYRO_WAKE_UP_SRC);
				temp &= 0x20; //read bit[5] to determine if FF flag was raised by device
				if (temp){
					accflag = WARNING;
					sprintf(message_print, "\r\nFall detected\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
				}
	}
}


int main (void) {

   //initialise_monitor_handles(); // for semi-hosting support (printf)
   	float accel_data[3], gyro_data[3], magneto_data[3];
   	float temp_data, hum_data;
   	int16_t accel_data_i16[3] = { 0 };	// array to store the x, y and z readings.
   	float gyro_data_i16[3] = { 0 };	//array to store gyro ODR data
   	float gyro_total;
   	float pres_data;
   	int16_t magneto_data_i16[3] = { 0 };

/* Reset of all peripherals */
   HAL_Init();
   MX_GPIO_Init();
/* Peripheral initializations using BSP functions */
   BSP_ACCELERO_Init();
   BSP_TSENSOR_Init();
   BSP_GYRO_Init();
   BSP_HSENSOR_Init();
   BSP_MAGNETO_Init();
   BSP_PSENSOR_Init();
   UART1_Init();
/* sensor interrupt configuration*/
   accelero_interrupt_config();
/*enable NVIC EXTI interrupt*/
   HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

   int count = 1;
   //printf("Entering Normal Mode.\n");
   sprintf(message_print, "\r\nEntering Normal Mode.\r\n");
   HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
   reset();
   while (1) {
	  if (mode == NORMAL){
		  if(HAL_GetTick()-time_one > 1000){
			  float temp_data;
			  // read temperature sensor
			  temp_data = BSP_TSENSOR_ReadTemp();
			  BSP_ACCELERO_AccGetXYZ(accel_data_i16);
			  //humi data
			  hum_data = BSP_HSENSOR_ReadHumidity();

			  //pres data
			  pres_data = BSP_PSENSOR_ReadPressure();

			  //accel data
			  accel_data[0] = (float)accel_data_i16[0] / 100.0f;
			  accel_data[1] = (float)accel_data_i16[1] / 100.0f;
			  accel_data[2] = (float)accel_data_i16[2] / 100.0f;
			  //capture_time update
			  time_one = HAL_GetTick();
		  }
		  if(HAL_GetTick()-time_two > 200){
			  if (tempflag == WARNING || accflag == WARNING){
			  		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
			  		time_two = HAL_GetTick();
			  	}
		  }
		  //raise flag for temperature warning
		  if (temp_data >= TEMP_THRESHOLD && tempflag != WARNING){
		  	  tempflag = WARNING;
		  	  sprintf(message_print, "\r\nFever is detected\r\n");
		  	  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
		  }
		  hum_data = BSP_HSENSOR_ReadHumidity();
		  temp_data = BSP_TSENSOR_ReadTemp();
		  if (pres_data >= PRES_THRESHOLD || hum_data <= HUM_THRESHOLD){
			  if (pres_data >= PRES_THRESHOLD){
           sprintf(message_print, "\r\nPressure!\r\n");
			  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			  } else{
           sprintf(message_print, "\r\nHumidity!\r\n");
			  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			  }
			  sprintf(message_print, "\r\nCheck patient's breath!\r\n");
			  HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			  mode_switch();
		  }

		  if (HAL_GetTick()-time_ten > 10000){
		  	  	if (tempflag == WARNING){
		  	  		sprintf(message_print, "Fever is detected\r\n");
		  	  		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
		  	  	  }
		  	  	if (accflag == WARNING){
		  	  		sprintf(message_print, "Fall detected\r\n");
		  	  		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
		  	  	  }
		  	    time_ten = HAL_GetTick();
		  }
	  }




	  if (mode == INTENSIVE){
		        pressureflag = WARNING;
		        humflag = WARNING;

		        if(HAL_GetTick()-time_one > 1000){
	  				//Accelerometer
	  				BSP_ACCELERO_AccGetXYZ(accel_data_i16);		// read accelerometer
	  				// the function above returns 16 bit integers which are 100 * acceleration_in_m/s2. Converting to float to print the actual acceleration.
	  				accel_data[0] = ((float)accel_data_i16[0] / 100.0f) / 9.8f;
	  				accel_data[1] = ((float)accel_data_i16[1] / 100.0f) / 9.8f;
	  				accel_data[2] = ((float)accel_data_i16[2] / 100.0f) / 9.8f;

	  				BSP_MAGNETO_GetXYZ(magneto_data_i16);
	  				magneto_data[0] = (float)magneto_data_i16[0] / 1000.0f;
	  				magneto_data[1] = (float)magneto_data_i16[1] / 1000.0f;
	  				magneto_data[2] = (float)magneto_data_i16[2] / 1000.0f;
	  				//temperature and humidity
	  				temp_data = BSP_TSENSOR_ReadTemp();			// read temperature sensor
	  				hum_data = BSP_HSENSOR_ReadHumidity();		//read humidity sensor

	  				//gyroscope
	  				BSP_GYRO_GetXYZ(gyro_data_i16);		//read gyro
	  				//convert gyro data into meaningful value in terms of dps;
	  				gyro_data[0] = (gyro_data_i16[0] + 630.0f) / 1000.0f;
	  				gyro_data[1] = (gyro_data_i16[1] + 280.0f) / 1000.0f;
	  				gyro_data[2] = (gyro_data_i16[2] + 140.0f) / 1000.0f;
	  				//get root mean square value
	  				gyro_total = sqrt(pow(gyro_data[0],2)+pow(gyro_data[1],2)+pow(gyro_data[2],2));

	  				//pressure
	  				pres_data = BSP_PSENSOR_ReadPressure()*100.0f;	//pressure in pascal
	  				time_one = HAL_GetTick();
	  			}

	  			//raising flags for warnings (done through INT)
	  				if((magneto_data[0] >= MAG_THRESHOLD || magneto_data[1] >= MAG_THRESHOLD || magneto_data[2] >= MAG_THRESHOLD )&& magflag != WARNING){
	  					magflag = WARNING;
	  					sprintf(message_print, "\r\nCheck patient's abnormal orientation!\r\n");
	  					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	  				}
	  				if ((temp_data >= TEMP_THRESHOLD)&& tempflag != WARNING){
	  					tempflag = WARNING;
	  					sprintf(message_print, "\r\nFever is detected\r\n");
	  					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	  				}
	  				if ((hum_data <= HUM_THRESHOLD || pres_data>=PRES_THRESHOLD)&&(humflag != WARNING)){
	  					humflag = WARNING;
	  					sprintf(message_print, "\r\nCheck patient's breath!\r\n");
	  					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	  				}
	  				if (gyro_total >= GYRO_THRESHOLD && gyroflag != WARNING){
	  					gyroflag = WARNING;
	  					sprintf(message_print, "Patient in pain!\r\n");
	  					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	  				}



	  			if (HAL_GetTick()-time_ten > 10000){
	  				//readings transmission
	  				sprintf(message_print, "%03d TEMP %0.2f (DegreeC) ACC %0.2f %0.2f %0.2f (g)\r\n", count, temp_data, accel_data[0], accel_data[1], accel_data[2]);
	  				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	  				sprintf(message_print, "%03d GYRO %0.1f (dps) MAGNETO %0.2f %0.2f %0.2f (Gauss)\r\n", count, gyro_total, magneto_data[0], magneto_data[1], magneto_data[2]);
	  				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	  				sprintf(message_print, "%03d HUMIDITY %0.2f (%rH)and PRESSURE %0.2f\r\n(pascal)",count,  hum_data, pres_data);
	  				HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	  				//warnings transmission
	  				count++;
	  				if (tempflag == WARNING){
	  					sprintf(message_print, "Fever is detected\r\n");
	  					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	  				}
	  				if (accflag == WARNING){
	  					sprintf(message_print, "Fall detected\r\n");
	  					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	  				}
	  				if (gyroflag == WARNING){
	  					sprintf(message_print, "Patient in pain!\r\n");
	  					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	  				}
	  				if (pressureflag == WARNING || humflag == WARNING){
	  					sprintf(message_print, "Check patient's breath!\r\n");
	  					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	  				}
	  				if (magflag == WARNING){
	  					sprintf(message_print, "Check patient's abnormal orientation!\r\n");
	  					HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
	  				}
	  				time_ten = HAL_GetTick();
	  			}
	  		}
   }
}


static void MX_GPIO_Init(void){

	__HAL_RCC_GPIOB_CLK_ENABLE();	// Enable AHB2 Bus for GPIOB
	__HAL_RCC_GPIOC_CLK_ENABLE();	// Enable AHB2 Bus for GPIOC
	__HAL_RCC_GPIOD_CLK_ENABLE();		//for

	HAL_GPIO_WritePin(GPIOB, LED2_Pin, 0); // Reset the LED2_Pin as 0

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitTypeDef GPIO_PushButton = {0};
	GPIO_InitTypeDef GPIO_lsm6dsl = {0};
	GPIO_InitTypeDef GPIO_lps22hb = {0};

	// Configuration of LED2_Pin (GPIO-B Pin-14) as GPIO output
	GPIO_InitStruct.Pin = LED2_Pin;1.
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Configuration of BUTTON_EXTI13_Pin (GPIO-C Pin-13) as AF,
	GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	//configure GPIO Pin lsm6dsl
	  GPIO_lsm6dsl.Pin = LSM6DSL_INT1_EXTI11_Pin;
	  GPIO_lsm6dsl.Mode = GPIO_MODE_IT_RISING;
	  GPIO_lsm6dsl.Pull = GPIO_PULLDOWN;
	  GPIO_lsm6dsl.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOD, &GPIO_lsm6dsl);
}

static void UART1_Init(void) {
/* Pin configuration for UART. BSP_COM_Init() can do this automatically */
__HAL_RCC_GPIOB_CLK_ENABLE();
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
/* Configuring UART1 */
huart1.Instance = USART1;
huart1.Init.BaudRate = 115200;
huart1.Init.WordLength = UART_WORDLENGTH_8B;
huart1.Init.StopBits = UART_STOPBITS_1;
huart1.Init.Parity = UART_PARITY_NONE;
huart1.Init.Mode = UART_MODE_TX_RX;
huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
huart1.Init.OverSampling = UART_OVERSAMPLING_16;
huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
if(HAL_UART_Init(&huart1) != HAL_OK) {
   while(1);
}
}


void accelero_interrupt_config(void){
	uint8_t Buffer;
	Buffer = 0x80; // |= (1<<7) 1000 0000 set bit[7] to enable basic interrupts
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_TAP_CFG1, Buffer);
	Buffer = 0x08; // 0000 1000 FF_Dur [4:0] = 00001, FF_Ths [2:0] = 000
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_FREE_FALL, Buffer);
	Buffer = SENSOR_IO_Read(0xD4,0x5E);
	Buffer |= (1 << 4); //set bit[4] to route FF interrupt to INT1
	SENSOR_IO_Write(LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW, LSM6DSL_ACC_GYRO_MD1_CFG, Buffer);
}

/* Switch function */
void mode_switch(void){
	if (mode == INTENSIVE){
		mode = NORMAL;
		sprintf(message_print, "Entering Normal Mode\r\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
		HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
		reset();
	} else {
			mode = INTENSIVE;
			sprintf(message_print, "Entering Intensive Care Mode\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)message_print, strlen(message_print),0xFFFF);
			HAL_GPIO_WritePin(GPIOB, LED2_Pin, 1);
			reset();
		}
}
void reset(void){
	accflag = SAFE;
	gyroflag = SAFE;
	pressureflag = SAFE;
	tempflag = SAFE;
	magflag = SAFE;
	humflag = SAFE;
}

