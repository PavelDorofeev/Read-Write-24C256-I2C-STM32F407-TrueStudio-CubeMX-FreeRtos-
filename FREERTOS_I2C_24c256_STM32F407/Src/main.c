/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "iic_func.h"
//#include "semphr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

osThreadId_t defaultTaskHandle;
osThreadId_t myTask02Handle;
/* USER CODE BEGIN PV */

uint8_t LCD_address = 0x7C; //0x78;
uint8_t I2C_SLAVE_ADDR_24c256=(0x50 << 1) ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void *argument); // for v2
void StartTask02(void *argument); // for v2

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//SemaphoreHandle_t xSemaphore;

uint8_t transferDirection;
uint8_t transferRequested=0;
uint16_t rxPos=0;
uint8_t txBuf[50]={"1111111122222222222222222333333\n"};
uint8_t rxBuf[50]={"0000000000000000000000000000000\n"};
uint16_t txPos=0;
uint8_t masterWaitTransmit=0;
uint8_t masterWaitTransmit_OK=0;
uint8_t masterWaitRead_OK=0;

int8_t t_frac, t_int;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */


	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */
	printf("start\n");
	/* USER CODE END 2 */

	osKernelInitialize(); // Initialize CMSIS-RTOS

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	const osThreadAttr_t defaultTask_attributes = {
			.name = "defaultTask",
			.priority = (osPriority_t) osPriorityNormal,
			.stack_size = 1024
	};
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* definition and creation of myTask02 */
	const osThreadAttr_t myTask02_attributes = {
			.name = "myTask02",
			.priority = (osPriority_t) osPriorityLow,
			.stack_size = 1024
	};
	myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void)
{

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 124;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED1_Pin */
	GPIO_InitStruct.Pin = LED1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : KEY2_Pin */
	GPIO_InitStruct.Pin = KEY2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : KEY1_Pin */
	GPIO_InitStruct.Pin = KEY1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED2_Pin */
	GPIO_InitStruct.Pin = LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t nn=0;
	if (hi2c-> Instance ==  I2C1)
		nn=1;//printf("1 HAL_I2C_MasterTxCpltCallback I2C1\n");
	else if (hi2c-> Instance ==  I2C2)
		nn=2;//printf("2 HAL_I2C_MasterTxCpltCallback I2C2\n");
	else
		nn=0;//printf(".....\n");
	uint16_t XferCount=hi2c->XferCount;
	uint16_t XferSize=hi2c->XferSize;
	uint16_t more=XferSize-XferCount;
	txPos=txPos+XferCount;
	uint8_t buf[50];
	sprintf(buf,"I2C%x XferSize=%x XferCount=%x txPos=%x more=%x\n",nn,XferSize,XferCount,txPos,more);
	printf(buf);
	if(XferCount >0)
		;
	else
		masterWaitTransmit_OK=1;

	//HAL_I2C_Master_Transmit_IT(hi2c,I2C_SLAVE_ADDR,&txBuf[txPos],more);

}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	// возникает когда XferCount становится = 0 см. I2C_MasterTransmit_BTF
	// BTF (Byte transfer finished)

	uint8_t nn=0;
	if (hi2c-> Instance ==  I2C1)
	{
		nn=1;
		masterWaitTransmit_OK=1;
		if(hi2c->XferCount>0)
			printf("не нормально см. I2C_MasterTransmit_BTF\n"); // BTF (Byte transfer finished)
	}
	else if (hi2c-> Instance ==  I2C2)
		nn=2;
	else
		nn=0;



}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{

}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//см. HAL_I2C_EV_IRQHandler
	if (hi2c-> Instance ==  I2C1)
	{
		if(hi2c->XferCount>0)
			printf("не нормально см. I2C_MasterTransmit_BTF\n"); // BTF (Byte transfer finished)
	}

	masterWaitRead_OK=1;
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	UNUSED(AddrMatchCode);
	uint8_t nn=0;
	uint8_t buf[15];
	if(hi2c->Instance == I2C1)
	{
		nn=1;//printf("1 HAL_I2C_AddrCallback AddrMatchCode=%x TransferDirection=%x\n",AddrMatchCode,TransferDirection);
	}
	else if(hi2c->Instance == I2C2)
	{
		nn=2;//printf("2 HAL_I2C_AddrCallback AddrMatchCode=%x TransferDirection=%x\n",AddrMatchCode,TransferDirection);
		//transferRequested = 1;
		transferDirection = TransferDirection;
		transferRequested=1;
	}
	/*sprintf(buf,"\nI2C%x AddrCallback \n",nn);
	printf(buf);*/



}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//Это когда уже передали стоп

	if (hi2c-> Instance ==  I2C1)
	{
		//printf("1 HAL_I2C_ListenCpltCallback I2C1\n");
	}
	else if (hi2c-> Instance ==  I2C2)
	{
		//printf("2 HAL_I2C_ListenCpltCallback \n");
		//HAL_I2C_EnableListen_IT(hi2c);

	}
}
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
	printf("2 HAL_I2C_AbortCpltCallback \n");
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t nn=0;
	if (hi2c-> Instance ==  I2C1)
		nn=1;
	else if (hi2c-> Instance ==  I2C2)
		nn=2;
	else
		nn=0;//printf(".....\n");
	/*uint16_t XferCount=hi2c->XferCount;
	uint16_t XferSize=hi2c->XferSize;
	uint16_t more=XferSize-XferCount;
	rxPos=rxPos+XferCount;
	uint8_t buf[100];
	sprintf(buf,"SLAVE I2C%x XferSize=%x XferCount=%x txPos=%x more=%x\n",nn,XferSize,XferCount,rxPos,more);
	printf(buf);*/
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c-> Instance ==  I2C1)
		;//printf("1 HAL_I2C_SlaveTxCpltCallback\n");
	else if (hi2c-> Instance ==  I2C2)
		;//printf("2 HAL_I2C_SlaveTxCpltCallback\n");
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t buf[50];
	uint8_t nn=0;
	if (hi2c-> Instance ==  I2C1)
		nn=1;
	else if (hi2c-> Instance ==  I2C2)
		nn=2;
	else
		nn=0;

	sprintf(buf,"\n I2C%x ErrorCallback\n",nn);
	printf(buf);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{

	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	/**xSemaphore = xSemaphoreCreateMutex();

	if( xSemaphore != NULL )
	{
		printf("xSemaphore != NULL\n");
	}*/
	/*osDelay(250);
	osDelay(250);*/

	HAL_StatusTypeDef res;
	uint8_t buf[100];

	printf("1 StartDefaultTask %x I2C_SLAVE_ADDR_24c256=0x%0.2X rxBuf[0]=0x%X\n",txBuf[0],I2C_SLAVE_ADDR_24c256,rxBuf[0]);
	uint16_t txAddr=0;
	uint16_t rxAddr=0;
	uint8_t batch=16;
	for(;;)
	{
		if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin) == SET)
		{
			HAL_GPIO_WritePin(KEY2_GPIO_Port,KEY2_Pin,RESET);
			HAL_GPIO_WritePin(KEY2_GPIO_Port,KEY2_Pin,RESET);
			HAL_GPIO_WritePin(KEY2_GPIO_Port,KEY2_Pin,RESET);
			vTaskDelay(1);

			printf("\nKEY2_Pin Transmit\n");

			//HAL_I2C_Master_Sequential_Transmit_IT(&hi2c1, I2C_SLAVE_ADDR_24c256, txBuf, 1, I2C_FIRST_FRAME);

			//while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
			//	;

			while(HAL_I2C_IsDeviceReady(&hi2c1, I2C_SLAVE_ADDR_24c256 , 1, HAL_MAX_DELAY)!= HAL_OK)
			{
				vTaskDelay(1);
			}
			/*txBuf[0]=(uint8_t)txAddr>>8; // старшая часть адреса
			txBuf[1]=(uint8_t)txAddr & 0xFF;   // младшая часть адреса
			txBuf[2]="=";*/

			for(int ii=0; ii < batch; ii++)
			{
				txBuf[ii]=ii;
				printf("%0.2X ",txBuf[ii]);
			}
			printf("\n");

			res=HAL_I2C_Mem_Write_IT(&hi2c1, I2C_SLAVE_ADDR_24c256, txAddr, I2C_MEMADD_SIZE_16BIT,txBuf, batch);
			txAddr=txAddr+batch;
			/*if(res == HAL_OK)
				printf(" HAL_I2C_Mem_Write_IT == HAL_OK\n");
			else
				printf(" HAL_I2C_Mem_Write_IT != HAL_OK\n");*/

			while(masterWaitTransmit_OK ==0 )
			{
				vTaskDelay(1);
				//printf(" masterWaitTransmit_OK =%x\n",masterWaitTransmit_OK);
			}

			printf("masterWaitTransmit_OK == 1  txAddr:0x%0.4X\n",txAddr);
			masterWaitTransmit_OK=0;

			while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
				;

			//HAL_I2C_Master_Sequential_Receive_IT(&hi2c1, I2C_SLAVE_ADDR, txBuf, 1, I2C_LAST_FRAME);
			//res=HAL_I2C_Master_Transmit_IT(&hi2c1,I2C_SLAVE_ADDR, &txBuf[0],1);
			//masterWaitTransmit=1;
			/*if(res != HAL_OK)
						printf("!= HAL_OK\n");
					else
						printf("== HAL_OK\n");*/
			vTaskDelay(1);

		}
		else if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==SET)
		{
			printf("\n KEY1_Pin \n");

			HAL_GPIO_WritePin(KEY1_GPIO_Port,KEY1_Pin,RESET);
			HAL_GPIO_WritePin(KEY1_GPIO_Port,KEY1_Pin,RESET);
			HAL_GPIO_WritePin(KEY1_GPIO_Port,KEY1_Pin,RESET);
			vTaskDelay(1);

			while(HAL_I2C_IsDeviceReady(&hi2c1, I2C_SLAVE_ADDR_24c256 , 1, HAL_MAX_DELAY)!= HAL_OK)
			{
				vTaskDelay(1);
			}

			res=HAL_I2C_Mem_Read_IT(&hi2c1, I2C_SLAVE_ADDR_24c256, rxAddr, I2C_MEMADD_SIZE_16BIT,rxBuf, batch);

			/*if(res == HAL_OK)
				printf(" HAL_I2C_Mem_Read_IT != HAL_OK rxBuf[0]=%x\n",rxBuf[0]);
			else
				printf(" HAL_I2C_Mem_Read_IT == HAL_OK\n");*/

			while(masterWaitRead_OK == 0 )
			{
				vTaskDelay(1);
				//printf(" masterWaitRead_OK = %x\n",masterWaitRead_OK);
			}

			printf(" masterWaitRead_OK = %x \n",masterWaitRead_OK);
			printf("\n0x%0.4X : ",rxAddr);

			for(uint8_t ii=0;ii<batch; ii++)
			{
				printf("%0.2X ",rxBuf[ii]);
			}
			printf("\n");

			//sprintf(buf,"%s\n",rxBuf);
			//printf(buf);

			rxAddr=rxAddr+batch;


			masterWaitRead_OK=0;

			//HAL_I2C_Master_Sequential_Receive_IT(&hi2c1, I2C_SLAVE_ADDR, txBuf, 1, I2C_LAST_FRAME);

			while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
				;

			vTaskDelay(100);

		}


		/*while(HAL_I2C_IsDeviceReady(&hi2c1, LCD_address , 1, 1)!= HAL_OK) //HAL_MAX_DELAY);
		{
			  vTaskDelay(1);
		}*/
		vTaskDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
	/* USER CODE BEGIN StartTask02 */
	/* Infinite loop */
	char buf[100];
	printf("2 StartTask02\n");
	transferRequested=0;
	uint8_t *dirWR="\nTRANSFER_DIR_WRITE\n";
	uint8_t *dirRD="\nTRANSFER_DIR_READ\n";

	HAL_StatusTypeDef res;
	while(HAL_I2C_EnableListen_IT(&hi2c2)!=HAL_OK)
		;
	uint32_t lastConversion=0;

	for(;;)
	{
		HAL_I2C_EnableListen_IT(&hi2c2);

		if(transferRequested>0)
		{
			if(HAL_GetTick() - lastConversion > 1000L)
			{
				/*while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_LISTEN)
				;*/
				/*switch(transferDirection)
				{
				case TRANSFER_DIR_WRITE :
					//printf(dirWR);

					if(HAL_I2C_Slave_Receive_IT(&hi2c2, buf, 1)==HAL_OK)
						printf("\n< %x\n",buf[0]);
					else
						printf("HAL_I2C_Slave_Receive_IT != HAL_OK %x\n",buf[0]);
					//transferRequested=0;

				case TRANSFER_DIR_READ :
					printf(dirRD);
					HAL_I2C_Slave_Receive_IT(&hi2c2, buf, 1);

				default :
					;
				}*/
				lastConversion = HAL_GetTick();
				//printf("\n AddrCallback transferDirection=%x\n",transferDirection);
			}
		}
		transferRequested = 0;

		if(transferDirection == TRANSFER_DIR_WRITE)
		{
			/* Master is sending register address */
			HAL_I2C_Slave_Sequential_Receive_IT(&hi2c2, buf, 1, I2C_FIRST_FRAME);

			while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_LISTEN)
				;

			switch(buf[0])
			{
			case WHO_AM_I_REGISTER:
				buf[0] = WHO_AM_I_VALUE;
				break;
			case TEMP_OUT_INT_REGISTER:
				buf[0] = t_int;
				break;
			case TEMP_OUT_FRAC_REGISTER:
				buf[0] = t_frac;
				break;
			default:
				buf[0] = 0xFF;
				break;
			}

			HAL_I2C_Slave_Sequential_Transmit_IT(&hi2c2, buf, 1, I2C_LAST_FRAME);

			while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY)
				;
		}


		vTaskDelay(1);
	}

	/* USER CODE END StartTask02 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM9 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM9) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	printf("Error_Handler\r\n");
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{ 
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	printf("Wrong parameters value: file %s on line %d\r\n", file, line);
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
