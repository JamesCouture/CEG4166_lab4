/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Keypad4X4.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include "fonts.h"
#include "ssd1306.h"
#include "task.h"

#include "queue.h"
#include "semphr.h"
#include "timers.h"

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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

///* Definitions for oledTask */
//osThreadId oledTaskHandle;
//osThreadId keypadTaskHandle;
//osThreadId indicatorTaskHandle;
//osThreadId updaterTaskHandle;
//osThreadId uartTaskHandle;



/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void startOLEDTask(void *argument);
void startKeypadTask(void *argument);
void startIndicatorTask(void *argument);
void startUpdaterTask(void *argument);
void startUARTTask(void *argument);
void enqueueMessage(const char* message);
void startPIRTask(void *argument);
void timerCallback(TimerHandle_t xTimer);
void timerCallback2(TimerHandle_t xTimer);


#define BUFFER_SIZE 6
extern char key;

char buffer[BUFFER_SIZE];
int bufferIndex = 0;
bool timerFlag = false;
bool timerFlag2 = false;

int alarmState = 0;

QueueHandle_t xQueue;
SemaphoreHandle_t xIndicatorSemaphore;
SemaphoreHandle_t xOLEDSemaphore;
TimerHandle_t xTimer;
TimerHandle_t xTimer2;

BaseType_t indicatorTaskHandle;
BaseType_t oledTaskHandle;
BaseType_t uartTaskHandle;
BaseType_t keypadTaskHandle;
BaseType_t updaterTaskHandle;
BaseType_t pirTaskHandle;

char alarmCode[] = "000000";
char actionCode = '\0';

int timerCount = 60;
char timerCountStr[20];

void enqueueMessage(const char* message) {
    if (message == NULL) {
        return;
    }

    size_t msgLen = strlen(message) + 1;
    char msgBuffer[50];

    strncpy(msgBuffer, message, sizeof(msgBuffer));
    xQueueSend(xQueue,&msgBuffer,0);
}

void clearBuffer() {
    memset(buffer, 0, BUFFER_SIZE);
    bufferIndex = 0;
}

void addToBuffer(char c) {
    if (bufferIndex < BUFFER_SIZE) {
        buffer[bufferIndex++] = c;
    } else {
        clearBuffer();
        enqueueMessage("BUFFER CLEARED BY SYSTEM\n");
    }
}

void printBuffer() {
    char uartMsg[50];
    char startMsg[] = "BUFFER: ";

    snprintf(uartMsg, sizeof(uartMsg), "%s", startMsg);
    for (int i = 0; i < bufferIndex; i++) {
        char temp[2] = {buffer[i], '\0'};
        strncat(uartMsg, temp, 1);
    }
    strncat(uartMsg, "\n", 1);
    enqueueMessage(uartMsg);
}

void printUART(char *str) {
	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

void startOLEDTask(void *argument) {
    SSD1306_Init();

    for (;;) {
        if (xSemaphoreTake(xOLEDSemaphore,portMAX_DELAY)) {
            SSD1306_Clear();
            SSD1306_UpdateScreen();

            SSD1306_GotoXY(0, 0);
            if (alarmState == 0) {
                SSD1306_Puts("DISARMED", &Font_11x18, 1);
            } else {

				sprintf(timerCountStr, "%d", timerCount);

				// Concatenate a string to the converted integer string
				strcat(timerCountStr, " - ARMED ");

                SSD1306_Puts(timerCountStr, &Font_11x18, 1);
            }

            SSD1306_GotoXY(0, 30);
            SSD1306_Puts("CODE:", &Font_11x18, 1);

            char anonBuf[BUFFER_SIZE + 1];
            for (int i = 0; i < bufferIndex; i++) {
                anonBuf[i] = '*';
            }
            anonBuf[bufferIndex] = '\0';

            SSD1306_Puts(anonBuf, &Font_11x18, 1);
            SSD1306_UpdateScreen();
        }
        vTaskDelay(1);
    }
}

void startKeypadTask(void *argument) {
    for (;;) {
        key = Get_Key();
        actionCode = '\0';
        if (key == '*' || key == '#') {
            actionCode = key;
        } else if (bufferIndex < BUFFER_SIZE){
            buffer[bufferIndex++] = key;
        } else {
            clearBuffer();

            enqueueMessage("BUFFER CLEARED BY SYSTEM\n");
            printUART("BUFFER CLEARED BY SYSTEM\n");
        }

        xSemaphoreGive(xOLEDSemaphore);
        xSemaphoreGive(xIndicatorSemaphore);

        printBuffer();
        HAL_Delay(200);
        vTaskDelay(1);
    }
}

void startIndicatorTask(void *argument) {
    for (;;) {
        if (xSemaphoreTake(xIndicatorSemaphore,0)) {
            if (alarmState == 0) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

//                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

//                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
            }
        }
        vTaskDelay(1);
    }
}

void startUpdaterTask(void *argument) {
    for (;;) {
        if (actionCode != '\0') {
            if (actionCode == '*') {
                clearBuffer();
                enqueueMessage("BUFFER CLEARED BY USER\n");
                printUART("BUFFER CLEARED BY USER\n");
            } else if (actionCode == '#') {
                if (bufferIndex == 4 || bufferIndex == 6) {
                    if (alarmState == 0) {
                        strcpy(alarmCode, buffer);
                        alarmState = 1;
                        clearBuffer();
                        enqueueMessage("SYSTEM ARMED\n");

                        printUART("SYSTEM ARMED\n");
                        actionCode = '\0';

                    } else {
                        if (strcmp(buffer, alarmCode) == 0) {
                            alarmState = 0;
                			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
                			timerCount = 60;
                            clearBuffer();
                            enqueueMessage("SYSTEM DISARMED\n");

                            printUART("SYSTEM DISARMED\n");

                        } else {
                            char errMsg[25];
                            snprintf(errMsg, sizeof(errMsg), "REQUIRED CODE: %s\n", alarmCode);
                            enqueueMessage(errMsg);

                            printUART(errMsg);

                            snprintf(errMsg, sizeof(errMsg), "RECEIVED CODE: %s\n", buffer);
                            enqueueMessage(errMsg);

                            printUART(errMsg);

                            clearBuffer();
                        }
                    }
                } else {
                    enqueueMessage("CODE: INVALID LENGTH\n");
                    printUART("CODE: INVALID LENGTH\n");

                    clearBuffer();
                }
            }
            xSemaphoreGive(xOLEDSemaphore);
            xSemaphoreGive(xIndicatorSemaphore);
        }
        vTaskDelay(100);
    }
}

void startPIRTask(void *argument){

	for (;;){

		if (alarmState == 1 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET ) {
            printUART("ARMED AND DETECTED MOVEMENT\n");
			for (int i = 60; i >= 0; i--) {
				timerCount = i;
				vTaskDelay(pdMS_TO_TICKS(1000));
			}
			for (int i = 0; i < 1000000; i++) {
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET) {
                    printUART("ALARM RING\n");
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
					break;
				}
			}
		} else {

		}
	}

}

void startUARTTask(void *argument) {
    char msgBuf[50];
    for (;;) {
        if (xQueueReceive(xQueue, &msgBuf, 100)) {
            HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
        }
        vTaskDelay(1);
    }
}

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  SSD1306_Init();

  /* USER CODE END 2 */

  xQueue = xQueueCreate(10, 50);

  //Enter binary semapohores
  xOLEDSemaphore = xSemaphoreCreateBinary();
  xIndicatorSemaphore = xSemaphoreCreateBinary();

  xSemaphoreGive(xOLEDSemaphore);
  xSemaphoreGive(xIndicatorSemaphore);

  /* Init scheduler */
//  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */

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
  /* creation of adcTask */
  TaskHandle_t xHandleOLED = NULL;
  TaskHandle_t xHandleIndicator = NULL;
  TaskHandle_t xHandleUART = NULL;
  TaskHandle_t xHandleKeypad= NULL;
  TaskHandle_t xHandleUpdater = NULL;
  TaskHandle_t xHandlePIR = NULL;

//  osThreadDef(indicatorTask, startIndicatorTask, osPriorityLow, 0, 128);
//  indicatorTaskHandle = osThreadCreate(osThread(indicatorTask), NULL);
//
//  osThreadDef(oledtask, startOLEDTask, osPriorityNormal, 0, 128);
//  oledTaskHandle = osThreadCreate(osThread(oledtask), NULL);
//
//  osThreadDef(uarttask, startUARTTask, osPriorityLow, 0, 128);
//  uartTaskHandle = osThreadCreate(osThread(uarttask), NULL);
//
//  osThreadDef(keypadtask, startKeypadTask, osPriorityLow, 0, 128);
//  keypadTaskHandle = osThreadCreate(osThread(keypadtask), NULL);
//
//  osThreadDef(updatertask, startUpdaterTask, osPriorityLow, 0, 128);
//  updaterTaskHandle = osThreadCreate(osThread(updatertask), NULL);


  indicatorTaskHandle = xTaskCreate(startIndicatorTask, "indicatorTask", 128 * 4, (void *)1, 2, &xHandleIndicator);
  oledTaskHandle = xTaskCreate(startOLEDTask, "oledtask", 128 * 4,(void *)1, 3, &xHandleOLED);
  uartTaskHandle = xTaskCreate(startUARTTask, "uarttask", 128 * 4,(void *)1, 2, &xHandleUART);
  keypadTaskHandle = xTaskCreate(startKeypadTask, "keypadtask", 128 * 4,(void *)1, 2, &xHandleKeypad);
  updaterTaskHandle = xTaskCreate(startUpdaterTask, "updatertask", 128 * 4,(void *)1, 2, &xHandleUpdater);
  pirTaskHandle = xTaskCreate(startPIRTask, "pirTask", 128 * 4,(void *)1, 2, &xHandlePIR);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, KC0_Pin|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|KC3_Pin
                          |KC1_Pin|KC2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PIR_IN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KC0_Pin PB13 PB14 KC3_Pin
                           KC1_Pin KC2_Pin */
  GPIO_InitStruct.Pin = KC0_Pin|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|KC3_Pin
                          |KC1_Pin|KC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KR1_Pin */
  GPIO_InitStruct.Pin = KR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KR1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KR3_Pin KR2_Pin */
  GPIO_InitStruct.Pin = KR3_Pin|KR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KR0_Pin */
  GPIO_InitStruct.Pin = KR0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KR0_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
