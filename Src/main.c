/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_hid.h" //Add
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
__IO uint16_t ADC_value[2] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern USBD_HandleTypeDef hUsbDeviceFS; 
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*
 * Keyboard buffer:
 * buf[1]: MOD (2=SHIFT, 1= CTRL,3=?
 * buf[2]: reserved
 * buf[3]..buf[8] - keycodes 1..6
 */
 
uint8_t buffer[8]= {0,0,0,0,0,0,0,0};//

void keyboard_write(uint8_t char1){
    buffer[2]=char1;
    USBD_HID_SendReport(&hUsbDeviceFS,buffer,8); //sizeof(buffer));
    HAL_Delay(100);
		buffer[0] = 0; buffer[2]=0; USBD_HID_SendReport(&hUsbDeviceFS,buffer,8);
}

void ReadKeyboard (void)
{
	//GPIOD
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15)){} keyboard_write(KEY_J);}}	//J
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_14)){} keyboard_write(KEY_K);}}	//K
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_13)){} keyboard_write(KEY_L);}}	//L		
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)){} keyboard_write(KEY_F8);}}		// ,
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11)){} keyboard_write(KEY_F9);}}		// =
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10)){} keyboard_write(KEY_F21);}}	//F11 -> F21
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9)){} keyboard_write(KEY_F20);}}			//F10->20
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8)){} keyboard_write(KEY_F4);}}			//^ ???
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7)){} keyboard_write(KEY_G);}}			//G
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)){} keyboard_write(KEY_H);}}			//H
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_5)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_5)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_5)){} keyboard_write(KEY_U);}}			//U
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4)){} keyboard_write(KEY_I);}}			//I
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_3)){} keyboard_write(KEY_O);}}		//O
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)){} keyboard_write(KEY_P);}}			//P
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_1)){} keyboard_write(KEY_F13);}}		//[ 
	if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_0)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_0)) { while (!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_0)){} keyboard_write(KEY_F14);}}		//] 
 // GPIOA
 	//if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)) { while (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7)){} keyboard_write(KEY_G);}}			//LED
	if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)) { while (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)){} keyboard_write(KEY_X);}}			//X
	if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)) { while (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)){} keyboard_write(KEY_Z);}}			//Z
	if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)) { while (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)){} keyboard_write(KEY_HOME);}}			//"
	if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)) { while (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)){} keyboard_write(KEY_A);}}		//; 
	if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)) { while (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)){} keyboard_write(KEY_BACKSPACE);}}			//<--
	if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)) { while (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_1)){} keyboard_write(KEY_KEYPAD_0_INSERT);}}		//0
	if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)) { while (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)){} keyboard_write(KEY_KEYPAD_9_PAGEUP);}}		//9
	//GPIOB
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)) { while (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)){} keyboard_write(KEY_F5);}}	//&
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)) { while (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)){} keyboard_write(KEY_F7);}}	//(
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)) { while (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13)){} keyboard_write(KEY_F6);}}		//*	
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)) { while (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12)){} keyboard_write(KEY_ESCAPE);}}		//!
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)) { while (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)){} keyboard_write(KEY_F1);}}		//F1
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)) { while (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)){} keyboard_write(KEY_F2);}}		//F2
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)) { while (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)){} keyboard_write(KEY_S);}}			//S
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)) { while (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)){} keyboard_write(KEY_D);}}			//D
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)) { while (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)){} keyboard_write(KEY_F);}}			//F
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)) { while (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)){} keyboard_write(KEY_T);}}			//T
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)) { while (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)){} keyboard_write(KEY_Y);}}			//Y
				// Joytick Button ....
	if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)) { while (!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)){} keyboard_write(KEY_ENTER);}}			//KEY_ENTER
	
	//GPIOC
	if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)) { while (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)){} keyboard_write(KEY_V);}}		//V
	if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)) { while (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)){} keyboard_write(KEY_KEYPAD_4_LEFT_ARROW);}}		//4
	if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_12)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_12)) { while (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_12)){} keyboard_write(KEY_PAGEUP);}}		//{
	if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11)) { while (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11)){} keyboard_write(KEY_PAGEDOWN);}}		//}
	if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10)) { while (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10)){} keyboard_write(KEY_LEFTARROW);}}		//LEFT
	if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)) { while (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)){} keyboard_write(KEY_RIGHTARROW);}}	//RIGHT
	if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)) { while (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)){} keyboard_write(KEY_UPARROW);}}			//UP
	if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)) { while (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)){} keyboard_write(KEY_DOWNARROW);}}			//DOWN
	if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6)) { while (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6)){} keyboard_write(KEY_C);}}		//C CENTER
	if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3)) { while (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3)){} keyboard_write(KEY_KEYPAD_8_UP_ARROW);}}		//8
	if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)) { while (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)){} keyboard_write(KEY_KEYPAD_7_HOME);}}		//7
	if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)) { while (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)){} keyboard_write(KEY_KEYPAD_6_RIGHT_ARROW);}}		//6
	if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)) { while (!HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)){} keyboard_write(KEY_KEYPAD_5);}}		//5
	//GPIOE
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_14)){} keyboard_write(KEY_F3);}}	 // F3
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13)){} keyboard_write(KEY_F12 );}}		// <	,
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12)){} keyboard_write(KEY_END1);}}		// > .
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_11)){} keyboard_write(KEY_DELETE);}}		// / 
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_10)){} keyboard_write(KEY_INSERT);}}		// ? => `
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9)){} keyboard_write(KEY_B);}}			//B
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8)){} keyboard_write(KEY_N);}}			//N
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7)){} keyboard_write(KEY_M);}}			//M
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_6)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_6)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_6)){} keyboard_write(KEY_KEYPAD_3_PAGEDN);}}			//3
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5)){} keyboard_write(KEY_KEYPAD_2_DOWN_ARROW);}}			//2
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)){} keyboard_write(KEY_KEYPAD_1_END);}}			//1
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)){} keyboard_write(KEY_W);}}		//W
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_2)){} keyboard_write(KEY_Q);}}			//Q
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1)){} keyboard_write(KEY_E);}}		//E
	if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0)){} keyboard_write(KEY_R);}}		//R
}
void ReadAdc (void)
{
	//if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0)) { HAL_Delay(10); if (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0)) { while (!HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_0)){} keyboard_write(KEY_R);}}		//R
	if (ADC_value[0] > 4000 ) { HAL_Delay(100);  keyboard_write(KEY_F18);} // Phai
	if (ADC_value[0] < 100 ) { HAL_Delay(100);   keyboard_write(KEY_F17);} // Trai
	if (ADC_value[1] > 4000 ) { HAL_Delay(100);  keyboard_write(KEY_F15);} // Up
	if (ADC_value[1] < 100 ) { HAL_Delay(100);   keyboard_write(KEY_F16);} // Down
}
/* USER CODE END 0 */
uint8_t item = 4;
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
	MX_DMA_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
	//HAL_Delay(4000); 
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_value,2); //???
	
	//TestKey(); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */
	 ReadKeyboard(); 
	 ReadAdc(); 
  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Configure the Systick interrupt time 
    */
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
	//
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}
/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE7 PE8 PE9 PE10 
                           PE11 PE12 PE13 PE14 
                           PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2 
                           PC3 PC6 PC7 PC8 
                           PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2 
                          |GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB11 PB12 
                           PB13 PB14 PB15 PB5 
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 
                           PD12 PD13 PD14 PD15 
                           PD0 PD1 PD2 PD3 
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET); //A7

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED2_Pin|LED3_Pin, GPIO_PIN_SET); //C4+C5

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
