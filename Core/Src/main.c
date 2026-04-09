/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "font.h"
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Display dimensions
#define NUMROWS 160
#define NUMCOLS 128
// Common 16-bit RGB colors: bbbbb gggggg rrrrr
#define BLACK 0x0000 // 00000 000000 00000
#define WHITE 0xFFFF // 11111 111111 11111 (red + green + blue)
#define RED 0x001F // 00000 000000 11111
#define GREEN 0x07E0 // 00000 111111 00000
#define BLUE 0xF800 // 11111 000000 00000
#define CYAN 0xFFE0 // 11111 111111 00000 (green + blue)
#define MAGENTA 0xF81F // 11111 000000 11111 (red + blue)
#define YELLOW 0x07FF // 00000 111111 11111 (red + green)
#define ORANGE 0x041F // 00000 100000 11111 (red + 50% green)
#define PURPLE 0x8010 // 10000 000000 10000 (50% red + 50% blue)
#define GRAY 0x8410 // 10000 100000 10000 (50% red + 50% green + 50% blue)
#define LIGHTGRAY 0xC618 // 11000 110000 11000 (75% red + 75% green + 75% blue)
#define DARKGRAY 0x4208 // 01000 010000 01000 (25% red + 25% green + 25% blue)
// ST7735 command codes
#define SWRESET 0x01
#define SLPOUT 0x11
#define COLMOD 0x3A
#define MADCTL 0x36
#define DISPON 0x29
#define CASET 0x2A
#define RASET 0x2B
#define RAMWR 0x2C
#define COLOR_MODE_16BIT 0x05
#define MADCTL_DEFAULT 0xC8
// MPU6050
#define MPU6050_ADDR (0x68 << 1)  // shift for HAL
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t cursor_x = 0; // start at left
uint16_t cursor_y = 0; // start at top
int16_t Accel_X, Accel_Y, Accel_Z; // Initialize variables for the accelerometer
int16_t Gyro_X, Gyro_Y, Gyro_Z; // Initialize variables for the gyroscope
float angle_x = 0.0f;
float angle_y = 0.0f;
float dt = 0.01f;   // 10 ms loop time
int prev_x = 64;
int prev_y = 80;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void ST7735_Reset();
void ST7735_Command(uint8_t cmd);
void ST7735_Init(void);
void ST7735_Data(uint8_t data[], uint16_t size);
void ST7735_SetAddressWindow(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
void ST7735_DrawPixel(uint8_t x, uint8_t y, uint16_t color);
void ST7735_DrawScreen(uint16_t frame[NUMROWS][NUMCOLS]);
void ST7735_UpdateCharCursor(uint16_t x, uint16_t y);
void ST7735_PrintChar(char character);
void ST7735_PrintString(char* string);
void ST7735_FillScreen(uint16_t color);
void MPU6050_Init();
void MPU6050_Read_Accel();
void MPU6050_Read_Gyro();
/* USER CODE END PFP */

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
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  ST7735_Init();
  MPU6050_Init();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  ST7735_FillScreen(WHITE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	MPU6050_Read_Accel(); // read the sensor data of the accelerometer
    MPU6050_Read_Gyro(); // read the sensor data of the gyroscope

    // set the acceleration in each direction (g's)
    float Ax = Accel_X / 16384.0;
    float Ay = Accel_Y / 16384.0;
    float Az = Accel_Z / 16384.0;

    // convert gyroscope data to degrees/sec
    float Gx = Gyro_X / 131.0;
    float Gy = Gyro_Y / 131.0;

    float accel_angle_x = atan2(Ay, Az) * 180 / M_PI; // calculate the x tilt angle and convert from rads to degrees
    float accel_angle_y = atan2(-Ax, sqrt(Ay*Ay + Az*Az)) * 180 / M_PI; // calculate the y tilt angle and convert from rads to degrees

    // integrate the gyroscope and the accelerometer for smooth motion tracking
    // The first part is the gyroscope, which tracks the motion; the second part is the correction from the accelerometer
    // to keep the system from drifting
    angle_x = 0.98 * (angle_x + Gx * dt) + 0.02 * accel_angle_x;
    angle_y = 0.98 * (angle_y + Gy * dt) + 0.02 * accel_angle_y;

    // multiply the angle by a value like 2 or 3 to increase the sensitivity, higher value equals more sensitive to tilt
    int x = 64 + angle_y * 3; // map angle data to x screen position for the pixel
    int y = 80 + angle_x * 3; // map angle data to y screen position for the pixel

    int dx = abs(x - 64); // compare where the pixel x value is in relation to the center
	int dy = abs(y - 80); // compare where the pixel y value is in relation to the center

	uint16_t color; // initialize a color variable

	if (dx < 20 && dy < 25)  // if within the middle of the screen, display green pixels
		color = GREEN;
	else if (dx < 40 && dy < 50) // if outside the middle but before the outer ring of the screen, display yellow pixels
		color = YELLOW;
	else // if in the outer ring of the screen, display red pixels
		color = RED;

	static uint16_t last_color = 0xFFFF; // Initialize a last_color variable for the loop below

	if (color != last_color) { // If the last color is different update how the buzzer should sound
		if (color == YELLOW) {
			__HAL_TIM_SET_AUTORELOAD(&htim2, 1000); // set the ARR to 1000 for a longer count cycle
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500); // lower pitch
		}
		else if (color == RED) {
			__HAL_TIM_SET_AUTORELOAD(&htim2, 500); // set the ARR to 500 for a shorter count cycle
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 250); // higher pitch
		}
		else { // Green
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);  // no sound
		}
		__HAL_TIM_SET_COUNTER(&htim2, 0); // reset the counter
		last_color = color; // set the new last color
	}

    // erase old pixel
	ST7735_DrawPixel(prev_x, prev_y, WHITE);
	ST7735_DrawPixel(prev_x, prev_y + 1, WHITE);
	ST7735_DrawPixel(prev_x + 1, prev_y, WHITE);
	ST7735_DrawPixel(prev_x + 1, prev_y + 1, WHITE);
	// draw new pixel
	ST7735_DrawPixel(x, y, color);
	ST7735_DrawPixel(x, y + 1, color);
	ST7735_DrawPixel(x + 1, y, color);
	ST7735_DrawPixel(x + 1, y + 1, color);
	// save new position
	prev_x = x;
	prev_y = y;
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CS_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin LD2_Pin */
  GPIO_InitStruct.Pin = CS_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DC_Pin */
  GPIO_InitStruct.Pin = DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DC_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ST7735_Reset() {
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

void ST7735_Command(uint8_t cmd) {
    HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}
void ST7735_Data(uint8_t data[], uint16_t size) {
 // Change the DC (Data/Command) pin to "Data" mode
 HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
 // Send the data
 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // select the device
 HAL_SPI_Transmit(&hspi2, data, size, HAL_MAX_DELAY); // transmit bytes
 HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // deselect the device
}
void ST7735_Init(void) {
 // Perform a hardware reset (reset low pulse)
 HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET); // RST pin low
 HAL_Delay(5); // wait 5ms
 HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET); // RST pin high
 HAL_Delay(5); // wait 5ms
 // Perform a software reset
 ST7735_Command(0x01); // send a SWRESET (Software reset) command
 HAL_Delay(150);
 // Wake the display (exit low-power sleep mode)
 ST7735_Command(0x11); // Send a SLPOUT (Sleep out) command
 HAL_Delay(150);
 // Enter "Interface Pixel Format" mode and set to 16-bit color
 ST7735_Command(0x3A); // Send a COLMOD (Color Mode) command
 uint8_t colorMode = 0x05; // color mode = 16-bit/pixel
 ST7735_Data(&colorMode, 1); // send color mode value as a data message
 // Enter "Memory Data Access Control" mode and set to row/column order
 ST7735_Command(0x36); // Send a MADCTL (Memory Access Data Control) command
 uint8_t accessMode = 0xC8; // access mode = row/column order
 ST7735_Data(&accessMode, 1); // send access mode value as a data message
 // Turn on display
 ST7735_Command(0x29); // send a DISPON (Display On) command
 HAL_Delay(10);
}
void ST7735_DrawScreen(uint16_t frame[NUMROWS][NUMCOLS]) {
 // Select entire display
 ST7735_SetAddressWindow(0, 0, NUMCOLS-1, NUMROWS-1);
 // Transmit entire frame to display
 ST7735_Data((uint8_t*) frame, NUMROWS*NUMCOLS*2);
}
/**
 * @brief Updated the character cursor
 * @param x: the desired x coordinate (0 to 127)
 * @param y: the desired y coordinate (0 to 159)
 * @retval 0: success, 1: failure
 */
void ST7735_UpdateCharCursor(uint16_t x, uint16_t y){
 // Update
 cursor_x = x;
 cursor_y = y;
 // Perform any necessary wrap-around
 if (cursor_x > (NUMCOLS-8)){ // cursor goes off the right
 cursor_x = 0; // wrap back to left
 cursor_y += 8; // move cursor down one row
 }
 if (cursor_y > (NUMROWS-9)){ // cursor goes off the bottom
 cursor_x = 0;
 cursor_y = 0; // wrap back to top
 }
}
/**
 * @brief Draws an 8x8 character to the ST7735 (black on white) using the cursor
 * @param character: the ASCII value of the character to print (' ' through '~')
 * @retval None
 */
void ST7735_PrintChar(char character) {
 // Early exit if character is not supported
 if ((character < ' ') || (character > '~')) return;
 // Realign ASCII character to start of font table
 int fontIndex = character - 0x20; // 0x20 is where printable characters begin in ASCII
 // Get character data from font table
 const uint8_t* characterData = fontTable[fontIndex];
 // Set address window for 8x8 character
 ST7735_SetAddressWindow(cursor_x, cursor_y, cursor_x + 7, cursor_y + 7);
 // Process each row of the character, one at a time
 for (int row = 0; row < 8; row++) {
 // Get the data for the current row from the character array
 uint8_t rowData = characterData[row];
 // Process each pixel in the row (left to right, msb to lsb)
 for (int col = 7; col >= 0; col--) {
 // See if that pixel is black (1) or white (0)
 uint16_t color;
 if ((rowData & (1 << col)) == 0){ // bitmask one bit at a time
 color = WHITE;
 } else {
 color = BLACK;
 }
 // Transmit pixel data
 uint8_t pixelData[2] = { color >> 8, color & 0xFF }; // swap for Big Endian
 ST7735_Data(pixelData, 2); // send the data
 }
 }
 // Move cursor to next character window
 ST7735_UpdateCharCursor(cursor_x+8, cursor_y);
}
/**
 * @brief Draws the characters (A-Z) from string to display
 * @param x: the x coordinate of the top-left corner (0 to NUMCOLS-8)
 * @param y: the y coordinate of the top-left corner (0 to NUMROWS-8)
 * @param string: the character array
 * @retval None
 */
void ST7735_PrintString(char* string) {
 // Print each character, one at a time
 for (int i = 0; string[i] != '\0'; i++){
 ST7735_PrintChar(string[i]);
 }
}

/**
 * @brief Specifies the address window where the next pixel data will go
 * @param xStart: the starting x address
 * @param yStart: the starting y address
 * @param xEnd: the ending x address
 * @param yEnd: the ending y address
 * @retval None
 */
void ST7735_SetAddressWindow(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd) {
 // Early exit if any dimensions are out of bounds
 if ((xStart > xEnd) || (yStart > yEnd) || (xEnd >= NUMCOLS) || (yEnd >= NUMROWS))
 return;
 // Adjust x,y internal offsets if needed by changing the zeros below:
 xStart += 0;
 xEnd += 0;
 yStart += 0;
 yEnd += 0;
 // Data array for configuration data
 uint8_t address[4];
 // Set column start/end
 ST7735_Command(0x2A); // Enter "Column address set" mode
 address[0] = xStart >> 8; // upper 8 bits of x starting address
 address[1] = xStart & 0xFF; // lower 8 bits of x starting address
 address[2] = xEnd >> 8; // upper 8 bits of x ending address
 address[3] = xEnd & 0xFF; // lower 8 bits of x ending address
 ST7735_Data(address, 4); // Transmit the 4 bytes of parameter data
 // Set row start/end
 ST7735_Command(0x2B); // Enter "Row address set" mode
 address[0] = yStart >> 8; // upper 8 bits of y starting address
 address[1] = yStart & 0xFF; // lower 8 bits of y starting address
 address[2] = yEnd >> 8; // upper 8 bits of y ending address
 address[3] = yEnd & 0xFF; // lower 8 bits of y ending address
 ST7735_Data(address, 4); // Transmit the 4 bytes of parameter data
 // Get ready to send pixel data
 ST7735_Command(0x2C); // Enter "RAM Write" mode (pixel writing mode)
}
/**
 * @brief Updates a specified pixel to a specified color
 * @param x: the x address of the pixel
 * @param y: the y address of the pixel
 * @param color: two-byte color value (bbbbb gggggg rrrrr)
 * @retval None
 */
void ST7735_DrawPixel(uint8_t x, uint8_t y, uint16_t color) {
 // Early exit if (x,y) is out of bounds
 if ((x >= NUMCOLS) || (y >= NUMROWS)) return;
 // Single pixel, so start and stop on the same (x,y) pixel
 ST7735_SetAddressWindow(x, y, x, y);
 // Organize the color as a two-byte value
 uint8_t pixelData[2] = { color >> 8, color & 0xFF }; // swap for Big Endian
 // Send the data
 ST7735_Data(pixelData, 2);
}
void ST7735_FillScreen(uint16_t color)
{
    ST7735_SetAddressWindow(0, 0, NUMCOLS-1, NUMROWS-1); // define the window to fill

    uint8_t data[2] = { color >> 8, color & 0xFF };

    for (int i = 0; i < NUMCOLS * NUMROWS; i++)
    {
        ST7735_Data(data, 2);
    }
}
void MPU6050_Init() {
    uint8_t data;

    // Wake up (write 0 to register 0x6B), turn the device on
    data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, HAL_MAX_DELAY);
}
void MPU6050_Read_Accel() { // reads raw acceleration data
    uint8_t buffer[6]; // create a 6 byte array (each axis needs two bytes)

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, buffer, 6, HAL_MAX_DELAY); // read 6 bytes for x, y, and z information
    // data is stored in the buffer array and used below to get accelerometer data for each axis
    Accel_X = (int16_t)(buffer[0] << 8 | buffer[1]); // shift the upper byte 8 bits then add on the lower byte (same for y,z)
    Accel_Y = (int16_t)(buffer[2] << 8 | buffer[3]);
    Accel_Z = (int16_t)(buffer[4] << 8 | buffer[5]);
}
void MPU6050_Read_Gyro() { // reads angular velocity
    uint8_t buffer[6]; // create a 6 byte array (each axis needs two bytes)

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, buffer, 6, HAL_MAX_DELAY); // read 6 bytes for x, y, and z information
    // data is stored in the buffer array and used below to get accelerometer data for each axis
    Gyro_X = (int16_t)(buffer[0] << 8 | buffer[1]); // shift the upper byte 8 bits then add on the lower byte (same for y,z)
    Gyro_Y = (int16_t)(buffer[2] << 8 | buffer[3]);
    Gyro_Z = (int16_t)(buffer[4] << 8 | buffer[5]);
}
/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
