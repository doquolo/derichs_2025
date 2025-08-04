/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "u8g2/u8g2.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define bitmap_width 64
#define bitmap_height 64
static unsigned char logoTruong[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0xfc, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0,
		0x07, 0xe0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x1e, 0x00,
		0x00, 0x00, 0x00, 0x0e, 0x88, 0x15, 0x73, 0x00, 0x00, 0x00, 0x00, 0x03,
		0xce, 0xd6, 0xc1, 0x00, 0x00, 0x00, 0xc0, 0xe0, 0xc3, 0x5d, 0x01, 0x03,
		0x00, 0x00, 0x60, 0x30, 0xd7, 0x74, 0x19, 0x06, 0x00, 0x00, 0x10, 0x74,
		0x0d, 0xd0, 0x31, 0x0c, 0x00, 0x00, 0x08, 0xec, 0xf0, 0x0f, 0x48, 0x10,
		0x00, 0x00, 0xc4, 0x09, 0x07, 0xe0, 0x68, 0x21, 0x00, 0x00, 0x42, 0xcb,
		0x00, 0x00, 0xa3, 0x4e, 0x00, 0x00, 0x51, 0x32, 0x00, 0x00, 0xcc, 0x8f,
		0x00, 0x80, 0xf0, 0x09, 0x00, 0x00, 0x10, 0x89, 0x01, 0xc0, 0x78, 0xf6,
		0x1f, 0xf8, 0x6f, 0x17, 0x03, 0x40, 0x1c, 0x13, 0x80, 0x01, 0xc8, 0x1c,
		0x02, 0x20, 0xbc, 0x10, 0x00, 0x00, 0x88, 0x75, 0x04, 0x30, 0xd3, 0x10,
		0x80, 0x01, 0x08, 0x5b, 0x0c, 0x10, 0x5e, 0x10, 0x80, 0x01, 0x08, 0xe2,
		0x08, 0x98, 0x23, 0x10, 0x80, 0x03, 0x08, 0xd4, 0x09, 0x88, 0x17, 0x10,
		0xc0, 0x03, 0x08, 0xe8, 0x10, 0x88, 0x11, 0x10, 0xe0, 0x03, 0x08, 0x48,
		0x10, 0x4c, 0x0a, 0x10, 0xe0, 0x07, 0x08, 0x10, 0x32, 0xe4, 0x08, 0x10,
		0xe0, 0x07, 0x18, 0x10, 0x27, 0x24, 0x05, 0x10, 0xf0, 0x0f, 0x18, 0xa0,
		0x24, 0x04, 0x04, 0x10, 0x70, 0x0e, 0x18, 0x20, 0x26, 0x06, 0x04, 0x10,
		0x70, 0x0e, 0x18, 0xa0, 0x67, 0xf2, 0x01, 0x10, 0xf8, 0x1f, 0x18, 0x20,
		0x4e, 0xd2, 0x03, 0x10, 0xf8, 0x1f, 0x18, 0x40, 0x41, 0xf2, 0x02, 0x10,
		0xfc, 0x3f, 0x18, 0x40, 0x43, 0x42, 0x02, 0x10, 0xec, 0x37, 0x18, 0x40,
		0x5e, 0xf2, 0x02, 0x14, 0xee, 0x37, 0x38, 0x40, 0x5f, 0x62, 0x02, 0x14,
		0xfc, 0x3f, 0x38, 0x40, 0x41, 0xf2, 0x02, 0x14, 0xf8, 0x1f, 0x38, 0x40,
		0x4f, 0xc2, 0x02, 0x14, 0xe0, 0x07, 0x38, 0x40, 0x46, 0xb2, 0x03, 0x14,
		0xe0, 0x07, 0x38, 0xc0, 0x47, 0x12, 0x01, 0x14, 0xe0, 0x07, 0x38, 0x20,
		0x40, 0xe6, 0x04, 0x14, 0xe0, 0x07, 0x38, 0x20, 0x60, 0x84, 0x05, 0x14,
		0xe0, 0x07, 0x38, 0xa0, 0x27, 0xe4, 0x05, 0x14, 0xe0, 0x07, 0x38, 0x20,
		0x25, 0x04, 0x89, 0x05, 0x08, 0x10, 0x30, 0x10, 0x23, 0xcc, 0xcb, 0xfc,
		0x37, 0xec, 0x3f, 0xd0, 0x31, 0x88, 0xf3, 0x00, 0x00, 0x00, 0x00, 0xa8,
		0x13, 0x88, 0xf7, 0xf8, 0x7f, 0xfe, 0x1f, 0xe8, 0x10, 0x98, 0xe3, 0x03,
		0xfc, 0x3f, 0x00, 0x34, 0x09, 0x10, 0x4c, 0x3f, 0xe0, 0x07, 0x00, 0xda,
		0x08, 0x30, 0xc7, 0xf8, 0xff, 0xff, 0x01, 0x13, 0x0c, 0x20, 0x82, 0x01,
		0xfe, 0xff, 0x9f, 0xe1, 0x04, 0x40, 0x00, 0x03, 0x00, 0xfc, 0xdf, 0x00,
		0x02, 0xc0, 0x00, 0x06, 0x00, 0xfc, 0x7f, 0x00, 0x03, 0x80, 0x01, 0x08,
		0x00, 0xfe, 0x1f, 0x80, 0x01, 0x00, 0x01, 0x31, 0xc0, 0xff, 0x0f, 0x81,
		0x00, 0x00, 0x82, 0xc1, 0xfc, 0xff, 0x83, 0x41, 0x00, 0x00, 0x84, 0x01,
		0xff, 0xff, 0x80, 0x20, 0x00, 0x00, 0x08, 0x00, 0xf0, 0x0f, 0x00, 0x10,
		0x00, 0x00, 0x30, 0xc0, 0x04, 0x06, 0x01, 0x0c, 0x00, 0x00, 0x60, 0x40,
		0x05, 0xe6, 0x02, 0x06, 0x00, 0x00, 0xc0, 0x60, 0xef, 0xde, 0x02, 0x03,
		0x00, 0x00, 0x00, 0xe3, 0xef, 0xe6, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x0e,
		0xaf, 0x7f, 0x71, 0x00, 0x00, 0x00, 0x00, 0x70, 0xa0, 0x09, 0x0e, 0x00,
		0x00, 0x00, 0x00, 0xc0, 0x07, 0xe0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
		0xfc, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00 };

// Structure example to send data
#pragma pack(1) // Disable padding
struct CONTROLLER_READOUT {
	struct {
		uint8_t A2, A3;
		uint8_t B8, B13, A4;
		uint8_t A0;
	} LS; // left shoulder
	struct {
		uint8_t A8, A11, B2, B12;
	} LD; // left dpad

	struct {
		uint8_t C13, A1;
		uint8_t A5, A12, B9;
		uint8_t PB11;
	} RS; // right shoulder
	struct {
		uint8_t B5, B4, A15, B3;
	} RD; // right dpad

	struct {
		uint8_t C15, C14;
	} ALT; // alternate button

	struct {
		uint8_t BTN;
		uint16_t VALUE[2];
	} JOY; // joystick

	struct {
		uint8_t BTN;
		uint8_t VALUE;
	} ENC; // encoder

} controller;
#pragma pack()

struct {
	struct {
		uint32_t A2, A3;
		uint32_t B8, B13, A4;
		uint32_t A0;
	} LS; // left shoulder
	struct {
		uint32_t A8, A11, B2, B12;
	} LD; // left dpad

	struct {
		uint32_t C13, A1;
		uint32_t A5, A12, B9;
		uint32_t PB11;
	} RS; // right shoulder
	struct {
		uint32_t B5, B4, A15, B3;
	} RD; // right dpad

	struct {
		uint32_t C15, C14;
	} ALT; // alternate button

	struct {
		uint32_t BTN;
	} JOY; // joystick

	struct {
		uint32_t BTN;
	} ENC; // encoder

} controller_debounced;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

uint8_t u8x8_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr) {
	switch (msg) {
	case U8X8_MSG_DELAY_MILLI:
		HAL_Delay(arg_int);
		break;
	}
	return 1;
}

uint8_t u8x8_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
	static uint8_t buffer[32]; /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
	static uint8_t buf_idx;
	uint8_t *data;

	switch (msg) {
	case U8X8_MSG_BYTE_SEND:
		data = (uint8_t*) arg_ptr;
		while (arg_int > 0) {
			buffer[buf_idx++] = *data;
			data++;
			arg_int--;
		}
		break;
	case U8X8_MSG_BYTE_START_TRANSFER:
		buf_idx = 0;
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		HAL_I2C_Master_Transmit(&hi2c1, 0x78, buffer, buf_idx, 1000);
		break;
	default:
		return 0;
	}
	return 1;
}

static u8g2_t u8g2;

const uint32_t buttonDebounceTime = 50; // ms
static uint8_t handleButtonDebounce(uint32_t *prev_time, uint32_t now,
		uint8_t newState, uint8_t currentState) {
	if (newState == currentState)
		return currentState;
	else {
		if (now - *prev_time > buttonDebounceTime) {
			*prev_time = now;
			return newState;
		} else
			return currentState;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		uint32_t now = HAL_GetTick(); // Current time in ms

		// Left shoulder
		controller.LS.A2 = handleButtonDebounce(&controller_debounced.LS.A0,
				now, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0), controller.LS.A0);
		controller.LS.A3 = handleButtonDebounce(&controller_debounced.LS.A3,
				now, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3), controller.LS.A3);
		controller.LS.B8 = handleButtonDebounce(&controller_debounced.LS.B8,
				now, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8), controller.LS.B8);
		controller.LS.B13 = handleButtonDebounce(&controller_debounced.LS.B13,
				now, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13), controller.LS.B13);
		controller.LS.A4 = handleButtonDebounce(&controller_debounced.LS.A4,
				now, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4), controller.LS.A4);
		controller.LS.A0 = handleButtonDebounce(&controller_debounced.LS.A2,
				now, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2), controller.LS.A2);

		// Left dpad
		controller.LD.A8 = handleButtonDebounce(&controller_debounced.LD.A8,
				now, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8), controller.LD.A8);
		controller.LD.A11 = handleButtonDebounce(&controller_debounced.LD.A11,
				now, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11), controller.LD.A11);
		controller.LD.B2 = handleButtonDebounce(&controller_debounced.LD.B2,
				now, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2), controller.LD.B2);
		controller.LD.B12 = handleButtonDebounce(&controller_debounced.LD.B12,
				now, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12), controller.LD.B12);

		// Right shoulder
		controller.RS.C13 = handleButtonDebounce(&controller_debounced.RS.C13,
				now, HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13), controller.RS.C13);
		controller.RS.A1 = handleButtonDebounce(&controller_debounced.RS.A1,
				now, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1), controller.RS.A1);
		controller.RS.A5 = handleButtonDebounce(&controller_debounced.RS.A5,
				now, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5), controller.RS.A5);
		controller.RS.A12 = handleButtonDebounce(&controller_debounced.RS.A12,
				now, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12), controller.RS.A12);
		controller.RS.B9 = handleButtonDebounce(&controller_debounced.RS.B9,
				now, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9), controller.RS.B9);
		controller.RS.PB11 = handleButtonDebounce(&controller_debounced.RS.PB11,
				now, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11), controller.RS.PB11);

		// Right dpad
		controller.RD.B5 = handleButtonDebounce(&controller_debounced.RD.B5,
				now, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5), controller.RD.B5);
		controller.RD.B4 = handleButtonDebounce(&controller_debounced.RD.B4,
				now, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4), controller.RD.B4);
		controller.RD.A15 = handleButtonDebounce(&controller_debounced.RD.A15,
				now, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15), controller.RD.A15);
		controller.RD.B3 = handleButtonDebounce(&controller_debounced.RD.B3,
				now, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3), controller.RD.B3);

		// Alternate buttons
		controller.ALT.C15 = handleButtonDebounce(&controller_debounced.ALT.C15,
				now, HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15), controller.ALT.C15);
		controller.ALT.C14 = handleButtonDebounce(&controller_debounced.ALT.C14,
				now, HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14), controller.ALT.C14);

		// Joystick
		controller.JOY.BTN = handleButtonDebounce(&controller_debounced.JOY.BTN,
				now, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14), controller.JOY.BTN);

		// Encoder
		controller.ENC.BTN = handleButtonDebounce(&controller_debounced.ENC.BTN,
				now, HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15), controller.ENC.BTN);
		controller.ENC.VALUE = ((TIM3->CNT) >> 2);
	}
}

#define RX_BUFFER_SIZE 100

uint8_t rx_byte;                      // Temporary byte buffer
uint8_t rx_line_buffer[RX_BUFFER_SIZE]; // Line buffer
volatile uint16_t rx_index = 0;
volatile uint8_t line_received = 0;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rx_byte == '\n' || rx_index >= RX_BUFFER_SIZE - 1) {
            rx_line_buffer[rx_index] = '\0'; // Null-terminate
            u8g2_ClearBuffer(&u8g2); // Clear the buffer before drawing
			u8g2_DrawStr(&u8g2, 80, 10, rx_line_buffer);
			u8g2_SendBuffer(&u8g2); // Transfer the buffer to the display
            rx_index = 0; // Reset for next line
        } else {
            rx_line_buffer[rx_index++] = rx_byte;
        }

        // Continue receiving next byte
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint8_t START_MARKER[] = { 0x7E }; // '~'
const uint8_t END_MARKER[] = { 0x7F };   // DEL (127)
uint8_t controllerBuffer[sizeof(controller)];
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_ADC_Start_DMA(&hadc1, controller.JOY.VALUE, 2);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	/* Initialize Display */
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_i2c,
			u8x8_gpio_and_delay);
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);

	u8g2_ClearBuffer(&u8g2); // Clear the buffer before drawing
	u8g2_DrawXBM(&u8g2, 32, 0, 64, 64, logoTruong);
	u8g2_SendBuffer(&u8g2); // Transfer the buffer to the display
	HAL_Delay(2000);
	u8g2_ClearBuffer(&u8g2); // Clear the buffer before drawing
	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
	u8g2_DrawStr(&u8g2, 80, 10, "No data!");
	u8g2_SendBuffer(&u8g2); // Transfer the buffer to the display
	HAL_Delay(150);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		memcpy(controllerBuffer, &controller, sizeof(controller));
		HAL_UART_Transmit(&huart1, START_MARKER, sizeof(START_MARKER), 1000);
		HAL_UART_Transmit(&huart1, controllerBuffer, sizeof(controllerBuffer),
				1000);
		HAL_Delay(25); // wait for 25ms for next transsmition
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 143;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : RS_C13_Pin ALT_C14_Pin ALT_C15_Pin */
  GPIO_InitStruct.Pin = RS_C13_Pin|ALT_C14_Pin|ALT_C15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LS_A0_Pin RS_A1_Pin LS_A2_Pin LS_A3_Pin
                           LS_A4_Pin RS_A5_Pin LD_A8_Pin LD_A11_Pin
                           RS_A12_Pin RD_A15_Pin */
  GPIO_InitStruct.Pin = LS_A0_Pin|RS_A1_Pin|LS_A2_Pin|LS_A3_Pin
                          |LS_A4_Pin|RS_A5_Pin|LD_A8_Pin|LD_A11_Pin
                          |RS_A12_Pin|RD_A15_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD_B2_Pin RS_B11_Pin LD_B12_Pin RS_B13_Pin
                           JOY_BTN_Pin ENC_BTN_Pin RD_B3_Pin RD_B4_Pin
                           RD_B5_Pin RS_B8_Pin RS_B9_Pin */
  GPIO_InitStruct.Pin = LD_B2_Pin|RS_B11_Pin|LD_B12_Pin|RS_B13_Pin
                          |JOY_BTN_Pin|ENC_BTN_Pin|RD_B3_Pin|RD_B4_Pin
                          |RD_B5_Pin|RS_B8_Pin|RS_B9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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
	while (1) {
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
