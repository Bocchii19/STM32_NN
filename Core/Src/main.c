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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RED_Min 0
#define RED_Max 600
#define GREEN_Min 0
#define GREEN_Max 600
#define BLUE_Min 0
#define BLUE_Max 600
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define S0_Pin GPIO_PIN_0
#define S1_Pin GPIO_PIN_1
#define S2_Pin GPIO_PIN_2
#define S3_Pin GPIO_PIN_3
#define OUT_Pin GPIO_PIN_4
#define BUTTON GPIO_PIN_5

#define S0_GPIO_Port GPIOA
#define S1_GPIO_Port GPIOA
#define S2_GPIO_Port GPIOA
#define S3_GPIO_Port GPIOA
#define OUT_GPIO_Port GPIOA
#define BUTTON_Port GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
volatile uint32_t pulse_count = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void TCS3200_Init(void);
void TCS3200_SetFilter(uint8_t filter);
uint32_t TCS3200_ReadFrequency();
uint8_t NormalizedToRGB(uint32_t freq, uint32_t minFreq, uint32_t maxFreq);
void TCS3200_ReadRGB(uint32_t *red, uint32_t *green, uint32_t *blue);
uint32_t* CheckAndStoreRGB();

	// Khai báo các nguyên mẫu hàm
void forward_nn(const float* input, float* output);
void forward_layer(const float* inputs, int n_inputs, const float* weights, int n_neurons,
                   const float* biases, float* outputs, float (*activation)(float));
float relu(float x);
float sigmoid(float x);
void softmax(const float* inputs, int n_inputs, float* outputs);
int check(const float* ouput_nn);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PrintRGB(uint32_t red, uint32_t green, uint32_t blue){
	char buffer[50];
	sprintf(buffer, "R: %lu  G: %lu  B:%lu", red, green, blue);
	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init();
  TCS3200_Init();
  uint32_t rgb[3] = {0, 136, 255};
  float input_nn[3];
  float output_nn[3];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	    input_nn[0] = (float)rgb[0] / 255.0f;
	    input_nn[1] = (float)rgb[1] / 255.0f;
	    input_nn[2] = (float)rgb[2] / 255.0f;

	    forward_nn(input_nn, output_nn);
	    int k = check(output_nn);
	    if (k == 0){
	    	SSD1306_GotoXY(20,20);
	    	SSD1306_Puts("Red", &Font_11x18, 1);
	    	SSD1306_UpdateScreen();
	    }
	    if (k == 1){
	    	SSD1306_GotoXY(20,20);
	    	SSD1306_Puts("Green", &Font_11x18, 1);
	    	SSD1306_UpdateScreen();
	    }
	    if (k == 2){
	    	SSD1306_GotoXY(20,20);
	    	SSD1306_Puts("Blue", &Font_11x18, 1);
	    	SSD1306_UpdateScreen();
	    }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

const float layer1_weights[] = {
-1.000034, 0.004155, -1.034409, -1.027560, -1.032790, -0.007708, -0.004044, 1.057768, 0.002526, -0.974304, 1.134888, 1.140078, -0.977461, 1.150575, -0.009843, 1.081702, 0.964246, 0.004067, -0.001869, -1.000862, -1.024731, -0.000676, -0.021501, -0.672442,
0.643971, -0.007820, 0.691880, 0.702061, 0.678230, -0.014878, -0.000420, -0.453437, -0.013700, 0.609220, -0.499153, -0.501184, 0.606021, -0.528430, 0.006427, -0.467120, -0.373622, -0.008900, 0.000548, 0.642126, 0.679425, -0.009382, -0.016044, 0.280958,
0.608352, -0.009833, 0.630960, 0.644207, 0.639461, -0.001313, 0.000562, -0.359813, -0.004356, 0.561741, -0.447902, -0.447503, 0.565181, -0.429904, -0.011587, -0.394288, -0.283903, -0.014454, -0.018812, 0.598503, 0.630526, -0.003772, -0.000805, 0.152787,
};

// Layer 1 Biases
const float layer1_biases[] = {
0.355653, -0.000430, 0.367235, 0.354171, 0.367850, 0.000000, 0.000000, 0.221959, 0.000000, 0.356736, 0.288363, 0.286460, 0.358947, 0.283901, -0.006070, 0.247893, 0.152289, 0.000000, -0.001120, 0.359816, 0.362563, 0.000000, 0.000000, 0.261204
};

// Layer 2 Weights
const float layer2_weights[] = {
-0.977964, -0.952362, -0.970754, -0.983245, -0.947568, -0.954500, -0.967063, -0.942508, -0.952682, -0.957146, -0.976307, -0.942296, -0.963111, -0.949174, -0.956381, -0.965147, -0.939090, -0.964289, -0.977227, -0.983043, -0.966927, -0.954989, -0.976909, -0.957271,
-0.004724, 0.004561, -0.009111, 0.000555, -0.001146, -0.006371, -0.001408, 0.005455, -0.006713, -0.006045, 0.002585, -0.009571, -0.005784, -0.010664, 0.005972, -0.001788, 0.001867, -0.011293, -0.014591, -0.001409, 0.000653, 0.001608, -0.003952, -0.012540,
-0.993978, -0.988615, -1.003021, -0.986998, -0.990625, -0.977515, -0.980318, -0.987654, -0.989481, -0.982562, -0.982100, -0.980499, -0.976879, -0.985292, -0.984112, -0.983956, -0.981718, -0.974840, -0.983070, -0.997978, -0.993305, -0.990871, -0.981412, -0.988999,
-0.992169, -0.980134, -0.978873, -0.981735, -1.003546, -0.995124, -0.992078, -0.974659, -0.972109, -0.974354, -0.995764, -0.974773, -0.994749, -0.988822, -1.005403, -0.993519, -0.985495, -0.991591, -0.991705, -0.977019, -0.981126, -0.982530, -0.977578, -0.984449,
-0.990112, -0.975291, -0.989336, -0.984617, -0.988878, -0.997253, -0.997109, -0.994210, -0.984836, -0.976541, -0.978432, -0.978198, -0.998636, -0.989421, -0.972820, -0.987584, -0.975363, -0.999032, -0.996662, -0.975241, -0.970538, -0.988034, -0.998338, -0.989443,
-0.017071, -0.008245, 0.003149, -0.005190, 0.013534, 0.017015, -0.011400, 0.000644, -0.009594, -0.001658, 0.000265, 0.013528, 0.001477, -0.009716, 0.012072, 0.000548, -0.022252, 0.020675, -0.008265, 0.001558, 0.006044, -0.016456, -0.000867, 0.013351,
0.005091, 0.010833, -0.000455, -0.017761, 0.005478, 0.009585, 0.001558, -0.000599, -0.018361, 0.006487, -0.006781, -0.000812, 0.001999, -0.004785, -0.010292, 0.005240, -0.000985, -0.000575, 0.005027, 0.003210, 0.015201, -0.002357, 0.005080, -0.002374,
1.024532, 0.989531, 1.032925, 1.006256, 1.024231, 0.986704, 1.028330, 1.002301, 1.004058, 1.013034, 1.028709, 0.979779, 1.024480, 1.009968, 1.024724, 1.008145, 1.008232, 1.016595, 1.015576, 1.029513, 1.025644, 1.015912, 1.020594, 1.038560,
0.024587, 0.012398, -0.001527, 0.014166, -0.004131, -0.003341, -0.008540, 0.009200, 0.011017, 0.019078, -0.033032, 0.002761, -0.004734, -0.004210, 0.004106, 0.006064, -0.003645, -0.008141, 0.001051, 0.004198, -0.010171, 0.008796, 0.001997, -0.026429,
-0.939189, -0.942186, -0.946432, -0.947894, -0.938395, -0.936469, -0.948127, -0.946159, -0.906008, -0.945930, -0.939571, -0.938771, -0.933788, -0.943910, -0.933362, -0.944811, -0.918290, -0.942031, -0.932185, -0.923117, -0.938802, -0.947545, -0.925490, -0.940373,
1.096174, 1.066767, 1.104821, 1.087246, 1.076780, 1.065879, 1.094341, 1.075940, 1.054573, 1.050834, 1.084085, 1.061942, 1.077839, 1.088003, 1.098074, 1.057152, 1.048092, 1.076169, 1.090747, 1.074581, 1.083462, 1.089248, 1.070228, 1.069712,
1.091608, 1.061747, 1.094111, 1.094434, 1.071927, 1.065299, 1.093656, 1.081941, 1.071489, 1.054365, 1.077773, 1.047609, 1.089329, 1.076277, 1.076393, 1.071731, 1.052464, 1.078670, 1.089402, 1.087498, 1.077184, 1.090429, 1.081429, 1.078816,
-0.938709, -0.926159, -0.936881, -0.928310, -0.934837, -0.930198, -0.947545, -0.936190, -0.951743, -0.953751, -0.950158, -0.930011, -0.944200, -0.951424, -0.947322, -0.928068, -0.916346, -0.937094, -0.948283, -0.949215, -0.938106, -0.934070, -0.950714, -0.943177,
1.078475, 1.052996, 1.099559, 1.074296, 1.085068, 1.071081, 1.089251, 1.064202, 1.050381, 1.070359, 1.083037, 1.042940, 1.082509, 1.084478, 1.085936, 1.081869, 1.048278, 1.087533, 1.098415, 1.075363, 1.083926, 1.078777, 1.090015, 1.075791,
0.002167, -0.001000, 0.001209, 0.013213, -0.000683, -0.000971, 0.010349, 0.003685, 0.013428, -0.015831, 0.006723, 0.011399, -0.007143, 0.006549, 0.003624, -0.001474, 0.011737, -0.002339, -0.021087, 0.018054, 0.018015, -0.003858, -0.014829, -0.006041,
1.037678, 1.022021, 1.054318, 1.048194, 1.041433, 1.036852, 1.042539, 1.036615, 1.019543, 1.018127, 1.050834, 1.021467, 1.057269, 1.016126, 1.036312, 1.015947, 0.998391, 1.042581, 1.036444, 1.049245, 1.041138, 1.046263, 1.052071, 1.042414,
0.951848, 0.942258, 0.953528, 0.969106, 0.959511, 0.960546, 0.963093, 0.945859, 0.958053, 0.950018, 0.960913, 0.938410, 0.953153, 0.947221, 0.961750, 0.948390, 0.936266, 0.964054, 0.963048, 0.964604, 0.960818, 0.969174, 0.970996, 0.980364,
-0.023795, 0.012464, 0.005587, 0.005185, -0.004393, 0.003450, 0.006795, 0.012716, 0.002912, -0.007577, -0.006003, -0.017392, -0.000392, 0.001595, -0.006368, -0.004667, 0.014614, -0.016418, 0.004042, 0.014475, -0.016755, -0.007632, 0.014189, 0.004636,
-0.008575, 0.005731, -0.001876, 0.005973, 0.012221, 0.005179, -0.000424, -0.002900, -0.006983, 0.000012, 0.000814, 0.013364, 0.006881, -0.001269, 0.006107, -0.007253, 0.003332, -0.008857, 0.010273, -0.000477, 0.000846, 0.008077, 0.017924, -0.001932,
-0.964535, -0.955855, -0.965593, -0.967863, -0.941611, -0.951006, -0.972186, -0.951514, -0.949830, -0.952031, -0.964373, -0.958403, -0.954907, -0.949880, -0.979188, -0.949190, -0.960972, -0.948668, -0.967677, -0.969856, -0.958674, -0.950303, -0.982456, -0.974523,
-0.977848, -0.968150, -0.984700, -0.981496, -0.976173, -0.998369, -0.970167, -0.997988, -0.974491, -0.976396, -0.965050, -0.965500, -0.980113, -0.978764, -0.987321, -0.974165, -0.975974, -0.988648, -0.981221, -0.993607, -0.996828, -0.993010, -0.972848, -0.983213,
-0.000564, -0.008670, 0.006019, -0.007589, 0.009306, 0.011373, -0.003176, 0.015763, -0.007257, -0.002331, 0.011035, -0.003986, -0.010109, 0.000310, 0.011067, -0.014651, -0.007004, 0.003789, 0.008066, -0.008020, 0.000168, 0.004369, -0.010328, -0.006242,
-0.005252, 0.006386, 0.002290, 0.009113, -0.001689, -0.001574, -0.002545, -0.009686, 0.008210, 0.005115, 0.009872, 0.002293, 0.013762, -0.007923, -0.001220, -0.012876, -0.009290, -0.006074, -0.001947, 0.015458, -0.005963, 0.005661, 0.004013, -0.005040,
-0.730601, -0.727313, -0.726390, -0.726195, -0.727828, -0.727466, -0.739819, -0.730288, -0.713538, -0.730609, -0.719401, -0.721971, -0.727489, -0.720882, -0.737932, -0.725800, -0.717022, -0.729350, -0.736997, -0.728827, -0.724656, -0.744340, -0.738007, -0.747748,
};

// Layer 2 Biases
const float layer2_biases[] = {
-0.240845, -0.322985, -0.233244, -0.256385, -0.295037, -0.270935, -0.240746, -0.287444, -0.325714, -0.308183, -0.266573, -0.340071, -0.264941, -0.280685, -0.248936, -0.290679, -0.354252, -0.265352, -0.238501, -0.251605, -0.274119, -0.273859, -0.255438, -0.257769
};

// Layer 3 Weights
const float layer3_weights[] = {
0.913269, 0.535794, 0.711259, 0.681742, 0.563343, 0.562511, 0.557755, 0.544639, 0.552358, 0.545055, 0.568144, 0.546438, 0.664979, 0.513305, 0.511589, 0.545290, 0.508781, 0.558810, 0.833428, 0.556837, 0.544198, 0.536582, 0.535147, 0.545799,
0.887340, 0.461638, 0.662624, 0.619046, 0.496791, 0.503994, 0.526772, 0.491416, 0.460710, 0.458664, 0.469428, 0.470994, 0.616403, 0.467018, 0.442777, 0.492066, 0.452326, 0.482043, 0.766976, 0.494048, 0.451130, 0.481426, 0.483310, 0.442578,
0.938386, 0.550261, 0.726957, 0.715201, 0.573446, 0.568804, 0.564179, 0.583563, 0.557087, 0.547814, 0.571381, 0.569441, 0.683681, 0.552145, 0.528599, 0.548762, 0.525030, 0.573971, 0.828330, 0.584071, 0.544326, 0.558874, 0.543626, 0.543204,
0.915060, 0.528915, 0.725918, 0.661118, 0.529181, 0.551714, 0.551161, 0.562935, 0.535152, 0.526239, 0.545862, 0.537159, 0.662719, 0.518905, 0.509646, 0.557040, 0.540608, 0.527578, 0.820274, 0.549341, 0.539526, 0.536277, 0.530548, 0.511145,
0.895328, 0.509476, 0.686920, 0.658494, 0.522956, 0.545645, 0.543894, 0.539715, 0.528051, 0.514465, 0.537588, 0.539918, 0.633596, 0.497617, 0.477536, 0.518239, 0.490919, 0.522196, 0.794241, 0.524808, 0.488907, 0.513998, 0.495936, 0.515191,
0.912187, 0.517038, 0.688746, 0.668816, 0.502354, 0.547719, 0.524206, 0.530027, 0.484537, 0.513070, 0.520065, 0.514056, 0.654950, 0.475022, 0.493508, 0.521036, 0.490867, 0.516924, 0.801748, 0.524192, 0.504228, 0.503789, 0.497726, 0.506355,
0.934101, 0.548500, 0.726161, 0.703149, 0.555954, 0.569495, 0.558331, 0.558223, 0.542257, 0.548266, 0.567751, 0.554967, 0.671520, 0.544001, 0.525619, 0.555849, 0.520670, 0.539775, 0.824673, 0.550724, 0.531504, 0.535570, 0.527480, 0.528719,
0.895455, 0.498493, 0.701733, 0.650811, 0.506814, 0.559244, 0.538279, 0.541257, 0.513805, 0.502633, 0.522932, 0.501714, 0.626891, 0.480559, 0.486996, 0.523302, 0.491352, 0.509751, 0.779012, 0.515972, 0.494582, 0.499570, 0.487983, 0.512426,
0.884218, 0.471107, 0.674539, 0.637488, 0.479151, 0.503699, 0.521065, 0.499077, 0.483609, 0.478153, 0.497277, 0.466783, 0.627562, 0.436810, 0.442551, 0.508003, 0.450217, 0.471960, 0.773304, 0.495218, 0.456830, 0.503848, 0.446415, 0.446639,
0.870016, 0.478073, 0.677634, 0.633149, 0.509129, 0.521651, 0.527066, 0.514840, 0.496923, 0.461014, 0.495489, 0.489589, 0.622862, 0.471139, 0.456305, 0.493285, 0.472755, 0.497475, 0.762571, 0.490954, 0.476228, 0.486872, 0.474093, 0.484586,
0.915361, 0.522785, 0.719539, 0.687879, 0.539505, 0.560779, 0.543710, 0.543686, 0.535280, 0.521473, 0.538955, 0.518531, 0.643528, 0.511048, 0.505891, 0.539271, 0.500034, 0.540396, 0.809747, 0.547861, 0.518104, 0.525718, 0.528267, 0.493788,
0.855205, 0.437044, 0.674374, 0.626270, 0.455115, 0.485590, 0.491017, 0.498847, 0.449689, 0.456238, 0.473377, 0.469393, 0.588277, 0.444549, 0.417675, 0.449468, 0.443880, 0.451633, 0.761267, 0.475431, 0.438264, 0.448027, 0.444861, 0.443801,
0.911837, 0.529132, 0.709479, 0.681728, 0.540533, 0.547583, 0.544944, 0.548433, 0.515606, 0.504154, 0.538065, 0.529623, 0.668136, 0.521629, 0.516918, 0.545116, 0.509755, 0.547582, 0.816676, 0.550541, 0.524641, 0.535272, 0.527847, 0.518945,
0.906693, 0.507666, 0.690177, 0.663988, 0.517183, 0.554113, 0.523603, 0.541257, 0.521889, 0.493261, 0.528317, 0.521334, 0.627885, 0.508658, 0.491370, 0.513038, 0.499560, 0.521826, 0.807023, 0.524293, 0.493589, 0.513500, 0.505225, 0.507846,
0.915110, 0.524858, 0.727629, 0.679902, 0.541393, 0.555220, 0.557210, 0.568244, 0.533113, 0.531052, 0.574771, 0.544293, 0.661065, 0.518945, 0.534003, 0.568140, 0.505098, 0.556637, 0.812233, 0.555282, 0.537479, 0.546249, 0.518593, 0.529054,
0.890152, 0.496737, 0.682753, 0.658960, 0.514674, 0.541243, 0.528620, 0.524789, 0.513386, 0.494314, 0.513782, 0.513924, 0.628191, 0.482000, 0.457239, 0.506693, 0.478959, 0.527297, 0.788868, 0.508732, 0.483067, 0.506035, 0.484257, 0.488778,
0.863349, 0.445973, 0.646116, 0.592809, 0.461420, 0.482470, 0.490622, 0.474986, 0.460717, 0.444573, 0.461585, 0.447028, 0.592298, 0.429472, 0.432877, 0.457834, 0.433728, 0.447525, 0.731258, 0.456434, 0.442022, 0.459493, 0.415973, 0.442469,
0.926311, 0.504662, 0.685767, 0.695556, 0.520336, 0.567799, 0.555238, 0.539408, 0.521527, 0.529497, 0.558335, 0.520710, 0.648500, 0.524425, 0.492081, 0.544058, 0.498995, 0.519215, 0.826312, 0.533955, 0.525193, 0.519918, 0.515583, 0.511750,
0.930402, 0.554998, 0.733308, 0.692396, 0.536225, 0.558101, 0.562023, 0.557619, 0.555751, 0.555264, 0.558269, 0.551969, 0.660623, 0.521380, 0.529996, 0.563639, 0.527059, 0.548435, 0.820223, 0.559065, 0.529598, 0.543142, 0.522344, 0.541688,
0.938254, 0.530222, 0.712039, 0.689402, 0.552507, 0.551864, 0.559771, 0.560598, 0.546983, 0.538237, 0.542388, 0.529005, 0.636491, 0.519913, 0.495428, 0.558549, 0.520639, 0.533001, 0.813860, 0.560185, 0.538804, 0.528493, 0.515868, 0.530353,
0.903770, 0.523548, 0.709471, 0.661765, 0.518641, 0.545458, 0.520960, 0.545556, 0.525795, 0.521823, 0.544754, 0.540523, 0.648509, 0.520681, 0.511549, 0.538654, 0.495734, 0.532434, 0.821463, 0.534488, 0.521583, 0.536783, 0.500298, 0.503500,
0.916041, 0.527515, 0.721621, 0.693103, 0.551017, 0.556113, 0.554459, 0.553821, 0.524127, 0.509990, 0.547190, 0.530661, 0.643064, 0.514386, 0.495249, 0.543030, 0.525604, 0.520317, 0.793009, 0.544804, 0.509340, 0.534241, 0.525909, 0.522523,
0.918477, 0.545878, 0.714213, 0.685639, 0.549117, 0.545414, 0.551436, 0.572747, 0.525454, 0.528409, 0.554162, 0.529585, 0.656096, 0.516276, 0.515618, 0.553641, 0.524259, 0.548511, 0.806653, 0.542361, 0.530774, 0.550770, 0.512368, 0.537538,
0.917226, 0.528890, 0.708500, 0.701507, 0.524553, 0.578164, 0.567331, 0.563033, 0.520056, 0.512990, 0.554956, 0.538504, 0.649503, 0.498574, 0.512457, 0.554464, 0.497798, 0.541444, 0.804740, 0.550086, 0.546168, 0.510107, 0.538702, 0.520821,
};

// Layer 3 Biases
const float layer3_biases[] = {
-1.204860, -1.544643, -1.344519, -1.289125, -1.522836, -1.449620, -1.509803, -1.524840, -1.563771, -1.510726, -1.449943, -1.542299, -0.736005, -1.521236, -1.535392, -1.472404, -1.560066, -1.459518, -1.012629, -1.497970, -1.506934, -1.546568, -1.523892, -1.511606
};

// Layer 4 Weights
const float layer4_weights[] = {
0.010348, 0.045596, 0.057966, 0.034409, -0.014032, -0.029192, 0.051519, 0.177874, 0.051122, 0.239135, 0.004237, 0.047064, -0.019755, -0.000310, 0.036990, 0.175784,
-0.265048, -0.282979, -0.282177, -0.279422, -0.271747, -0.286740, -0.266301, 0.353704, -0.283595, 0.349810, -0.281230, -0.275144, -0.295889, -0.283685, -0.275550, 0.348529,
-0.070821, -0.058467, -0.050747, -0.056606, -0.067669, -0.084592, -0.034982, 0.209825, -0.035136, 0.247256, -0.067503, -0.033963, -0.101194, -0.062456, -0.025562, 0.204531,
-0.040597, -0.044382, -0.020545, -0.018247, -0.060118, -0.059132, -0.033609, 0.165180, 0.003912, 0.175607, -0.040420, -0.017346, -0.068544, -0.051161, -0.026487, 0.162850,
-0.261441, -0.243976, -0.236977, -0.251461, -0.266802, -0.253845, -0.258148, 0.340105, -0.243111, 0.336035, -0.243201, -0.245544, -0.256698, -0.231499, -0.241278, 0.322251,
-0.153729, -0.159786, -0.141594, -0.169061, -0.171463, -0.165965, -0.148288, 0.252334, -0.140265, 0.260069, -0.162021, -0.154610, -0.167416, -0.164961, -0.166604, 0.245207,
-0.216283, -0.198306, -0.213756, -0.223983, -0.224513, -0.210454, -0.206086, 0.288749, -0.216942, 0.328258, -0.205207, -0.199136, -0.226241, -0.204674, -0.188137, 0.299154,
-0.241103, -0.225486, -0.228578, -0.212015, -0.246422, -0.243911, -0.215351, 0.323453, -0.233274, 0.339087, -0.239100, -0.218704, -0.255802, -0.238086, -0.233388, 0.329413,
-0.304847, -0.287085, -0.307323, -0.295458, -0.310259, -0.313709, -0.303543, 0.376216, -0.313711, 0.390571, -0.294059, -0.297516, -0.311258, -0.296294, -0.296487, 0.375271,
-0.217914, -0.229478, -0.215358, -0.234568, -0.247989, -0.238286, -0.232155, 0.295823, -0.223857, 0.298421, -0.231855, -0.222261, -0.222389, -0.227157, -0.240854, 0.299180,
-0.162653, -0.176890, -0.157731, -0.175943, -0.154225, -0.169734, -0.158317, 0.252277, -0.136014, 0.250649, -0.149830, -0.178022, -0.185849, -0.166740, -0.169612, 0.248679,
-0.261170, -0.279681, -0.292524, -0.265176, -0.266780, -0.275913, -0.275866, 0.348918, -0.261656, 0.357906, -0.291737, -0.276752, -0.298086, -0.284335, -0.254399, 0.361050,
0.170998, 0.185972, 0.218882, 0.166266, 0.130143, 0.125822, 0.207663, 0.000845, 0.225498, 0.034377, 0.165059, 0.199160, 0.115466, 0.166054, 0.201588, -0.001904,
-0.244240, -0.239509, -0.259257, -0.254482, -0.245852, -0.261003, -0.275260, 0.327845, -0.243334, 0.304855, -0.241238, -0.260507, -0.261075, -0.242144, -0.255794, 0.305054,
-0.262369, -0.275947, -0.281812, -0.281453, -0.263662, -0.265605, -0.285828, 0.319824, -0.281818, 0.322446, -0.265669, -0.279961, -0.259281, -0.277209, -0.264224, 0.329859,
-0.177394, -0.191063, -0.185986, -0.187560, -0.181953, -0.202623, -0.171419, 0.259802, -0.174333, 0.270700, -0.194381, -0.168647, -0.190615, -0.206095, -0.176232, 0.275192,
-0.310038, -0.312492, -0.300441, -0.307190, -0.298109, -0.290612, -0.292233, 0.364772, -0.313477, 0.351343, -0.308990, -0.304464, -0.295496, -0.308267, -0.309070, 0.360953,
-0.175110, -0.165109, -0.172139, -0.171878, -0.187417, -0.190172, -0.172428, 0.250453, -0.171697, 0.255836, -0.158734, -0.154894, -0.172946, -0.175786, -0.169117, 0.241810,
0.064745, 0.066084, 0.133683, 0.072268, 0.067525, 0.035096, 0.099658, 0.079127, 0.125942, 0.095986, 0.084686, 0.085626, 0.033547, 0.072868, 0.073520, 0.086544,
-0.208860, -0.231427, -0.217444, -0.217444, -0.203091, -0.231627, -0.194087, 0.293810, -0.192536, 0.299964, -0.218287, -0.203585, -0.216252, -0.227561, -0.186490, 0.299053,
-0.233541, -0.250484, -0.214894, -0.237452, -0.233332, -0.223827, -0.242661, 0.303794, -0.214954, 0.305105, -0.233577, -0.253658, -0.220600, -0.222947, -0.261878, 0.298778,
-0.286328, -0.263249, -0.250804, -0.273322, -0.255779, -0.275577, -0.293039, 0.346063, -0.272705, 0.353297, -0.285357, -0.265840, -0.273028, -0.281448, -0.267481, 0.351843,
-0.225299, -0.256529, -0.259740, -0.254723, -0.253067, -0.241791, -0.264847, 0.311848, -0.269408, 0.332879, -0.268680, -0.261072, -0.252463, -0.252950, -0.261695, 0.315886,
-0.258970, -0.238380, -0.234600, -0.219763, -0.244969, -0.236586, -0.223126, 0.307003, -0.212818, 0.294109, -0.240604, -0.263787, -0.234627, -0.228886, -0.249354, 0.306809,
};

// Layer 4 Biases
const float layer4_biases[] = {
1.347267, 1.347538, 1.342754, 1.347336, 1.300633, 1.294787, 1.364140, -1.269043, 1.310920, -1.272810, 1.336237, 1.370207, 1.286474, 1.330343, 1.370780, -1.278986
};

// Layer 5 Weights
const float output_weights[] = {
0.491593, 0.286852, -0.888003,
0.480447, 0.278488, -0.893616,
0.449433, 0.280336, -0.884925,
0.491023, 0.288876, -0.883289,
0.475922, 0.252246, -0.870545,
0.465721, 0.233341, -0.846295,
0.495788, 0.307898, -0.924619,
-0.771749, -0.190709, 0.957530,
0.442715, 0.284792, -0.872831,
-0.811963, -0.170090, 0.971072,
0.482242, 0.274759, -0.870472,
0.508888, 0.317137, -0.932261,
0.452927, 0.217646, -0.840118,
0.467070, 0.256742, -0.876104,
0.513034, 0.320380, -0.933255,
-0.788683, -0.206095, 0.960171,
};

// Layer 5 Biases
const float output_biases[] = {
-0.324474, -0.071484, 0.677330
};

void forward_nn(const float* input, float* output) {
    float layer1_output[24];
    float layer2_output[24];
    float layer3_output[24];
    float layer4_output[16];
    float final_output[3];

    forward_layer(input, 3, layer1_weights, 24, layer1_biases, layer1_output, relu);
    forward_layer(layer1_output, 24, layer2_weights, 24, layer2_biases, layer2_output, sigmoid);
    forward_layer(layer2_output, 24, layer3_weights, 24, layer3_biases, layer3_output, sigmoid);
    forward_layer(layer3_output, 24, layer4_weights, 16, layer4_biases, layer4_output, sigmoid);
    forward_layer(layer4_output, 16, output_weights, 3, output_biases, final_output, NULL);
    softmax(final_output, 3, output);
}

// Hàm kích hoạt ReLU
float relu(float x) {
    return (x > 0) ? x : 0;
}

// Hàm kích hoạt Sigmoid
float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}

// Hàm Softmax
void softmax(const float* inputs, int n_inputs, float* outputs) {
    float max_input = inputs[0];
    for (int i = 1; i < n_inputs; i++) {
        if (inputs[i] > max_input) {
            max_input = inputs[i];
        }
    }

    float sum_exp = 0.0f;
    for (int i = 0; i < n_inputs; i++) {
        outputs[i] = expf(inputs[i] - max_input);
        sum_exp += outputs[i];
    }

    for (int i = 0; i < n_inputs; i++) {
        outputs[i] /= sum_exp;
    }
}

// Hàm forward propagation cho từng lớp
void forward_layer(const float* inputs, int n_inputs, const float* weights, int n_neurons,
                   const float* biases, float* outputs, float (*activation)(float)) {
    for (int i = 0; i < n_neurons; i++) {
        outputs[i] = biases[i];
        for (int j = 0; j < n_inputs; j++) {
            outputs[i] += inputs[j] * weights[i * n_inputs + j];
        }
        if (activation != NULL) {
            outputs[i] = activation(outputs[i]);
        }
    }
}

void TCS3200_Init(void){
	HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, GPIO_PIN_RESET);
}

void TCS3200_SetFilter(uint8_t filter) {
    switch (filter) {
        case 0: // Red
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_RESET);
            break;
        case 1: // Green
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_SET);
            break;
        case 2: // Blue
            HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, GPIO_PIN_SET);
            break;
        default:
            break;
    }
}
uint32_t TCS3200_ReadFrequency() {
    uint32_t count = 0;
    uint32_t timeout = HAL_GetTick() + 1000; // 1 giây timeout
    while (HAL_GetTick() < timeout) {
        if (HAL_GPIO_ReadPin(OUT_GPIO_Port, OUT_Pin) == GPIO_PIN_SET) {
            count++;
            while (HAL_GPIO_ReadPin(OUT_GPIO_Port, OUT_Pin) == GPIO_PIN_SET);
        }
    }
    return count;
}

uint8_t NormalizeToRGB(uint32_t freq, uint32_t minFreq, uint32_t maxFreq) {
    if (freq < minFreq) freq = minFreq;
    if (freq > maxFreq) freq = maxFreq;
    return (uint8_t)(((freq - minFreq) * 255) / (maxFreq - minFreq));
}

void TCS3200_ReadRGB(uint32_t *red, uint32_t *green, uint32_t *blue) {
    uint32_t rawRed, rawGreen, rawBlue;

    // Đọc giá trị tần số thô từ cảm biến
    TCS3200_SetFilter(0); // Chọn bộ lọc Red
    HAL_Delay(50);       // Chờ ổn định
    rawRed = TCS3200_ReadFrequency();

    TCS3200_SetFilter(1); // Chọn bộ lọc Green
    HAL_Delay(50);
    rawGreen = TCS3200_ReadFrequency();

    TCS3200_SetFilter(2); // Chọn bộ lọc Blue
    HAL_Delay(50);
    rawBlue = TCS3200_ReadFrequency();

    // Chuẩn hóa giá trị tần số thành thang đo 0-255
    *red = NormalizeToRGB(rawRed, RED_Min, RED_Max);
    *green = NormalizeToRGB(rawGreen, GREEN_Min, GREEN_Max);
    *blue = NormalizeToRGB(rawBlue, BLUE_Min, BLUE_Max);
}
int check(const float* output_nn){
	int status = 0;
	float max = output_nn[0];
	for(int i = 0; i < 3; i ++){
		if(output_nn[i] > max){
			max = output_nn[i];
			status = i;
		}
	}
	return status;
}

uint32_t* CheckAndStoreRGB() {
    static uint32_t rgb[3]; // Mảng RGB (static để không bị xóa sau khi hàm kết thúc)
    static uint8_t buttonPressed = 0; // Trạng thái nút nhấn
    uint32_t red, green, blue;
	TCS3200_ReadRGB(&red, &green, &blue);
    // Kiểm tra trạng thái nút
    if (HAL_GPIO_ReadPin(BUTTON_Port, BUTTON) == GPIO_PIN_SET) {
        if (buttonPressed == 0) { // Nếu nút vừa được nhấn
            buttonPressed = 1; // Đánh dấu nút đã được nhấn

            // Đọc giá trị RGB từ cảm biến


            // Lưu vào mảng RGB
            rgb[0] = red;
            rgb[1] = green;
            rgb[2] = blue;
        }
    } else {
        buttonPressed = 0; // Reset trạng thái khi nút được thả
    }

    return rgb; // Trả về con trỏ tới mảng RGB
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
