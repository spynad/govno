/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SEQ_LEN 8
#define BLINK_DELAY 100
#define LIGHT_DELAY 1000
#define WAIT 20
#define INCORRECT_TIMEOUT 10000
#define DURATION 30000
#define BUF_SIZE 1024

/* USER CODE END PD */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int interrupt_enable;

    uint32_t pmask;
} it_status;

typedef enum {
	NOT_PRESSED,
	PRESSED
} button_state;


typedef enum {

	NEW_PASS,
	CURRENT_PASS,
	APPROVAL
} console_mode;

struct ring_buffer_proto {
  char data[BUF_SIZE];
  uint8_t head;
  uint8_t tail;
  bool empty;
};

typedef struct ring_buffer_proto ring_buffer;


/* USER CODE END PTD */



/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static char	correct_seq_c[SEQ_LEN];
static char approval_seq[SEQ_LEN];
static int cur_index 				= 0;
static int incorrect_presses 		= 0;
static it_status status;
static char el[2];
console_mode mode 					= CURRENT_PASS;

static char APPROVAL_MSG[] 			= "Do you want to set this new password (y to accept, anything otherwise)? ";
static char WRONG_PASSWORD[] 		= "3 incorrect attempts, sleeping for 10 sec.";
static bool transmit_busy = false;
static button_state current_state = NOT_PRESSED;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern void initialise_monitor_handles(void);

static void buf_init(ring_buffer *buf) {
  buf->head = 0;
  buf->tail = 0;
  buf->empty = true;
}

static void buf_push(ring_buffer *buf, char *el) {
  uint64_t size = strlen(el);

  if (buf->head + size + 1 > BUF_SIZE) {
    buf->head = 0;
  }

  strcpy(&buf->data[buf->head], el);
  //    buf->data[buf->head] = el;
  buf->head += size + 1;

  if (buf->head == BUF_SIZE) {
    buf->head = 0;
  }

  buf->empty = false;
}

static bool buf_pop(ring_buffer *buf, /* out */ char *el) {
  if (buf->empty) {
    return false;
  }

  uint64_t size = strlen(&buf->data[buf->tail]);

  strcpy(el, &buf->data[buf->tail]);
  buf->tail += size + 1;

  if (buf->tail == BUF_SIZE || buf->tail == '\0') {
    buf->tail = 0;
  }

  if (buf->tail == buf->head) {
    buf->empty = true;
  }

  return true;
}

static ring_buffer r_buff;
static ring_buffer ring_buffer_tx;

void enable_interrupt(it_status *status) {
	HAL_NVIC_EnableIRQ(USART6_IRQn);
	status->interrupt_enable = true;
}

void disable_interrupt(it_status *status) {
	HAL_UART_AbortReceive(&huart6);
	HAL_NVIC_DisableIRQ(USART6_IRQn);
    status->interrupt_enable = false;
}

void transmit_uart(const it_status *status, char *buf, size_t size) {
  if (status->interrupt_enable) {
    if (transmit_busy) {
      buf_push(&ring_buffer_tx, buf);
    } else {
      HAL_UART_Transmit_IT(&huart6, buf, size);
      transmit_busy = true;
    }
    return;
  }
  HAL_UART_Transmit(&huart6, buf, size, 100);
}


void receive_uart(const it_status *status) {
  if (status->interrupt_enable) {
    HAL_UART_Receive_IT(&huart6, el, sizeof(char));
    return;
  }
  HAL_StatusTypeDef stat = HAL_UART_Receive(&huart6, el, sizeof(char), 0);
  switch (stat) {
  case HAL_OK: {
    buf_push(&r_buff, el);
    transmit_uart(status, el, 1);
    break;
  }
  case HAL_ERROR:
  case HAL_BUSY:
  case HAL_TIMEOUT:
    break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart6.Instance) {
  buf_push(&r_buff, el);
  transmit_uart(&status, el, 1);}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart6.Instance) {
  char buf[1024];
  if (buf_pop(&ring_buffer_tx, buf)) {
    HAL_UART_Transmit_IT(&huart6, buf, strlen(buf));
  } else {
    transmit_busy = false;
  }
	}
}

button_state get_button_state() {
	int start_time;
	int hal_button_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);
	button_state current_state = NOT_PRESSED;

	if(!hal_button_state){
		start_time = HAL_GetTick();
	}

	while (!hal_button_state){
	 if (HAL_GetTick() - start_time >= WAIT){
		current_state = PRESSED;
	 }

	  hal_button_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);
	}

	return current_state;
}

bool is_allowed_symbol(char symbol) {
	return symbol < 127 && symbol > 20;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//initialise_monitor_handles();

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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t tickstart = HAL_GetTick();

  //disable_interrupt(&status);
  enable_interrupt(&status);
  buf_init(&r_buff);
  transmit_uart(&status, "\f", 1);

  while (1)
  {
	  if (get_button_state() == PRESSED) {
		  if (status.interrupt_enable) {
			  disable_interrupt(&status);
		  } else {
			  enable_interrupt(&status);
		  }
	  }

	  if (HAL_GetTick() - tickstart > DURATION) {
		  cur_index = 0;
		  tickstart = HAL_GetTick();
	  }

	  receive_uart(&status);

	  char c[2];


	  if (!buf_pop(&r_buff, c)) {
		  continue;
	  }

	  if (c[0] == '+') {
		  cur_index = 0;
		  mode = NEW_PASS;
		  //transmit_uart(&status, "\r\n", 2);
		  continue;
	  }

	  if (c[0] == '\r' && mode == NEW_PASS) {
		  cur_index = 0;
		  mode = APPROVAL;
		  transmit_uart(&status, "\r\n", 2);
		  transmit_uart(&status, APPROVAL_MSG, sizeof(APPROVAL_MSG));
		  continue;
	  }

	  //HAL_UART_Transmit( &huart6, (uint8_t *) s, sizeof( s ), 10 );
	  //HAL_Delay( 1000 );
	  if (mode == CURRENT_PASS && is_allowed_symbol(c[0])) {
		  if (correct_seq_c[cur_index] == c[0]) {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			  HAL_Delay(BLINK_DELAY);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			  cur_index++;
		  } else {
			  incorrect_presses++;
			  if (incorrect_presses == 3) {
				  transmit_uart(&status, "\r\n", 2);
				  transmit_uart(&status, WRONG_PASSWORD, sizeof(WRONG_PASSWORD));
				  for (size_t i = 0; i < 3; ++i) {
					  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
					  HAL_Delay(LIGHT_DELAY);
					  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
					  HAL_Delay(LIGHT_DELAY);
				  }
				  incorrect_presses = 0;
				  HAL_Delay(INCORRECT_TIMEOUT);
			  } else {
				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
				  HAL_Delay(BLINK_DELAY);
				  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			  }
			  cur_index = 0;
		  }

		  if (cur_index == SEQ_LEN || correct_seq_c[cur_index] == '\0') {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			  HAL_Delay(LIGHT_DELAY * 3);
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			  cur_index = 0;
		  }
	  } else {
		  if (is_allowed_symbol(c[0]) && mode == NEW_PASS) {
			  approval_seq[cur_index] = c[0];
			  cur_index++;

			  if (cur_index == SEQ_LEN) {
				  mode = APPROVAL;
			  }
		  } else {
			  if (c[0] == 'y') {
				  memcpy(correct_seq_c, approval_seq, SEQ_LEN);
			  }

			  mode = CURRENT_PASS;
			  transmit_uart(&status, "\r\n", 2);
		  }
	  }


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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
