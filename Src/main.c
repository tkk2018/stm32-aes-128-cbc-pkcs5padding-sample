/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "crypto.h"
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
uint8_t IV[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

uint8_t KEY[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 
                 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
                 
int32_t status;
                 
uint8_t * padded_plain_text;
uint16_t * padded_plain_text_len;
uint8_t cipher_buffer[128];
uint8_t * cipher_text;
uint32_t cipher_text_len;

uint8_t decrypted_buffer[128];                 
uint8_t * decrypted_text;
uint32_t decrypted_text_len;
uint8_t * unpadded_decrypted_text;
uint16_t unpadded_decrypted_text_len;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void aes_128_pkcs5padding(uint8_t * in, uint16_t len, uint8_t ** out, uint16_t * out_len);
void aes_128_pkcs5unpadding(uint8_t * in, uint16_t len, uint8_t ** out, uint16_t * out_len);
int32_t STM32_AES_CBC_Encrypt(uint8_t* InputMessage,
                        uint32_t InputMessageLength,
                        uint8_t  *AES128_Key,
                        uint8_t  *InitializationVector,
                        uint32_t  IvLength,
                        uint8_t  *OutputMessage,
                        uint32_t *OutputMessageLength);
int32_t STM32_AES_CBC_Decrypt(uint8_t* InputMessage,
                        uint32_t InputMessageLength,
                        uint8_t  *AES128_Key,
                        uint8_t  *InitializationVector,
                        uint32_t  IvLength,
                        uint8_t  *OutputMessage,
                        uint32_t *OutputMessageLength);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void aes_128_pkcs5padding(uint8_t * in, uint16_t len, uint8_t ** out, uint16_t * out_len) 
{
  int blockSize = 16;
  int mod = len % blockSize;
  uint8_t pad = blockSize - mod;
  *out_len = len + blockSize - mod;
  *out = malloc(*out_len * sizeof(uint8_t));
  if (NULL == out) 
  {
    return;
  }
  memcpy(*out, in, len);
  for (int i = len; i < *out_len; i++) 
  {
    (*out)[i] = pad;
  }
}

void aes_128_pkcs5unpadding(uint8_t * in, uint16_t len, uint8_t ** out, uint16_t * out_len)
{
  uint8_t remove_len = in[len - 1];
  *out_len = len - remove_len;
  *out = malloc(*out_len * sizeof(uint8_t));
  memcpy(*out, in, *out_len);
}

int32_t STM32_AES_CBC_Encrypt(uint8_t* InputMessage,
                        uint32_t InputMessageLength,
                        uint8_t  *AES128_Key,
                        uint8_t  *InitializationVector,
                        uint32_t  IvLength,
                        uint8_t  *OutputMessage,
                        uint32_t *OutputMessageLength)
{
  AESCBCctx_stt AESctx;

  uint32_t error_status = AES_SUCCESS;

  int32_t outputLength = 0;

  /* Set flag field to default value */
  AESctx.mFlags = E_SK_DEFAULT;

  /* Set key size to 16 (corresponding to AES-128) */
  AESctx.mKeySize = 16;

  /* Set iv size field to IvLength*/
  AESctx.mIvSize = IvLength;

  /* Initialize the operation, by passing the key.
   * Third parameter is NULL because CBC doesn't use any IV */
  error_status = AES_CBC_Encrypt_Init(&AESctx, AES128_Key, InitializationVector );

  /* check for initialization errors */
  if (error_status == AES_SUCCESS)
  {
    /* Encrypt Data */
    error_status = AES_CBC_Encrypt_Append(&AESctx,
                                          InputMessage,
                                          InputMessageLength,
                                          OutputMessage,
                                          &outputLength);

    if (error_status == AES_SUCCESS)
    {
      /* Write the number of data written*/
      *OutputMessageLength = outputLength;
      /* Do the Finalization */
      error_status = AES_CBC_Encrypt_Finish(&AESctx, OutputMessage + *OutputMessageLength, &outputLength);
      /* Add data written to the information to be returned */
      *OutputMessageLength += outputLength;
    }
  }

  return error_status;
}

int32_t STM32_AES_CBC_Decrypt(uint8_t* InputMessage,
                        uint32_t InputMessageLength,
                        uint8_t  *AES128_Key,
                        uint8_t  *InitializationVector,
                        uint32_t  IvLength,
                        uint8_t  *OutputMessage,
                        uint32_t *OutputMessageLength)
{
  AESCBCctx_stt AESctx;

  uint32_t error_status = AES_SUCCESS;

  int32_t outputLength = 0;

  /* Set flag field to default value */
  AESctx.mFlags = E_SK_DEFAULT;

  /* Set key size to 16 (corresponding to AES-128) */
  AESctx.mKeySize = 16;

  /* Set iv size field to IvLength*/
  AESctx.mIvSize = IvLength;

  /* Initialize the operation, by passing the key.
   * Third parameter is NULL because CBC doesn't use any IV */
  error_status = AES_CBC_Decrypt_Init(&AESctx, AES128_Key, InitializationVector );

  /* check for initialization errors */
  if (error_status == AES_SUCCESS)
  {
    /* Decrypt Data */
    error_status = AES_CBC_Decrypt_Append(&AESctx,
                                          InputMessage,
                                          InputMessageLength,
                                          OutputMessage,
                                          &outputLength);

    if (error_status == AES_SUCCESS)
    {
      /* Write the number of data written*/
      *OutputMessageLength = outputLength;
      /* Do the Finalization */
      error_status = AES_CBC_Decrypt_Finish(&AESctx, OutputMessage + *OutputMessageLength, &outputLength);
      /* Add data written to the information to be returned */
      *OutputMessageLength += outputLength;
    }
  }

  return error_status;
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  status = AES_SUCCESS;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  __CRC_CLK_ENABLE();
  char plain_text[] = "HelloWorld";
  int len = strlen(plain_text);
  uint8_t * padded_plain_text;
  uint16_t padded_plain_text_len = 0;
  
  aes_128_pkcs5padding((uint8_t *)plain_text, strlen(plain_text), &padded_plain_text, &padded_plain_text_len);
  status = STM32_AES_CBC_Encrypt(padded_plain_text, padded_plain_text_len, KEY, IV, sizeof(IV), cipher_buffer, &cipher_text_len);
 
  if (status == AES_SUCCESS) {
    cipher_text = malloc(cipher_text_len);
    memcpy(cipher_text, cipher_buffer, cipher_text_len);
    
    status = STM32_AES_CBC_Decrypt(cipher_text, cipher_text_len, KEY, IV, sizeof(IV), decrypted_buffer, &decrypted_text_len);
    
    if (status == AES_SUCCESS) {
      decrypted_text = malloc(decrypted_text_len);
      memcpy(decrypted_text, decrypted_buffer, decrypted_text_len);
      
      aes_128_pkcs5unpadding(decrypted_text, decrypted_text_len, &unpadded_decrypted_text, &unpadded_decrypted_text_len);
    }
  }
  
  free(padded_plain_text);
  free(cipher_text);
  free(decrypted_text);
  free(unpadded_decrypted_text);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

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

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
