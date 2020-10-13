/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define tim2_ch1_pwm4_Pin GPIO_PIN_0
#define tim2_ch1_pwm4_GPIO_Port GPIOA
#define tim2_ch2_pwm5_Pin GPIO_PIN_1
#define tim2_ch2_pwm5_GPIO_Port GPIOA
#define APC_TX_Pin GPIO_PIN_2
#define APC_TX_GPIO_Port GPIOA
#define APC_RX_2_Pin GPIO_PIN_3
#define APC_RX_2_GPIO_Port GPIOA
#define SRF04_Pin GPIO_PIN_5
#define SRF04_GPIO_Port GPIOA
#define tim3_ch1_pwm0_Pin GPIO_PIN_6
#define tim3_ch1_pwm0_GPIO_Port GPIOA
#define tim3_ch2_pwm1_Pin GPIO_PIN_7
#define tim3_ch2_pwm1_GPIO_Port GPIOA
#define tim3_ch3_pwm2_Pin GPIO_PIN_0
#define tim3_ch3_pwm2_GPIO_Port GPIOB
#define tim3_ch4_pwm3_Pin GPIO_PIN_1
#define tim3_ch4_pwm3_GPIO_Port GPIOB
#define Tim1_CH1_RC1_Pin GPIO_PIN_8
#define Tim1_CH1_RC1_GPIO_Port GPIOA
#define Tim1_CH2_RC2_Pin GPIO_PIN_9
#define Tim1_CH2_RC2_GPIO_Port GPIOA
#define Tim1_CH3_RC3_Pin GPIO_PIN_10
#define Tim1_CH3_RC3_GPIO_Port GPIOA
#define Tim1_CH4_RC4_Pin GPIO_PIN_11
#define Tim1_CH4_RC4_GPIO_Port GPIOA
#define Tim4_CH1_RC5_Pin GPIO_PIN_6
#define Tim4_CH1_RC5_GPIO_Port GPIOB
#define Tim4_CH2_RC6_Pin GPIO_PIN_7
#define Tim4_CH2_RC6_GPIO_Port GPIOB
#define I2C_1_SRF02___gy955_SCL_Pin GPIO_PIN_8
#define I2C_1_SRF02___gy955_SCL_GPIO_Port GPIOB
#define I2C_1_SRF02___gy955_SDA_Pin GPIO_PIN_9
#define I2C_1_SRF02___gy955_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
