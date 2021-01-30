/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_tim_ex.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* basically we want to connect the motor phases to
 * the correct timer channels up here
 * see the timer 2 ISR for what this does
 */
#define PHASE_U1 TIM_CHANNEL_1
#define PHASE_U2 TIM_CHANNEL_1
#define PHASE_V1 TIM_CHANNEL_2
#define PHASE_V2 TIM_CHANNEL_2
#define PHASE_W1 TIM_CHANNEL_3
#define PHASE_W2 TIM_CHANNEL_3
#define CCMR_CHAN1  0b0000000001110000
#define CCMR_CHAN2  0b0111000000000000
#define CCMR_CHAN3  0b0000000001110000
#define PWM_CHAN_U_FORCE_ACTIVE  0b0000000001010000
#define PWM_CHAN_V_FORCE_ACTIVE  0b0101000000000000
#define PWM_CHAN_W_FORCE_ACTIVE  0b0000000001010000
#define PWM_CHAN_U_FORCE_INACTIVE  0b0000000001000000
#define PWM_CHAN_V_FORCE_INACTIVE  0b0100000000000000
#define PWM_CHAN_W_FORCE_INACTIVE  0b0000000001000000
#define PWM_CHAN_U_MODE1  0b0000000001100000
#define PWM_CHAN_V_MODE1  0b0110000000000000
#define PWM_CHAN_W_MODE1  0b0000000001100000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

static void TIM_CCxNChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelNState)
{
  /* we're borrowing this function from stm32f4xx_hal_tim_ex.c because its static for
   * some reason even tho the regular (non complimentary) version of it is public
   * I would just modify in place but Cubemx would overwrite so here we are
   */
  uint32_t tmp;

  tmp = TIM_CCER_CC1NE << (Channel & 0x1FU); /* 0x1FU = 31 bits max shift */

  /* Reset the CCxNE Bit */
  TIMx->CCER &=  ~tmp;

  /* Set or reset the CCxNE Bit */
  TIMx->CCER |= (uint32_t)(ChannelNState << (Channel & 0x1FU)); /* 0x1FU = 31 bits max shift */
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//inline void high_side_on(TIM_HandleTypeDef tim_handle, )

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern DMA_HandleTypeDef hdma_usart2_tx;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim1;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /*
   * read in the state of the hall sensors
   * and pack them into a single variable
   */
  int hall_value = 0;
  /* hall A PA15*/
  hall_value = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
  /* hall B PB3*/
  hall_value = (hall_value << 1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
  /* hall C PB10*/
  hall_value = (hall_value << 1) | HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);

  /*
   * turn off all of the output TIM1 channels need to do this to clear
   * the last state
   */

  /* this is a mask for accessing the OCxM bits that
   * control the timer output state
   */
  uint32_t temp_CCMR1 = 0;
  uint32_t temp_CCMR2 = 0;
  /* store temp reg values, write to them and push at the end */
  temp_CCMR1 = htim1.Instance->CCMR1;
  temp_CCMR2 = htim1.Instance->CCMR2;
  /* clear the output OCxM bits*/
  temp_CCMR1 &= !CCMR_CHAN1;
  temp_CCMR2 &= !CCMR_CHAN2;
  temp_CCMR2 &= !CCMR_CHAN3;

  /* set all channels to their forced inactive state */
  //temp_CCMR1 |= PWM_CHAN1_FORCE_INACTIVE | PWM_CHAN2_FORCE_INACTIVE;
  //temp_CCMR2 |= PWM_CHAN3_FORCE_INACTIVE;
  //temp_CCMR1 |= PWM_CHAN1_FORCE_ACTIVE | PWM_CHAN2_FORCE_ACTIVE;
  //temp_CCMR2 |= PWM_CHAN3_FORCE_ACTIVE;


  /* clear the existing value */
   TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
   TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
   TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
  
   TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCxN_ENABLE);
   TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCxN_ENABLE);
   TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_ENABLE);


  /*
   * Now set all the the output TIM1 pwm phases to what they need to be
   * the table below shows the the relationship between the motor phase
   * connections and the timer channels
   *
   * U1 = channel1
   * U2 = channel1N
   * V1 = channel2
   * V2 = channel2N
   * W1 = channel3
   * W2 = channel3N
   */
  switch(hall_value)
  {
  case 6:
	   /* Hall CBA = 110, Phase W1, V2 = high */
	  TIM_CCxNChannelCmd(htim1.Instance, PHASE_W2, TIM_CCxN_DISABLE);
	  TIM_CCxChannelCmd(htim1.Instance, PHASE_V1, TIM_CCx_DISABLE);
      
      /* W = chan3 set to pwm1 mode */
      temp_CCMR2 |= PWM_CHAN_W_MODE1;
      /* V = chan2 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_V_MODE1;

      temp_CCMR1 |= PWM_CHAN_U_FORCE_INACTIVE;
	  TIM_CCxChannelCmd(htim1.Instance, PHASE_U1, TIM_CCx_DISABLE);

	  break;
  case 4:
	   /* Hall CBA = 100, Phase U1, V2 = high */
	  TIM_CCxNChannelCmd(htim1.Instance, PHASE_U2, TIM_CCxN_DISABLE);
	  TIM_CCxChannelCmd(htim1.Instance, PHASE_V1, TIM_CCx_DISABLE);

      /* U = chan3 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_U_MODE1;
      /* V = chan2 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_V_MODE1;
      
      temp_CCMR2 |= PWM_CHAN_W_FORCE_INACTIVE;
	  TIM_CCxChannelCmd(htim1.Instance, PHASE_W1, TIM_CCx_DISABLE);

	  break;
  case 5:
	  /* Hall CBA = 101, Phase W2, U1 = high */
	  TIM_CCxChannelCmd(htim1.Instance, PHASE_W1, TIM_CCx_DISABLE);
	  TIM_CCxNChannelCmd(htim1.Instance, PHASE_U2, TIM_CCxN_DISABLE);

      /* W = chan3 set to pwm1 mode */
      temp_CCMR2 |= PWM_CHAN_W_MODE1;
      /* U = chan2 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_U_MODE1;

      temp_CCMR1 |= PWM_CHAN_V_FORCE_INACTIVE;
	  TIM_CCxChannelCmd(htim1.Instance, PHASE_V1, TIM_CCx_DISABLE);

	  break;
  case 1:
	  /* Hall CBA = 001, Phase W2, V1 = high */
	  TIM_CCxChannelCmd(htim1.Instance, PHASE_W1, TIM_CCx_DISABLE);
	  TIM_CCxNChannelCmd(htim1.Instance, PHASE_V2, TIM_CCxN_DISABLE);

      /* W = chan3 set to pwm1 mode */
      temp_CCMR2 |= PWM_CHAN_W_MODE1;
      /* V = chan2 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_V_MODE1;

      temp_CCMR1 |= PWM_CHAN_U_FORCE_INACTIVE;
	  TIM_CCxChannelCmd(htim1.Instance, PHASE_U1, TIM_CCx_DISABLE);
      
	  break;
  case 3:
	  /* Hall CBA = 011, Phase V1, U2 = high */
	  TIM_CCxNChannelCmd(htim1.Instance, PHASE_V2, TIM_CCxN_DISABLE);
	  TIM_CCxChannelCmd(htim1.Instance, PHASE_U1, TIM_CCx_DISABLE);

      /* V = chan3 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_V_MODE1;
      /* U = chan2 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_U_MODE1;

      temp_CCMR2 |= PWM_CHAN_W_FORCE_INACTIVE;
	  TIM_CCxChannelCmd(htim1.Instance, PHASE_W1, TIM_CCx_DISABLE);

	  break;
  case 2:
	  /* Hall CBA = 010, Phase W1, U2 = high */
	  TIM_CCxNChannelCmd(htim1.Instance, PHASE_W2, TIM_CCxN_DISABLE);
	  TIM_CCxChannelCmd(htim1.Instance, PHASE_U1, TIM_CCx_DISABLE);

      /* W = chan3 set to pwm1 mode */
      temp_CCMR2 |= PWM_CHAN_W_MODE1;
      /* U = chan2 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_U_MODE1;

      temp_CCMR1 |= PWM_CHAN_V_FORCE_INACTIVE;
	  TIM_CCxChannelCmd(htim1.Instance, PHASE_V1, TIM_CCx_DISABLE);

	  break;
  }

  /*
   * write the control bits for pwm-mode/force output 
   */
  htim1.Instance->CCMR1 = temp_CCMR1;
  htim1.Instance->CCMR2 = temp_CCMR2;

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
    uint8_t adc_val;

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
