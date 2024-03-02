/****************************************************************************************************************************
  timer.h

  For Portenta_H7 boards
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/Portenta_H7_TimerInterrupt
  Licensed under MIT license

  Now even you use all these new 16 ISR-based timers,with their maximum interval practically unlimited (limited only by
  unsigned long miliseconds), you just consume only one Portenta_H7 STM32 timer and avoid conflicting with other cores' tasks.
  The accuracy is nearly perfect compared to software timers. The most important feature is they're ISR-based timers
  Therefore, their executions are not blocked by bad-behaving functions / tasks.
  This important feature is absolutely necessary for mission-critical tasks.

  Version: 1.4.0

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.2.1   K.Hoang      15/09/2021 Initial coding for Portenta_H7
  1.3.0   K.Hoang      17/09/2021 Add PWM features and examples
  1.3.1   K.Hoang      21/09/2021 Fix warnings in PWM examples
  1.4.0   K.Hoang      22/01/2022 Fix `multiple-definitions` linker error. Fix bug
 *****************************************************************************************************************************/

// Modified from stm32 core v2.0.0

/*
 *******************************************************************************
   Copyright (c) 2019, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIMER_H
#define __TIMER_H

/* Includes ------------------------------------------------------------------*/
#include "PinNames.h"

#ifdef __cplusplus
extern "C" {
#endif

#if defined(HAL_TIM_MODULE_ENABLED) && !defined(HAL_TIM_MODULE_ONLY)

/* Exported constants --------------------------------------------------------*/
#ifndef TIM_IRQ_PRIO
#if (__CORTEX_M == 0x00U)
#define TIM_IRQ_PRIO       3
#else
#define TIM_IRQ_PRIO       14
#endif /* __CORTEX_M */

#endif /* TIM_IRQ_PRIO */

#ifndef TIM_IRQ_SUBPRIO
#define TIM_IRQ_SUBPRIO    0
#endif

#if defined(TIM1_BASE) && !defined(TIM1_IRQn)
#define TIM1_IRQn TIM1_UP_IRQn
#define TIM1_IRQHandler TIM1_UP_IRQHandler
#endif

#if defined(TIM8_BASE) && !defined(TIM8_IRQn)
#define TIM8_IRQn TIM8_UP_TIM13_IRQn
#define TIM8_IRQHandler TIM8_UP_TIM13_IRQHandler
#endif

#if defined(TIM12_BASE) && !defined(TIM12_IRQn)
#define TIM12_IRQn TIM8_BRK_TIM12_IRQn
#define TIM12_IRQHandler TIM8_BRK_TIM12_IRQHandler
#endif

#if defined(TIM13_BASE) && !defined(TIM13_IRQn)
#define TIM13_IRQn TIM8_UP_TIM13_IRQn
#endif

#if defined(TIM14_BASE) && !defined(TIM14_IRQn)
#define TIM14_IRQn TIM8_TRG_COM_TIM14_IRQn
#define TIM14_IRQHandler TIM8_TRG_COM_TIM14_IRQHandler
#endif


typedef enum
{
#if defined(TIM1_BASE)
  TIMER1_INDEX,
#endif
#if defined(TIM2_BASE)
  TIMER2_INDEX,
#endif
#if defined(TIM3_BASE)
  TIMER3_INDEX,
#endif
#if defined(TIM4_BASE)
  TIMER4_INDEX,
#endif
#if defined(TIM5_BASE)
  TIMER5_INDEX,
#endif
#if defined(TIM6_BASE)
  TIMER6_INDEX,
#endif
#if defined(TIM7_BASE)
  TIMER7_INDEX,
#endif
#if defined(TIM8_BASE)
  TIMER8_INDEX,
#endif
#if defined(TIM9_BASE)
  TIMER9_INDEX,
#endif
#if defined(TIM10_BASE)
  TIMER10_INDEX,
#endif
#if defined(TIM11_BASE)
  TIMER11_INDEX,
#endif
#if defined(TIM12_BASE)
  TIMER12_INDEX,
#endif
#if defined(TIM13_BASE)
  TIMER13_INDEX,
#endif
#if defined(TIM14_BASE)
  TIMER14_INDEX,
#endif
#if defined(TIM15_BASE)
  TIMER15_INDEX,
#endif
#if defined(TIM16_BASE)
  TIMER16_INDEX,
#endif
#if defined(TIM17_BASE)
  TIMER17_INDEX,
#endif
#if defined(TIM18_BASE)
  TIMER18_INDEX,
#endif
#if defined(TIM19_BASE)
  TIMER19_INDEX,
#endif
#if defined(TIM20_BASE)
  TIMER20_INDEX,
#endif
#if defined(TIM21_BASE)
  TIMER21_INDEX,
#endif
#if defined(TIM22_BASE)
  TIMER22_INDEX,
#endif

  TIMER_NUM,
  UNKNOWN_TIMER = 0XFFFF
} timer_index_t;


// This structure is used to be able to get HardwareTimer instance (C++ class)
// from handler (C structure) specially for interrupt management
typedef struct
{
  // Those 2 first fields must remain in this order at the beginning of the structure
  void    *__this;
  TIM_HandleTypeDef handle;
  uint32_t preemptPriority;
  uint32_t subPriority;
} timerObj_t;

/* Exported functions ------------------------------------------------------- */
timerObj_t *get_timer_obj(TIM_HandleTypeDef *htim);

void enableTimerClock(TIM_HandleTypeDef *htim);
void disableTimerClock(TIM_HandleTypeDef *htim);

uint32_t getTimerIrq(TIM_TypeDef *tim);
uint8_t getTimerClkSrc(TIM_TypeDef *tim);

IRQn_Type getTimerUpIrq(TIM_TypeDef *tim);
IRQn_Type getTimerCCIrq(TIM_TypeDef *tim);

#endif /* HAL_TIM_MODULE_ENABLED && !HAL_TIM_MODULE_ONLY */

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif
#if defined(HAL_TIM_MODULE_ENABLED) && !defined(HAL_TIM_MODULE_ONLY)

/* Private Functions */
/* Aim of the function is to get _timerObj pointer using htim pointer */
/* Highly inspired from magical linux kernel's "container_of" */
/* (which was not directly used since not compatible with IAR toolchain) */
timerObj_t *get_timer_obj(TIM_HandleTypeDef *htim)
{
  timerObj_t *obj;
  obj = (timerObj_t *)((char *)htim - offsetof(timerObj_t, handle));
  return (obj);
}

/**
    @brief  TIMER Initialization - clock init and nvic init
    @param  htim_base: TIM handle
    @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
  timerObj_t *obj = get_timer_obj(htim_base);
  enableTimerClock(htim_base);

  // configure Update interrupt
  HAL_NVIC_SetPriority(getTimerUpIrq(htim_base->Instance), obj->preemptPriority, obj->subPriority);
  HAL_NVIC_EnableIRQ(getTimerUpIrq(htim_base->Instance));

  if (getTimerCCIrq(htim_base->Instance) != getTimerUpIrq(htim_base->Instance))
  {
    // configure Capture Compare interrupt
    HAL_NVIC_SetPriority(getTimerCCIrq(htim_base->Instance), obj->preemptPriority, obj->subPriority);
    HAL_NVIC_EnableIRQ(getTimerCCIrq(htim_base->Instance));
  }
}

/**
    @brief  TIMER Deinitialization - clock and nvic
    @param  htim_base: TIM handle
    @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base)
{
  disableTimerClock(htim_base);
  HAL_NVIC_DisableIRQ(getTimerUpIrq(htim_base->Instance));
  HAL_NVIC_DisableIRQ(getTimerCCIrq(htim_base->Instance));
}

/**
    @brief  Initializes the TIM Output Compare MSP.
    @param  htim: TIM handle
    @retval None
*/
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{
  timerObj_t *obj = get_timer_obj(htim);
  enableTimerClock(htim);

  // configure Update interrupt
  HAL_NVIC_SetPriority(getTimerUpIrq(htim->Instance), obj->preemptPriority, obj->subPriority);
  HAL_NVIC_EnableIRQ(getTimerUpIrq(htim->Instance));

  if (getTimerCCIrq(htim->Instance) != getTimerUpIrq(htim->Instance))
  {
    // configure Capture Compare interrupt
    HAL_NVIC_SetPriority(getTimerCCIrq(htim->Instance), obj->preemptPriority, obj->subPriority);
    HAL_NVIC_EnableIRQ(getTimerCCIrq(htim->Instance));
  }
}

/**
    @brief  DeInitialize TIM Output Compare MSP.
    @param  htim: TIM handle
    @retval None
*/
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim)
{
  disableTimerClock(htim);
  HAL_NVIC_DisableIRQ(getTimerUpIrq(htim->Instance));
  HAL_NVIC_DisableIRQ(getTimerCCIrq(htim->Instance));
}

/**
    @brief  Initializes the TIM Input Capture MSP.
    @param  htim: TIM handle
    @retval None
*/
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  enableTimerClock(htim);
}

/**
    @brief  DeInitialize TIM Input Capture MSP.
    @param  htim: TIM handle
    @retval None
*/
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim)
{
  disableTimerClock(htim);
}

/* Exported functions */
/**
    @brief  Enable the timer clock
    @param  htim: TIM handle
    @retval None
*/
void enableTimerClock(TIM_HandleTypeDef *htim)
{
  // Enable TIM clock
#if defined(TIM1_BASE)
  if (htim->Instance == TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }

#endif
#if defined(TIM2_BASE)

  if (htim->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }

#endif
#if defined(TIM3_BASE)

  if (htim->Instance == TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }

#endif
#if defined(TIM4_BASE)

  if (htim->Instance == TIM4)
  {
    __HAL_RCC_TIM4_CLK_ENABLE();
  }

#endif
#if defined(TIM5_BASE)

  if (htim->Instance == TIM5)
  {
    __HAL_RCC_TIM5_CLK_ENABLE();
  }

#endif
#if defined(TIM6_BASE)

  if (htim->Instance == TIM6)
  {
    __HAL_RCC_TIM6_CLK_ENABLE();
  }

#endif
#if defined(TIM7_BASE)

  if (htim->Instance == TIM7)
  {
    __HAL_RCC_TIM7_CLK_ENABLE();
  }

#endif
#if defined(TIM8_BASE)

  if (htim->Instance == TIM8)
  {
    __HAL_RCC_TIM8_CLK_ENABLE();
  }

#endif
#if defined(TIM9_BASE)

  if (htim->Instance == TIM9)
  {
    __HAL_RCC_TIM9_CLK_ENABLE();
  }

#endif
#if defined(TIM10_BASE)

  if (htim->Instance == TIM10)
  {
    __HAL_RCC_TIM10_CLK_ENABLE();
  }

#endif
#if defined(TIM11_BASE)

  if (htim->Instance == TIM11)
  {
    __HAL_RCC_TIM11_CLK_ENABLE();
  }

#endif
#if defined(TIM12_BASE)

  if (htim->Instance == TIM12)
  {
    __HAL_RCC_TIM12_CLK_ENABLE();
  }

#endif
#if defined(TIM13_BASE)

  if (htim->Instance == TIM13)
  {
    __HAL_RCC_TIM13_CLK_ENABLE();
  }

#endif
#if defined(TIM14_BASE)

  if (htim->Instance == TIM14)
  {
    __HAL_RCC_TIM14_CLK_ENABLE();
  }

#endif
#if defined(TIM15_BASE)

  if (htim->Instance == TIM15)
  {
    __HAL_RCC_TIM15_CLK_ENABLE();
  }

#endif
#if defined(TIM16_BASE)

  if (htim->Instance == TIM16)
  {
    __HAL_RCC_TIM16_CLK_ENABLE();
  }

#endif
#if defined(TIM17_BASE)

  if (htim->Instance == TIM17)
  {
    __HAL_RCC_TIM17_CLK_ENABLE();
  }

#endif
#if defined(TIM18_BASE)

  if (htim->Instance == TIM18)
  {
    __HAL_RCC_TIM18_CLK_ENABLE();
  }

#endif
#if defined(TIM19_BASE)

  if (htim->Instance == TIM19)
  {
    __HAL_RCC_TIM19_CLK_ENABLE();
  }

#endif
#if defined(TIM20_BASE)

  if (htim->Instance == TIM20)
  {
    __HAL_RCC_TIM20_CLK_ENABLE();
  }

#endif
#if defined(TIM21_BASE)

  if (htim->Instance == TIM21)
  {
    __HAL_RCC_TIM21_CLK_ENABLE();
  }

#endif
#if defined(TIM22_BASE)

  if (htim->Instance == TIM22)
  {
    __HAL_RCC_TIM22_CLK_ENABLE();
  }

#endif
}

/**
    @brief  Disable the timer clock
    @param  htim: TIM handle
    @retval None
*/
void disableTimerClock(TIM_HandleTypeDef *htim)
{
  // Enable TIM clock
#if defined(TIM1_BASE)
  if (htim->Instance == TIM1)
  {
    __HAL_RCC_TIM1_CLK_DISABLE();
  }

#endif
#if defined(TIM2_BASE)

  if (htim->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_DISABLE();
  }

#endif
#if defined(TIM3_BASE)

  if (htim->Instance == TIM3)
  {
    __HAL_RCC_TIM3_CLK_DISABLE();
  }

#endif
#if defined(TIM4_BASE)

  if (htim->Instance == TIM4)
  {
    __HAL_RCC_TIM4_CLK_DISABLE();
  }

#endif
#if defined(TIM5_BASE)

  if (htim->Instance == TIM5)
  {
    __HAL_RCC_TIM5_CLK_DISABLE();
  }

#endif
#if defined(TIM6_BASE)

  if (htim->Instance == TIM6)
  {
    __HAL_RCC_TIM6_CLK_DISABLE();
  }

#endif
#if defined(TIM7_BASE)

  if (htim->Instance == TIM7)
  {
    __HAL_RCC_TIM7_CLK_DISABLE();
  }

#endif
#if defined(TIM8_BASE)

  if (htim->Instance == TIM8)
  {
    __HAL_RCC_TIM8_CLK_DISABLE();
  }

#endif
#if defined(TIM9_BASE)

  if (htim->Instance == TIM9)
  {
    __HAL_RCC_TIM9_CLK_DISABLE();
  }

#endif
#if defined(TIM10_BASE)

  if (htim->Instance == TIM10)
  {
    __HAL_RCC_TIM10_CLK_DISABLE();
  }

#endif
#if defined(TIM11_BASE)

  if (htim->Instance == TIM11)
  {
    __HAL_RCC_TIM11_CLK_DISABLE();
  }

#endif
#if defined(TIM12_BASE)

  if (htim->Instance == TIM12)
  {
    __HAL_RCC_TIM12_CLK_DISABLE();
  }

#endif
#if defined(TIM13_BASE)

  if (htim->Instance == TIM13)
  {
    __HAL_RCC_TIM13_CLK_DISABLE();
  }

#endif
#if defined(TIM14_BASE)

  if (htim->Instance == TIM14)
  {
    __HAL_RCC_TIM14_CLK_DISABLE();
  }

#endif
#if defined(TIM15_BASE)

  if (htim->Instance == TIM15)
  {
    __HAL_RCC_TIM15_CLK_DISABLE();
  }

#endif
#if defined(TIM16_BASE)

  if (htim->Instance == TIM16)
  {
    __HAL_RCC_TIM16_CLK_DISABLE();
  }

#endif
#if defined(TIM17_BASE)

  if (htim->Instance == TIM17)
  {
    __HAL_RCC_TIM17_CLK_DISABLE();
  }

#endif
#if defined(TIM18_BASE)

  if (htim->Instance == TIM18)
  {
    __HAL_RCC_TIM18_CLK_DISABLE();
  }

#endif
#if defined(TIM19_BASE)

  if (htim->Instance == TIM19)
  {
    __HAL_RCC_TIM19_CLK_DISABLE();
  }

#endif
#if defined(TIM20_BASE)

  if (htim->Instance == TIM20)
  {
    __HAL_RCC_TIM20_CLK_DISABLE();
  }

#endif
#if defined(TIM21_BASE)

  if (htim->Instance == TIM21)
  {
    __HAL_RCC_TIM21_CLK_DISABLE();
  }

#endif
#if defined(TIM22_BASE)

  if (htim->Instance == TIM22)
  {
    __HAL_RCC_TIM22_CLK_DISABLE();
  }

#endif
}

/**
    @brief  This function return IRQ number corresponding to update interrupt event of timer instance.
    @param  tim: timer instance
    @retval IRQ number
*/
IRQn_Type getTimerUpIrq(TIM_TypeDef *tim)
{
  IRQn_Type IRQn = NonMaskableInt_IRQn;

  if (tim != (TIM_TypeDef *)NC)
  {
    /* Get IRQn depending on TIM instance */
    switch ((uint32_t)tim)
    {
#if defined(TIM1_BASE)

      case (uint32_t)TIM1_BASE:
        IRQn = TIM1_IRQn;
        break;
#endif
#if defined(TIM2_BASE)

      case (uint32_t)TIM2_BASE:
        IRQn = TIM2_IRQn;
        break;
#endif
#if defined(TIM3_BASE)

      case (uint32_t)TIM3_BASE:
        IRQn = TIM3_IRQn;
        break;
#endif
#if defined(TIM4_BASE)

      case (uint32_t)TIM4_BASE:
        IRQn = TIM4_IRQn;
        break;
#endif
#if defined(TIM5_BASE)

      case (uint32_t)TIM5_BASE:
        IRQn = TIM5_IRQn;
        break;
#endif

        // KH
#if 0
#if defined(TIM6_BASE)

      case (uint32_t)TIM6_BASE:
        IRQn = TIM6_IRQn;
        break;
#endif
#endif
        //////

#if defined(TIM7_BASE)

      case (uint32_t)TIM7_BASE:
        IRQn = TIM7_IRQn;
        break;
#endif
#if defined(TIM8_BASE)

      case (uint32_t)TIM8_BASE:
        IRQn = TIM8_IRQn;
        break;
#endif
#if defined(TIM9_BASE)

      case (uint32_t)TIM9_BASE:
        IRQn = TIM9_IRQn;
        break;
#endif
#if defined(TIM10_BASE)

      case (uint32_t)TIM10_BASE:
        IRQn = TIM10_IRQn;
        break;
#endif
#if defined(TIM11_BASE)

      case (uint32_t)TIM11_BASE:
        IRQn = TIM11_IRQn;
        break;
#endif
#if defined(TIM12_BASE)

      case (uint32_t)TIM12_BASE:
        IRQn = TIM12_IRQn;
        break;
#endif
#if defined(TIM13_BASE)

      case (uint32_t)TIM13_BASE:
        IRQn = TIM13_IRQn;
        break;
#endif
#if defined(TIM14_BASE)

      case (uint32_t)TIM14_BASE:
        IRQn = TIM14_IRQn;
        break;
#endif
#if defined(TIM15_BASE)

      case (uint32_t)TIM15_BASE:
        IRQn = TIM15_IRQn;
        break;
#endif
#if defined(TIM16_BASE)

      case (uint32_t)TIM16_BASE:
        IRQn = TIM16_IRQn;
        break;
#endif
#if defined(TIM17_BASE)

      case (uint32_t)TIM17_BASE:
        IRQn = TIM17_IRQn;
        break;
#endif
#if defined(TIM18_BASE)

      case (uint32_t)TIM18_BASE:
        IRQn = TIM18_IRQn;
        break;
#endif
#if defined(TIM19_BASE)

      case (uint32_t)TIM19_BASE:
        IRQn = TIM19_IRQn;
        break;
#endif
#if defined(TIM20_BASE)

      case (uint32_t)TIM20_BASE:
        IRQn = TIM20_IRQn;
        break;
#endif
#if defined(TIM21_BASE)

      case (uint32_t)TIM21_BASE:
        IRQn = TIM21_IRQn;
        break;
#endif
#if defined(TIM22_BASE)

      case (uint32_t)TIM22_BASE:
        IRQn = TIM22_IRQn;
        break;
#endif

      default:
        //_Error_Handler("TIM: Unknown timer IRQn", (int)tim);
        break;
    }
  }

  return IRQn;
}

/**
    @brief  This function return IRQ number corresponding to Capture or Compare interrupt event of timer instance.
    @param  tim: timer instance
    @retval IRQ number
*/
IRQn_Type getTimerCCIrq(TIM_TypeDef *tim)
{
  IRQn_Type IRQn = NonMaskableInt_IRQn;

  if (tim != (TIM_TypeDef *)NC)
  {
    /* Get IRQn depending on TIM instance */
    switch ((uint32_t)tim)
    {
#if defined(TIM1_BASE)

      case (uint32_t)TIM1_BASE:
        IRQn = TIM1_CC_IRQn;
        break;
#endif
#if defined(TIM2_BASE)

      case (uint32_t)TIM2_BASE:
        IRQn = TIM2_IRQn;
        break;
#endif
#if defined(TIM3_BASE)

      case (uint32_t)TIM3_BASE:
        IRQn = TIM3_IRQn;
        break;
#endif
#if defined(TIM4_BASE)

      case (uint32_t)TIM4_BASE:
        IRQn = TIM4_IRQn;
        break;
#endif
#if defined(TIM5_BASE)

      case (uint32_t)TIM5_BASE:
        IRQn = TIM5_IRQn;
        break;
#endif

#if 0
        // KH
#if defined(TIM6_BASE)

      case (uint32_t)TIM6_BASE:
        IRQn = TIM6_IRQn;
        break;
#endif
#endif
        //////

#if defined(TIM7_BASE)

      case (uint32_t)TIM7_BASE:
        IRQn = TIM7_IRQn;
        break;
#endif
#if defined(TIM8_BASE)

      case (uint32_t)TIM8_BASE:
        IRQn = TIM8_CC_IRQn;
        break;
#endif
#if defined(TIM9_BASE)

      case (uint32_t)TIM9_BASE:
        IRQn = TIM9_IRQn;
        break;
#endif
#if defined(TIM10_BASE)

      case (uint32_t)TIM10_BASE:
        IRQn = TIM10_IRQn;
        break;
#endif
#if defined(TIM11_BASE)

      case (uint32_t)TIM11_BASE:
        IRQn = TIM11_IRQn;
        break;
#endif
#if defined(TIM12_BASE)

      case (uint32_t)TIM12_BASE:
        IRQn = TIM12_IRQn;
        break;
#endif
#if defined(TIM13_BASE)

      case (uint32_t)TIM13_BASE:
        IRQn = TIM13_IRQn;
        break;
#endif
#if defined(TIM14_BASE)

      case (uint32_t)TIM14_BASE:
        IRQn = TIM14_IRQn;
        break;
#endif
#if defined(TIM15_BASE)

      case (uint32_t)TIM15_BASE:
        IRQn = TIM15_IRQn;
        break;
#endif
#if defined(TIM16_BASE)

      case (uint32_t)TIM16_BASE:
        IRQn = TIM16_IRQn;
        break;
#endif
#if defined(TIM17_BASE)

      case (uint32_t)TIM17_BASE:
        IRQn = TIM17_IRQn;
        break;
#endif
#if defined(TIM18_BASE)

      case (uint32_t)TIM18_BASE:
        IRQn = TIM18_IRQn;
        break;
#endif
#if defined(TIM19_BASE)

      case (uint32_t)TIM19_BASE:
        IRQn = TIM19_IRQn;
        break;
#endif
#if defined(TIM20_BASE)

      case (uint32_t)TIM20_BASE:
        IRQn = TIM20_CC_IRQn;
        break;
#endif
#if defined(TIM21_BASE)

      case (uint32_t)TIM21_BASE:
        IRQn = TIM21_IRQn;
        break;
#endif
#if defined(TIM22_BASE)

      case (uint32_t)TIM22_BASE:
        IRQn = TIM22_IRQn;
        break;
#endif
        break;

      default:
        //_Error_Handler("TIM: Unknown timer IRQn", (int)tim);
        break;
    }
  }

  return IRQn;
}

/**
    @brief  This function return the timer clock source.
    @param  tim: timer instance
    @retval 1 = PCLK1 or 2 = PCLK2
*/
uint8_t getTimerClkSrc(TIM_TypeDef *tim)
{
  uint8_t clkSrc = 0;

  if (tim != (TIM_TypeDef *)NC)
#if defined(STM32F0xx) || defined(STM32G0xx)
    /* TIMx source CLK is PCKL1 */
    clkSrc = 1;

#else
  {
    /* Get source clock depending on TIM instance */
    switch ((uint32_t)tim)
    {
#if defined(TIM2_BASE)

      case (uint32_t)TIM2_BASE:
#endif
#if defined(TIM3_BASE)
      case (uint32_t)TIM3_BASE:
#endif
#if defined(TIM4_BASE)
      case (uint32_t)TIM4_BASE:
#endif
#if defined(TIM5_BASE)
      case (uint32_t)TIM5_BASE:
#endif
#if defined(TIM6_BASE)
      case (uint32_t)TIM6_BASE:
#endif
#if defined(TIM7_BASE)
      case (uint32_t)TIM7_BASE:
#endif
#if defined(TIM12_BASE)
      case (uint32_t)TIM12_BASE:
#endif
#if defined(TIM13_BASE)
      case (uint32_t)TIM13_BASE:
#endif
#if defined(TIM14_BASE)
      case (uint32_t)TIM14_BASE:
#endif
#if defined(TIM18_BASE)
      case (uint32_t)TIM18_BASE:
#endif
        clkSrc = 1;
        break;
#if defined(TIM1_BASE)

      case (uint32_t)TIM1_BASE:
#endif
#if defined(TIM8_BASE)
      case (uint32_t)TIM8_BASE:
#endif
#if defined(TIM9_BASE)
      case (uint32_t)TIM9_BASE:
#endif
#if defined(TIM10_BASE)
      case (uint32_t)TIM10_BASE:
#endif
#if defined(TIM11_BASE)
      case (uint32_t)TIM11_BASE:
#endif
#if defined(TIM15_BASE)
      case (uint32_t)TIM15_BASE:
#endif
#if defined(TIM16_BASE)
      case (uint32_t)TIM16_BASE:
#endif
#if defined(TIM17_BASE)
      case (uint32_t)TIM17_BASE:
#endif
#if defined(TIM19_BASE)
      case (uint32_t)TIM19_BASE:
#endif
#if defined(TIM20_BASE)
      case (uint32_t)TIM20_BASE:
#endif
#if defined(TIM21_BASE)
      case (uint32_t)TIM21_BASE:
#endif
#if defined(TIM22_BASE)
      case (uint32_t)TIM22_BASE:
#endif
        clkSrc = 2;
        break;

      default:
        ////_Error_Handler("TIM: Unknown timer instance", (int)tim);
        break;
    }
  }
#endif
  return clkSrc;
}


#endif /* HAL_TIM_MODULE_ENABLED && !HAL_TIM_MODULE_ONLY */

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
