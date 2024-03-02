#ifndef PORTENTA_DRIVER_DEF
#define PORTENTA_DRIVER_DEF
#include <drivers/hardware_api.h>

#pragma message("")
#pragma message("SimpleFOC: compiling for Arduino/Portenta_H7")
#pragma message("")

#include "pwmout_api.h"
#include "pinDefinitions.h"
#include "platform/mbed_critical.h"
#include "platform/mbed_power_mgmt.h"
#include "platform/mbed_assert.h"
#include "PeripheralPins.h"
#include "pwmout_device.h"

// default pwm parameters
#define _PWM_FREQUENCY 25000 // 25khz
#define _PWM_FREQUENCY_MAX 50000 // 50khz

typedef struct PortentaDriverParams {
  pwmout_t pins[4];
  long pwm_frequency;
} PortentaDriverParams;

class PortentaDriver3PWM : public BLDCDriver
{
  public:
    PortentaDriver3PWM(int phA,int phB,int phC, int en1 = NOT_SET, int en2 = NOT_SET, int en3 = NOT_SET) {
      // Pin initialization
      pwmA = phA;
      pwmB = phB;
      pwmC = phC;

      // enable_pin pin
      enableA_pin = en1;
      enableB_pin = en2;
      enableC_pin = en3;

      // default power-supply value
      voltage_power_supply = DEF_POWER_SUPPLY;
      voltage_limit = NOT_SET;
      pwm_frequency = NOT_SET;
    }
    
  	int init() {
      // PWM pins
      pinMode(pwmA, OUTPUT);
      pinMode(pwmB, OUTPUT);
      pinMode(pwmC, OUTPUT);
      if( _isset(enableA_pin)) pinMode(enableA_pin, OUTPUT);
      if( _isset(enableB_pin)) pinMode(enableB_pin, OUTPUT);
      if( _isset(enableC_pin)) pinMode(enableC_pin, OUTPUT);

      // sanity check for the voltage limit configuration
      if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

      // Set the pwm frequency to the pins
      // hardware specific function - depending on driver and mcu
      params = configure3PWM(pwm_frequency, pwmA, pwmB, pwmC);
      initialized = (params != SIMPLEFOC_DRIVER_INIT_FAILED);
      return params != SIMPLEFOC_DRIVER_INIT_FAILED;
    }

  	void disable() {
      // set zero to PWM
      setPwm(0, 0, 0);
      // disable the driver - if enable_pin pin available
      if ( _isset(enableA_pin) ) digitalWrite(enableA_pin, !enable_active_high);
      if ( _isset(enableB_pin) ) digitalWrite(enableB_pin, !enable_active_high);
      if ( _isset(enableC_pin) ) digitalWrite(enableC_pin, !enable_active_high);
    }

    void enable() {
      // enable_pin the driver - if enable_pin pin available
      if ( _isset(enableA_pin) ) digitalWrite(enableA_pin, enable_active_high);
      if ( _isset(enableB_pin) ) digitalWrite(enableB_pin, enable_active_high);
      if ( _isset(enableC_pin) ) digitalWrite(enableC_pin, enable_active_high);
      // set zero to PWM
      setPwm(0,0,0);
    }

    // hardware variables
  	int pwmA; //!< phase A pwm pin number
  	int pwmB; //!< phase B pwm pin number
  	int pwmC; //!< phase C pwm pin number
    int enableA_pin; //!< enable pin number
    int enableB_pin; //!< enable pin number
    int enableC_pin; //!< enable pin number
    bool enable_active_high = true;

    void setPwm(float Ua, float Ub, float Uc) {
      // limit the voltage in driver
      Ua = _constrain(Ua, 0.0f, voltage_limit);
      Ub = _constrain(Ub, 0.0f, voltage_limit);
      Uc = _constrain(Uc, 0.0f, voltage_limit);
      // calculate duty cycle
      // limited in [0,1]
      dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
      dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
      dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

      core_util_critical_section_enter();
      pwm_write(&(((PortentaDriverParams*)params)->pins[0]), (float)dc_a);
      pwm_write(&(((PortentaDriverParams*)params)->pins[1]), (float)dc_b);
      pwm_write(&(((PortentaDriverParams*)params)->pins[2]), (float)dc_c);
      core_util_critical_section_exit();
    }

    virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) {
      // disable if needed
      if( _isset(enableA_pin) &&  _isset(enableB_pin)  && _isset(enableC_pin) ){
        digitalWrite(enableA_pin, sa == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
        digitalWrite(enableB_pin, sb == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
        digitalWrite(enableC_pin, sc == PhaseState::PHASE_ON ? enable_active_high:!enable_active_high);
      }
    }
  private:

    void pwm_write(pwmout_t *obj, float value){
      TIM_HandleTypeDef TimHandle;
      int channel = 0;

      TimHandle.Instance = (TIM_TypeDef *)(obj->pwm);
      
      if (value < (float)0.0) {
          value = 0.0;
      } else if (value > (float)1.0) {
          value = 1.0;
      }

      obj->pulse = (uint32_t)((float)obj->period * value + 0.5);

      switch (obj->channel) {
          case 1: channel = TIM_CHANNEL_1; break;
          case 2: channel = TIM_CHANNEL_2; break;
          case 3: channel = TIM_CHANNEL_3; break;
          case 4: channel = TIM_CHANNEL_4; break;
          default: return;
      }
      
      // If channel already enabled, only update compare value to avoid glitch
      __HAL_TIM_SET_COMPARE(&TimHandle, channel, obj->pulse / obj->prescaler);
    }

    void* configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
      if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
      else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max

      PortentaDriverParams* params = new PortentaDriverParams();
      params->pwm_frequency = pwm_frequency;

      core_util_critical_section_enter();
      pwm_init(&(params->pins[0]), pinA, (long)pwm_frequency);
      pwm_init(&(params->pins[1]), pinB, (long)pwm_frequency);
      pwm_init(&(params->pins[2]), pinC, (long)pwm_frequency);
      // allign the timers
      alignPWMTimers(&(params->pins[0]), &(params->pins[1]), &(params->pins[2]));
      core_util_critical_section_exit();

      return params;
    }

    int pwm_init(pwmout_t *obj, uint32_t pin, long frequency) {
        int peripheral = (int)pinmap_peripheral(digitalPinToPinName(pin), PinMap_TIM);
        int function = (int)pinmap_find_function(digitalPinToPinName(pin), PinMap_TIM);

        const PinMap static_pinmap = {digitalPinToPinName(pin), peripheral, function};

        pwmout_init_direct(obj, &static_pinmap);

        TIM_HandleTypeDef TimHandle;
        TimHandle.Instance = (TIM_TypeDef *)(obj->pwm);
        RCC_ClkInitTypeDef RCC_ClkInitStruct;
        uint32_t PclkFreq = 0;
        uint32_t APBxCLKDivider = RCC_HCLK_DIV1;
        uint8_t i = 0;

        __HAL_TIM_DISABLE(&TimHandle);

        // Get clock configuration
        // Note: PclkFreq contains here the Latency (not used after)
        HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &PclkFreq);

        /*  Parse the pwm / apb mapping table to find the right entry */
        while (pwm_apb_map_table[i].pwm != obj->pwm) i++;
        // sanity check
        if (pwm_apb_map_table[i].pwm == 0) return -1;
        

        if (pwm_apb_map_table[i].pwmoutApb == PWMOUT_ON_APB1) {
            PclkFreq = HAL_RCC_GetPCLK1Freq();
            APBxCLKDivider = RCC_ClkInitStruct.APB1CLKDivider;
        } else {
    #if !defined(PWMOUT_APB2_NOT_SUPPORTED)
            PclkFreq = HAL_RCC_GetPCLK2Freq();
            APBxCLKDivider = RCC_ClkInitStruct.APB2CLKDivider;
    #endif
        }

        long period_us = 500000.0/((float)frequency);
        /* By default use, 1us as SW pre-scaler */
        obj->prescaler = 1;
        // TIMxCLK = PCLKx when the APB prescaler = 1 else TIMxCLK = 2 * PCLKx
        if (APBxCLKDivider == RCC_HCLK_DIV1) {
            TimHandle.Init.Prescaler = (((PclkFreq) / 1000000)) - 1; // 1 us tick
        } else {
            TimHandle.Init.Prescaler = (((PclkFreq * 2) / 1000000)) - 1; // 1 us tick
        }
        TimHandle.Init.Period = (period_us - 1);

        /*  In case period or pre-scalers are out of range, loop-in to get valid values */
        while ((TimHandle.Init.Period > 0xFFFF) || (TimHandle.Init.Prescaler > 0xFFFF)) {
            obj->prescaler = obj->prescaler * 2;
            if (APBxCLKDivider == RCC_HCLK_DIV1) {
                TimHandle.Init.Prescaler = (((PclkFreq) / 1000000) * obj->prescaler) - 1;
            } else {
                TimHandle.Init.Prescaler = (((PclkFreq * 2) / 1000000) * obj->prescaler) - 1;
            }
            TimHandle.Init.Period = (period_us - 1) / obj->prescaler;
            /*  Period decreases and prescaler increases over loops, so check for
            *  possible out of range cases */
            if ((TimHandle.Init.Period < 0xFFFF) && (TimHandle.Init.Prescaler > 0xFFFF)) {
                break;
            }
        }

        TimHandle.Init.ClockDivision = 0;
        TimHandle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3; // center aligned

        if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) {
            return -1;
        }
        
        TIM_OC_InitTypeDef sConfig;
        // Configure channels
        sConfig.OCMode       = TIM_OCMODE_PWM1;
        sConfig.Pulse        = obj->pulse / obj->prescaler;
        sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
        sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
    #if defined(TIM_OCIDLESTATE_RESET)
        sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
    #endif
    #if defined(TIM_OCNIDLESTATE_RESET)
        sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
        sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    #endif

        int channel = 0;
        switch (obj->channel) {
            case 1: channel = TIM_CHANNEL_1; break;
            case 2: channel = TIM_CHANNEL_2; break;
            case 3: channel = TIM_CHANNEL_3; break;
            case 4: channel = TIM_CHANNEL_4; break;
            default: return -1;
        }
        
        if (LL_TIM_CC_IsEnabledChannel(TimHandle.Instance, _TIM_ChannelConvert_HAL2LL(channel, obj)) == 0) {
            // If channel is not enabled, proceed to channel configuration
            if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, channel) != HAL_OK) {
                return -1;
            }
        } 

        // Save for future use
        obj->period = period_us;
#if !defined(PWMOUT_INVERTED_NOT_SUPPORTED)
        if (obj->inverted) {
            HAL_TIMEx_PWMN_Start(&TimHandle, channel);
        } else
#endif
        {
            HAL_TIM_PWM_Start(&TimHandle, channel);
        }
        
        return 0;
    }

    /* Convert STM32 Cube HAL channel to LL channel */
    uint32_t _TIM_ChannelConvert_HAL2LL(uint32_t channel, pwmout_t *obj) {
#if !defined(PWMOUT_INVERTED_NOT_SUPPORTED)
      if (obj->inverted) {
          switch (channel) {
              case TIM_CHANNEL_1: return LL_TIM_CHANNEL_CH1N;
              case TIM_CHANNEL_2: return LL_TIM_CHANNEL_CH2N;
              case TIM_CHANNEL_3: return LL_TIM_CHANNEL_CH3N;
#if defined(LL_TIM_CHANNEL_CH4N)
              case TIM_CHANNEL_4: return LL_TIM_CHANNEL_CH4N;
#endif
              default: return 0;
          }
      } else
#endif
      {
          switch (channel) {
              case TIM_CHANNEL_1: return LL_TIM_CHANNEL_CH1;
              case TIM_CHANNEL_2: return LL_TIM_CHANNEL_CH2;
              case TIM_CHANNEL_3: return LL_TIM_CHANNEL_CH3;
              case TIM_CHANNEL_4: return LL_TIM_CHANNEL_CH4;
              default: return 0;
          }
      }
    }

    // align the timers to end the init
    void alignPWMTimers(pwmout_t *t1, pwmout_t *t2, pwmout_t *t3) {
        TIM_HandleTypeDef TimHandle1, TimHandle2, TimHandle3;
        TimHandle1.Instance = (TIM_TypeDef *)(t1->pwm);
        TimHandle2.Instance = (TIM_TypeDef *)(t2->pwm);
        TimHandle3.Instance = (TIM_TypeDef *)(t3->pwm);
        __HAL_TIM_DISABLE(&TimHandle1);
        __HAL_TIM_DISABLE(&TimHandle2);
        __HAL_TIM_DISABLE(&TimHandle3);
        __HAL_TIM_ENABLE(&TimHandle1); 
        __HAL_TIM_ENABLE(&TimHandle2); 
        __HAL_TIM_ENABLE(&TimHandle3); 
    }
};

#endif