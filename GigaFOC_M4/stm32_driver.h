#ifndef STM32_DRIVER_MCU_DEF
#define STM32_DRIVER_MCU_DEF
#include <drivers/hardware_api.h>
#include "HardwareTimer.h"

// default pwm parameters
#define _PWM_RESOLUTION 12 // 12bit
#define _PWM_RANGE 4095.0f // 2^12 -1 = 4095
#define _PWM_FREQUENCY 25000 // 25khz
#define _PWM_FREQUENCY_MAX 50000 // 50khz

// pinmap.h
#define NP 0U
// PinNames.h
#define ALTX_MASK 0x700

typedef struct STM32DriverParams {
  HardwareTimer* timers[6] = {NULL};
  uint32_t channels[6];
  long pwm_frequency;
  float dead_zone;
  uint8_t interface_type;
} STM32DriverParams;

// timer synchornisation functions
void _stopTimers(HardwareTimer **timers_to_stop, int timer_num=6);
void _startTimers(HardwareTimer **timers_to_start, int timer_num=6);

// timer query functions
bool _getPwmState(void* params);

#define SIMPLEFOC_STM32_DEBUG
#pragma message("")
#pragma message("SimpleFOC: compiling for STM32")
#pragma message("")

void printTimerCombination(int numPins, PinMap* timers[], int score);
int getTimerNumber(int timerIndex);


#ifndef SIMPLEFOC_STM32_MAX_PINTIMERSUSED
#define SIMPLEFOC_STM32_MAX_PINTIMERSUSED 15
#endif
int numTimerPinsUsed;
PinMap* timerPinsUsed[SIMPLEFOC_STM32_MAX_PINTIMERSUSED];

bool _getPwmState(void* params) {
  // assume timers are synchronized and that there's at least one timer
  HardwareTimer* pHT = ((STM32DriverParams*)params)->timers[0];
  TIM_HandleTypeDef* htim = pHT->getHandle();
  
  bool dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(htim);

  return dir;
}

// setting pwm to hardware pin - instead analogWrite()
void _setPwm(HardwareTimer *HT, uint32_t channel, uint32_t value, int resolution)
{
  // TODO - remove commented code
  // PinName pin = digitalPinToPinName(ulPin);
  // TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_PWM);
  // uint32_t index = get_timer_index(Instance);
  // HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  
  HT->setCaptureCompare(channel, value, (TimerCompareFormat_t)resolution);
}

int getLLChannel(PinMap* timer) {
#if defined(TIM_CCER_CC1NE)
  if (STM_PIN_INVERTED(timer->function)) {
    switch (STM_PIN_CHANNEL(timer->function)) {
      case 1: return LL_TIM_CHANNEL_CH1N;
      case 2: return LL_TIM_CHANNEL_CH2N;
      case 3: return LL_TIM_CHANNEL_CH3N;
#if defined(LL_TIM_CHANNEL_CH4N)
      case 4: return LL_TIM_CHANNEL_CH4N;
#endif
      default: return -1;
    }
  } else
#endif
  {
    switch (STM_PIN_CHANNEL(timer->function)) {
      case 1: return LL_TIM_CHANNEL_CH1;
      case 2: return LL_TIM_CHANNEL_CH2;
      case 3: return LL_TIM_CHANNEL_CH3;
      case 4: return LL_TIM_CHANNEL_CH4;
      default: return -1;
    }
  }
  return -1;
}

// init pin pwm
HardwareTimer* _initPinPWM(uint32_t PWM_freq, PinMap* timer) {
  // sanity check
  if (timer==NP)
    return NP;
  uint32_t index = get_timer_index((TIM_TypeDef*)timer->peripheral);
  bool init = false;
  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef*)timer->peripheral);
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HardwareTimer_Handle[index]->handle.Init.RepetitionCounter = 1;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
    init = true;
  }
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  uint32_t channel = STM_PIN_CHANNEL(timer->function);
  HT->pause();
  if (init)
    HT->setOverflow(PWM_freq, HERTZ_FORMAT);
  HT->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, timer->pin);
  #if SIMPLEFOC_PWM_ACTIVE_HIGH==false
  LL_TIM_OC_SetPolarity(HT->getHandle()->Instance, getLLChannel(timer), LL_TIM_OCPOLARITY_LOW);
  #endif
#ifdef SIMPLEFOC_STM32_DEBUG
  SIMPLEFOC_DEBUG("STM32-DRV: Configuring high timer ", (int)getTimerNumber(get_timer_index(HardwareTimer_Handle[index]->handle.Instance)));
  SIMPLEFOC_DEBUG("STM32-DRV: Configuring high channel ", (int)channel);
#endif
  return HT;
}

// init high side pin
HardwareTimer* _initPinPWMHigh(uint32_t PWM_freq, PinMap* timer) {
  HardwareTimer* HT = _initPinPWM(PWM_freq, timer);
  #if SIMPLEFOC_PWM_HIGHSIDE_ACTIVE_HIGH==false && SIMPLEFOC_PWM_ACTIVE_HIGH==true
  LL_TIM_OC_SetPolarity(HT->getHandle()->Instance, getLLChannel(timer), LL_TIM_OCPOLARITY_LOW);
  #endif
  return HT;
}

// init low side pin
HardwareTimer* _initPinPWMLow(uint32_t PWM_freq, PinMap* timer)
{
  uint32_t index = get_timer_index((TIM_TypeDef*)timer->peripheral);

  bool init = false;
  if (HardwareTimer_Handle[index] == NULL) {
    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef*)timer->peripheral);
    HardwareTimer_Handle[index]->handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    HardwareTimer_Handle[index]->handle.Init.RepetitionCounter = 1;
    HAL_TIM_Base_Init(&(HardwareTimer_Handle[index]->handle));
    init = true;
  }
  HardwareTimer *HT = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
  uint32_t channel = STM_PIN_CHANNEL(timer->function);

#ifdef SIMPLEFOC_STM32_DEBUG
  SIMPLEFOC_DEBUG("STM32-DRV: Configuring low timer ", (int)getTimerNumber(get_timer_index(HardwareTimer_Handle[index]->handle.Instance)));
  SIMPLEFOC_DEBUG("STM32-DRV: Configuring low channel ", (int)channel);
#endif

  HT->pause();
  if (init)
    HT->setOverflow(PWM_freq, HERTZ_FORMAT);
  // sets internal fields of HT, but we can't set polarity here
  HT->setMode(channel, TIMER_OUTPUT_COMPARE_PWM2, timer->pin);
  #if SIMPLEFOC_PWM_LOWSIDE_ACTIVE_HIGH==false
  LL_TIM_OC_SetPolarity(HT->getHandle()->Instance, getLLChannel(timer), LL_TIM_OCPOLARITY_LOW);
  #endif
  return HT;
}

// align the timers to end the init
void _alignPWMTimers(HardwareTimer *HT1, HardwareTimer *HT2, HardwareTimer *HT3)
{
  HT1->pause();
  HT1->refresh();
  HT2->pause();
  HT2->refresh();
  HT3->pause();
  HT3->refresh();
  HT1->resume();
  HT2->resume();
  HT3->resume();
}

// align the timers to end the init
void _alignPWMTimers(HardwareTimer *HT1, HardwareTimer *HT2, HardwareTimer *HT3, HardwareTimer *HT4)
{
  HT1->pause();
  HT1->refresh();
  HT2->pause();
  HT2->refresh();
  HT3->pause();
  HT3->refresh();
  HT4->pause();
  HT4->refresh();
  HT1->resume();
  HT2->resume();
  HT3->resume();
  HT4->resume();
}


// align the timers to end the init
void _stopTimers(HardwareTimer **timers_to_stop, int timer_num)
{
  // TODO - stop each timer only once
  // stop timers
  for (int i=0; i < timer_num; i++) {
    if(timers_to_stop[i] == NP) return;
    timers_to_stop[i]->pause();
    timers_to_stop[i]->refresh();
    #ifdef SIMPLEFOC_STM32_DEBUG
      SIMPLEFOC_DEBUG("STM32-DRV: Stopping timer ", getTimerNumber(get_timer_index(timers_to_stop[i]->getHandle()->Instance)));
    #endif
  }
}

#if defined(STM32G4xx)
// function finds the appropriate timer source trigger for the master/slave timer combination
// returns -1 if no trigger source is found
// currently supports the master timers to be from TIM1 to TIM4 and TIM8
int _getInternalSourceTrigger(HardwareTimer* master, HardwareTimer* slave) { // put master and slave in temp variables to avoid arrows
  TIM_TypeDef *TIM_master = master->getHandle()->Instance;
  #if defined(TIM1) && defined(LL_TIM_TS_ITR0)
    if (TIM_master == TIM1) return LL_TIM_TS_ITR0;// return TIM_TS_ITR0;
  #endif
  #if defined(TIM2) &&  defined(LL_TIM_TS_ITR1)
    else if (TIM_master == TIM2) return LL_TIM_TS_ITR1;//return TIM_TS_ITR1;
  #endif
  #if defined(TIM3) &&  defined(LL_TIM_TS_ITR2)
    else if (TIM_master == TIM3) return LL_TIM_TS_ITR2;//return TIM_TS_ITR2;
  #endif  
  #if defined(TIM4) &&  defined(LL_TIM_TS_ITR3)
    else if (TIM_master == TIM4) return LL_TIM_TS_ITR3;//return TIM_TS_ITR3;
  #endif 
  #if defined(TIM5) &&  defined(LL_TIM_TS_ITR4)
    else if (TIM_master == TIM5) return LL_TIM_TS_ITR4;//return TIM_TS_ITR4;
  #endif
  #if defined(TIM8) &&  defined(LL_TIM_TS_ITR5)
    else if (TIM_master == TIM8) return LL_TIM_TS_ITR5;//return TIM_TS_ITR5;
  #endif
  return -1;
}
#elif defined(STM32F4xx) || defined(STM32F1xx) || defined(STM32L4xx) || defined(STM32F7xx)

// function finds the appropriate timer source trigger for the master/slave timer combination
// returns -1 if no trigger source is found
// currently supports the master timers to be from TIM1 to TIM4 and TIM8
int _getInternalSourceTrigger(HardwareTimer* master, HardwareTimer* slave) {
  // put master and slave in temp variables to avoid arrows
  TIM_TypeDef *TIM_master = master->getHandle()->Instance;
  TIM_TypeDef *TIM_slave = slave->getHandle()->Instance;
  #if defined(TIM1) && defined(LL_TIM_TS_ITR0)
    if (TIM_master == TIM1){
      #if defined(TIM2)
      if(TIM_slave == TIM2) return LL_TIM_TS_ITR0;
      #endif
      #if defined(TIM3)
      else if(TIM_slave == TIM3) return LL_TIM_TS_ITR0;
      #endif
      #if defined(TIM4)
      else if(TIM_slave == TIM4) return LL_TIM_TS_ITR0;
      #endif
      #if defined(TIM8)
      else if(TIM_slave == TIM8) return LL_TIM_TS_ITR0;
      #endif
    }
  #endif
  #if defined(TIM2) &&  defined(LL_TIM_TS_ITR1)
    else if (TIM_master == TIM2){
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR1;
      #endif
      #if defined(TIM3)
      else if(TIM_slave == TIM3) return LL_TIM_TS_ITR1;
      #endif
      #if defined(TIM4)
      else if(TIM_slave == TIM4) return LL_TIM_TS_ITR1;
      #endif
      #if defined(TIM8)
      else if(TIM_slave == TIM8) return LL_TIM_TS_ITR1;
      #endif
      #if defined(TIM5)
      else if(TIM_slave == TIM5) return LL_TIM_TS_ITR0;
      #endif
    }
  #endif
  #if defined(TIM3) &&  defined(LL_TIM_TS_ITR2)
    else if (TIM_master == TIM3){
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR2;
      #endif
      #if defined(TIM2)
      else if(TIM_slave == TIM2) return LL_TIM_TS_ITR2;
      #endif
      #if defined(TIM4)
      else if(TIM_slave == TIM4) return LL_TIM_TS_ITR2;
      #endif
      #if defined(TIM5)
      else if(TIM_slave == TIM5) return LL_TIM_TS_ITR1;
      #endif
    }
  #endif  
  #if defined(TIM4) &&  defined(LL_TIM_TS_ITR3)
    else if (TIM_master == TIM4){
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR3;
      #endif
      #if defined(TIM2)
      else if(TIM_slave == TIM2) return LL_TIM_TS_ITR3;
      #endif
      #if defined(TIM3)
      else if(TIM_slave == TIM3) return LL_TIM_TS_ITR3;
      #endif
      #if defined(TIM8)
      else if(TIM_slave == TIM8) return LL_TIM_TS_ITR2;
      #endif
      #if defined(TIM5)
      else if(TIM_slave == TIM5) return LL_TIM_TS_ITR1;
      #endif
    }
  #endif 
  #if defined(TIM5) 
    else if (TIM_master == TIM5){
      #if !defined(STM32L4xx) // only difference between F4,F1 and L4
      #if defined(TIM1)
      if(TIM_slave == TIM1) return LL_TIM_TS_ITR0;
      #endif
      #if defined(TIM3)
      else if(TIM_slave == TIM3) return LL_TIM_TS_ITR2;
      #endif
      #endif
      #if defined(TIM8)
      if(TIM_slave == TIM8) return LL_TIM_TS_ITR3;
      #endif
    }
  #endif
  #if defined(TIM8)
    else if (TIM_master == TIM8){
      #if defined(TIM2)
      if(TIM_slave==TIM2) return LL_TIM_TS_ITR1;
      #endif
      #if defined(TIM4)
      else if(TIM_slave == TIM4) return LL_TIM_TS_ITR3;
      #endif
      #if defined(TIM5)
      else if(TIM_slave == TIM5) return LL_TIM_TS_ITR3;
      #endif
    }
  #endif
  return -1; // combination not supported
}
#else
// Alignment not supported for this architecture
int _getInternalSourceTrigger(HardwareTimer* master, HardwareTimer* slave) {
  return -1;
}
#endif

void syncTimerFrequency(long pwm_frequency, HardwareTimer *timers[], uint8_t num_timers) {
  uint32_t max_frequency = 0;
  uint32_t min_frequency = UINT32_MAX;
  for (size_t i=0; i<num_timers; i++) {
    uint32_t freq = timers[i]->getTimerClkFreq();
    if (freq > max_frequency) {
      max_frequency = freq;
    } else if (freq < min_frequency) {
      min_frequency = freq; 
    }
  }
  if (max_frequency==min_frequency) return;
  uint32_t overflow_value = min_frequency/pwm_frequency;
  for (size_t i=0; i<num_timers; i++) {
    uint32_t prescale_factor = timers[i]->getTimerClkFreq()/min_frequency;
    #ifdef SIMPLEFOC_DEBUG
      SIMPLEFOC_DEBUG("STM32-DRV: Setting prescale to ", (float)prescale_factor);
      SIMPLEFOC_DEBUG("STM32-DRV: Setting Overflow to ", (float)overflow_value);
    #endif
    timers[i]->setPrescaleFactor(prescale_factor);
    timers[i]->setOverflow(overflow_value,TICK_FORMAT);
    timers[i]->refresh();
  }
}

void _alignTimersNew() {
  int numTimers = 0;
  HardwareTimer *timers[numTimerPinsUsed];

  // find the timers used
  for (int i=0; i<numTimerPinsUsed; i++) {
    uint32_t index = get_timer_index((TIM_TypeDef*)timerPinsUsed[i]->peripheral);
    HardwareTimer *timer = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);
    bool found = false;
    for (int j=0; j<numTimers; j++) {
      if (timers[j] == timer) {
        found = true;
        break;
      }
    }
    if (!found)
      timers[numTimers++] = timer;
  }

  #ifdef SIMPLEFOC_STM32_DEBUG
    SIMPLEFOC_DEBUG("STM32-DRV: Syncronising timers! Timer no. ", numTimers);
  #endif

  // see if there is more then 1 timers used for the pwm
  // if yes, try to align timers
  if(numTimers > 1){
    // find the master timer
    int16_t master_index = -1;
    int triggerEvent = -1;
    for (int i=0; i<numTimers; i++) {
      // check if timer can be master
      if(IS_TIM_MASTER_INSTANCE(timers[i]->getHandle()->Instance)) {
        // check if timer already configured in TRGO update mode (used for ADC triggering)
        // in that case we should not change its TRGO configuration
        if(timers[i]->getHandle()->Instance->CR2 & LL_TIM_TRGO_UPDATE) continue;
        // check if the timer has the supported internal trigger for other timers
        for (int slave_i=0; slave_i<numTimers; slave_i++) {
          if (i==slave_i) continue; // skip self
          // check if it has the supported internal trigger
          triggerEvent = _getInternalSourceTrigger(timers[i],timers[slave_i]); 
          if(triggerEvent == -1) break; // not supported keep searching
        }
        if(triggerEvent == -1) continue; // cannot be master, keep searching
        // otherwise the master has been found, remember the index
        master_index = i; // found the master timer
        break;
      }
    }
    
    // if no master timer found do not perform alignment
    if (master_index == -1) {
      #ifdef SIMPLEFOC_STM32_DEBUG
        SIMPLEFOC_DEBUG("STM32-DRV: ERR: No master timer found, cannot align timers!");
      #endif
    }else{
      #ifdef SIMPLEFOC_STM32_DEBUG
        SIMPLEFOC_DEBUG("STM32-DRV: Aligning PWM to master timer: ",  getTimerNumber(get_timer_index(timers[master_index]->getHandle()->Instance)));
      #endif
      // make the master timer generate ITRGx event
      // if it was already configured in slave mode
      LL_TIM_SetSlaveMode(timers[master_index]->getHandle()->Instance, LL_TIM_SLAVEMODE_DISABLED );
      // Configure the master  timer to send a trigger signal on enable 
      LL_TIM_SetTriggerOutput(timers[master_index]->getHandle()->Instance, LL_TIM_TRGO_ENABLE);
      // configure other timers to get the input trigger from the master timer
      for (int slave_index=0; slave_index < numTimers; slave_index++) {
        if (slave_index == master_index)
          continue;
        // Configure the slave timer to be triggered by the master enable signal
        LL_TIM_SetTriggerInput(timers[slave_index]->getHandle()->Instance, _getInternalSourceTrigger(timers[master_index], timers[slave_index]));
        LL_TIM_SetSlaveMode(timers[slave_index]->getHandle()->Instance, LL_TIM_SLAVEMODE_TRIGGER);
      }
    }
  }

  // enable timer clock
  for (int i=0; i<numTimers; i++) {
    timers[i]->pause();
    // timers[i]->refresh();
    #ifdef SIMPLEFOC_STM32_DEBUG
      SIMPLEFOC_DEBUG("STM32-DRV: Restarting timer ", getTimerNumber(get_timer_index(timers[i]->getHandle()->Instance)));
    #endif
  }

  for (int i=0; i<numTimers; i++) {
    timers[i]->resume();
  }

}

// align the timers to end the init
void _startTimers(HardwareTimer **timers_to_start, int timer_num)
{
  // // TODO - start each timer only once
  // // start timers
  // for (int i=0; i < timer_num; i++) {
  //   if(timers_to_start[i] == NP) return;
  //   timers_to_start[i]->resume();
  //   #ifdef SIMPLEFOC_STM32_DEBUG
  //     SIMPLEFOC_DEBUG("STM32-DRV: Starting timer ", getTimerNumber(get_timer_index(timers_to_start[i]->getHandle()->Instance)));
  //   #endif
  // }
  _alignTimersNew();
}

/*
  timer combination scoring function
  assigns a score, and also checks the combination is valid
  returns <0 if combination is invalid, >=0 if combination is valid. lower (but positive) score is better
  for 6 pwm, hardware 6-pwm is preferred over software 6-pwm
  hardware 6-pwm is possible if each low channel is the inverted counterpart of its high channel
  inverted channels are not allowed except when using hardware 6-pwm (in theory they could be but lets not complicate things)
*/
int scoreCombination(int numPins, PinMap* pinTimers[]) {
  // check already used - TODO move this to outer loop also...
  for (int i=0; i<numTimerPinsUsed; i++) {
    if (pinTimers[i]->peripheral == timerPinsUsed[i]->peripheral
        && STM_PIN_CHANNEL(pinTimers[i]->function) == STM_PIN_CHANNEL(timerPinsUsed[i]->function))
      return -2; // bad combination - timer channel already used
  }
  
  // TODO LPTIM and HRTIM should be ignored for now
  
  // check for inverted channels
  if (numPins < 6) {
    for (int i=0; i<numPins; i++) {
      if (STM_PIN_INVERTED(pinTimers[i]->function))
        return -3; // bad combination - inverted channel used in non-hardware 6pwm
    }
  }
  // check for duplicated channels
  for (int i=0; i<numPins-1; i++) {
    for (int j=i+1; j<numPins; j++) {
      if (pinTimers[i]->peripheral == pinTimers[j]->peripheral
          && STM_PIN_CHANNEL(pinTimers[i]->function) == STM_PIN_CHANNEL(pinTimers[j]->function)
          && STM_PIN_INVERTED(pinTimers[i]->function) == STM_PIN_INVERTED(pinTimers[j]->function))
        return -4; // bad combination - duplicated channel
    }
  }
  int score = 0;
  for (int i=0; i<numPins; i++) {
    // count distinct timers
    bool found = false;
    for (int j=i+1; j<numPins; j++) {
      if (pinTimers[i]->peripheral == pinTimers[j]->peripheral)
        found = true;
    }
    if (!found) score++;
  }
  if (numPins==6) {
    // check for inverted channels - best: 1 timer, 3 channels, 3 matching inverted channels
    //                                     >1 timer, 3 channels, 3 matching inverted channels
    //                                     1 timer, 6 channels (no inverted channels)
    //                                     >1 timer, 6 channels (no inverted channels)
    // check for inverted high-side channels - TODO is this a configuration we should allow? what if all 3 high side channels are inverted and the low-side non-inverted?
    if (STM_PIN_INVERTED(pinTimers[0]->function) || STM_PIN_INVERTED(pinTimers[2]->function) || STM_PIN_INVERTED(pinTimers[4]->function))
      return -5; // bad combination - inverted channel used on high-side channel
    if (pinTimers[0]->peripheral == pinTimers[1]->peripheral
        && pinTimers[2]->peripheral == pinTimers[3]->peripheral
        && pinTimers[4]->peripheral == pinTimers[5]->peripheral
        && STM_PIN_CHANNEL(pinTimers[0]->function) == STM_PIN_CHANNEL(pinTimers[1]->function)
        && STM_PIN_CHANNEL(pinTimers[2]->function) == STM_PIN_CHANNEL(pinTimers[3]->function)
        && STM_PIN_CHANNEL(pinTimers[4]->function) == STM_PIN_CHANNEL(pinTimers[5]->function)
        && STM_PIN_INVERTED(pinTimers[1]->function) && STM_PIN_INVERTED(pinTimers[3]->function) && STM_PIN_INVERTED(pinTimers[5]->function)) {
          // hardware 6pwm, score <10

          // TODO F37xxx doesn't support dead-time insertion, it has no TIM1/TIM8
          // F301, F302 --> 6 channels, but only 1-3 have dead-time insertion
          // TIM2/TIM3/TIM4/TIM5 don't do dead-time insertion
          // TIM15/TIM16/TIM17 do dead-time insertion only on channel 1

          // TODO check these defines
          #if defined(STM32F4xx_HAL_TIM_H) || defined(STM32F3xx_HAL_TIM_H) || defined(STM32F2xx_HAL_TIM_H) || defined(STM32F1xx_HAL_TIM_H) || defined(STM32F100_HAL_TIM_H) || defined(STM32FG0x1_HAL_TIM_H)  || defined(STM32G0x0_HAL_TIM_H) 
          if (STM_PIN_CHANNEL(pinTimers[0]->function)>3 || STM_PIN_CHANNEL(pinTimers[2]->function)>3 || STM_PIN_CHANNEL(pinTimers[4]->function)>3 )
            return -8; // channel 4 does not have dead-time insertion
          #endif
          #ifdef STM32G4xx_HAL_TIM_H
          if (STM_PIN_CHANNEL(pinTimers[0]->function)>4 || STM_PIN_CHANNEL(pinTimers[2]->function)>4 || STM_PIN_CHANNEL(pinTimers[4]->function)>4 )
            return -8; // channels 5 & 6 do not have dead-time insertion
          #endif
        }
    else {
      // check for inverted low-side channels
      if (STM_PIN_INVERTED(pinTimers[1]->function) || STM_PIN_INVERTED(pinTimers[3]->function) || STM_PIN_INVERTED(pinTimers[5]->function))
        return -6; // bad combination - inverted channel used on low-side channel in software 6-pwm
      if (pinTimers[0]->peripheral != pinTimers[1]->peripheral
        || pinTimers[2]->peripheral != pinTimers[3]->peripheral
        || pinTimers[4]->peripheral != pinTimers[5]->peripheral)
        return -7; // bad combination - non-matching timers for H/L side in software 6-pwm
      score += 10; // software 6pwm, score >10
    }
  }
  return score;
}

int findIndexOfFirstPinMapEntry(int pin) {
  PinName pinName = digitalPinToPinName(pin);
  int i = 0;
  while (PinMap_TIM[i].pin!=NC) {
    if (pinName == PinMap_TIM[i].pin)
      return i;
    i++;
  }
  return -1;
}

int findIndexOfLastPinMapEntry(int pin) {
  PinName pinName = digitalPinToPinName(pin);
  int i = 0;
  while (PinMap_TIM[i].pin!=NC) {
    if (   pinName == (PinMap_TIM[i].pin & ~ALTX_MASK) 
        && pinName != (PinMap_TIM[i+1].pin & ~ALTX_MASK))
      return i;
    i++;
  }
  return -1;
}

#define NOT_FOUND 1000

int findBestTimerCombination(int numPins, int index, int pins[], PinMap* pinTimers[]) {
  PinMap* searchArray[numPins];
  for (int j=0;j<numPins;j++)
        searchArray[j] = pinTimers[j];
  int bestScore = NOT_FOUND;
  int startIndex = findIndexOfFirstPinMapEntry(pins[index]);
  int endIndex = findIndexOfLastPinMapEntry(pins[index]);
  if (startIndex == -1 || endIndex == -1) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: no timer on pin ", pins[index]);
    return -1; // pin is not connected to any timer
  }
  for (int i=startIndex;i<=endIndex;i++) {
    searchArray[index] = (PinMap*)&PinMap_TIM[i];
    int score = NOT_FOUND;
    if (index<numPins-1)
      score = findBestTimerCombination(numPins, index+1, pins, searchArray);
    else {
      score = scoreCombination(numPins, searchArray);
      #ifdef SIMPLEFOC_STM32_DEBUG
      printTimerCombination(numPins, searchArray, score);
      #endif
    }
    if (score==-1)
      return -1; // pin not connected to any timer, propagate driectly
    if (score>=0 && score<bestScore) {
      bestScore = score;
      for (int j=index;j<numPins;j++)
        pinTimers[j] = searchArray[j];
    }
  }
  return bestScore;
}

int findBestTimerCombination(int numPins, int pins[], PinMap* pinTimers[]) {
  int bestScore = findBestTimerCombination(numPins, 0, pins, pinTimers);
  if (bestScore == NOT_FOUND) {
    #ifdef SIMPLEFOC_STM32_DEBUG
    SimpleFOCDebug::println("STM32-DRV: no workable combination found on these pins");
    #endif
    return -10; // no workable combination found
  }
  else if (bestScore >= 0) {
    #ifdef SIMPLEFOC_STM32_DEBUG
    SimpleFOCDebug::print("STM32-DRV: best: ");
    printTimerCombination(numPins, pinTimers, bestScore);
    #endif
  }
  return bestScore;
};

TIM_MasterConfigTypeDef sMasterConfig;
TIM_SlaveConfigTypeDef sSlaveConfig;

void* _configure3PWM(long pwm_frequency,const int pinA, const int pinB, const int pinC) {
  if (numTimerPinsUsed+3 > SIMPLEFOC_STM32_MAX_PINTIMERSUSED) {
    SIMPLEFOC_DEBUG("STM32-DRV: ERR: too many pins used");
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;
  }
  if( !pwm_frequency || !_isset(pwm_frequency) ) pwm_frequency = _PWM_FREQUENCY; // default frequency 25khz
  else pwm_frequency = _constrain(pwm_frequency, 0, _PWM_FREQUENCY_MAX); // constrain to 50kHz max
  // center-aligned frequency is uses two periods
  pwm_frequency *=2;

  int pins[3] = { pinA, pinB, pinC };
  PinMap* pinTimers[3] = { NP, NP, NP };
  if (findBestTimerCombination(3, pins, pinTimers)<0)
    return (STM32DriverParams*)SIMPLEFOC_DRIVER_INIT_FAILED;

  HardwareTimer* HT1 = _initPinPWM(pwm_frequency, pinTimers[0]);
  HardwareTimer* HT2 = _initPinPWM(pwm_frequency, pinTimers[1]);
  HardwareTimer* HT3 = _initPinPWM(pwm_frequency, pinTimers[2]);

  HardwareTimer *timers[3] = {HT1, HT2, HT3};
  syncTimerFrequency(pwm_frequency, timers, 3); 

  uint32_t channel1 = STM_PIN_CHANNEL(pinTimers[0]->function);
  uint32_t channel2 = STM_PIN_CHANNEL(pinTimers[1]->function);
  uint32_t channel3 = STM_PIN_CHANNEL(pinTimers[2]->function);

  STM32DriverParams* params = new STM32DriverParams {
    .timers = { HT1, HT2, HT3 },
    .channels = { channel1, channel2, channel3 },
    .pwm_frequency = pwm_frequency
  };
  timerPinsUsed[numTimerPinsUsed++] = pinTimers[0];
  timerPinsUsed[numTimerPinsUsed++] = pinTimers[1];
  timerPinsUsed[numTimerPinsUsed++] = pinTimers[2];

  _alignTimersNew();

  return params;
}

// function setting the pwm duty cycle to the hardware
// - BLDC motor - 3PWM setting
//- hardware speciffic
void _writeDutyCycle3PWM(float dc_a,  float dc_b, float dc_c, void* params){
  // transform duty cycle from [0,1] to [0,4095]
  _setPwm(((STM32DriverParams*)params)->timers[0], ((STM32DriverParams*)params)->channels[0], _PWM_RANGE*dc_a, _PWM_RESOLUTION);
  _setPwm(((STM32DriverParams*)params)->timers[1], ((STM32DriverParams*)params)->channels[1], _PWM_RANGE*dc_b, _PWM_RESOLUTION);
  _setPwm(((STM32DriverParams*)params)->timers[2], ((STM32DriverParams*)params)->channels[2], _PWM_RANGE*dc_c, _PWM_RESOLUTION);
}

void _setSinglePhaseState(PhaseState state, HardwareTimer *HT, int channel1,int channel2) {
  _UNUSED(channel2);
  switch (state) {
    case PhaseState::PHASE_OFF:
      // Due to a weird quirk in the arduino timer API, pauseChannel only disables the complementary channel (e.g. CC1NE).
      // To actually make the phase floating, we must also set pwm to 0.
      HT->pauseChannel(channel1);
      break;
    default:
      HT->resumeChannel(channel1);
      break;
  }
}

int getTimerNumber(int timerIndex) {
  #if defined(TIM1_BASE)
    if (timerIndex==TIMER1_INDEX) return 1;
  #endif
  #if defined(TIM2_BASE)
    if (timerIndex==TIMER2_INDEX) return 2;
  #endif
  #if defined(TIM3_BASE)
    if (timerIndex==TIMER3_INDEX) return 3;
  #endif
  #if defined(TIM4_BASE)
    if (timerIndex==TIMER4_INDEX) return 4;
  #endif
  #if defined(TIM5_BASE)
    if (timerIndex==TIMER5_INDEX) return 5;
  #endif
  #if defined(TIM6_BASE)
    if (timerIndex==TIMER6_INDEX) return 6;
  #endif
  #if defined(TIM7_BASE)
    if (timerIndex==TIMER7_INDEX) return 7;
  #endif
  #if defined(TIM8_BASE)
    if (timerIndex==TIMER8_INDEX) return 8;
  #endif
  #if defined(TIM9_BASE)
    if (timerIndex==TIMER9_INDEX) return 9;
  #endif
  #if defined(TIM10_BASE)
    if (timerIndex==TIMER10_INDEX) return 10;
  #endif
  #if defined(TIM11_BASE)
    if (timerIndex==TIMER11_INDEX) return 11;
  #endif
  #if defined(TIM12_BASE)
    if (timerIndex==TIMER12_INDEX) return 12;
  #endif
  #if defined(TIM13_BASE)
    if (timerIndex==TIMER13_INDEX) return 13;
  #endif
  #if defined(TIM14_BASE)
    if (timerIndex==TIMER14_INDEX) return 14;
  #endif
  #if defined(TIM15_BASE)
    if (timerIndex==TIMER15_INDEX) return 15;
  #endif
  #if defined(TIM16_BASE)
    if (timerIndex==TIMER16_INDEX) return 16;
  #endif
  #if defined(TIM17_BASE)
    if (timerIndex==TIMER17_INDEX) return 17;
  #endif
  #if defined(TIM18_BASE)
    if (timerIndex==TIMER18_INDEX) return 18;
  #endif
  #if defined(TIM19_BASE)
    if (timerIndex==TIMER19_INDEX) return 19;
  #endif
  #if defined(TIM20_BASE)
    if (timerIndex==TIMER20_INDEX) return 20;
  #endif
  #if defined(TIM21_BASE)
    if (timerIndex==TIMER21_INDEX) return 21;
  #endif
  #if defined(TIM22_BASE)
    if (timerIndex==TIMER22_INDEX) return 22;
  #endif
  return -1;
}

void printTimerCombination(int numPins, PinMap* timers[], int score) {
  for (int i=0; i<numPins; i++) {
    if (timers[i] == NP)
      SimpleFOCDebug::print("NP");
    else {
      SimpleFOCDebug::print("TIM");
      SimpleFOCDebug::print(getTimerNumber(get_timer_index((TIM_TypeDef*)timers[i]->peripheral)));
      SimpleFOCDebug::print("-CH");
      SimpleFOCDebug::print(STM_PIN_CHANNEL(timers[i]->function));
      if (STM_PIN_INVERTED(timers[i]->function))
        SimpleFOCDebug::print("N");
    }
    SimpleFOCDebug::print(" ");
  }
  SimpleFOCDebug::println("score: ", score);
}

class GigaBLDCDriver3PWM: public BLDCDriver
{
  public:

  GigaBLDCDriver3PWM(int phA,int phB,int phC, int en1 = NOT_SET, int en2 = NOT_SET, int en3 = NOT_SET) {
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
    
  int init() override {
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
    params = _configure3PWM(pwm_frequency, pwmA, pwmB, pwmC);
    initialized = (params != SIMPLEFOC_DRIVER_INIT_FAILED);
    return params != SIMPLEFOC_DRIVER_INIT_FAILED;
  }

  void disable() override {
    // set zero to PWM
    setPwm(0, 0, 0);
    // disable the driver - if enable_pin pin available
    if ( _isset(enableA_pin) ) digitalWrite(enableA_pin, !enable_active_high);
    if ( _isset(enableB_pin) ) digitalWrite(enableB_pin, !enable_active_high);
    if ( _isset(enableC_pin) ) digitalWrite(enableC_pin, !enable_active_high);
  }

  void enable() override {
    // enable_pin the driver - if enable_pin pin available
    if ( _isset(enableA_pin) ) digitalWrite(enableA_pin, enable_active_high);
    if ( _isset(enableB_pin) ) digitalWrite(enableB_pin, enable_active_high);
    if ( _isset(enableC_pin) ) digitalWrite(enableC_pin, enable_active_high);
    // set zero to PWM
    setPwm(0,0,0);
  }

  void setPwm(float Ua, float Ub, float Uc) override {
    // limit the voltage in driver
    Ua = _constrain(Ua, 0.0f, voltage_limit);
    Ub = _constrain(Ub, 0.0f, voltage_limit);
    Uc = _constrain(Uc, 0.0f, voltage_limit);
    // calculate duty cycle
    // limited in [0,1]
    dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
    dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
    dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

    // hardware specific writing
    // hardware specific function - depending on driver and mcu
    _writeDutyCycle3PWM(dc_a, dc_b, dc_c, params);
  }

  virtual void setPhaseState(PhaseState sa, PhaseState sb, PhaseState sc) override {
    // disable if needed
    if( _isset(enableA_pin) &&  _isset(enableB_pin)  && _isset(enableC_pin) ){
      digitalWrite(enableA_pin, sa == PhaseState::PHASE_ON ? enable_active_high : !enable_active_high);
      digitalWrite(enableB_pin, sb == PhaseState::PHASE_ON ? enable_active_high : !enable_active_high);
      digitalWrite(enableC_pin, sc == PhaseState::PHASE_ON ? enable_active_high : !enable_active_high);
    }
  }

  int pwmA; //!< phase A pwm pin number
  int pwmB; //!< phase B pwm pin number
  int pwmC; //!< phase C pwm pin number
  int enableA_pin; //!< enable pin number
  int enableB_pin; //!< enable pin number
  int enableC_pin; //!< enable pin number
};

#endif

