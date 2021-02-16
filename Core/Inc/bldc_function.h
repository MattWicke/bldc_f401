#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_gpio.h"

/* 
 * basically we want to connect the motor phases to
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

/*
 * note: this function is inlined because it will be 
 * called from an interupt
 */
inline void TIM_CCxNChannelCmd(TIM_TypeDef *TIMx
        , uint32_t Channel, uint32_t ChannelNState)
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

static inline void bldc_update(TIM_HandleTypeDef *pwm_timer);
/*
 * note: this function is inlined because it will be 
 * called from an interupt
 */
inline void bldc_update(TIM_HandleTypeDef *pwm_timer)
{
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


  /* this is a mask for accessing the OCxM bits that
   * control the timer output state
   */
  uint32_t temp_CCMR1 = 0;
  uint32_t temp_CCMR2 = 0;
  /* store temp reg values, write to them and push at the end */
  temp_CCMR1 = pwm_timer->Instance->CCMR1;
  temp_CCMR2 = pwm_timer->Instance->CCMR2;
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
   TIM_CCxChannelCmd(pwm_timer->Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
   TIM_CCxChannelCmd(pwm_timer->Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
   TIM_CCxChannelCmd(pwm_timer->Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
  
   TIM_CCxNChannelCmd(pwm_timer->Instance, TIM_CHANNEL_1, TIM_CCxN_ENABLE);
   TIM_CCxNChannelCmd(pwm_timer->Instance, TIM_CHANNEL_2, TIM_CCxN_ENABLE);
   TIM_CCxNChannelCmd(pwm_timer->Instance, TIM_CHANNEL_3, TIM_CCxN_ENABLE);


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
	  TIM_CCxNChannelCmd(pwm_timer->Instance, PHASE_W2, TIM_CCxN_DISABLE);
	  TIM_CCxChannelCmd(pwm_timer->Instance, PHASE_V1, TIM_CCx_DISABLE);
      
      /* W = chan3 set to pwm1 mode */
      temp_CCMR2 |= PWM_CHAN_W_MODE1;
      /* V = chan2 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_V_MODE1;

      temp_CCMR1 |= PWM_CHAN_U_FORCE_INACTIVE;
	  TIM_CCxChannelCmd(pwm_timer->Instance, PHASE_U1, TIM_CCx_DISABLE);

	  break;
  case 4:
	   /* Hall CBA = 100, Phase U1, V2 = high */
	  TIM_CCxNChannelCmd(pwm_timer->Instance, PHASE_U2, TIM_CCxN_DISABLE);
	  TIM_CCxChannelCmd(pwm_timer->Instance, PHASE_V1, TIM_CCx_DISABLE);

      /* U = chan3 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_U_MODE1;
      /* V = chan2 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_V_MODE1;
      
      temp_CCMR2 |= PWM_CHAN_W_FORCE_INACTIVE;
	  TIM_CCxChannelCmd(pwm_timer->Instance, PHASE_W1, TIM_CCx_DISABLE);

	  break;
  case 5:
	  /* Hall CBA = 101, Phase W2, U1 = high */
	  TIM_CCxChannelCmd(pwm_timer->Instance, PHASE_W1, TIM_CCx_DISABLE);
	  TIM_CCxNChannelCmd(pwm_timer->Instance, PHASE_U2, TIM_CCxN_DISABLE);

      /* W = chan3 set to pwm1 mode */
      temp_CCMR2 |= PWM_CHAN_W_MODE1;
      /* U = chan2 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_U_MODE1;

      temp_CCMR1 |= PWM_CHAN_V_FORCE_INACTIVE;
	  TIM_CCxChannelCmd(pwm_timer->Instance, PHASE_V1, TIM_CCx_DISABLE);

	  break;
  case 1:
	  /* Hall CBA = 001, Phase W2, V1 = high */
	  TIM_CCxChannelCmd(pwm_timer->Instance, PHASE_W1, TIM_CCx_DISABLE);
	  TIM_CCxNChannelCmd(pwm_timer->Instance, PHASE_V2, TIM_CCxN_DISABLE);

      /* W = chan3 set to pwm1 mode */
      temp_CCMR2 |= PWM_CHAN_W_MODE1;
      /* V = chan2 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_V_MODE1;

      temp_CCMR1 |= PWM_CHAN_U_FORCE_INACTIVE;
	  TIM_CCxChannelCmd(pwm_timer->Instance, PHASE_U1, TIM_CCx_DISABLE);
      
	  break;
  case 3:
	  /* Hall CBA = 011, Phase V1, U2 = high */
	  TIM_CCxNChannelCmd(pwm_timer->Instance, PHASE_V2, TIM_CCxN_DISABLE);
	  TIM_CCxChannelCmd(pwm_timer->Instance, PHASE_U1, TIM_CCx_DISABLE);

      /* V = chan3 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_V_MODE1;
      /* U = chan2 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_U_MODE1;

      temp_CCMR2 |= PWM_CHAN_W_FORCE_INACTIVE;
	  TIM_CCxChannelCmd(pwm_timer->Instance, PHASE_W1, TIM_CCx_DISABLE);

	  break;
  case 2:
	  /* Hall CBA = 010, Phase W1, U2 = high */
	  TIM_CCxNChannelCmd(pwm_timer->Instance, PHASE_W2, TIM_CCxN_DISABLE);
	  TIM_CCxChannelCmd(pwm_timer->Instance, PHASE_U1, TIM_CCx_DISABLE);

      /* W = chan3 set to pwm1 mode */
      temp_CCMR2 |= PWM_CHAN_W_MODE1;
      /* U = chan2 set to pwm1 mode */
      temp_CCMR1 |= PWM_CHAN_U_MODE1;

      temp_CCMR1 |= PWM_CHAN_V_FORCE_INACTIVE;
	  TIM_CCxChannelCmd(pwm_timer->Instance, PHASE_V1, TIM_CCx_DISABLE);

	  break;
  }

  /*
   * write the control bits for pwm-mode/force output 
   */
  pwm_timer->Instance->CCMR1 = temp_CCMR1;
  pwm_timer->Instance->CCMR2 = temp_CCMR2;
}

