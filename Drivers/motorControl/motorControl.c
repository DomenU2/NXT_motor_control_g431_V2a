/*
 * motorControl.c
 *
 *  Created on: Jul 19, 2024
 *      Author: Domen
 */
#include "motorControl.h"



//Variables

uint8_t motorEnable = 0;
uint8_t motorDriverEnable = 0;
uint8_t IN1_H = 0; //test drv8833
uint8_t IN2_H = 0; //test drv8833

Motor_state_t motor_state1={0};
Motor_state_t motor_state2={0};


int16_t ADC_max = 4095;

int32_t speed_ramp=0;
int32_t ramp_step = 5;

//Private variables
//debug
float limitHighDbg = 360; //deg
float position_deg = 0;
float limitLowDbg = 0; //deg

//Functions

void InitMotorControl(Motor_state_t *ms1,Motor_state_t *ms2){


	TIM_HandleTypeDef *p_encT1 = &htim3;
	TIM_HandleTypeDef *p_encT2 = &htim4;
	TIM_HandleTypeDef *p_pwmT1 = &htim1;
	TIM_HandleTypeDef *p_pwmT2 = &htim2;


	  //ENCODER MOTOR 1
	  HAL_TIM_Encoder_Start(p_encT1,TIM_CHANNEL_ALL);
	  //ENCODER MOTOR 2
	  HAL_TIM_Encoder_Start(p_encT2,TIM_CHANNEL_ALL);
	  //PWM MOTOR 1
	  HAL_TIM_PWM_Start(p_pwmT1, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(p_pwmT1, TIM_CHANNEL_3);
	  //PWM MOTOR 2
	  HAL_TIM_PWM_Start(p_pwmT2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(p_pwmT2 , TIM_CHANNEL_2);

    //definicija vrednosti konstant
	//PID variables
    ms1->motorID = 0;
    ms1->p_encoderTimer = p_encT1;
    ms1->p_pwmTimer = p_pwmT1;
    ms1->posLimMin = -PI;
    ms1->posLimMax = PI;
    ms1->error_max = ms1->posLimMax - ms1->posLimMin;
    ms1->Kpp=1000; //6000
    ms1->Tip=0;
    ms1->Tdp=0; //500
	///PWM variables
    ms1->PWM_period = p_pwmT1->Init.Period;


    ms2->motorID = 1;
    ms2->p_encoderTimer = p_encT2;
	ms2->p_pwmTimer = p_pwmT2;
	ms2->posLimMin = -3*PI;
	ms2->posLimMax = 3*PI;
	ms2->error_max = ms2->posLimMax - ms2->posLimMin;
	ms2->Kpp=1000;
	ms2->Tip=0;
	ms2->Tdp=0;
	///PWM variables
	ms2->PWM_period = p_pwmT2->Init.Period;
}

int Set_Duty_Cycle(Motor_state_t *ms, uint16_t dc1,uint16_t dc2){


		  int rtn = 0;
		  switch (ms->motorID){
		  case 0:
			  ms->p_pwmTimer->Instance->CCR2 = dc1;
			  ms->p_pwmTimer->Instance->CCR3 = dc2;
			  break;
		  case 1:
			  ms->p_pwmTimer->Instance->CCR1 = dc1;
			  ms->p_pwmTimer->Instance->CCR2 = dc2;
			  break;
		  default:
			  rtn = -1;
			  break;
	}
		  return rtn;
}
void PWM_Control_Motor(Motor_state_t *ms, int16_t speed){


	ms->dir = speed >=0 ? 1: -1;
	ms->duty_cycle = speed * ms->dir;

	if(ms->duty_cycle>ms->PWM_period){ms->duty_cycle=ms->PWM_period;}
	else if(ms->duty_cycle<100){ms->duty_cycle=0;}

	if(motorDriverEnable==1){
		HAL_GPIO_WritePin(MOT_SLEEP_GPIO_Port, MOT_SLEEP_Pin, GPIO_PIN_SET);
		//Pull sleep pin on DRV8833 HIGH
	}else{
		HAL_GPIO_WritePin(MOT_SLEEP_GPIO_Port, MOT_SLEEP_Pin, GPIO_PIN_RESET);
	}


if(motorEnable==1){
	//Pull sleep pin on DRV8833 HIGH

	if(ms->dir==1){
		Set_Duty_Cycle(ms, ms->duty_cycle, 0);
	}
	else if(ms->dir==-1){
		Set_Duty_Cycle(ms, 0, ms->duty_cycle);
	}
}
else{
		Set_Duty_Cycle(ms, 0, 0);
	}
}


void UpdateEncoder(Motor_state_t *ms){



	uint32_t temp_counter = __HAL_TIM_GET_COUNTER(ms->p_encoderTimer);
	  if(temp_counter == ms->last_counter_value)
	  {
	    ms->tick_velocity = 0;
	  }
	  else if(temp_counter > ms->last_counter_value)
	  {
	    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(ms->p_encoderTimer))
	    {
	      ms->tick_velocity = -ms->last_counter_value -
		(__HAL_TIM_GET_AUTORELOAD(ms->p_encoderTimer)-temp_counter);
	    }
	    else
	    {
	      ms->tick_velocity = temp_counter -
	           ms->last_counter_value;
	    }
	  }
	  else
	  {
	    if (__HAL_TIM_IS_TIM_COUNTING_DOWN(ms->p_encoderTimer))
	    {
		ms->tick_velocity = temp_counter -
	            ms->last_counter_value;
	    }
	    else
	    {
		ms->tick_velocity = temp_counter +
		(__HAL_TIM_GET_AUTORELOAD(ms->p_encoderTimer) -
	              ms->last_counter_value);
	    }
	   }
	ms->tick_position += ms->tick_velocity;
	ms->last_counter_value = temp_counter;

	//Sprememba predznaka zaradi vezave enkoderjev?
	ms->position = (float)ms->tick_position * DIRECTION_SIGN* 2* PI / TICK_PER_REV; //[rad]

	//velocity and low pass filter
	ms->velocity = (float)ms->tick_velocity * DIRECTION_SIGN * 2* PI/ (TICK_PER_REV * Ts); //[rad/s]

	//Low pass filter 10rad/s
	ms->velocity_f =  0.9048 *  ms->velocity_f_k_1 + 0.09516 * ms->velocity_k_1;
	ms->velocity_k_1=ms->velocity;
	ms->velocity_f_k_1 = ms->velocity_f;
	ms->velocity = ms->velocity_f;

	//debug
	position_deg = RAD_2_DEG(ms->position);
	}


void PD_position(Motor_state_t *ms){

 //Regulacijska napaka in omejitve
/*
if(position_ref>0 && position_ref_k_1>=0)
	{
		   error= position_ref - position;
		   position_ref_k_1=position_ref;

	}else if(position_ref<0 && position_ref_k_1<=0)
	{
		 error= position_ref - position;
		 position_ref_k_1=position_ref;
	}else
	{
	  position_ref=0; //Reset value
		position_ref_k_1=0;
	}
*/
	//
	ms->error= ms->position_ref - ms->position;
	ms->position_ref_k_1=ms->position_ref;
	float P,I,D;
	if(ms->error<=ms->error_max && ms->error>=-ms->error_max){
		//P - proporcionalni del

		P = ms->Kpp * ms->error;

		//I - Integralni del
		// I =  I + error * Ts; //Integracija
		// Id = 1/(Tip) * I * 0;

		//D - Diferencialni del
		D =    ms->Tdp/Ts * (ms->error - ms->error_k_1);


		// Izhod regulatorja
	//	u_k = P +Id + D;
		ms->u_k = P + D;
		ms->error_k_1 = ms->error;

		}
else{
	ms->u_k = 0;
}


#if 1
	PWM_Control_Motor(ms, ms->u_k);
#endif

}//End PID position

//set position reference with potenciometeer
void SetPosRefAd(Motor_state_t *ms, uint16_t ad_val){

	ms->position_ref = (ms->posLimMax - ms->posLimMin) /ADC_max * (float)ad_val + ms->posLimMin;

}
int16_t speed = 0;
int16_t dir = 0;

// open loop control
void SetSpeedAd(Motor_state_t *ms, uint16_t ad_val){
		UpdateEncoder(ms);
		int16_t speed = (int16_t)2 * (float)ms->PWM_period/ADC_max*ad_val-ms->PWM_period;
		PWM_Control_Motor(ms, speed);
/*
		if(ad_val >ADC_max * 0.51){
		PWM_Control_Motor(ms, ms->PWM_period);
		}
		else if(ad_val <ADC_max * 0.49){
				PWM_Control_Motor(ms, -ms->PWM_period);
		}
		else{
			    PWM_Control_Motor(ms, 0);
		}
*/
}
//on off test of drv8833
void test_DRV833(Motor_state_t *ms){

uint32_t dc1, dc2;
if(IN1_H == 1){dc1 = ms->PWM_period;} else {dc1=0;}
if(IN2_H == 1){dc2 = ms->PWM_period;} else {dc2=0;}
Set_Duty_Cycle(ms, dc1, dc2);
}

void speed_ramp_test(Motor_state_t *ms){

	static int8_t sgn = 1;
	speed_ramp+=sgn * ramp_step;
	if((speed_ramp > ms->PWM_period) || (speed_ramp < -ms->PWM_period)){
		sgn = - sgn;
	}
PWM_Control_Motor(ms, speed_ramp);
}
// en korak nadzorne zanke
void MotorControl(Motor_state_t *ms){
	UpdateEncoder(ms);
	PD_position(ms);
}
