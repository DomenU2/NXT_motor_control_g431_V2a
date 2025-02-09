#ifndef _MOTOR_CONTROL_H
#define _MOTOR_CONTROL_H
#include "main.h"
#include <math.h>

//DEFINES
//#define TICK_PER_REV 720 //n of encoder ticks per one revolution of axis
#define TICK_PER_REV 360 //n of encoder ticks per one revolution of axis
#define DIRECTION_SIGN -1
#define PI 3.141592f
#define RAD_2_DEG(x) (x*180/PI)

#define MOTOR_ID_1 1
#define MOTOR_ID_2 2

// assume 10 ms sample time
#define Ts 0.01f //Unit [s]

//Types


typedef struct{

	uint8_t motorID;

	// ENCODER
	TIM_HandleTypeDef *p_encoderTimer;
	int16_t tick_velocity;
	int64_t tick_position;

	uint32_t last_counter_value;


	//position
	float velocity;
	float velocity_f; //filtered velocity
	float position;
	int8_t pos_sign;

	//velocity
	float velocity_k_1;
	float velocity_f_k_1;


	//PID variables
	float position_ref;
	float posLimMin;
	float posLimMax;
	float position_ref_k_1;
	float error;
	float error_k_1;

	float error_max;
	float u_k;
	float u_k_1;
	//float Kpp=0.825, Tip=1000, Tdp=0.0184;
	float Kpp;
	float Tip;
	float Tdp;

	///PWM variables
	TIM_HandleTypeDef *p_pwmTimer; //Pointer to HAL timer handle
	uint16_t duty_cycle;
	int8_t dir; //dir 1 of -1
	int16_t PWM_period;
	int8_t pwm_sign;





}Motor_state_t ;

//VARIABLES
extern Motor_state_t motor_state1;
extern Motor_state_t motor_state2;

extern uint8_t motorEnable;
extern uint8_t motorDriverEnable;

void InitMotorControl(Motor_state_t *ms1,Motor_state_t *ms2);

int Set_Duty_Cycle(Motor_state_t *ms, uint16_t dc1,uint16_t dc2);

void PWM_Control_Motor(Motor_state_t *ms, int16_t speed);


// code counts the ticks and deals with uint16_t wrap around
// source: https://www.steppeschool.com/pages/blog/stm32-timer-encoder-mode
void UpdateEncoder(Motor_state_t *ms);

// PD position control;
void PD_position(Motor_state_t *ms);

void RefTraj1(Motor_state_t *ms);


//Set position reference with AD input
 void SetPosRefAd(Motor_state_t *ms, uint16_t ad_val);

 void SetSpeedAd(Motor_state_t *ms, uint16_t ad_val);

 void speed_ramp_test(Motor_state_t *ms);

 void MotorControl(Motor_state_t *ms);
#endif 
