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

Motor_state_t motor_state1={0};
Motor_state_t motor_state2={0};
Motor_state_t motor_stateDBG={0};

int16_t ADC_max = 4095;

int32_t speed_ramp=0;
int32_t ramp_step = 5;

//Private variables
//debug
float limitHighDbg = 360; //deg
float position_deg = 0;
float limitLowDbg = 0; //deg


// CAN messages TX
CAN_Message_t can_msg1={0};
struct mcan_m_pos_vel_1_t pos_vel_msg1={0};
CAN_Message_t can_msg2={0};
struct mcan_m_pos_vel_2_t pos_vel_msg2={0};


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
    ms1->motorID = MOTOR_ID_1;
    ms1->pos_sign=-1;
    ms1->pwm_sign=1;
    ms1->p_encoderTimer = p_encT1;
    ms1->p_pwmTimer = p_pwmT1;
    ms1->posLimMin = -PI;
    ms1->posLimMax = PI;
    ms1->error_max = ms1->posLimMax - ms1->posLimMin;
    ms1->Kpp=1000; //6000
    ms1->Tip=0;
    ms1->Tdp=100; //500

	///PWM variables
    ms1->PWM_period = p_pwmT1->Init.Period;


    ms2->motorID = MOTOR_ID_2;
    ms2->pos_sign=1;
    ms2->pwm_sign=-1;
    ms2->p_encoderTimer = p_encT2;
	ms2->p_pwmTimer = p_pwmT2;
	ms2->posLimMin = -3*PI;
	ms2->posLimMax = 3*PI;
	ms2->error_max = ms2->posLimMax - ms2->posLimMin;
	ms2->Kpp=1000;
	ms2->Tip=0;
	ms2->Tdp=100;
	///PWM variables
	ms2->PWM_period = p_pwmT2->Init.Period;
}


uint8_t Motor_Driver_Enable(Motor_state_t *ms1,Motor_state_t *ms2){

	if(ms1->driver_enable==1 || ms1->driver_enable==1){
		motorDriverEnable=1;
		HAL_GPIO_WritePin(MOT_SLEEP_GPIO_Port, MOT_SLEEP_Pin, GPIO_PIN_SET);
		//Pull sleep pin on DRV8833 HIGH
	}
	else{
	   motorDriverEnable=0;
	   HAL_GPIO_WritePin(MOT_SLEEP_GPIO_Port, MOT_SLEEP_Pin, GPIO_PIN_RESET);
	   //Pull sleep pin on DRV8833 HIGH
	}
}

int Set_Duty_Cycle(Motor_state_t *ms, uint16_t dc1,uint16_t dc2){


		  int rtn = 0;
		  switch (ms->motorID){
		  case MOTOR_ID_1:
			  ms->p_pwmTimer->Instance->CCR2 = dc1;
			  ms->p_pwmTimer->Instance->CCR3 = dc2;
			  break;
		  case MOTOR_ID_2:
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

	speed = ms->pwm_sign * speed;
	ms->dir = speed >=0 ? 1: -1;
	ms->duty_cycle = speed * ms->dir;

	if(ms->duty_cycle>ms->PWM_period){ms->duty_cycle=ms->PWM_period;}
	else if(ms->duty_cycle<100){ms->duty_cycle=0;}

if(ms->enable==1){

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

	ms->position = (float)ms->tick_position  * ms->pos_sign* 2* PI / TICK_PER_REV; //[rad]



	//velocity and low pass filter
	ms->velocity = (float)ms->tick_velocity  * ms->pos_sign * 2* PI/ (TICK_PER_REV * Ts); //[rad/s]

	// Motor 2 has some wrong wiring so encoder direction is wrong
	// quick fix
		if(ms->motorID==MOTOR_ID_2){
			ms->position = -ms->position;
			ms->velocity = -ms->velocity;
		}
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

	ms->error= ms->position_ref - ms->position;
	ms->position_ref_k_1=ms->position_ref;
	float P,I,D;
	if(ms->error<=ms->error_max && ms->error>=-ms->error_max){
		//P - proporcionalni del

		P = ms->Kpp * ms->error;

#if 0
		I - Integralni del
		 I =  I + error * Ts; //Integracija
		 Id = 1/(Tip) * I * 0;
#endif

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
}

void RefTraj1(Motor_state_t *ms){
	static float delta = 0.005;
	ms->position_ref = ms->position_ref + delta;

	if(ms->position_ref >= ms->posLimMax){
		ms->position_ref = ms->posLimMax;
		delta=-delta;
	}
	else if(ms->position_ref <= ms->posLimMin){
		ms->position_ref = ms->posLimMin;
		delta=-delta;
	}
}


// open loop pwm test motor
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


// CAN Send recieve functions for motor state and commands
uint8_t Motor_Send_Pos_Vel_CAN(Motor_state_t *ms){

	uint8_t ret=0;


	switch(ms->motorID){
	case MOTOR_ID_1:
		can_msg1.Identifier = MCAN_M_POS_VEL_1_FRAME_ID;
		can_msg1.IdType = FDCAN_STANDARD_ID;
		can_msg1.DataLength = FDCAN_DLC_BYTES_8;
		pos_vel_msg1.position = mcan_m_pos_vel_1_position_encode(ms->position);
		pos_vel_msg1.velocity= mcan_m_pos_vel_1_velocity_encode(ms->velocity_f);

		mcan_m_pos_vel_1_pack(&can_msg1.Data, &pos_vel_msg1, MCAN_M_POS_VEL_1_LENGTH);

		Send_CAN_Message(&can_msg1);
		break;
	case MOTOR_ID_2:
		can_msg2.Identifier = MCAN_M_POS_VEL_2_FRAME_ID;
		can_msg2.IdType = FDCAN_STANDARD_ID;
		can_msg2.DataLength = FDCAN_DLC_BYTES_8;
		pos_vel_msg2.position = mcan_m_pos_vel_2_position_encode(ms->position);
		pos_vel_msg2.velocity= mcan_m_pos_vel_2_velocity_encode(ms->velocity_f);

		mcan_m_pos_vel_2_pack(&can_msg2.Data, &pos_vel_msg2, MCAN_M_POS_VEL_2_LENGTH);

		Send_CAN_Message(&can_msg2);
		break;
	default:
		ret=1;
		break;
	}
	return ret;

}

uint8_t Motor_Send_Status_CAN(Motor_state_t *ms){

	uint8_t ret=0;
	CAN_Message_t can_msg={0};
	struct mcan_m_status_1_t status_msg={0};

	switch(ms->motorID){
	case MOTOR_ID_1:
		can_msg.Identifier = MCAN_M_STATUS_1_FRAME_ID;
		can_msg.IdType = FDCAN_STANDARD_ID;
		can_msg.DataLength = FDCAN_DLC_BYTES_8;

		status_msg.m_drive_en = ms->driver_enable;
		status_msg.m_en = ms->enable;
		status_msg.m_error = ms->error_code;
		status_msg.m_mode = ms->mode;

		mcan_m_status_1_pack(&can_msg.Data, &status_msg, MCAN_M_STATUS_1_LENGTH);
		Send_CAN_Message(&can_msg);
		break;
	case MOTOR_ID_2:
		can_msg.Identifier = MCAN_M_STATUS_2_FRAME_ID;
		can_msg.IdType = FDCAN_STANDARD_ID;
		can_msg.DataLength = FDCAN_DLC_BYTES_8;

		status_msg.m_drive_en = ms->driver_enable;
		status_msg.m_en = ms->enable;
		status_msg.m_error = ms->error_code;
		status_msg.m_mode = ms->mode;

		mcan_m_status_2_pack(&can_msg.Data, &status_msg, MCAN_M_STATUS_2_LENGTH);

		Send_CAN_Message(&can_msg);
		break;
	default:
		ret=1;
		break;
	}
	return ret;
}

uint8_t Motor_Send_Diag_CAN(Motor_state_t *ms){

	uint8_t ret=0;
	CAN_Message_t can_msg={0};
	struct mcan_m_diag_1_t diag_msg={0};

	switch(ms->motorID){
	case MOTOR_ID_1:
		can_msg.Identifier = MCAN_M_DIAG_1_FRAME_ID;
		can_msg.IdType = FDCAN_STANDARD_ID;
		can_msg.DataLength = FDCAN_DLC_BYTES_8;

		diag_msg.reg_out = mcan_m_diag_1_reg_out_encode(ms->u_k);
		diag_msg.er = mcan_m_diag_1_er_encode(ms->error);
		mcan_m_diag_1_pack(&can_msg.Data, &diag_msg,MCAN_M_DIAG_1_LENGTH);
		Send_CAN_Message(&can_msg);
		break;
	case MOTOR_ID_2:
		can_msg.Identifier = MCAN_M_DIAG_2_FRAME_ID;
		can_msg.IdType = FDCAN_STANDARD_ID;
		can_msg.DataLength = FDCAN_DLC_BYTES_8;

		diag_msg.reg_out = mcan_m_diag_2_reg_out_encode(ms->u_k);
		diag_msg.er = mcan_m_diag_2_er_encode(ms->error);
		mcan_m_diag_2_pack(&can_msg.Data, &diag_msg,MCAN_M_DIAG_2_LENGTH);
		Send_CAN_Message(&can_msg);
		break;
	default:
		ret=1;
		break;
	}
	return ret;
}

void Motor_Send_Messages_CAN(){
	//Send CAN statuses
	static uint32_t time_cnt=0;
	static uint8_t msgn=0;
	if(time_cnt % MCAN_MSG_PERIOD == 0){

		time_cnt=0;
		msgn++;
		msgn%=6;
	}

	switch(msgn){
	case 0:
		Motor_Send_Pos_Vel_CAN(&motor_state1);
		break;
	case 1:
		Motor_Send_Pos_Vel_CAN(&motor_state2);
			break;
	case 2:
		Motor_Send_Status_CAN(&motor_state1);
			break;
	case 3:
		Motor_Send_Status_CAN(&motor_state2);
			break;

	case 4:
		Motor_Send_Diag_CAN(&motor_state1);
			break;

	case 5:
		Motor_Send_Diag_CAN(&motor_state2);
			break;
	}
}


uint8_t Motor_Update_Ref_CAN(CAN_Message_t *can_msg, Motor_state_t *ms){

	uint8_t ret=0;
	struct mcan_m_ref_2_t m_ref_msg={0};
	mcan_m_ref_2_unpack(&m_ref_msg, can_msg->Data,MCAN_M_REF_1_LENGTH);
	ms->position_ref = mcan_m_ref_1_position_decode(m_ref_msg.position);
	ms->velocity_ref = mcan_m_ref_1_velocity_decode(m_ref_msg.velocity);

	return ret;
}

uint8_t Motor_Update_Command_CAN(CAN_Message_t *can_msg, Motor_state_t *ms){

	uint8_t ret=0;
	struct mcan_m_command_1_t m_command_msg={0};
	mcan_m_command_2_unpack(&m_command_msg, can_msg->Data,MCAN_M_COMMAND_1_LENGTH);

	ms->driver_enable = m_command_msg.m_drive_en;
	ms->enable = m_command_msg.m_en;
	ms->mode = m_command_msg.m_mode;
	return ret;

}

