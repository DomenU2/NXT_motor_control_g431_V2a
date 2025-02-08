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

    ms1->motorID = MOTOR_ID_1;
    ms1->pos_sign=-1;
    ms1->pwm_sign=1;
    ms1->p_encoderTimer = p_encT1;
    ms1->p_pwmTimer = p_pwmT1;
    ///PWM variables
    ms1->PWM_period = p_pwmT1->Init.Period;

    //PID position
    ms1->reg_pos.limMin = -30*PI;
    ms1->reg_pos.limMax = 30*PI;
    ms1->reg_pos.error_max = ms1->reg_pos.limMax - ms1->reg_pos.limMin;
    ms1->reg_pos.Kpp=600;
    ms1->reg_pos.Tip=1;
    ms1->reg_pos.Tdp=80;

    //PID speed
     ms1->reg_speed.limMin = -30;
     ms1->reg_speed.limMax = 30;
     ms1->reg_speed.error_max = ms1->reg_speed.limMax - ms1->reg_speed.limMin;
     ms1->reg_speed.Kpp=150;
     ms1->reg_speed.Tip=2;
     ms1->reg_speed.Tdp=1;


// MOTOR 2

     ms2->motorID = MOTOR_ID_2;
     ms2->pos_sign=1;
     ms2->pwm_sign=-1;
     ms2->p_encoderTimer = p_encT2;
     ms2->p_pwmTimer = p_pwmT2;
     ///PWM variables
     ms2->PWM_period = p_pwmT2->Init.Period;

     //PID position
     ms2->reg_pos.limMin = -30*PI;
     ms2->reg_pos.limMax = 30*PI;
     ms2->reg_pos.error_max = ms2->reg_pos.limMax - ms2->reg_pos.limMin;
     ms2->reg_pos.Kpp=600;
     ms2->reg_pos.Tip=1;
     ms2->reg_pos.Tdp=80;

     //PID speed
      ms2->reg_speed.limMin = -30;
      ms2->reg_speed.limMax = 30;
      ms2->reg_speed.error_max = ms2->reg_speed.limMax - ms2->reg_speed.limMin;
      ms2->reg_speed.Kpp=150;
      ms2->reg_speed.Tip=2;
      ms2->reg_speed.Tdp=1;
}


uint8_t Motor_Driver_Enable(Motor_state_t *ms1,Motor_state_t *ms2){

	if(ms1->driver_enable==1 || ms2->driver_enable==1){
		motorDriverEnable=1;
		HAL_GPIO_WritePin(MOT_SLEEP_GPIO_Port, MOT_SLEEP_Pin, GPIO_PIN_SET);
		//Pull sleep pin on DRV8833 HIGH
	}
	if(ms1->driver_enable==0 && ms2->driver_enable==0){
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

void Motor_Stop(Motor_state_t *ms){
	ms->duty_cycle = ms->PWM_period;
	Set_Duty_Cycle(ms, ms->PWM_period, ms->PWM_period);
}

void Motor_Coast(Motor_state_t *ms){
	ms->duty_cycle = 0;
	Set_Duty_Cycle(ms, 0, 0);
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

	ms->reg_pos.error= ms->reg_pos.ref - ms->position;
	ms->reg_pos.ref_k_1=ms->reg_pos.ref;
	float P,I,D, Id=0;
	if(ms->reg_pos.error<=ms->reg_pos.error_max && ms->reg_pos.error>=-ms->reg_pos.error_max){
		//P - proporcionalni del

		P = ms->reg_pos.Kpp * ms->reg_pos.error;

#if 1
		//I - Integralni del
		ms->reg_pos.I =  ms->reg_pos.I + ms->reg_pos.error * Ts; //Integracija
		ms->reg_pos.Id = 1/(ms->reg_pos.Tip) * ms->reg_pos.I;
#endif

		//D - Diferencialni del
		D =    ms->reg_pos.Tdp/Ts * (ms->reg_pos.error - ms->reg_pos.error_k_1);


		// Izhod regulatorja
	//	u_k = P +Id + D;
#if 0
		ms->reg_pos.u_k = P + D;
#endif
		ms->reg_pos.u_k = P +Id + D;

		ms->reg_pos.error_k_1 = ms->reg_pos.error;

		}
else{
	ms->reg_pos.u_k = 0;
}


#if 1
	PWM_Control_Motor(ms, ms->reg_pos.u_k);
#endif

}//End PID position

void PI_speed(Motor_state_t *ms){

 //Regulacijska napaka in omejitve

	ms->reg_speed.error= ms->reg_speed.ref - ms->velocity_f;
	ms->reg_speed.ref_k_1=ms->reg_speed.ref;
	float P, Id=0;
	//limiting
#if 0
	if(ms->reg_speed.error<=ms->reg_speed.error_max){
		ms->reg_speed.error=ms->reg_speed.error_max;
	}
	else if (ms->reg_speed.error>=-ms->reg_speed.error_max){
		ms->reg_speed.error=-ms->reg_speed.error_max;
		}
#endif
		//P - proporcionalni del

		P = ms->reg_speed.Kpp * ms->reg_speed.error;

#if 1
		//I - Integralni del
		ms->reg_speed.I =  ms->reg_speed.I + ms->reg_speed.error * Ts; //Integracija
		ms->reg_speed.Id = 1/(ms->reg_speed.Tip) * ms->reg_speed.I;
#endif

		// Izhod regulatorja


		ms->reg_speed.u_k = P + ms->reg_speed.Id;

		ms->reg_speed.error_k_1 = ms->reg_speed.error;




#if 1
	PWM_Control_Motor(ms, ms->reg_speed.u_k);
#endif

}//End PID position



void RefTraj1(Motor_state_t *ms){
	static float delta = 0.005;
	ms->reg_pos.ref = ms->reg_pos.ref + delta;

	if(ms->reg_pos.ref >= ms->reg_pos.limMax){
		ms->reg_pos.ref = ms->reg_pos.limMax;
		delta=-delta;
	}
	else if(ms->reg_pos.ref <= ms->reg_pos.limMin){
		ms->reg_pos.ref = ms->reg_pos.limMin;
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
	switch (ms->mode){
	case COAST_MODE:
		 Motor_Coast(ms);
	break;
	case POSITION_MODE:
		PD_position(ms);
	break;
	case VELOCITY_MODE:
		PI_speed(ms);
	break;
	case STOP_MODE:
 		Motor_Stop(ms);
	break;
	default:
		Motor_Coast(ms);
	break;
	}



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

		diag_msg.reg_out = mcan_m_diag_1_reg_out_encode(ms->reg_pos.u_k);
		diag_msg.er = mcan_m_diag_1_er_encode(ms->reg_pos.error);
		mcan_m_diag_1_pack(&can_msg.Data, &diag_msg,MCAN_M_DIAG_1_LENGTH);
		Send_CAN_Message(&can_msg);
		break;
	case MOTOR_ID_2:
		can_msg.Identifier = MCAN_M_DIAG_2_FRAME_ID;
		can_msg.IdType = FDCAN_STANDARD_ID;
		can_msg.DataLength = FDCAN_DLC_BYTES_8;

		diag_msg.reg_out = mcan_m_diag_2_reg_out_encode(ms->reg_pos.u_k);
		diag_msg.er = mcan_m_diag_2_er_encode(ms->reg_pos.error);
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
	static uint8_t time_cnt=0;
	time_cnt++;
	time_cnt%=6;
switch(time_cnt){
case 0:
	Motor_Send_Pos_Vel_CAN(&motor_state1);
	Motor_Send_Pos_Vel_CAN(&motor_state2);
break;

case 2:
	Motor_Send_Status_CAN(&motor_state1);
	Motor_Send_Status_CAN(&motor_state2);
break;

case 4:
	Motor_Send_Diag_CAN(&motor_state1);
	Motor_Send_Diag_CAN(&motor_state2);
break;

}







}


uint8_t Motor_Update_Ref_CAN(CAN_Message_t *can_msg, Motor_state_t *ms){

	uint8_t ret=0;
	struct mcan_m_ref_2_t m_ref_msg={0};
	mcan_m_ref_2_unpack(&m_ref_msg, can_msg->Data,MCAN_M_REF_1_LENGTH);
	ms->reg_pos.ref = mcan_m_ref_1_position_decode(m_ref_msg.position);
	ms->reg_speed.ref = mcan_m_ref_1_velocity_decode(m_ref_msg.velocity);

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

