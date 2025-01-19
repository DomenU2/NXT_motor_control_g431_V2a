/*
 * Can_Driver.c
 *
 *  Created on: Jan 19, 2025
 *      Author: Domen
 */


//includes

#include "Can_Driver.h"

//Defines



//Variables
//CAN variables

FDCAN_TxHeaderTypeDef CAN1_TxHeader={0};
FDCAN_RxHeaderTypeDef CAN1_RxHeader={0};



CAN_Message_t can_tx_message = {0};
CAN_Message_t can_rx_message = {0};



//functions

uint8_t Send_CAN_Message(CAN_Message_t *can_tx_msg){

	CAN1_TxHeader.Identifier = can_tx_msg->Identifier;
	CAN1_TxHeader.IdType = can_tx_msg->IdType;
	CAN1_TxHeader.DataLength = can_tx_msg->DataLength;

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN1_TxHeader,can_tx_msg->Data);


	return 0;
}


uint8_t Receive_CAN_Message(FDCAN_HandleTypeDef *hfdcan){

	(void)HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN1_RxHeader, can_rx_message.Data);

	can_rx_message.DataLength = CAN1_RxHeader.DataLength;
	can_rx_message.IdType = CAN1_RxHeader.IdType;
	can_rx_message.Identifier = CAN1_RxHeader.Identifier;

	//Data unpacking, scaling and saving here
return 0;
}
