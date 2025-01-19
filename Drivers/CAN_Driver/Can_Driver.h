/*
 * Can_Driver.h
 *
 *  Created on: Jan 19, 2025
 *      Author: Domen
 */

#ifndef CAN_DRIVER_CAN_DRIVER_H_
#define CAN_DRIVER_CAN_DRIVER_H_

#include "main.h"
// types
typedef struct
{
   uint32_t IdType;
   uint32_t Identifier;
   uint32_t DataLength;
   uint8_t Data [8];
} CAN_Message_t;

//variables

extern CAN_Message_t can_tx_message;
extern CAN_Message_t can_rx_message;


// functions
uint8_t Send_CAN_Message(CAN_Message_t *can_tx_msg);

uint8_t Receive_CAN_Message(FDCAN_HandleTypeDef *hfdcan);


#endif /* CAN_DRIVER_CAN_DRIVER_H_ */
