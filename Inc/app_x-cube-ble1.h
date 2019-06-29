/**
  ******************************************************************************
  * File Name          : app_x-cube-ble1.h
  * Description        : Header file
  *                    
  ******************************************************************************
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_X_CUBE_BLE1_H
#define APP_X_CUBE_BLE1_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32f3xx_nucleo.h"

/* Exported Functions --------------------------------------------------------*/
void Process_BlueNRG_MS_Init(void);
void Process_Initialization_Nodes(void);
void Process_Enable_Notification_BlueNRG_MS(void);
void Process_Routing_BlueNRG_MS(void);
void Process_Mesh_Start_BlueNRG_Connection(uint8_t Dest_Node_Num);
void Process_Mesh_Start_Listen_Connection(void);
void Process_Mesh_BlueNRG_Send_Node(uint8_t Dest_Node_Num, uint8_t* data_buffer, uint8_t Nb_bytes);
void Process_BlueNRG_MS(void);
static void User_Init(void);
void initialize_Node(void);
const static uint8_t NodeNum=2;



#ifdef __cplusplus
}
#endif
#endif /* APP_X_CUBE_BLE1_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
