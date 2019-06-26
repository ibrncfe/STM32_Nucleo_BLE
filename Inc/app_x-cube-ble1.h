/**
  ******************************************************************************
  * File Name          : app_x-cube-ble1.h
  * Description        : Header file
  *                    
  ******************************************************************************
  *
  * COPYRIGHT 2019 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
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
void Process_Mesh_Start_BlueNRG_Connection(void);
void Process_Mesh_Start_Listen_Connection(void);
void Process_BlueNRG_MS(void);

const static uint8_t NodeNum='1';


#ifdef __cplusplus
}
#endif
#endif /* APP_X_CUBE_BLE1_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
