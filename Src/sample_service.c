/**
  ******************************************************************************
  * @file    sample_service.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   Add a sample service using a vendor specific profile.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "sample_service.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_hal_aci.h"
#include "app_x-cube-ble1.h"
#include "routing.h"




/* Private variables ---------------------------------------------------------*/
volatile int connected = FALSE;
volatile uint8_t set_connectable=1;
volatile uint16_t connection_handle = 0;
volatile uint8_t notification_enabled = FALSE;
volatile uint8_t start_read_tx_char_handle = FALSE;
volatile uint8_t start_read_rx_char_handle = FALSE;
volatile uint8_t end_read_tx_char_handle = FALSE;
volatile uint8_t end_read_rx_char_handle = FALSE;
uint16_t conn_handle;
uint8_t reason;
uint8_t ret;
uint16_t tx_handle;
uint16_t rx_handle;

uint8_t Ack[1] = { '@' };
static BOOL acknow_signal=FALSE;
volatile BOOL forward_routing=FALSE;

uint16_t sampleServHandle, TXCharHandle, RXCharHandle;


extern uint8_t bnrg_expansion_board;
extern BLE_RoleTypeDef BLE_Role;
/**
 * @}
 */
 
/** @defgroup SAMPLE_SERVICE_Private_Macros
 * @{
 */
/* Private macros ------------------------------------------------------------*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
  do {\
  	uuid_struct.uuid128[0] = uuid_0; uuid_struct.uuid128[1] = uuid_1; uuid_struct.uuid128[2] = uuid_2; uuid_struct.uuid128[3] = uuid_3; \
	uuid_struct.uuid128[4] = uuid_4; uuid_struct.uuid128[5] = uuid_5; uuid_struct.uuid128[6] = uuid_6; uuid_struct.uuid128[7] = uuid_7; \
	uuid_struct.uuid128[8] = uuid_8; uuid_struct.uuid128[9] = uuid_9; uuid_struct.uuid128[10] = uuid_10; uuid_struct.uuid128[11] = uuid_11; \
	uuid_struct.uuid128[12] = uuid_12; uuid_struct.uuid128[13] = uuid_13; uuid_struct.uuid128[14] = uuid_14; uuid_struct.uuid128[15] = uuid_15; \
	}while(0)
/**
 * @}
 */

/** @defgroup SAMPLE_SERVICE_Exported_Functions 
 * @{
 */
 
/**
 * @brief  Add a sample service using a vendor specific profile
 * @param  None
 * @retval Status
 */
tBleStatus Add_Sample_Service(void)
{
  tBleStatus ret;
  
  /*
  UUIDs:
  D973F2E0-B19E-11E2-9E96-0800200C9A66
  D973F2E1-B19E-11E2-9E96-0800200C9A66
  D973F2E2-B19E-11E2-9E96-0800200C9A66
  */
  
  const uint8_t service_uuid[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9};
  const uint8_t charUuidTX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
  const uint8_t charUuidRX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};
  
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid, PRIMARY_SERVICE, 7, &sampleServHandle); /* original is 9?? */
  if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
  ret =  aci_gatt_add_char(sampleServHandle, UUID_TYPE_128, charUuidTX, 20, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                           16, 1, &TXCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  ret =  aci_gatt_add_char(sampleServHandle, UUID_TYPE_128, charUuidRX, 20, CHAR_PROP_WRITE|CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                           16, 1, &RXCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  PRINTF("Sample Service added.\nTX Char Handle %04X, RX Char Handle %04X\n", TXCharHandle, RXCharHandle);
  return BLE_STATUS_SUCCESS; 
  
fail:
  PRINTF("Error while adding Sample Service.\n");
  return BLE_STATUS_ERROR ;
}

/**
 * @brief  Make the device connectable
 * @param  None 
 * @retval None
 */
void Make_Connection(void)
{  
}

/**
 * @brief  Discovery TX characteristic handle by UUID 128 bits
 * @param  None 
 * @retval None
 */
void startReadTXCharHandle(void)
{
  if (!start_read_tx_char_handle)
  {    
    PRINTF("Start reading TX Char Handle\n");
    
    const uint8_t charUuid128_TX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
    aci_gatt_disc_charac_by_uuid(connection_handle, 0x0001, 0xFFFF, UUID_TYPE_128, charUuid128_TX);
    start_read_tx_char_handle = TRUE;
  }
}

/**
 * @brief  Discovery RX characteristic handle by UUID 128 bits
 * @param  None 
 * @retval None
 */
void startReadRXCharHandle(void)
{  
  if (!start_read_rx_char_handle)
  {
    PRINTF("Start reading RX Char Handle\n");
    
    const uint8_t charUuid128_RX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};
    aci_gatt_disc_charac_by_uuid(connection_handle, 0x0001, 0xFFFF, UUID_TYPE_128, charUuid128_RX);
    start_read_rx_char_handle = TRUE;
  }
}

/**
 * @brief  This function is used to receive data related to the sample service
 *         (received over the air from the remote board).
 * @param  data_buffer : pointer to store in received data
 * @param  Nb_bytes : number of bytes to be received
 * @retval None
 */
void receiveData(uint8_t* data_buffer, uint8_t Nb_bytes)
{
	uint8_t SenderNum=0;
	uint8_t DestNum=0;
	acknow_signal=0;
	forward_routing=FALSE;

  for(int i = 0; i < Nb_bytes; i++) {
    printf("%c", data_buffer[i]);
		if (data_buffer[i]=='@')
		{
			//acknowledgment signal reachout STATE 2 
			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);					
			HAL_Delay(500);
			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);					
			forward_routing=FALSE;
			acknow_signal=TRUE;
			ret=aci_gap_terminate(conn_handle, reason);
			if (ret != BLE_STATUS_SUCCESS) 
						HAL_GPIO_WritePin(GPIOB, LED11_Pin, GPIO_PIN_SET);				
			set_connectable=1;
			//add return
		}
	}
	
	//normal frame comming
	if(!acknow_signal)
	{
		//checking if it be forwarded or just getting it out 
		if(Process_frame_Deformulation(&SenderNum, &DestNum, data_buffer, Nb_bytes))
		{
			DestNum=data_buffer[3];
		//getting frame STATE 3
		if (DestNum==NodeNum+'0')
		{
			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);	
			BLE_Role = SERVER;
			sendData(Ack, sizeof(Ack));
			//wait until send
			uint16_t holdTime=1000;
			while(holdTime>0)
			{
				Process_Enable_Notification_BlueNRG_MS();
				hci_user_evt_proc();
				HAL_Delay(1);
				holdTime--;
			}
		  acknow_signal=FALSE;
			forward_routing=FALSE;
			GettingData(data_buffer, Nb_bytes);
			set_connectable=1;
		}
		else //forward it STATE 1
		{
			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_SET);		
			HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);				
			HAL_Delay(1000);
			HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);		
			BLE_Role = SERVER;
			sendData(Ack, sizeof(Ack));
			//wait until send
			uint16_t holdTime=1000;
			while(holdTime>0)
			{
				Process_Enable_Notification_BlueNRG_MS();
				hci_user_evt_proc();
				HAL_Delay(1);
				holdTime--;
			}			
		  acknow_signal=FALSE;			
			forward_routing=TRUE;
			BLE_Role = CLIENT;
			ForwardFrame(data_buffer, Nb_bytes);	
			holdTime=1000;
			while(holdTime>0)
			{
				Process_Enable_Notification_BlueNRG_MS();
				hci_user_evt_proc();
				HAL_Delay(1);
				holdTime--;
			}			
			set_connectable=1;
		}
	}
  }

	fflush(stdout);
}

/**
 * @brief  This function is used to forward a frame on a mesh network
 *         (to be sent over the air to the remote board).
 * @param  data_buffer : pointer to data to be sent
 * @param  Nb_bytes : number of bytes to send
 * @retval None
 */
fPrccStatus ForwardFrame(uint8_t* frame_buffer, uint8_t Nb_bytes)
{
	uint8_t Dest_num;
	Dest_num=frame_buffer[3]-'0';
	/////
	Process_BlueNRG_MS_Init();
	HAL_Delay(500);
	Process_Mesh_Start_Listen_Connection();
	HAL_Delay(1000);
	Process_Mesh_Start_BlueNRG_Connection(Dest_num);
	/////
	while(!(connected && notification_enabled))
	{	
		Process_Enable_Notification_BlueNRG_MS();
		hci_user_evt_proc();
		HAL_Delay(100);
		HAL_GPIO_TogglePin(GPIOB,LD3_Pin);
		
		if(notification_enabled)
			HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);		
	}
	
  uint8_t data[1] = {'p'};	
	HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
	Process_frame_formulation(2,3,data,sizeof(data));

	sendData(frame_buffer,Nb_bytes);

	return FRM_OK;	
}

/**
 * @brief  This function is used to formulate a mesh frame 
 *         (to be sent over the air to the remote board).
 * @param  data_buffer : pointer to data to be sent
 * @param  Nb_bytes : number of bytes to send
 * @retval None
 */
fPrccStatus GettingData(uint8_t* data_buffer, uint8_t Nb_bytes)
{

	return FRM_OK;
}




/**
 * @brief  This function is used to formulate a mesh frame 
 *         (to be sent over the air to the remote board).
 * @param  data_buffer : pointer to data to be sent
 * @param  Nb_bytes : number of bytes to send
 * @retval None
 */
fPrccStatus Process_frame_formulation(uint8_t Currentnum, uint8_t Targetnum, uint8_t* data_buffer, uint8_t Nb_bytes)
{
/*
| # | source addr | # | destination addr | # | data | *
length = 1+1+1+1+1+data_buffer+1=6+Nb_bytes
*/
	if (Targetnum == 0 || Currentnum == 0 || Targetnum == Currentnum)
	{
		HAL_GPIO_WritePin(GPIOB, LED11_Pin, GPIO_PIN_SET);
		return FRM_ERR;
	}

	const uint8_t sz = Nb_bytes + 6;
//	const uint8_t preamble = '#';
//	const uint8_t trail = '*';
	int i = 0;
	
	uint8_t fr[] = { '#' , Currentnum + '0' ,'#' , Targetnum + '0', '#' };
	uint8_t n = sizeof(fr);
	uint8_t* A = (uint8_t *) malloc(sz * sizeof(uint8_t));
	for (i = 0; i < n; ++i)
		A[i] = fr[i];
	for (i = 0; i < Nb_bytes; ++i)
		A[i+ n] = data_buffer[i];
	A[sz-1] = '*';

	sendData(A,sz);
	
	free(A);
	
	return FRM_OK;
}

/**
 * @brief  This function is used to formulate a mesh frame 
 *         (to be sent over the air to the remote board).
 * @param  data_buffer : pointer to data to be sent
 * @param  Nb_bytes : number of bytes to send
 * @retval None
 */
fPrccStatus Process_frame_Deformulation(uint8_t* SenderNum, uint8_t* TargetNum, uint8_t* frame_buffer, uint8_t Nb_bytes)
{
 /*
 | # | source addr | # | destination addr | # | data | *
 length = 1+1+1+1+1+data_buffer+1=6+Nb_bytes
 */
	int o = 0;
	for (int i = 0; i < Nb_bytes; ++i)
	{
	 if (frame_buffer[i] == '#')
		 o++;
	}

	if (o != 3 || frame_buffer[Nb_bytes-1]!='*')
	 return FRM_ERR;

	*SenderNum = frame_buffer[1];
	*TargetNum = frame_buffer[3];


	return FRM_OK;
}


/**
 * @brief  This function is used to send data related to the sample service
 *         (to be sent over the air to the remote board).
 * @param  data_buffer : pointer to data to be sent
 * @param  Nb_bytes : number of bytes to send
 * @retval None
 */
void sendData(uint8_t* data_buffer, uint8_t Nb_bytes)
{
  if(BLE_Role == SERVER) {    
    aci_gatt_update_char_value(sampleServHandle,TXCharHandle, 0, Nb_bytes, data_buffer);    
  }
  else {
    aci_gatt_write_without_response(connection_handle, rx_handle+1, Nb_bytes, data_buffer);
  }
}

/**
 * @brief  Enable notification
 * @param  None 
 * @retval None
 */
void enableNotification(void)
{
  uint8_t client_char_conf_data[] = {0x01, 0x00}; // Enable notifications
  
  uint32_t tickstart = HAL_GetTick();
  
  while(aci_gatt_write_charac_descriptor(connection_handle, tx_handle+2, 2, client_char_conf_data)==BLE_STATUS_NOT_ALLOWED){
    /* Radio is busy */
    if ((HAL_GetTick() - tickstart) > (10*HCI_DEFAULT_TIMEOUT_MS)) break;
  }
  notification_enabled = TRUE;
}

/**
 * @brief  This function is called when an attribute gets modified
 * @param  handle : handle of the attribute
 * @param  data_length : size of the modified attribute data
 * @param  att_data : pointer to the modified attribute data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data)
{
  if(handle == RXCharHandle + 1){
    receiveData(att_data, data_length);
  } else if (handle == TXCharHandle + 2) {        
    if(att_data[0] == 0x01)
      notification_enabled = TRUE;
  }
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  addr : Address of peer device
 * @param  handle : Connection handle
 * @retval None
 */
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  connected = TRUE;
  connection_handle = handle;
  
  printf("Connected to device:");
  for(int i = 5; i > 0; i--){
    printf("%02X-", addr[i]);
  }
  printf("%02X\n", addr[0]);
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;
  
  printf("Disconnected\n");
  /* Make the device connectable again. */
  set_connectable = TRUE;
  notification_enabled = FALSE;
  start_read_tx_char_handle = FALSE;
  start_read_rx_char_handle = FALSE;
  end_read_tx_char_handle = FALSE;
  end_read_rx_char_handle = FALSE;
}

/**
 * @brief  This function is called when there is a notification from the sever.
 * @param  attr_handle Handle of the attribute
 * @param  attr_len    Length of attribute value in the notification
 * @param  attr_value  Attribute value in the notification
 * @retval None
 */
void GATT_Notification_CB(uint16_t attr_handle, uint8_t attr_len, uint8_t *attr_value)
{
  if (attr_handle == tx_handle+1) {
    receiveData(attr_value, attr_len);
  }
}

/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  pData  Pointer to the ACI packet
 * @retval None
 */
void user_notify(void * pData)
{
  hci_uart_pckt *hci_pckt = pData;  
  /* obtain event packet */
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT)
    return;
  
  switch(event_pckt->evt){
    
  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;
    
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;
    
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){
        
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        {
          if (bnrg_expansion_board == IDB05A1) {
            evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
            Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);
          }
          else {
            evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
            Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);
          }
          
        }
        break;
      case EVT_BLUE_GATT_NOTIFICATION:
        {
          evt_gatt_attr_notification *evt = (evt_gatt_attr_notification*)blue_evt->data;
          GATT_Notification_CB(evt->attr_handle, evt->event_data_length - 2, evt->attr_value);
        }
        break;
      case EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP:
        if(BLE_Role == CLIENT) {
          PRINTF("EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP\n");
          
          evt_gatt_disc_read_char_by_uuid_resp *resp = (void*)blue_evt->data;
          
          if (start_read_tx_char_handle && !end_read_tx_char_handle)
          {
            tx_handle = resp->attr_handle;
            printf("TX Char Handle %04X\n", tx_handle);
          }
          else if (start_read_rx_char_handle && !end_read_rx_char_handle)
          {
            rx_handle = resp->attr_handle;
            printf("RX Char Handle %04X\n", rx_handle);
          }
        }
        break;
        
      case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
        if(BLE_Role == CLIENT) {
          /* Wait for gatt procedure complete event trigger related to Discovery Charac by UUID */
          //evt_gatt_procedure_complete *pr = (void*)blue_evt->data;
          
          if (start_read_tx_char_handle && !end_read_tx_char_handle)
          {
            end_read_tx_char_handle = TRUE;
          }
          else if (start_read_rx_char_handle && !end_read_rx_char_handle)
          {
            end_read_rx_char_handle = TRUE;
          }
        }
        break;
      }
    }
    break;
  }    
}
/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
