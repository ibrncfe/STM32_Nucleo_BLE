/**
  ******************************************************************************
  * File Name          : app_x-cube-ble1.c
  * Description        : Implementation file
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
#ifndef __APP_X_CUBE_BLE1_C
#define __APP_X_CUBE_BLE1_C
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_x-cube-ble1.h"

#include "hci_tl.h"
#include "sample_service.h"
#include "role_type.h"
#include "bluenrg_utils.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_hal_aci.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private defines -----------------------------------------------------------*/
/**
 * Define the role here only if it is not already defined in the project options
 * For the CLIENT_ROLE comment the line below 
 * For the SERVER_ROLE uncomment the line below
 */
//#define SERVER_ROLE

#define BDADDR_SIZE 6
 
/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
uint8_t bnrg_expansion_board = IDB04A1; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */  
static volatile uint8_t user_button_init_state = 1;
static volatile uint8_t user_button_pressed = 0;

//#ifdef SERVER_ROLE
  BLE_RoleTypeDef BLE_Role = SERVER;
//#else
//  BLE_RoleTypeDef BLE_Role = CLIENT;
//#endif

extern volatile uint8_t set_connectable;
extern volatile int     connected;
extern volatile uint8_t notification_enabled;

extern volatile uint8_t end_read_tx_char_handle;
extern volatile uint8_t end_read_rx_char_handle;

/* USER CODE BEGIN PV */
uint8_t bdaddr[BDADDR_SIZE];

const tBDAddr local_address[] = { 0x01, 0x00, 0x00, 0xE1, 0x80, 0xaa };
const char local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME,'N','O','D','E','N','R','G','_','0','0','0','1' };

uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void User_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

void MX_BlueNRG_MS_Init(void)
{
  /* USER CODE BEGIN SV */ 

  /* USER CODE END SV */
  
  /* USER CODE BEGIN BlueNRG_MS_Init_PreTreatment */
  
  /* USER CODE END BlueNRG_MS_Init_PreTreatment */
  uint8_t  hwVersion;
  uint16_t fwVersion;
  int ret;
  
  User_Init();
  
  /* Get the User Button initial state */
  user_button_init_state = BSP_PB_GetState(BUTTON_KEY);
  
  hci_init(user_notify, NULL);
      
  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  /* 
   * Reset BlueNRG again otherwise we won't
   * be able to change its MAC address.
   * aci_hal_write_config_data() must be the first
   * command after reset otherwise it will fail.
   */
  hci_reset();
  
  HAL_Delay(100);
  
  printf("HWver %d, FWver %d\n", hwVersion, fwVersion);
  
  if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
    bnrg_expansion_board = IDB05A1; 
  }
  
	BLUENRG_memcpy(bdaddr, local_address, sizeof(local_address));

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if (ret) {
    printf("Setting BD_ADDR failed 0x%02x.\n", ret);
  }
  
  ret = aci_gatt_init();    
  if (ret) {
    printf("GATT_Init failed.\n");
  }
  
//  if (BLE_Role == SERVER) {
  ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
//  }
//  else {
//      ret = aci_gap_init_IDB05A1(GAP_CENTRAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
//  }
  
  if (ret != BLE_STATUS_SUCCESS) {
    printf("GAP_Init failed.\n");
  }
    
  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     BONDING);

  if (ret == BLE_STATUS_SUCCESS) {
    printf("BLE Stack Initialized.\n");  
  }
  
//  if (BLE_Role == SERVER) {
    printf("SERVER: BLE Stack Initialized\n");
    ret = Add_Sample_Service();
    
    if (ret == BLE_STATUS_SUCCESS)
		{
      printf("Service added successfully.\n");
    }
		else
      printf("Error while adding service.\n");
    
//  } else {
//    printf("CLIENT: BLE Stack Initialized\n");
//  }
  
  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);

  /* USER CODE BEGIN BlueNRG_MS_Init_PostTreatment */
  
  /* USER CODE END BlueNRG_MS_Init_PostTreatment */
}

/**
 * @brief  Initilization the GAP Communication as Broadcasting Server
 *
 * @param  None
 * @retval None
 */
void Mesh_Start_Listen_Connection(void)
{
  /* Initialize the peripherals and the BLE Stack */
  
  int ret;
  
  hci_init(user_notify, NULL);

  hci_reset();
  
  HAL_Delay(100);
  
	BLUENRG_memcpy(bdaddr, local_address, sizeof(local_address));

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if (ret) {
    printf("Setting BD_ADDR failed 0x%02x.\n", ret);
  }
  
  ret = aci_gatt_init();    
  if (ret) {
    printf("GATT_Init failed.\n");
  }
  
	ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    printf("GAP_Init failed.\n");
  }
    
  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     BONDING);
  if (ret == BLE_STATUS_SUCCESS) {
    printf("BLE Stack Initialized.\n");
  }
  
	BLE_Role = SERVER;
	printf("SERVER: BLE Stack Initialized\n");
	ret = Add_Sample_Service();
    
	if (ret == BLE_STATUS_SUCCESS)
		printf("Service added successfully.\n");
	else
		printf("Error while adding service.\n");
  
  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);

/* USER CODE BEGIN BlueNRG_MS_Init_PostTreatment */
	const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'N','O','D','E','N','R','G','_','0','0','0','1'};
	
	/* disable scan response */
	hci_le_set_scan_resp_data(0,NULL);
	
	PRINTF("General Discoverable Mode ");
	/*
	Advertising_Event_Type, Adv_Interval_Min, Adv_Interval_Max, Address_Type, Adv_Filter_Policy,
	Local_Name_Length, Local_Name, Service_Uuid_Length, Service_Uuid_List, Slave_Conn_Interval_Min,
	Slave_Conn_Interval_Max
	*/
	ret = aci_gap_set_discoverable(ADV_DATA_TYPE, ADV_INTERV_MIN, ADV_INTERV_MAX, PUBLIC_ADDR, 
																 NO_WHITE_LIST_USE, 13, local_name, 0, NULL, 0, 0);
	PRINTF("%d\n",ret);  
/* USER CODE END BlueNRG_MS_Init_PostTreatment */
}

/**
 * @brief  Initilization the GAP Communication 
 *
 * @param  None
 * @retval None
 */
void Mesh_Start_P2P_Connection(void)
{
	int ret;

  hci_init(user_notify, NULL);
      
  hci_reset();
  
  HAL_Delay(100);
    
  BLUENRG_memcpy(bdaddr, local_address, sizeof(local_address));
 
	ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if (ret) {
    printf("Setting BD_ADDR failed 0x%02x.\n", ret);
  }
  
  ret = aci_gatt_init();    
  if (ret) {
    printf("GATT_Init failed.\n");
  }
  
  ret = aci_gap_init_IDB05A1(GAP_CENTRAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    printf("GAP_Init failed.\n");
  }
    
  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     BONDING);
  if (ret == BLE_STATUS_SUCCESS) {
    printf("BLE Stack Initialized.\n");
  }
  
  ret = Add_Sample_Service();
    
  if (ret == BLE_STATUS_SUCCESS)
    printf("Service added successfully.\n");
  else
    printf("Error while adding service.\n");
    
  
  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);
	
	//Check the setting
  if (ret != BLE_STATUS_SUCCESS) {
		HAL_GPIO_WritePin(GPIOB, LED11_Pin, GPIO_PIN_SET);
  }
	
  //Set output power level
  ret = aci_hal_set_tx_power_level(1,4);

	BLE_Role = CLIENT;
	printf("Client Create Connection\n");

	//remote address
	tBDAddr bdaddr = {0x02, 0x00, 0x00, 0xE1, 0x80, 0xaa};
	BSP_LED_On(LED2); //To indicate the start of the connection and discovery phase
	
	/*
	Scan_Interval, Scan_Window, Peer_Address_Type, Peer_Address, Own_Address_Type, Conn_Interval_Min, 
	Conn_Interval_Max, Conn_Latency, Supervision_Timeout, Conn_Len_Min, Conn_Len_Max    
	*/
	ret = aci_gap_create_connection(SCAN_P, SCAN_L, PUBLIC_ADDR, bdaddr, PUBLIC_ADDR, CONN_P1, CONN_P2, 0,
																	SUPERV_TIMEOUT, CONN_L1 , CONN_L2); 
	
	if (ret != 0){
		printf("Error while starting connection.\n");
		HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, LED11_Pin, GPIO_PIN_SET);
	}
	
}
void Process_En_Notification_BlueNRG_MS(void)
{
	//enable send and receive functions
	/* Start TX handle Characteristic dynamic discovery if not yet done */
	if (connected && !end_read_tx_char_handle){
		startReadTXCharHandle();
	}
	
	/* Start RX handle Characteristic dynamic discovery if not yet done */
	else if (connected && !end_read_rx_char_handle){      
		startReadRXCharHandle();
	}
		
	if (connected && end_read_tx_char_handle && end_read_rx_char_handle && !notification_enabled) 
	{
		BSP_LED_Off(LED2); //end of the connection and chars discovery phase
		enableNotification();
	}
	
}

void Routing_BlueNRG_MS(void)
{
/*
NODE1	
{'N','O','D','E','N','R','G','_','0','0','0','1'};
{0x01, 0x00, 0x00, 0xE1, 0x80, 0xaa};
NODE2
{'N','O','D','E','N','R','G','_','0','0','0','1'};
{0x01, 0x00, 0x00, 0xE1, 0x80, 0xaa};
	
	
	
*/
}

	
	
/*
 * BlueNRG-MS background task
 */
void MX_BlueNRG_MS_Process(void)
{
  /* USER CODE BEGIN BlueNRG_MS_Process_PreTreatment */

  /* USER CODE END BlueNRG_MS_Process_PreTreatment */
  if (set_connectable) 
  {
		//////////////////should be changed
		//Mesh_Start_Listen_Connection();
    set_connectable = FALSE;
    user_button_init_state = BSP_PB_GetState(BUTTON_KEY);
  }
  

  Process_En_Notification_BlueNRG_MS();

  /* Check if the User Button has been pushed */
  if (user_button_pressed) 
  {
    /* Debouncing */
    HAL_Delay(50);
    
    /* Wait until the User Button is released */
    while (BSP_PB_GetState(BUTTON_KEY) == !user_button_init_state);
    
    /* Debouncing */
    HAL_Delay(50);
    
		//Mesh_Start_P2P_Connection();

    if (connected && notification_enabled)
    {
			/* Send a toggle command to the remote device */
      uint8_t data[1] = {'p'};
      sendData(data, sizeof(data));
      
			
			
			

      //BSP_LED_Toggle(LED2);  /* Toggle the LED2 locally. */
                               /* If uncommented be sure the BSP_LED_Init(LED2)
                                * is called in main().
                                * E.g. it can be enabled for debugging. */
    }
    
    /* Reset the User Button flag */
    user_button_pressed = 0;
  }
	
  hci_user_evt_proc();

  /* USER CODE BEGIN BlueNRG_MS_Process_PostTreatment */
  
  /* USER CODE END BlueNRG_MS_Process_PostTreatment */
}
/**
 * @brief  Initialize User process.
 *
 * @param  None
 * @retval None
 */
static void User_Init(void)
{
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
  BSP_LED_Init(LED2);
  BSP_COM_Init(COM1); 
}


/**
  * @brief  BSP Push Button callback
  * @param  Button Specifies the pin connected EXTI line
  * @retval None.
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  /* Set the User Button flag */
  user_button_pressed = 1;
}

#ifdef __cplusplus
}
#endif
#endif /* __APP_X_CUBE_BLE1_C */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
