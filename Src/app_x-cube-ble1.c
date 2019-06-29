/**
  ******************************************************************************
  * File Name          : app_x-cube-ble1.c
  * Description        : Implementation file
  *             
  ******************************************************************************
  *
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
#include "routing.h"

#define maxNodes	(4)

#define Working 0 
#define Failed 1 
#define Freezed 2 

#define LOCAL_NODE 2
#define NAME_NODE_SIZE (13)
#define BDADDR_SIZE 6

//tBDAddr LOCAL_ADDR[]={ 0x01, 0x00, 0x00, 0xE1, 0x80, 0xaa };
char LOCAL_NAME[NAME_NODE_SIZE];


tBDAddr Node_address_1[] = { 0x01, 0x00, 0x00, 0xE1, 0x80, 0xaa };
const char Node_name_1[] = { AD_TYPE_COMPLETE_LOCAL_NAME,'N','O','D','E','N','R','G','_','0','0','0','1' };

tBDAddr Node_address_2[] = { 0x02, 0x00, 0x00, 0xE1, 0x80, 0xaa };
const char Node_name_2[] = { AD_TYPE_COMPLETE_LOCAL_NAME,'N','O','D','E','N','R','G','_','0','0','0','2' };

tBDAddr Node_address_3[] = { 0x03, 0x00, 0x00, 0xE1, 0x80, 0xaa };
const char Node_name_3[] = { AD_TYPE_COMPLETE_LOCAL_NAME,'N','O','D','E','N','R','G','_','0','0','0','3' };


//should be changed and in header
//tBDAddr local_address[] = { 0x01, 0x00, 0x00, 0xE1, 0x80, 0xaa };
//const char local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME, 'N','O','D','E','N','R','G','_','0','0','0','1' };

uint8_t bnrg_expansion_board = IDB04A1; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */  
static volatile uint8_t user_button_init_state = 1;
static volatile uint8_t user_button_pressed = 0;

BLE_RoleTypeDef BLE_Role = SERVER;
uint8_t bdaddr[BDADDR_SIZE];
tBDAddr bdaddr_r;

extern volatile uint8_t set_connectable;
extern volatile int     connected;
extern volatile uint8_t notification_enabled;

extern volatile uint8_t end_read_tx_char_handle;
extern volatile uint8_t end_read_rx_char_handle;

uint16_t service_handle, dev_name_char_handle, appearance_char_handle;


typedef struct Node
{
	const char* name;
	uint8_t num;
	int state;
	tBDAddr* addr;
	const char* local_name;
	int distance;
	int connected[maxNodes-1];
}Node;


struct Node nodes[3];

/* Define Functions  -----------------------------------------------*/

void initialize_Node(void)
{
	
	nodes[0].name = "Node 1";
	nodes[0].num = 1;
	nodes[0].addr = Node_address_1;
	nodes[0].local_name = Node_name_1;
	nodes[0].state = Freezed;
	nodes[0].distance = 1;
	nodes[0].connected[0] = 2;

	nodes[1].name = "Node 2";
	nodes[1].num = 2;
	nodes[1].addr = Node_address_2;
	nodes[1].local_name = Node_name_2;
	nodes[1].state = Freezed;
	nodes[1].distance = 2;
	nodes[1].connected[0] = 1;
	nodes[1].connected[1] = 3;

	nodes[2].name = "Node 3";
	nodes[2].num = 3;
	nodes[2].addr = Node_address_3;
	nodes[2].local_name = Node_name_3;
	nodes[2].state = Freezed;
	nodes[2].distance = 2;
	nodes[2].connected[0] = 2;
	nodes[2].connected[1] = 4;

	#if (LOCAL_NODE==1) 

	memcpy(bdaddr, nodes[LOCAL_NODE-1].addr, 6);
	memcpy(LOCAL_NAME,nodes[LOCAL_NODE-1].local_name,13);
	// char local_name[]={ AD_TYPE_COMPLETE_LOCAL_NAME,'N','O','D','E','N','R','G','_','0','0','0','1' };

	#elif (LOCAL_NODE==2) 
	memcpy(bdaddr, nodes[LOCAL_NODE-1].addr, 6);
	memcpy(LOCAL_NAME,nodes[LOCAL_NODE-1].local_name,13);

	#elif (LOCAL_NODE==3)
	const tBDAddr local_address[]={ 0x03, 0x00, 0x00, 0xE1, 0x80, 0xaa };
	const char local_name[]={ AD_TYPE_COMPLETE_LOCAL_NAME,'N','O','D','E','N','R','G','_','0','0','0','3' };

	#endif

}

/*
 * BlueNRG-MS Initialization task
 */
void Process_BlueNRG_MS_Init(void)
{
  uint8_t  hwVersion;
  uint16_t fwVersion;
  int ret;
  
  User_Init();
  
  /* Get the User Button initial state */
  user_button_init_state = BSP_PB_GetState(BUTTON_KEY);
  
  hci_init(user_notify, NULL);
      
  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  hci_reset();
  
  HAL_Delay(100);
  
  printf("HWver %d, FWver %d\n", hwVersion, fwVersion);
  
  if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
    bnrg_expansion_board = IDB05A1; 
  }
  
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
 
    printf("SERVER: BLE Stack Initialized\n");
    ret = Add_Sample_Service();
    
    if (ret == BLE_STATUS_SUCCESS)
		{
      printf("Service added successfully.\n");
    }
		else
      printf("Error while adding service.\n");
  
  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);

}

/**
 * @brief  Initilization the GAP Communication as Broadcasting Server
 *
 * @param  None
 * @retval None
 */
void Process_Mesh_Start_Listen_Connection(void)
{
  /* Initialize the peripherals and the BLE Stack */
  int ret;
  
  hci_init(user_notify, NULL);

  hci_reset();
  
  HAL_Delay(100);
  
	//BLUENRG_memcpy(bdaddr, LOCAL_ADDR, sizeof(LOCAL_ADDR));

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
	
	/* disable scan response */
	hci_le_set_scan_resp_data(0,NULL);
	
	PRINTF("General Discoverable Mode ");
	/*
	Advertising_Event_Type, Adv_Interval_Min, Adv_Interval_Max, Address_Type, Adv_Filter_Policy,
	Local_Name_Length, Local_Name, Service_Uuid_Length, Service_Uuid_List, Slave_Conn_Interval_Min,
	Slave_Conn_Interval_Max
	*/
	ret = aci_gap_set_discoverable(ADV_DATA_TYPE, ADV_INTERV_MIN, ADV_INTERV_MAX, PUBLIC_ADDR, 
																 NO_WHITE_LIST_USE, 13, LOCAL_NAME, 0, NULL, 0, 0);
	PRINTF("%d\n",ret);  
}

/**
 * @brief  Initilization the GAP Communication 2
 *
 * @param  None
 * @retval None
 */
void Process_Mesh_Start_BlueNRG_Connection(uint8_t Dest_Node_Num)
{
	int ret;

  hci_init(user_notify, NULL);
      
  hci_reset();
  
  HAL_Delay(100);
    
  //BLUENRG_memcpy(bdaddr, LOCAL_ADDR, sizeof(LOCAL_ADDR));
 
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

	BLE_Role = CLIENT;
	printf("Client Create Connection\n");

	//remote address
	//tBDAddr bdaddr;

	memcpy(bdaddr_r, nodes[Dest_Node_Num-1].addr, 6);
	//tBDAddr bdaddr = {0x02, 0x00, 0x00, 0xE1, 0x80, 0xaa};

	BSP_LED_On(LED2); //To indicate the start of the connection and discovery phase
	
	/*
	Scan_Interval, Scan_Window, Peer_Address_Type, Peer_Address, Own_Address_Type, Conn_Interval_Min, 
	Conn_Interval_Max, Conn_Latency, Supervision_Timeout, Conn_Len_Min, Conn_Len_Max    
	*/
	ret = aci_gap_create_connection(SCAN_P, SCAN_L, PUBLIC_ADDR, bdaddr_r, PUBLIC_ADDR, CONN_P1, CONN_P2, 0,
																	SUPERV_TIMEOUT, CONN_L1 , CONN_L2); 
	
	if (ret != 0){
		printf("Error while starting connection.\n");
		HAL_GPIO_WritePin(GPIOB, LED11_Pin, GPIO_PIN_SET);
	}
	
	connected=FALSE;
}

/**
 * @brief  Initilization the notification Communication 3
 *
 * @param  None
 * @retval None
 */
void Process_Enable_Notification_BlueNRG_MS(void)
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

/**
 * @brief  Initilization the notification Communication 1
 *
 * @param  None
 * @retval None
 */

/*
 * BlueNRG-MS background task
 */
void Process_BlueNRG_MS(void)
{
  if (set_connectable) 
  {
		//should be changed
		Process_BlueNRG_MS_Init();
		Process_Mesh_Start_Listen_Connection();
    set_connectable = FALSE;
    user_button_init_state = BSP_PB_GetState(BUTTON_KEY);
		HAL_GPIO_WritePin(GPIOB,LED11_Pin,GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB,LED11_Pin,GPIO_PIN_RESET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB,LED11_Pin,GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOB,LED11_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);
  }
	
	//This start the notification process and authentication
	Process_Enable_Notification_BlueNRG_MS();
  hci_user_evt_proc();


  /* Check if the User Button has been pushed */
  if (user_button_pressed) 
  {
    /* Debouncing */
    HAL_Delay(50);
    
    /* Wait until the User Button is released */
    while (BSP_PB_GetState(BUTTON_KEY) == !user_button_init_state);
    
    /* Debouncing */
    HAL_Delay(50);
		
    uint8_t data[1] = {'p'};

		Process_Mesh_Start_BlueNRG_Connection(3);
		
		while(!(connected && notification_enabled))
		{
			Process_Enable_Notification_BlueNRG_MS();
			hci_user_evt_proc();
			HAL_Delay(100);
			HAL_GPIO_TogglePin(GPIOB,LD3_Pin);
			
			if(notification_enabled)
				HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_RESET);		
		}
		HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);

		
		Process_frame_formulation(2,3,data,sizeof(data));
		
    /* Reset the User Button flag */
    user_button_pressed = 0;
  }
	

}
/**
 * @brief  Initilization the full sending function
 *
 * @param  None
 * @retval None
 */
void Process_Mesh_BlueNRG_Send_Node(uint8_t Dest_Node_Num, uint8_t* data_buffer, uint8_t Nb_bytes)
{

	Process_Mesh_Start_BlueNRG_Connection(Dest_Node_Num);

	Process_Enable_Notification_BlueNRG_MS();

	//while(!(connected && notification_enabled))
	while (!(connected ))
	{
		//Process_Enable_Notification_BlueNRG_MS();
		hci_user_evt_proc();
	}
	
	Process_frame_formulation(LOCAL_NODE,Dest_Node_Num,data_buffer,sizeof(Nb_bytes));

//	while(connected && notification_enabled)
//	{
//		Process_Enable_Notification_BlueNRG_MS();
//		hci_user_evt_proc();		
//	}
	set_connectable=0;
	
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

/**
 * @brief  This function is used to formulate a mesh frame 
 *         (to be sent over the air to the remote board).
 * @param  data_buffer : pointer to data to be sent
 * @param  Nb_bytes : number of bytes to send
 * @retval None
 */
fPrccStatus Process_frame_Routing(uint8_t Currentnum, uint8_t* routedNum, uint8_t* frame_buffer, uint8_t Nb_bytes)
{
//	uint8_t SenderNum = frame_buffer[1] - '0';
	uint8_t TargetNum = frame_buffer[3] - '0';
	int j = 0; 
	for (int i = 0; i < maxNodes; i++)
	{
		if (nodes[i].num == Currentnum) //if node is not exist
		{
			if (nodes[i].distance > 0) //if node hasn't connected with others
			{
				for (j = 0; j < nodes[i].distance; j++)
				{
					if (TargetNum == nodes[i].connected[j])
					{
						*routedNum = nodes[i].connected[j];
						return FRM_OK;
					}
					else
					{
						*routedNum = 255;
						return FRM_OK;
					}

				}
			}
		}
	}

//broadcasting
*routedNum = 0;
return FRM_ERR;
}
#ifdef __cplusplus
}
#endif
#endif /* __APP_X_CUBE_BLE1_C */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
