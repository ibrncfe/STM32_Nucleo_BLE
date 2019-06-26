#include "routing.h"



void initialize_Node(void)
{
	
//	nodes[0].name = "Node 1";
//	nodes[0].num = 1;
//	nodes[0].addr = Node_address_1;
//	nodes[0].local_name = Node_name_1;
//	nodes[0].state = Freezed;
//	nodes[0].distance = 1;
//	nodes[0].connected[0] = 2;

//	nodes[1].name = "Node 2";
//	nodes[1].num = 2;
//	nodes[1].addr = Node_address_2;
//	nodes[1].local_name = Node_name_2;
//	nodes[1].state = Freezed;
//	nodes[1].distance = 2;
//	nodes[1].connected[0] = 1;
//	nodes[1].connected[1] = 3;

//	nodes[2].name = "Node 3";
//	nodes[2].num = 3;
//	nodes[2].addr = Node_address_3;
//	nodes[2].local_name = Node_name_3;
//	nodes[2].state = Freezed;
//	nodes[2].distance = 2;
//	nodes[2].connected[0] = 2;
//	nodes[2].connected[1] = 4;

}


/**
 * @brief  This function is used to formulate a mesh frame 
 *         (to be sent over the air to the remote board).
 * @param  data_buffer : pointer to data to be sent
 * @param  Nb_bytes : number of bytes to send
 * @retval None
 */
//fPrccStatus Process_frame_Routing(uint8_t Currentnum, uint8_t* routedNum, uint8_t* frame_buffer, uint8_t Nb_bytes)
//{
////	uint8_t SenderNum = frame_buffer[1] - '0';
//	uint8_t TargetNum = frame_buffer[3] - '0';
//	int j = 0; 
//	for (int i = 0; i < maxNodes; i++)
//	{
//		if (nodes[i].num == Currentnum) //if node is not exist
//		{
//			if (nodes[i].distance > 0) //if node hasn't connected with others
//			{
//				for (j = 0; j < nodes[i].distance; j++)
//				{
//					if (TargetNum == nodes[i].connected[j])
//					{
//						*routedNum = nodes[i].connected[j];
//						return FRM_OK;
//					}
//					else
//					{
//						*routedNum = 255;
//						return FRM_OK;
//					}

//				}
//			}
//		}
//	}

////broadcasting
//*routedNum = 0;
//return FRM_ERR;
//}


