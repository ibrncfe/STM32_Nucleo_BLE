#ifndef _ROUTING_H_
#define _ROUTING_H_

#ifdef __cplusplus
 extern "C" {
#endif 


#include "main.h"
#include "app_x-cube-ble1.h"
#include "link_layer.h"
#include "sample_service.h"
#include "stdio.h"



//#if (LOCAL_NODE==1) 
//const tBDAddr local_address[]={ 0x01, 0x00, 0x00, 0xE1, 0x80, 0xaa };
//const char local_name[]={ AD_TYPE_COMPLETE_LOCAL_NAME,'N','O','D','E','N','R','G','_','0','0','0','1' };

//#elif (LOCAL_NODE==2) 
//const tBDAddr local_address[]={ 0x02, 0x00, 0x00, 0xE1, 0x80, 0xaa };
//const char local_name[]={ AD_TYPE_COMPLETE_LOCAL_NAME,'N','O','D','E','N','R','G','_','0','0','0','2' };

//#elif (LOCAL_NODE==3)
//const tBDAddr local_address[]={ 0x03, 0x00, 0x00, 0xE1, 0x80, 0xaa };
//const char local_name[]={ AD_TYPE_COMPLETE_LOCAL_NAME,'N','O','D','E','N','R','G','_','0','0','0','3' };

//#endif




void initialize_Node(void);




#ifdef __cplusplus
}
#endif

#endif
