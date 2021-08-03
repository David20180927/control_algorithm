/**
  * @brief      ,
  *             
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2021    		 David           1. Íê³É
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  */

#ifndef USB_RECEIVE_H
#define USB_RECEIVE_H
#include "struct_typedef.h"
#include "remote_control.h"
#include "define.h"




typedef struct
{
  const RC_ctrl_t *usb_RC;
}usb_receive_t;

extern void usb_receive(void const *pvParameters);





#endif
