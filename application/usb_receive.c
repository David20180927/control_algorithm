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
//----------include
#include "usb_receive.h"
#include "usbd_cdc_if.h"
#include "chassis_movement.h"
#include "usb_device.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "define.h"
#include "cmsis_os.h"

//----------define

upper_order ai_order;

//----------function
static void usbtask_init(usb_receive_t *usb_init);
static void usb_loop(usb_receive_t *usb_loop);

//----------main task----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t usb_buf[256];
uint8_t buffer[64];
usb_receive_t usb;
void usb_receive(void const *pvParameters)
{

	vTaskDelay(usb_task_init_time);
	MX_USB_DEVICE_Init();
	usbtask_init(&usb);
	while(1)
	{
		

		CDC_Transmit_FS(usb_buf, 256);

		usb_loop(&usb);
		if(buffer[0] == 1) buzzer_high();
		if(buffer[0] == 2) buzzer_low();
		//os delay
		vTaskDelay(usb_task_control_time);
		

	}
	





}//end task
//----------END main task----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

static void usbtask_init(usb_receive_t *usb_init)
{
	usb_init->usb_RC = get_remote_control_point();


}


static void usb_loop(usb_receive_t *usb_loop)
{

	//copy buffer into ai_order
	ai_order.test = buffer[0];
	
	ai_order.vx_set_order = buffer[11];
	ai_order.vy_set_order = buffer[12];
	ai_order.wz_set_order = buffer[13];
	
	ai_order.pwm_one = buffer[21];
	ai_order.pwm_two = buffer[22]; 
	
	ai_order.x_point =  buffer[31];
	ai_order.y_point =  buffer[32];
	ai_order.z_point =  buffer[33]; 
	
	


}







