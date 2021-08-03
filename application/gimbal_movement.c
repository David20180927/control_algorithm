/**
  * @brief      ,
  *             
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2021    		 David           1. 完成
  *
  @verbatim
  ==============================================================================

                                        --
         -n2(pitch)-  --- -n3(roll)- -n4-
        ---                             --
      ---
    -n1(yaw)-
---------------
---chassis-----
---------------
n1-> servo s1
n2->
n3->
n4->

...dbus
...
...
... s1
...
  ==============================================================================
  @endverbatim
  */
//----------include
#include "gimbal_movement.h"
#include "chassis_movement.h"
#include "usb_receive.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "CAN_Receive.h"
//----------define
extern upper_order ai_order;
//----------function
void gimbal_set(gimbal_control_t *gimbal);
static void gimbal_init(gimbal_control_t *gimbal_move_init);
static void gimbal_set_mode(gimbal_control_t *gimbal_mode); //mode change between manual and auto
static void gimbal_data_determine(gimbal_control_t *gimbal_data);

//----------main task----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
gimbal_control_t gimbal;
void gimbal_movement(void const *pvParameters)
{
	vTaskDelay(gimbal_task_init_time);
	gimbal_init(&gimbal);

	while(1)
	{
		gimbal_set_mode(&gimbal);
		gimbal_data_determine(&gimbal);
		gimbal_set(&gimbal);
		
		
		//os delay
		vTaskDelay(gimbal_task_control_time);
	}









}
//end task
//----------END main task----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//init gimbal data
static void gimbal_init(gimbal_control_t *init)
{

//    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
//    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    //电机数据指针获取
//    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
//    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    //陀螺仪数据指针获取
//    init->gimbal_INT_angle_point = get_INS_angle_point();
//    init->gimbal_INT_gyro_point = get_gyro_data_point();
    //遥控器数据指针获取
    init->gimbal_rc_ctrl = get_remote_control_point();
    //初始化电机模式
//    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
//    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //初始化yaw电机pid
//    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
//    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
//    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    //初始化pitch电机pid
//    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
//    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
//    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

    //清除所有PID
//    gimbal_total_pid_clear(init);

//    gimbal_feedback_update(init);

//    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
//    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
//    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;


 //   init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
 //   init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
 //   init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;


}

static void gimbal_set_mode(gimbal_control_t *gimbal_mode)
{
    if (gimbal_mode == NULL)
    {
        return;
    }
	if (switch_is_mid(gimbal_mode->gimbal_rc_ctrl->rc.s[MODE_CHANNEL]))
    {
      
		gimbal_mode->gimbal_mode = GIMBAL_MID; //write into struct 
    }
    else if (switch_is_down(gimbal_mode->gimbal_rc_ctrl->rc.s[MODE_CHANNEL]))
    {
        
		gimbal_mode->gimbal_mode = GIMBAL_MANUAL;
    }
    else if (switch_is_up(gimbal_mode->gimbal_rc_ctrl->rc.s[MODE_CHANNEL]))
    {
        
		gimbal_mode->gimbal_mode = GIMBAL_AUTO;
    }
	
}

static void gimbal_data_determine(gimbal_control_t *gimbal_data)
{
	if (gimbal_data == NULL)
    {
        return;
    }

    if (gimbal_data->gimbal_mode == GIMBAL_MID)
    {//500 ~ 2500
        gimbal_data->arm_servo.s1 = node1_rawpoint;
		gimbal_data->arm_servo.s2 = node2_rawpoint;
		gimbal_data->arm_servo.s3 = node3_rawpoint;
		gimbal_data->arm_servo.s4 = node4_rawpoint;
    }
    else if (gimbal_data->gimbal_mode == GIMBAL_MANUAL)
    {
		if (switch_is_mid(gimbal_data->gimbal_rc_ctrl->rc.s[1]))
		{
			gimbal_data->arm_servo.s1 = 1500;
		}
		else if (switch_is_down(gimbal_data->gimbal_rc_ctrl->rc.s[1]))
		{
			gimbal_data->arm_servo.s1 = 800;
		}
		else if (switch_is_up(gimbal_data->gimbal_rc_ctrl->rc.s[1]))
		{
			gimbal_data->arm_servo.s1 = 1000;
		}
			
    }
    else if (gimbal_data->gimbal_mode == GIMBAL_AUTO)
    {
        //gimbal_auto_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }




}















void gimbal_set(gimbal_control_t *gimbal)
{
	//servo_set
	servo_pwm_set(gimbal->arm_servo.s1,1);
	servo_pwm_set(gimbal->arm_servo.s2,2);
	servo_pwm_set(gimbal->arm_servo.s3,3);
	servo_pwm_set(gimbal->arm_servo.s4,4);
	//motor_set
	CAN_cmd_gimbal(gimbal->gimbal_yaw_motor.current_set, 0,0,0);
}














