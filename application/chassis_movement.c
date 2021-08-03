/**
  * @brief      ,
  *             
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2021    		 David           1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  */
//----------include
#include "chassis_movement.h"
#include "gimbal_movement.h"
#include "usb_receive.h"
#include "Define.h"
#include "bsp_buzzer.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"


//----------define
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
extern upper_order ai_order;
//----------function
static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_mode(chassis_move_t *chassis_mode); //mode change between manual and auto
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_manual_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_auto_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_control(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
static void chassis_vector_to_mecanum_wheel_speed(chassis_move_t *chassis_move_mode, const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4]);
//----------main task----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
chassis_move_t chassis;
static chassis_mode_e chassis_behaviour = CHASSIS_ZERO;
	
void chassis_movement(void const *pvParameters)
{

	vTaskDelay(chassis_task_init_time);
	chassis_init(&chassis);//chassis initialization
	while(1)
	{
		chassis_set_mode(&chassis);
		chassis_feedback_update(&chassis);
		chassis_set_control(&chassis);
		chassis_control_loop(&chassis);
		CAN_cmd_chassis(chassis.motor_chassis[0].give_current, chassis.motor_chassis[1].give_current, chassis.motor_chassis[2].give_current, chassis.motor_chassis[3].give_current);
		//os delay
		vTaskDelay(chassis_task_control_time);
		
	}

}//end task
//----------END main task----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//init chassis data
static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
    //chassis motor speed PID
    //�����ٶȻ�pidֵ
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    
    //chassis angle PID
    //���̽Ƕ�pidֵ
   // const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
   
    uint8_t i;

    //in beginning�� chassis mode is raw 
    //���̿���״̬Ϊԭʼ
    //chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //get remote control point
    //��ȡң����ָ��
    chassis_move_init->chassis_RC = get_remote_control_point();
    //get gyro sensor euler angle point
    //��ȡ��������̬��ָ��
    //chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //get gimbal motor data point
    //��ȡ��̨�������ָ��
    //chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    //chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    //get chassis motor data point,  initialize motor speed PID
    //��ȡ���̵������ָ�룬��ʼ��PID 
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //initialize angle PID
    //��ʼ���Ƕ�PID
    //PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    //first order low-pass filter  replace ramp function
    //��һ���˲�����б����������
	const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, chassis_task_control_time, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, chassis_task_control_time, chassis_y_order_filter);

    //max and min speed
    //��� ��С�ٶ�
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //update data
    //����һ������
    chassis_feedback_update(chassis_move_init);
}




//input: read rc value  
//output: change the static value of chassis_behaviour
static void chassis_set_mode(chassis_move_t *chassis_mode)
{
    if (chassis_mode == NULL)
    {
        return;
    }
	if (switch_is_mid(chassis_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour = CHASSIS_ZERO;
		chassis_mode->chassis_mode = CHASSIS_ZERO; //write into struct 
    }
    else if (switch_is_down(chassis_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour = CHASSIS_MANUAL;
		chassis_mode->chassis_mode = CHASSIS_MANUAL;
    }
    else if (switch_is_up(chassis_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour = CHASSIS_AUTO;
		chassis_mode->chassis_mode = CHASSIS_AUTO;
    }
	
}



static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //update motor speed, accel is differential of speed PID
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //calculate vertical speed, horizontal speed ,rotation speed, left hand rule 
    //���µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
    //���������̬�Ƕ�, �����������������������ⲿ�ִ���
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
 //   chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
 //   chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}

//get vx,vy,wz set
static void chassis_set_control(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    //get three control set-point, ��ȡ������������ֵ
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);
    if (chassis_move_control->chassis_mode == CHASSIS_AUTO)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //rotate chassis direction, make sure vertial direction follow gimbal 
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
        //sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        //cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
       // chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
       // chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //set control relative angle  set-point
        //���ÿ��������̨�Ƕ�
        //chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
        //calculate ratation speed
        //������תPID���ٶ�
        chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
        //speed limit
        //�ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }

    else if (chassis_move_control->chassis_mode == CHASSIS_MANUAL)
    {
        //"angle_set" is rotation speed set-point
        //��angle_set�� ����ת�ٶȿ���
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
	chassis_move_control->last_chassis_mode = chassis_move_control->chassis_mode;
}

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    //mecanum wheel speed calculation
    //�����˶��ֽ�
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop, chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

//    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
//    {
        
//        for (i = 0; i < 4; i++)
//        {
//            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
 //       }
        //in raw mode, derectly return
        //raw����ֱ�ӷ���
//        return;
//    }

    //calculate the max speed in four wheels, limit the max speed
    //�������ӿ�������ٶȣ�������������ٶ�
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //calculate pid
    //����pid
    for (i = 0; i < 4; i++)
    {
        PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }


    //��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}

static void chassis_vector_to_mecanum_wheel_speed(chassis_move_t *chassis_move_mode, const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //because the gimbal is in front of chassis, when chassis rotates, wheel 0 and wheel 1 should be slower and wheel 2 and wheel 3 should be faster
    //��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
        //can change to CHASSIS_ZERO_FORCE,CHASSIS_NO_MOVE,CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,
        //CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,CHASSIS_NO_FOLLOW_YAW,CHASSIS_OPEN
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;  
    
}
//input: read the static value of chassis_behaviour  
//output: call control function of manual,zero and auto
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour == CHASSIS_ZERO)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour == CHASSIS_MANUAL)
    {
        chassis_manual_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
			
    }
    else if (chassis_behaviour == CHASSIS_AUTO)
    {
        chassis_auto_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }

}

//input: read rc value   output : vx,vy (m/s)
//@note: manual control
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, RC_DEADLINE);
    rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    //keyboard set speed set-point
    //���̿���


    //first order low-pass replace ramp function, calculate chassis speed set-point to improve control performance
    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
	
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
	
    //stop command, need not slow change, set zero derectly
    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if (vx_set_channel < RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }
	
	//read from filter function
    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}


//control function of manual,zero and auto
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}
static void chassis_manual_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

//havent
static void chassis_auto_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
	
    *vx_set = ai_order.vx_set_order;
	*vy_set = ai_order.vy_set_order;
	*wz_set = ai_order.wz_set_order;
    
}

