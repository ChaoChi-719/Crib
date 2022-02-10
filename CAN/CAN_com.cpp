#include "CAN_com.h"
#include "../structs.h"

#define P_MIN -6.28f
#define P_MAX 6.28f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define T_MIN -9.0f
#define T_MAX 9.0f
#define TIME_MIN 0.0f
#define TIME_MAX 10.0f

#define T_MIN_Reply -9.0f
#define T_MAX_Reply 9.0f

#define KP_MIN 0.0f
#define KP_MAX 512.0f
//#define KI_MIN 0.0f
//#define KI_MAX 1.0f
#define KD_MIN 0.0f
#define KD_MAX 2.0f

void update_cmd(ControllerStruct *controller, float time);

/// CAN Receive Command Packet Structure ///

/**
 * Unpack cmd message
 *
 * @param msg - msg from cmd
 * @param controller - mode : 0-initial mode , 1-continuous mode
 *                   - p_cmd : 18 bits position , between -2pi and 2 pi
 *                   - time_cmd : 20 bits time , between 0 and 10
 *                   - t_ff : 8 bits torque , between -9 and 9
 *                   - kp : 9 bits , between 0 and 512
 *                   - kd : 8 bits , between 0 and 2
 */
void unpack_cmd(CANMessage msg, ControllerStruct *controller)
{
        int p_int = ((msg.data[0] & 0x7f) << 11) | (msg.data[1] << 3) | (msg.data[2] >> 5);
        int time_int = (msg.data[2] & 0x1f) << 15 | (msg.data[3] << 7) | msg.data[4] >> 1;
        int t_int = (msg.data[4] & 0x1) << 7 | msg.data[5] >> 1;
        int kp_int = (msg.data[5] & 0x1) << 8 | msg.data[6];
        //        int ki_int = (msg.data[5]&0x3)<<6 | msg.data[6]>>2;
        int kd_int = (msg.data[7] & 0xff);

        controller->mode = msg.data[0] >> 7;
        controller->p_cmd = uint_to_float(p_int, P_MIN, P_MAX, 18);
        controller->time_cmd = uint_to_float(time_int, TIME_MIN, TIME_MAX, 20);
        controller->t_ff = uint_to_float(t_int, T_MIN, T_MAX, 8);
        controller->kp = uint_to_float(kp_int, KP_MIN, KP_MAX, 9);
        //        controller->ki = uint_to_float(ki_int, KI_MIN, KI_MAX, 8);
        controller->kd = uint_to_float(kd_int, KD_MIN, KD_MAX, 8);

        update_cmd(controller, controller->time_cmd);
}
/**
 * The parameter is assigned to controller in this function
 *
 * @param controller - p_cmd : Command from CAN bus
 *                   - theta_mech : Current position of motor
 *                   - v_cmd : The velocity is calculated here
 *                   - delta theta : The displacement in every 1/40k sec
 * @param time - The time of next command coming
 */
void update_cmd(ControllerStruct *controller, float time)
{
        controller->p_goal = controller->p_cmd + int(controller->theta_mech / 6.28) * 6.28;
        if (controller->p_cmd <= 0 && int(controller->theta_mech / 6.28) > 0) //馬達旋轉大於1圈後, 尋找就近相對位置
                controller->p_goal += 6.28;
        else if (controller->p_cmd >= 0 && int(controller->theta_mech / 6.28) < 0)
                controller->p_goal -= 6.28;
        if (controller->p_goal<controller->theta_mech &&int(controller->theta_mech / 6.28)> 0)) //維持正轉至目標點
                controller->p_goal += 6.28;
        else if (controller->p_goal > controller->theta_mech && int(controller->theta_mech / 6.28) < 0))//維持反轉至目標點
                controller->p_goal -= 6.28;
        controller->p_des = controller->theta_mech;
        controller->v_cmd = (controller->p_goal - controller->theta_mech) / time;
        controller->abs_v_cmd = (controller->p_cmd) / time;
        controller->delta_theta = controller->v_cmd / F_TORQUE;
        controller->flag = 0;
}

/// CAN Reply Packet Structure ///
/// 16 bit position, between -4*pi and 4*pi
/// 12 bit velocity, between -30 and + 30 rad/s
/// 12 bit current, between -40 and 40;
/// CAN Packet is 5 8-bit words
/// Formatted as follows.  For each quantity, bit 0 is LSB
/// 0: [position[15-8]]
/// 1: [position[7-0]]
/// 2: [velocity[11-4]]
/// 3: [velocity[3-0], current[11-8]]
/// 4: [current[7-0]]
void pack_reply(CANMessage *msg, float p, float v, float t)
{
        int p_int = float_to_uint(p, P_MIN, P_MAX, 16);
        int v_int = float_to_uint(v, V_MIN, V_MAX, 12);
        int t_int = float_to_uint(t, T_MIN_Reply, T_MAX_Reply, 12);
        msg->data[0] = CAN_ID;
        msg->data[1] = p_int >> 8;
        msg->data[2] = p_int & 0xFF;
        msg->data[3] = v_int >> 4;
        msg->data[4] = ((v_int & 0xF) << 4) + (t_int >> 8);
        msg->data[5] = t_int & 0xFF;
}

void pack_echo(CANMessage *msg)
{
        msg->data[0] = CAN_ID;
        msg->data[1] = 0xFF;
        msg->data[2] = 0xFF;
        msg->data[3] = 0xFF;
        msg->data[4] = 0xFF;
        msg->data[5] = 0xFF;
}