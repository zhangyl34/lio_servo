#include "motor_drive_control.h"
#include "stdio.h"
#include "QThread"
#include <unistd.h>
#include <math.h>

DriveStatusStr drive_status_list[] = {
	{ SW_NOT_READY_TO_SWITCH_ON, MASK_NOT_READY_TO_SWITCH_ON, NOT_READY_TO_SWITCH_ON, "NOT_READY_TO_SWITCH_ON"},
	{ SW_SWITCH_ON_DISABLED, MASK_SWITCH_ON_DISABLED, SWITCH_ON_DISABLED, "SWITCH_ON_DISABLED" },
	{ SW_READY_TO_SWITCH_ON, MASK_READY_TO_SWITCH_ON, READY_TO_SWITCH_ON, "READY_TO_SWITCH_ON" },
	{ SW_SWITCHED_ON, MASK_SWITCHED_ON, SWITCHED_ON, "SWICHED_ON" },
	{ SW_OPERATION_ENABLED, MASK_OPERATION_ENABLED, OPERATION_ENABLED, "OPERATION_ENABLED" },
	{ SW_QUICK_STOP_ACTIVE, MASK_QUICK_STOP_ACTIVE, QUICK_STOP_ACTIVE, "QUICK_STOP_ACTIVE" },
	{ SW_FAULT_REACTION_ACTIVE, MASK_FAULT_REACTION_ACTIVE, FAULT_REACTION_ACTIVE, "FAULT_REACTION_ACTIVE" },
	{ SW_FAULT, MASK_FAULT, FAULT, "FAULT" },
};

MotorDriveControl::MotorDriveControl(U8 node_id) :
	Canopen(node_id) {	
	this->node_id = node_id;
	cur_status = NOT_READY_TO_SWITCH_ON;
	ctrl_target_velocity = 0.0F;
	
	this->start();  // 开启 run 线程，控制状态机的切换。

	resetNode();  // 0x81
    usleep(100*1000);
}

void MotorDriveControl::recvTpdo(U16 pdo, U8* const data) {
	static U16 status_word = 0U;
	if (pdo == 0x180)
	{
		status_word = data[0] + (data[1] << 8);
		for (int i = 0; i < sizeof(drive_status_list) / sizeof(drive_status_list[0]); i++)
		{
			if ((status_word & drive_status_list[i].mask) == drive_status_list[i].status_word)
			{
				//cur_status = drive_status_list[i].status;
				MotorCtrlMsg msg;
				msg.event = MCE_STATUS_CHANGE;
				msg.obj.motor_status = drive_status_list[i].status;
                motor_ctrl_queue.enqueue(msg);
				break;
			}
		}
	}
}

void MotorDriveControl::recvEmergency(U8* const data) {
    printf("recv emergency msg, node id=%d, :0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x!\n",
            node_id, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
}

void MotorDriveControl::recvBootup()
{
    printf("recv boot up msg, node id=%d\n", node_id);
    MotorCtrlMsg msg;
	msg.event = MCE_RESET_NODE;
    motor_ctrl_queue.enqueue(msg);
}

void MotorDriveControl::ctrlMotorMoveByVelocity(F32 velocity)
{
	MotorCtrlMsg msg;
	msg.event = MCE_CTRL_MOTOR_VELOCITY_MODE;
	msg.obj.velocity = velocity;
    motor_ctrl_queue.enqueue(msg);
}

void MotorDriveControl::ctrlMotorQuickStop()
{
	printf("@%d, quick_stop!\n", node_id);
	sendControlWord(CW_QUICK_STOP);
	ctrlMotorMoveByVelocity(0.0);
}

MotorStatus MotorDriveControl::currentStatus()
{
	return cur_status;
}

// get current rpm. can't be used directly. OD should only be visited in MotorDriveControl::run() thread.
F32 MotorDriveControl::currentVelocity()
{
	MotorCtrlMsg msg;
	msg.event = MCE_GET_VEL;
    motor_ctrl_queue.enqueue(msg);
	return cur_velocity;
}

void MotorDriveControl::configPdo()
{
    U8 data[4] = { 0xFE };
	writeOD(data, 1, 0x1800, 0x2);  // 异步
    usleep(100*1000);
	data[0] = 0xE8;  // 1000ms
	data[1] = 0x3;
	writeOD(data, 2, 0x1800, 0x5);  // 事件定时器触发时间
    usleep(100*1000);
	data[0] = 0x10;
	data[1] = 0;
	data[2] = 0;
	data[3] = 0;
	writeOD(data, 4, 0x6065, 0x0);
	
	U32 val = 0x80000300U+node_id;
	writeOD((unsigned char*)&val, 4, 0x1401, 1);

	val = 0x607E0008U;
	writeOD((unsigned char*)&val, 4, 0x1601, 1);

	val = 0x60FF0020U;
	writeOD((unsigned char*)&val, 4, 0x1601, 2);

	val = 0x2U;
	writeOD((unsigned char*)&val, 1, 0x1601, 0);

	val = 0x00000300U+node_id;
	writeOD((unsigned char*)&val, 4, 0x1401, 1);

    data[0] = 0xE8;
	data[1] = 0x3;
	data[2] = 0;
	data[3] = 0;
	writeOD(data, 2, 0x1017, 0x0); // heartbeet time

    if (false == setNodeRun())
	{
        printf("set node %d run status error!\n", node_id);
	}
}

void MotorDriveControl::run()
{
	MotorCtrlMsg buf;
	while (true) {
        if (motor_ctrl_queue.isEmpty() != true) {
            buf = motor_ctrl_queue.dequeue();
			printf("id=0x%x, msg:%d, cur_s=%d, get_s=%d\n", node_id, buf.event, cur_status, buf.obj.motor_status);
			if (buf.event == MCE_STATUS_CHANGE)	{
				//FILE_LOG("change status:cur status node id=%d, =%d, %d!\n", node_id, buf.obj.motor_status, cur_status);
				if ((buf.obj.motor_status == READY_TO_SWITCH_ON) && (cur_status == SWITCH_ON_DISABLED)) {
					sendControlWord(CW_SWITCH_ON);
				} else if ((buf.obj.motor_status == SWITCHED_ON) && (cur_status == READY_TO_SWITCH_ON)) {
					sendControlWord(CW_ENABLE_OPERATION);
				} else if ((buf.obj.motor_status == OPERATION_ENABLED) && (cur_status == SWITCHED_ON)) {
                    U8 sdo_data[] = { 0x3 };
					writeOD(sdo_data, 1, 0x6060, 0);
					//sendTargetVelocity(ctrl_target_velocity);
				} else if (buf.obj.motor_status == SW_FAULT) {
					sendFaultReset();
				} else if (buf.obj.motor_status == SWITCH_ON_DISABLED) {
					sendControlWord(CW_SHUTDOWN);
				} else if (buf.obj.motor_status == SWITCHED_ON) {
					sendControlWord(CW_ENABLE_OPERATION);
				} else if (buf.obj.motor_status == READY_TO_SWITCH_ON) {
					sendControlWord(CW_SWITCH_ON);
				}

				cur_status = buf.obj.motor_status;
			} else if (buf.event == MCE_CTRL_MOTOR_VELOCITY_MODE) {
				ctrl_target_velocity = buf.obj.velocity;
				if (cur_status == SWITCH_ON_DISABLED) {
					//sendTargetVelocity(0.0);
					sendControlWord(CW_SHUTDOWN);
				} else if (cur_status == OPERATION_ENABLED) {
					sendTargetVelocity(ctrl_target_velocity);
				} else {
                    printf("the state cannot send speed:%d!\n", cur_status);
				}
				
			} else if (buf.event == MCE_RESET_NODE) {
                configPdo();
            } else if (buf.event == MCE_GET_VEL) {
				sendGetCurVelocity();
			}
		} else {
            usleep(1*1000);
        }
	}
}

void MotorDriveControl::sendFaultReset() {
	sendControlWord(CW_FAULT_RESET);
    usleep(20*1000);
	sendControlWord(CW_DISABLE_VOLTAGE);
}

void MotorDriveControl::sendControlWord(U16 ctrl_val)
{
	U8 send_data[2] = { ctrl_val, ctrl_val >> 8};
    sendRpdo(0x200, send_data, 2);
}

// input rpm, transfer to count/ms, then write to
// rpm 上限 42
void MotorDriveControl::sendTargetVelocity(F32 velocity)
{
	printf("send vel:@%d, val=%f\n", node_id, velocity);
    U8 send_data[8] = { 0 };
	// 2000: pulse/round; 66.67: 减速比
	F32 t_v = fabs((velocity * 500.0 * 4.0 * 66.666667) / (60.0 * 1000.0));  // pulse per millisecond. //32.0 * 4.0 * 53.0
	S16 t_v_integer = trunc(t_v);
	U16 t_v_decimal = (t_v - t_v_integer) * 65535.0;

	S16 send_val = 0;  // right wheels, +
	if (velocity < 0)  // left wheels, -
	{
		send_val = 64;  // means: -
	}
	//FILE_LOG("polarity:%d", send_val);
    U8 send_polarity[8] = { send_val };  // 64 means negative polarity.
	//FILE_LOG("ver:%f, %d, %d\n", t_v, t_v_integer, t_v_decimal);
	send_data[0] = t_v_decimal;
	send_data[1] = t_v_decimal >> 8;
	send_data[2] = t_v_integer;
	send_data[3] = t_v_integer >> 8;
	//FILE_LOG("send val:0x%x, 0x%x, 0x%x, 0x%x\n", send_data[0], send_data[1], send_data[2], send_data[3]);
	writeOD(send_polarity, 1, 0x607E, 0);
	bool rc2 = writeOD(send_data, 4, 0x60FF, 0);
	//U8 data[5] = { send_val, send_data[0], send_data[1], send_data[2], send_data[3]};

    //sendRpdo(0x300, data, 5);
}

void MotorDriveControl::sendGetCurVelocity() {
    U8 sdo_buffer[4] = { 0 };
	if (true == readOD(sdo_buffer, 4, 0x606C, 0))
	{
        printf("read od 0x606C fail\n");
	}
	//U16 t_v_decimal = sdo_buffer[0] + (sdo_buffer[1] << 8);
	S16 t_v_integer = sdo_buffer[2] + (sdo_buffer[3] << 8);
	F32 t_v = (F32)t_v_integer;

	cur_velocity = (t_v * 60.0 * 1000.0) / (500.0 * 4.0 * 66.666667);
}
