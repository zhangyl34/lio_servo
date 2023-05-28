#pragma once

#include "can_adapter.h"
#include "QQueue"

typedef unsigned short U16;
typedef unsigned char U8;
typedef signed char S8;
typedef signed short S16;
typedef float F32;
typedef unsigned int U32;

typedef U16 CtrlWord;
const CtrlWord CW_SHUTDOWN = 0x0006;
const CtrlWord CW_SWITCH_ON = 0x0007;
const CtrlWord CW_ENABLE_OPERATION = 0x000F;
const CtrlWord CW_DISABLE_VOLTAGE = 0x0000;
const CtrlWord CW_QUICK_STOP = 0x0002;
const CtrlWord CW_FAULT_RESET = 0x0080;

typedef U16 StatusWord;
const StatusWord SW_NOT_READY_TO_SWITCH_ON = 0x0000;
const StatusWord SW_SWITCH_ON_DISABLED = 0x0040;
const StatusWord SW_READY_TO_SWITCH_ON = 0x0021;
const StatusWord SW_SWITCHED_ON = 0x0023;
const StatusWord SW_OPERATION_ENABLED = 0x0027;
const StatusWord SW_QUICK_STOP_ACTIVE = 0x0007;
const StatusWord SW_FAULT_REACTION_ACTIVE = 0x000F;
const StatusWord SW_FAULT = 0x0008;

const U16 MASK_NOT_READY_TO_SWITCH_ON = 0x004F;
const U16 MASK_SWITCH_ON_DISABLED = 0x004F;
const U16 MASK_READY_TO_SWITCH_ON = 0x006F;
const U16 MASK_SWITCHED_ON = 0x006F;
const U16 MASK_OPERATION_ENABLED = 0x006F;
const U16 MASK_QUICK_STOP_ACTIVE = 0x006F;
const U16 MASK_FAULT_REACTION_ACTIVE = 0x004F;
const U16 MASK_FAULT = 0x004F;

typedef enum {
	NOT_READY_TO_SWITCH_ON,
	SWITCH_ON_DISABLED,
	READY_TO_SWITCH_ON,
	SWITCHED_ON,
	OPERATION_ENABLED,
	QUICK_STOP_ACTIVE,
	FAULT_REACTION_ACTIVE,
	FAULT,
} MotorStatus;

typedef struct {
	StatusWord status_word;
	U16 mask;
	MotorStatus status;
	char* status_name;
} DriveStatusStr;

typedef enum {
	MCE_STATUS_CHANGE,
	MCE_CTRL_MOTOR_VELOCITY_MODE,
    MCE_RESET_NODE,
	MCE_GET_VEL,
} MotorCtrlEvent;

typedef struct {
	MotorCtrlEvent event;
	union {
		MotorStatus motor_status;
		F32 velocity;
	}obj;

} MotorCtrlMsg;

extern DriveStatusStr drive_status_list[];

class MotorDriveControl : public Canopen, QThread {
public:
	MotorDriveControl(U8 node_id);
	void ctrlMotorMoveByVelocity(F32 velocity);
	void ctrlMotorQuickStop();
	MotorStatus currentStatus();
	F32 currentVelocity();
	void configPdo();

private:
    void recvTpdo(U16 pdo, U8* const data);
    void recvEmergency(U8* const data);
    void recvBootup();
	void run();
	void sendFaultReset();
	void sendControlWord(U16 ctrl_val);
	void sendTargetVelocity(F32 velocity);
	void sendGetCurVelocity();
	
    U8 node_id;
	MotorStatus cur_status;
	F32 ctrl_target_velocity;
	F32 cur_velocity;
    QQueue<MotorCtrlMsg> motor_ctrl_queue;
};
