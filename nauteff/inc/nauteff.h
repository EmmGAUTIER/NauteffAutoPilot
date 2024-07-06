/*
 * nauteff.h
 *
 *  Created on: 26 mars 2020
 *      Author: emmanuel
 */

#ifndef INC_NAUTEFF_H_
#define INC_NAUTEFF_H_
;
#include "message_buffer.h"




//----- Commandes vers la tâche principale -----
typedef enum {
	MSG_NONE = 0,
	MSG_KBD_AUTO,
	MSG_KBD_STDBY,
	MSG_KBD_STARBOARD_ONE,
	MSG_KBD_PORT_ONE,
	MSG_KBD_STARBOARD_TEN,
	MSG_KBD_PORT_TEN,
	MSG_KBD_STARBOARD_ONE_END,
	MSG_KBD_PORT_ONE_END,
	MSG_MOT_POS_ESTIMEE,
	MSG_MOT_EFFORT,
	MSG_MOT_BLOCKED,
	MSG_MOT_SHORT_CIRCUIT,
	MSG_MOT_END_MOVE,
	MSG_MEMS_VALUES,
	MSG_MEMS_DEFAULT,
	MSG_MOT_EMBRAYE,
	MSG_MOT_DEBRAYE,
	MSG_MOT_STARBOARD,
	MSG_MOT_PORT,
	MSG_MOT_MOVE_STARBOARD,
	MSG_MOT_MOVE_PORT,
	MSG_MOT_STOP,
	MSG_MOT_DMD_EFFORT,
	MSG_MOT_MOVING_CONTROL
} MsgInterTaskMessageType;

typedef uint16_t MsgCode_t;

typedef struct {
	MsgCode_t msgCode;
	float cap;
	float lacet;
} MsgMEMSInfo_t;

typedef struct {
	MsgCode_t msgCode;
} MsgKeyboard_t;

typedef struct {
	MsgCode_t msgCode;
	float effort;
} MsgMotorTorque_t;

typedef struct {
	MsgCode_t msgCode;
	float angle;
} MsgMotorMoveOrder_t;

typedef union {
	MsgCode_t msgCode;
	MsgMEMSInfo_t msgMEMs;
	MsgKeyboard_t msgKeyboard;
	MsgMotorTorque_t msgMotor;
	MsgMotorMoveOrder_t msgMotorMove;
} MessageCore_t;

extern MessageBufferHandle_t bufferCore;
#if 0
typedef struct {
	int16 MessageCode;
	union {
		float effort;
		float motorDisplacement;
		struct {
			float cap, lacet;
		} infoMEMs;
	};
} InterTaskMessage;
#endif

int   inttoa(char*, int, int);
void  dbgmsg(const char *);


#endif /* INC_NAUTEFF_H_ */
