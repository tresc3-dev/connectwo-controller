/*
 * AuCAR.h
 * Task Controller Class
 *
 *  Created on: Jun 23, 2019
 *      Author: colson
 *      dud3722000@naver.com
 */

/* task map */
/*
 * ======================================================================================================================================
 * packet |  2bytes  |  2bytes  |  2bytes  |  2bytes  |      varing-bytes      |    1byte   |
 *        |  s-bits  |   cmd1   |   cmd2   |  length  |  depend on length bits |  checksum  |
 * ======================================================================================================================================
 * cmd1 : command 1
 * request command
 * type, set mode, set value
 * --------------------------------------------------------------------------------------------------------------------------------------
 * == 0x0001
 * request ping (every 1sec)
 * If there is no ack signal from slaves, reboot slaves. (power control)
 *
 * == 0x0002
 * request cpu rate
 *--------------------------------------------------------------------------------------------------------------------------------------
 * == 0x@000
 * request motor@ type(absolute or incremental)
 *
 * == 0x@001
 * request motor@ set mode(PWM mode PID mode)
 *
 * == 0x@002
 * request motor@ set PID(long type setting)
 *
 * == 0x@003
 * request motor@ set value of PWM
 *
 * == 0x@004
 * request motor@ set value of PID (long type)
 *
 * == 0x@005
 * request motor@ set value of wheel size (for calculate speed)
 *
 * == 0x@006
 * request motor@ ~~~~~
 *
 * == 0x@010
 * request motor@ set target PWM
 *
 * == 0x@020
 * request motor@ set target PID
 *
 *
 * --------------------------------------------------------------------------------------------------------------------------------------
 *
 *
 * ======================================================================================================================================
 * cmd2 : command 2
 * ack command
 * Command2 bytes is ack for command1 request.
 * If cmd1 bytes is 0x0001, cmd2 bytes same as cmd1(0x0001) request -> ack
 * --------------------------------------------------------------------------------------------------------------------------------------
 *
 *
 *
 *
 *
 *
 *
 *
 * ======================================================================================================================================
 * --------------------------------------------------------------------------------------------------------------------------------------
 */
/*task map end */

#ifndef INC_AUCAR_H_
#define INC_AUCAR_H_

#include "main.h"
#include "AuCAR_conf.h"
#include "conf.h"
#include "stateMachine.h"

class AuCAR : public StateMachine{
private:
	int seq;
	stateMachineTask_ST task[TASK_MAX_SIZE];
public:
	void get_task(int index);
	void task_run(int index);
};





#endif /* INC_AUCAR_H_ */
