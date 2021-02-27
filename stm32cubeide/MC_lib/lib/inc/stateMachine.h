/*
 * stateMachine.h
 *
 *  Created on: Jun 23, 2019
 *      Author: colson
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

#ifndef INC_STATEMACHINE_H_
#define INC_STATEMACHINE_H_

#include <vector>
#include "main.h"
#include "stdlib.h"
#include "data_struct.h"


#ifndef BOOL
#define BOOL int
#endif

#define true 1
#define false 0

#define MACHINE_MAX_SIZE 3

typedef struct _stateMachineTask{
	uint16_t cmd1;
	uint16_t cmd2;
	uint16_t length;
	uint8_t *data;
	uint8_t checksum;
}stateMachineTask_ST;


typedef struct _stateMachine{
	/* state machine */
	int 	state;

	/* sampled packet */
	uint16_t 	cmd1;
	uint16_t 	cmd2;
	uint16_t 	length;
	uint8_t 	*data;
	uint8_t 	checksum;
	uint16_t 	count;

	/* raw data */
	std::vector<uint8_t> rdata;
	/* queue */
	stateMachineTask_ST rqueue[TASK_MAX_QUEUE_SIZE];
	int rqfront;
	int rqrear;
	int rqcount;
	int rqmax_count;

	stateMachineTask_ST squeue[TASK_MAX_QUEUE_SIZE];
	int sqfront;
	int sqrear;
	int sqcount;
	int sqmax_count;

}stateMachine_ST;


class StateMachine {
protected:
	int device;
	stateMachine_ST info[MACHINE_MAX_SIZE];



public:
	StateMachine(){
		for(int i = 0 ; i < MACHINE_MAX_SIZE; i++)
			machine_init(i);
	};
	/*
	 * state machine
	 * */
	void run(void);
	void machine_init(int index);
	void machine(int index, uint8_t data);

public:
	void add_task(int index);
	BOOL get_task(int index, stateMachineTask_ST *task);
	int send_task(int index, stateMachineTask_ST task, uint8_t *psend);

public:

public:
	int data_push_back(int index, uint8_t data)
	{
		if(index >= MACHINE_MAX_SIZE || index < 0)
			return -1;

		info[index].rdata.push_back(data);
		return 1;
	}
	int data_clear(int index)
	{
		if(index >= MACHINE_MAX_SIZE || index < 0)
			return -1;

		info[index].rdata.clear();
		return 1;
	}
	int get_vector_size(int index)
	{
		if(index >= MACHINE_MAX_SIZE || index < 0)
			return -1;

		return info[index].rdata.size();
	}

public:
	void init_task_queue(int index);
	BOOL is_task_empty(int index);
	BOOL is_task_full(int index);
	BOOL task_enqueue(int index, stateMachineTask_ST value);
	BOOL task_dequeue(int index, stateMachineTask_ST *value);

public:



};




#endif /* INC_STATEMACHINE_H_ */
