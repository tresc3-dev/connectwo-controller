#ifndef __STATEMACHINE_H__
#define __STATEMACHINE_H__

#include <main.h>
#include <stdio.h>

#define START 1
#define CONFIRM 2
#define CMD1 3
#define CMD2 4
#define LENGTH1 5
#define LENGTH2 6
#define DATA 7
#define CHECK 8

namespace tresc3
{
    enum machineSequence
    {
        start = 1, confirm, cmd1, cmd2, length1, length2, data, check,
    };

    struct packet{
        unsigned int cmd;
        unsigned int length;
        uint8_t data[255];
        uint8_t checkSum;
    };

    class StateMachine
    {
        public:
            StateMachine();
            ~StateMachine();

        private:
            int state;
            packet result;
            int dataCount;

        public:
            bool run(uint8_t data);
            void resetPacket();
            packet getPacket();
    };
}  // namespace tresc3
#endif // __STATEMACHINE_H__
