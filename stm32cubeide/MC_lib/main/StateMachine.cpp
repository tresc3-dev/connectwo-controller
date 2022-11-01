#include <StateMachine.h>
#include <string.h>

namespace tresc3
{
    StateMachine::StateMachine()
    {
        state = START;
    }
    StateMachine::~StateMachine()
    {

    }

    bool StateMachine::run(uint8_t data)
    {
        // printf("get Data: ");
        // printf(data, HEX);
        // Serial.print("state: ");
        // printf(state);
        if(state == START)
        {
            resetPacket();
            bool isEndPacket = false;
            dataCount = 0;
            if(data == 0xFF)
                state = CONFIRM;
            else
                state = START;
        }
        else if(state == CONFIRM)
        {
            if(data == 0xFF)
                state = CMD1;
            else
                state = START;
        }
        else if(state == CMD1)
        {
            result.cmd = (unsigned int)data;
            state = CMD2;
        }
        else if(state == CMD2)
        {
            result.cmd = result.cmd | (unsigned int)data << 8;
            state = LENGTH1;
        }
        else if(state == LENGTH1)
        {
            result.length = (unsigned int)data;
            state = LENGTH2;
        }
        else if(state == LENGTH2)
        {
            result.length = result.length | (unsigned int)data << 8;
            state = DATA;
        }
        else if(state == DATA)
        {
            result.data[dataCount++] = data;
            result.checkSum += data;
            if(dataCount == result.length)
                state = CHECK;
        }
        else if(state == CHECK)
        {
            printf("checksum: %d",result.checkSum);
            printf(" data: %d", data);
            state = START;
            if(result.checkSum == data)
            {
                printf("complete state machine");
                return true;
            }
        }
        else
        {
            printf("error");
        }
        return false;
    }
    

    void StateMachine::resetPacket()
    {
        result.cmd = 0x0000;
        result.length = 0x0000;
        memset(result.data, 0, sizeof(result.data));
        result.checkSum = 0x00;
    }

    packet StateMachine::getPacket()
    {
        return result;
    }
}  // namespace tresc3
