/*
 * hardware.h
 *
 *  Created on: Jun 22, 2019
 *      Author: colson
 */

#ifndef INC_PERIPHUSBCOM_H_
#define INC_PERIPHUSBCOM_H_

#include "main.h"
#include "string.h"


class PeriphUSBCOM {
  protected:
    UART_HandleTypeDef *huart;

    const static uint16_t rbuflen = 1024;
    uint8_t rbuf[rbuflen];
    uint32_t rind;
    inline uint32_t getRdmaInd(void){ return (rbuflen - huart->hdmarx->Instance->NDTR) & (rbuflen - 1); }

    const static uint16_t tbuflen = 2048;
    uint8_t tbuf[tbuflen];
    uint32_t twind, tfind;

  public:
    PeriphUSBCOM(){
    	reset_rbuf();
    }

    PeriphUSBCOM(UART_HandleTypeDef *huart_):
      huart(huart_), rind(0), twind(0), tfind(0){
    }

    void init(){
      reset_rbuf();
    }

    void reset_rbuf(void){
      HAL_UART_Receive_DMA(huart, rbuf, rbuflen);
    }

    int read(){
      int c = -1;
      if(rind != getRdmaInd()){
        c = rbuf[rind++];
        rind &= rbuflen - 1;
      }
      return c;
    }

    void flush(void){
      static bool mutex = false;

      if((huart->gState == HAL_UART_STATE_READY) && !mutex){
        mutex = true;

        if(twind != tfind){
          uint16_t len = tfind < twind ? twind - tfind : tbuflen - tfind;
          HAL_UART_Transmit_DMA(huart, &(tbuf[tfind]), len);
          tfind = (tfind + len) & (tbuflen - 1);
        }
        mutex = false;
      }
    }

    void write(uint8_t* data, int length){
      int n = length;
      n = n <= tbuflen ? n : tbuflen;

      int get_tail;

      //int n_tail = n <= tbuflen - twind ? n : tbuflen - twind;
      int n_tail;
      n <= (int)(tbuflen - twind) ? get_tail = n : get_tail = tbuflen - twind;


      n_tail = get_tail;

      memcpy(&(tbuf[twind]), data, n_tail);
      twind = (twind + n) & (tbuflen - 1);

      if(n != n_tail){
        memcpy(tbuf, &(data[n_tail]), n - n_tail);
      }

      flush();
    }
  protected:
};



#endif /* INC_PERIPHUSART_H_ */
