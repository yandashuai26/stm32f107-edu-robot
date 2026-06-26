#ifndef __485_H
#define __485_H

void RS485_Init(void);
void RS485_TX_Opr(uint8_t uAddr, uint8_t opr, uint16_t uReg, uint16_t uData);
uint32_t get_tempture(void);
uint32_t get_distance(void);


#endif