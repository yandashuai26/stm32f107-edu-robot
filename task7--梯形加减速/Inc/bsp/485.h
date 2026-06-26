#ifndef __485_H
#define __485_H

void Rs485Init(void);
void Rs485TxOpr(uint8_t uAddr, uint8_t opr, uint16_t uReg, uint16_t uData);
uint32_t GetTempture(void);
uint32_t GetDistance(void);

#endif
