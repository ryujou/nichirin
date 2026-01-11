#ifndef UI_H
#define UI_H

#include <stdint.h>

void Ui_Init(void);
void Ui_LoopOnce(void);
void Ui_TickInc1ms(void);
uint32_t Ui_GetSleepHintMs(void);

#endif /* UI_H */
