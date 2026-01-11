#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

void ModeManager_Init(void);
void ModeManager_SetLedMode(uint8_t mode_1_9);
uint8_t ModeManager_GetMode(void);
uint8_t ModeManager_GetLedMode(void);
bool ModeManager_IsStreamActive(void);
void ModeManager_EnterStream(void);
void ModeManager_ExitStream(void);
void ModeManager_Poll(void);

#endif /* MODE_MANAGER_H */
