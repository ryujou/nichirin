#ifndef LCD_STREAMER_H
#define LCD_STREAMER_H

#include <stdbool.h>
#include <stdint.h>

#define LCD_STREAMER_FIFO_SIZE 16384U
#define LCD_STREAMER_DMA_CHUNK_SIZE 512U

void LcdStreamer_Init(void);
void LcdStreamer_Start(void);
void LcdStreamer_Stop(void);
bool LcdStreamer_IsActive(void);
void LcdStreamer_OnPacketOk(uint16_t seq);
void LcdStreamer_OnFrameStart(uint16_t seq,
                              uint8_t width,
                              uint8_t height,
                              uint8_t pixfmt,
                              uint16_t frame_bytes,
                              uint8_t flags);
void LcdStreamer_OnFrameData(uint16_t seq, const uint8_t *data, uint16_t len);
void LcdStreamer_OnCrcError(void);
void LcdStreamer_OnDmaTxComplete(void);
void LcdStreamer_Poll(void);
uint8_t LcdStreamer_IsDmaBusy(void);
uint8_t LcdStreamer_IsNoSignal(void);
uint16_t LcdStreamer_GetLastSeqOk(void);
uint16_t LcdStreamer_GetFifoFill(void);
uint32_t LcdStreamer_GetFrameRemaining(void);

#endif /* LCD_STREAMER_H */
