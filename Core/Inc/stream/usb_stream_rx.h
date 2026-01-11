#ifndef USB_STREAM_RX_H
#define USB_STREAM_RX_H

#include <stdint.h>

void UsbStreamRx_Init(void);
void UsbStreamRx_Reset(void);
void UsbStreamRx_OnRxData(const uint8_t *data, uint32_t len);
void UsbStreamRx_Poll(void);

#endif /* USB_STREAM_RX_H */
