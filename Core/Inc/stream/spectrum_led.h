#ifndef SPECTRUM_LED_H
#define SPECTRUM_LED_H

#include <stdbool.h>
#include <stdint.h>

#define SPECTRUM_LED_COUNT 12U

void SpectrumLed_Init(void);
void SpectrumLed_SetActive(uint8_t active);
bool SpectrumLed_IsActive(void);
void SpectrumLed_OnBands(const uint8_t *bands_val, const uint8_t *bands_peak);
void SpectrumLed_GetLastBands(uint8_t out_val[SPECTRUM_LED_COUNT],
                              uint8_t out_peak[SPECTRUM_LED_COUNT]);

#endif /* SPECTRUM_LED_H */
