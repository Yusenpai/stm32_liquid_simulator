#ifndef LIQSIM_H
#define LIQSIM_H

#include <stdint.h>
#include "liqsim_const.h"

#define DMA_BUFFER_LENGTH (216 * 2)
extern uint16_t Liq_Display_Buffer[];
extern uint16_t Liq_DMA_Buffer_ODR[];
extern uint32_t Liq_DMA_Buffer_MODER[];
void Liq_Convert_Image_To_DMA(const uint16_t display_buffer[]);

#endif // LIQSIM_H
