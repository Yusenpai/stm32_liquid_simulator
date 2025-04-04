#include "liqsim.h"
extern uint16_t Liq_Display_Buffer[16];
uint16_t Liq_DMA_Buffer_ODR[DMA_BUFFER_LENGTH];
uint32_t Liq_DMA_Buffer_MODER[DMA_BUFFER_LENGTH];

void Liq_Convert_Image_To_DMA(const uint16_t display_buffer[])
{
	uint8_t index = 0;
	for (uint8_t i = 0; i < 16; i++)
		for (uint8_t j = 16; j--;)
			if (Liq_Display_Mask[i] & (1 << j))
			{
				if (display_buffer[i] & (1 << j))
				{
					Liq_DMA_Buffer_ODR[2 * index] = Liq_ODR[index];
					Liq_DMA_Buffer_MODER[2 * index] = Liq_MODER[index];
				}
				else 
				{
					Liq_DMA_Buffer_ODR[2 * index] = 0;
					Liq_DMA_Buffer_MODER[2 * index] = 0xFFFFFFFF;
				}
				index++;
				Liq_DMA_Buffer_ODR[2 * index + 1] = 0;
				Liq_DMA_Buffer_MODER[2 * index + 1] = 0xFFFFFFFF;
			}
}

