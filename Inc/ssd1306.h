#ifndef SSD1306_H
#define SSD1306_H

#include "main.h"
#include <stdint.h>

/* I2C-адрес дисплея (7-бит 0x3C сдвинут на 1 для HAL). У некоторых модулей 0x3D -> (0x3D << 1) */
#define SSD1306_I2C_ADDR   (0x3C << 1)

/* Размер дисплея: 128x64 или 128x32 */
#define SSD1306_WIDTH      128
#define SSD1306_HEIGHT     64

#define SSD1306_BLACK      0
#define SSD1306_WHITE      1

/* Возвращает 1 при успехе, 0 если дисплей не отвечает */
uint8_t SSD1306_Init(I2C_HandleTypeDef *hi2c);

void SSD1306_Fill(uint8_t color);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

/* Текст шрифтом 5x7, scale = 1, 2, 3... (ширина символа = 6*scale пикселей) */
void SSD1306_WriteString(uint8_t x, uint8_t y, const char *str, uint8_t scale);

/* Отправить буфер на экран. Возвращает HAL_OK при успехе */
HAL_StatusTypeDef SSD1306_UpdateScreen(void);

#endif /* SSD1306_H */
