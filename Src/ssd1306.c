#include "ssd1306.h"
#include <string.h>

#define SSD1306_PAGES (SSD1306_HEIGHT / 8)

static I2C_HandleTypeDef *oled_i2c;
static uint8_t fb[SSD1306_WIDTH * SSD1306_PAGES];   /* кадровый буфер в RAM */

/* ---------- Шрифт 5x7 (колонки, младший бит = верхний пиксель) ---------- */
typedef struct {
    char    c;
    uint8_t col[5];
} Glyph;

static const Glyph font[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', {0x08, 0x08, 0x08, 0x08, 0x08}},
    {'.', {0x00, 0x60, 0x60, 0x00, 0x00}},
    {':', {0x00, 0x36, 0x36, 0x00, 0x00}},
    {'0', {0x3E, 0x51, 0x49, 0x45, 0x3E}},
    {'1', {0x00, 0x42, 0x7F, 0x40, 0x00}},
    {'2', {0x42, 0x61, 0x51, 0x49, 0x46}},
    {'3', {0x21, 0x41, 0x45, 0x4B, 0x31}},
    {'4', {0x18, 0x14, 0x12, 0x7F, 0x10}},
    {'5', {0x27, 0x45, 0x45, 0x45, 0x39}},
    {'6', {0x3C, 0x4A, 0x49, 0x49, 0x30}},
    {'7', {0x01, 0x71, 0x09, 0x05, 0x03}},
    {'8', {0x36, 0x49, 0x49, 0x49, 0x36}},
    {'9', {0x06, 0x49, 0x49, 0x29, 0x1E}},
    {'A', {0x7E, 0x11, 0x11, 0x11, 0x7E}},
    {'B', {0x7F, 0x49, 0x49, 0x49, 0x36}},
    {'C', {0x3E, 0x41, 0x41, 0x41, 0x22}},
    {'D', {0x7F, 0x41, 0x41, 0x41, 0x3E}},
    {'E', {0x7F, 0x49, 0x49, 0x49, 0x41}},
    {'F', {0x7F, 0x09, 0x09, 0x09, 0x01}},
    {'G', {0x3E, 0x41, 0x49, 0x49, 0x7A}},
    {'H', {0x7F, 0x08, 0x08, 0x08, 0x7F}},
    {'I', {0x00, 0x41, 0x7F, 0x41, 0x00}},
    {'J', {0x20, 0x40, 0x41, 0x3F, 0x01}},
    {'K', {0x7F, 0x08, 0x14, 0x22, 0x41}},
    {'L', {0x7F, 0x40, 0x40, 0x40, 0x40}},
    {'M', {0x7F, 0x02, 0x0C, 0x02, 0x7F}},
    {'N', {0x7F, 0x04, 0x08, 0x10, 0x7F}},
    {'O', {0x3E, 0x41, 0x41, 0x41, 0x3E}},
    {'P', {0x7F, 0x09, 0x09, 0x09, 0x06}},
    {'Q', {0x3E, 0x41, 0x51, 0x21, 0x5E}},
    {'R', {0x7F, 0x09, 0x19, 0x29, 0x46}},
    {'S', {0x46, 0x49, 0x49, 0x49, 0x31}},
    {'T', {0x01, 0x01, 0x7F, 0x01, 0x01}},
    {'U', {0x3F, 0x40, 0x40, 0x40, 0x3F}},
    {'V', {0x1F, 0x20, 0x40, 0x20, 0x1F}},
    {'W', {0x3F, 0x40, 0x38, 0x40, 0x3F}},
    {'X', {0x63, 0x14, 0x08, 0x14, 0x63}},
    {'Y', {0x07, 0x08, 0x70, 0x08, 0x07}},
    {'Z', {0x61, 0x51, 0x49, 0x45, 0x43}},
};
#define FONT_COUNT (sizeof(font) / sizeof(font[0]))

/* ---------- Низкий уровень I2C ---------- */
static HAL_StatusTypeDef ssd1306_cmd(uint8_t c)
{
    /* Control byte 0x00 = следом идёт команда */
    return HAL_I2C_Mem_Write(oled_i2c, SSD1306_I2C_ADDR, 0x00,
                             I2C_MEMADD_SIZE_8BIT, &c, 1, 50);
}

uint8_t SSD1306_Init(I2C_HandleTypeDef *hi2c)
{
    oled_i2c = hi2c;

    HAL_Delay(100);   /* время на старт контроллера после подачи питания */

    if (HAL_I2C_IsDeviceReady(oled_i2c, SSD1306_I2C_ADDR, 3, 50) != HAL_OK) {
        return 0;
    }

    static const uint8_t init_seq[] = {
        0xAE,                         /* display OFF */
        0xD5, 0x80,                   /* clock divide / osc freq */
        0xA8, SSD1306_HEIGHT - 1,     /* multiplex ratio */
        0xD3, 0x00,                   /* display offset */
        0x40,                         /* start line = 0 */
        0x8D, 0x14,                   /* charge pump ON (питание от 3.3 В) */
        0x20, 0x02,                   /* page addressing mode */
        0xA1,                         /* segment remap (зеркало по X) */
        0xC8,                         /* COM scan direction remapped (зеркало по Y) */
#if SSD1306_HEIGHT == 64
        0xDA, 0x12,                   /* COM pins config для 128x64 */
#else
        0xDA, 0x02,                   /* COM pins config для 128x32 */
#endif
        0x81, 0x7F,                   /* контраст */
        0xD9, 0xF1,                   /* pre-charge */
        0xDB, 0x40,                   /* VCOMH level */
        0xA4,                         /* вывод из RAM */
        0xA6,                         /* нормальная (не инверсная) полярность */
        0xAF                          /* display ON */
    };

    for (uint32_t i = 0; i < sizeof(init_seq); i++) {
        if (ssd1306_cmd(init_seq[i]) != HAL_OK) {
            return 0;
        }
    }

    SSD1306_Fill(SSD1306_BLACK);
    return (SSD1306_UpdateScreen() == HAL_OK) ? 1 : 0;
}

/* ---------- Рисование в буфере ---------- */
void SSD1306_Fill(uint8_t color)
{
    memset(fb, color ? 0xFF : 0x00, sizeof(fb));
}

void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    uint16_t idx = x + (y / 8) * SSD1306_WIDTH;
    if (color) fb[idx] |=  (1 << (y % 8));
    else       fb[idx] &= ~(1 << (y % 8));
}

static void draw_char(uint8_t x, uint8_t y, char c, uint8_t scale)
{
    const Glyph *g = NULL;
    for (uint32_t i = 0; i < FONT_COUNT; i++) {
        if (font[i].c == c) { g = &font[i]; break; }
    }
    if (g == NULL) return;   /* неизвестный символ — пропускаем */

    for (uint8_t col = 0; col < 5; col++) {
        for (uint8_t row = 0; row < 7; row++) {
            if (g->col[col] & (1 << row)) {
                for (uint8_t dx = 0; dx < scale; dx++) {
                    for (uint8_t dy = 0; dy < scale; dy++) {
                        SSD1306_DrawPixel(x + col * scale + dx,
                                          y + row * scale + dy,
                                          SSD1306_WHITE);
                    }
                }
            }
        }
    }
}

void SSD1306_WriteString(uint8_t x, uint8_t y, const char *str, uint8_t scale)
{
    while (*str) {
        draw_char(x, y, *str, scale);
        x += 6 * scale;      /* 5 колонок символа + 1 пустая */
        str++;
    }
}

/* ---------- Передача буфера на экран ---------- */
HAL_StatusTypeDef SSD1306_UpdateScreen(void)
{
    for (uint8_t page = 0; page < SSD1306_PAGES; page++) {
        if (ssd1306_cmd(0xB0 + page) != HAL_OK) return HAL_ERROR;  /* номер страницы */
        if (ssd1306_cmd(0x00) != HAL_OK)        return HAL_ERROR;  /* колонка, младшие 4 бита */
        if (ssd1306_cmd(0x10) != HAL_OK)        return HAL_ERROR;  /* колонка, старшие 4 бита */

        /* Control byte 0x40 = следом идут данные */
        if (HAL_I2C_Mem_Write(oled_i2c, SSD1306_I2C_ADDR, 0x40,
                              I2C_MEMADD_SIZE_8BIT,
                              &fb[page * SSD1306_WIDTH],
                              SSD1306_WIDTH, 100) != HAL_OK) {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}
