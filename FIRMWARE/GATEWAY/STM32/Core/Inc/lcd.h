/* vim: set ai et ts=4 sw=4: */
#ifndef __ILI9341_H__
#define __ILI9341_H__

#include "font.h"
#include "touch.h"
#include "main.h"
#include <stdbool.h>

#define ILI9341_MADCTL_MY  0x80
#define ILI9341_MADCTL_MX  0x40
#define ILI9341_MADCTL_MV  0x20
#define ILI9341_MADCTL_ML  0x10
#define ILI9341_MADCTL_RGB 0x00
#define ILI9341_MADCTL_BGR 0x08
#define ILI9341_MADCTL_MH  0x04

/*** Redefine if necessary ***/


#define ILI9341_CS_Pin        CS_Pin
#define ILI9341_CS_GPIO_Port  CS_GPIO_Port
#define ILI9341_DC_Pin        RS_Pin
#define ILI9341_DC_GPIO_Port  RS_GPIO_Port
#define ILI9341_WR_Pin        WR_Pin
#define ILI9341_WR_GPIO_Port  WR_GPIO_Port
#define ILI9341_RD_Pin        RD_Pin
#define ILI9341_RD_GPIO_Port  RD_GPIO_Port


#define ILI9341_RES_Pin       RST_Pin
#define ILI9341_RES_GPIO_Port RST_GPIO_Port
#define ILI9341_LED_Pin       BL_Pin
#define ILI9341_LED_GPIO_Port BL_GPIO_Port

#define DATA_PORT   GPIOB

#define DB08_PORT   D8_GPIO_Port
#define DB08        D8_Pin
#define DB09_PORT   D9_GPIO_Port
#define DB09        D9_Pin
#define DB10_PORT   D10_GPIO_Port
#define DB10        D10_Pin
#define DB11_PORT   D11_GPIO_Port
#define DB11        D11_Pin
#define DB12_PORT   D12_GPIO_Port
#define DB12        D12_Pin
#define DB13_PORT   D13_GPIO_Port
#define DB13        D13_Pin
#define DB14_PORT   D14_GPIO_Port
#define DB14        D14_Pin
#define DB15_PORT   D15_GPIO_Port
#define DB15        D15_Pin


#define ROTATION    1

#if ROTATION == 0
#define ILI9341_WIDTH  240
#define ILI9341_HEIGHT 320
#define ILI9341_ROTATION (0x08)

#elif ROTATION == 1
#define ILI9341_WIDTH  320
#define ILI9341_HEIGHT 240
#define ILI9341_ROTATION (0x10 | 0x20 | 0x80 | 0x08)

#elif ROTATION == 2
#define ILI9341_WIDTH  320
#define ILI9341_HEIGHT 240
#define ILI9341_ROTATION (0x20 | 0x40 | 0x08)

#elif ROTATION == 3
#define ILI9341_WIDTH  240
#define ILI9341_HEIGHT 320
#define ILI9341_ROTATION (0x10 | 0x40 | 0x80 | 0x08)

#endif

/****************************/


/****************************/

// Color definitions
#define	ILI9341_BLACK       0x0000
#define	ILI9341_BLUE        0x001F
#define	ILI9341_RED         0xF800
#define	ILI9341_GREEN       0x07E0
#define ILI9341_CYAN        0x07FF
#define ILI9341_MAGENTA     0xF81F
#define ILI9341_YELLOW      0xFFE0
#define ILI9341_WHITE       0xFFFF
#define ILI9341_BROWN 	    0XBC40
#define ILI9341_NAVY        0x000F
#define ILI9341_PURPLE      0x780F
#define ILI9341_ORANGE      0xFD20
#define ILI9341_PINK        0xF81F
#define ILI9341_GRAY  		0X8430

#define ILI9341_BRRED 		0XFC07
#define ILI9341_DARKBLUE    0X01CF
#define ILI9341_LIGHTBLUE   0X7D7C 
#define ILI9341_GRAYBLUE    0X5458
#define ILI9341_LIGHTGREEN  0X841F
#define ILI9341_LIGHTGRAY   0XEF5B
#define ILI9341_LGRAY 	    0XC618
#define ILI9341_LGRAYBLUE   0XA651
#define ILI9341_LBBLUE      0X2B12

#define ILI9341_COLOR565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))



// call before initializing any SPI devices
void ILI9341_Unselect();

void ILI9341_Init(void);

void ILI9341_LCD_LED(bool state);

void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ILI9341_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ILI9341_WriteString(uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor);
void ILI9341_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);

void ILI9341_FillScreen(uint16_t color);
void ILI9341_InvertColors(bool invert);

void ILI9341_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ILI9341_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

void ILI9341_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void ILI9341_FillTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);

void ILI9341_DrawCircle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);
void ILI9341_FillCircle(uint16_t x, uint16_t y, uint16_t r, uint16_t color);

void ILI9341_PlotTimeGraph32( uint16_t x0, uint16_t y0, uint8_t graphSizeX, 
                            uint32_t* Ydata, uint8_t graphSizeY, uint8_t Ymin, uint8_t DataToPixelRatio,
                            uint16_t YlabelMin, uint16_t YlabelMid, uint16_t YlabelMax, 
                            uint16_t XlabelMin, uint16_t XlabelMid, uint16_t XlabelMax, 
                            char* Xunit, char* Yunit, uint16_t color);

void ILI9341_PlotTimeGraph8( uint16_t x0, uint16_t y0, uint8_t graphSizeX, 
                            uint8_t* Ydata, uint8_t graphSizeY, uint8_t Ymin, uint8_t DataToPixelRatio,
                            uint16_t YlabelMin, uint16_t YlabelMid, uint16_t YlabelMax, 
                            uint16_t XlabelMin, uint16_t XlabelMid, uint16_t XlabelMax, 
                            char* Xunit, char* Yunit, uint16_t color);

#endif // __ILI9341_H__
