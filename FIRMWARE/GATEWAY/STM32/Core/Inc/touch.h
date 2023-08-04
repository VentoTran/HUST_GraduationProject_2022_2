/* vim: set ai et ts=4 sw=4: */
#ifndef __ILI9341_TOUCH_H__
#define __ILI9341_TOUCH_H__

#include <stdbool.h>
#include "lcd.h"
#include "main.h"
#include "spi.h"
/*** Redefine if necessary ***/

// Warning! Use SPI bus with < 1.3 Mbit speed, better ~650 Kbit to be save.
#define ILI9341_TOUCH_SPI_PORT      (hspi2)
// extern SPI_HandleTypeDef ILI9341_TOUCH_SPI_PORT;
// extern UART_HandleTypeDef huart1;

#define ILI9341_TOUCH_IRQ_Pin       Touch_EXT_Pin
#define ILI9341_TOUCH_IRQ_GPIO_Port Touch_EXT_GPIO_Port
#define ILI9341_TOUCH_CS_Pin        SPI2_CS_Pin
#define ILI9341_TOUCH_CS_GPIO_Port  SPI2_CS_GPIO_Port

#define TOUCH_ROTATION    1

// change depending on screen orientation
#if (TOUCH_ROTATION == 1)
#define ILI9341_TOUCH_SCALE_X 320
#define ILI9341_TOUCH_SCALE_Y 240
#elif (TOUCH_ROTATION == 2)
#define ILI9341_TOUCH_SCALE_X 320
#define ILI9341_TOUCH_SCALE_Y 240
#elif (TOUCH_ROTATION == 0)
#define ILI9341_TOUCH_SCALE_X 240
#define ILI9341_TOUCH_SCALE_Y 320
#elif (TOUCH_ROTATION == 3)
#define ILI9341_TOUCH_SCALE_X 240
#define ILI9341_TOUCH_SCALE_Y 320
#endif

// to calibrate uncomment UART_Printf line in ili9341_touch.c
#define ILI9341_TOUCH_MIN_RAW_X 1200
#define ILI9341_TOUCH_MAX_RAW_X 28000
#define ILI9341_TOUCH_MIN_RAW_Y 1500
#define ILI9341_TOUCH_MAX_RAW_Y 31500

#define ILI9341_TOUCH_OFFSET_RAW_X  (-30)
#define ILI9341_TOUCH_OFFSET_RAW_Y  (0)

//------------ define type --------------

#define BUTTON_ON   (true)
#define BUTTON_OFF  (false)

typedef struct
{
    const uint16_t pos_x;
    const uint16_t pos_y;

    const uint16_t shape_r;
    const uint16_t shape_w;
    const uint16_t shape_h;

    const uint16_t color_on;
    const uint16_t color_off;

    volatile bool state;

} myButton_t;



// call before initializing any SPI devices
void ILI9341_TouchUnselect();

bool ILI9341_TouchPressed();
bool ILI9341_TouchGetCoordinates(uint16_t* x, uint16_t* y);

bool ILI9341_UpdateButton(const myButton_t* button);

bool ILI9341_checkButton(uint16_t x, uint16_t y, myButton_t* button, bool change_state);

#endif // __ILI9341_TOUCH_H__
