/**
 * @file button.c
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief
 * @version 1.0
 * @date 28-07_2023
 *
 * @copyright Copyright (c) 2023
 *
 */


#include "button.h"


//-------------------------------------------------------------------------------------------------------------------

const myButton_t Button1 = {
    .color_on = ILI9341_BLACK,
    .pos_x = 5,
    .pos_y = 45,
    .shape_w = 150,
    .shape_h = 90,
    .state = BUTTON_ON
};

const myButton_t Button2 = {
    .color_on = ILI9341_BLACK,
    .pos_x = 165,
    .pos_y = 45,
    .shape_w = 150,
    .shape_h = 90,
    .state = BUTTON_ON
};

const myButton_t Button3 = {
    .color_on = ILI9341_BLACK,
    .pos_x = 5,
    .pos_y = 145,
    .shape_w = 150,
    .shape_h = 90,
    .state = BUTTON_ON
};

const myButton_t Button4 = {
    .color_on = ILI9341_BLACK,
    .pos_x = 165,
    .pos_y = 145,
    .shape_w = 150,
    .shape_h = 90,
    .state = BUTTON_ON
};

const myButton_t Back_Top_Left = {
    .color_on = ILI9341_CYAN,
    .pos_x = 25,
    .pos_y = 22,
    .shape_r = 20,
    .state = BUTTON_ON
};

//-------------------------------------------------------------------------------------------------------------------

const myButton_t Promt_Window = {
    .color_on = ILI9341_ORANGE,
    .pos_x = 70,
    .pos_y = 100,
    .shape_w = 180,
    .shape_h = 100,
    .state = BUTTON_ON
};

const myButton_t Promt_Accept = {
    .color_on = ILI9341_GREEN,
    .pos_x = 76,
    .pos_y = 160,
    .shape_w = 80,
    .shape_h = 30,
    .state = BUTTON_ON
};

const myButton_t Promt_Reject = {
    .color_on = ILI9341_RED,
    .pos_x = 164,
    .pos_y = 160,
    .shape_w = 80,
    .shape_h = 30,
    .state = BUTTON_ON
};

//-------------------------------------------------------------------------------------------------------------------

const myButton_t To_Join_Request_Page = {
    .color_on = ILI9341_CYAN,
    .pos_x = 260,
    .pos_y = 180,
    .shape_r = 40,
    .state = BUTTON_ON
};

const myButton_t To_Network_Page = {
    .color_on = ILI9341_CYAN,
    .pos_x = 60,
    .pos_y = 180,
    .shape_r = 40,
    .state = BUTTON_ON
};

const myButton_t To_Control_Page = {
    .color_on = ILI9341_DARKBLUE,
    .pos_x = 160,
    .pos_y = 180,
    .shape_r = 45,
    .state = BUTTON_ON
};

const myButton_t Select_4G = {
    .color_on = ILI9341_BLACK,
    .pos_x = 10,
    .pos_y = 10,
    .shape_w = 100,
    .shape_h = 20,
    .state = BUTTON_ON
};

const myButton_t Select_WiFi = {
    .color_on = ILI9341_BLACK,
    .pos_x = 120,
    .pos_y = 10,
    .shape_w = 100,
    .shape_h = 20,
    .state = BUTTON_ON
};



//-------------------------------------------------------------------------------------------------------------------

const myButton_t Back_Top_Right = {
    .color_on = ILI9341_CYAN,
    .pos_x = 295,
    .pos_y = 22,
    .shape_r = 20,
    .state = BUTTON_ON
};

//-------------------------------------------------------------------------------------------------------------------

myButton_t Button5 = {
    .color_on = ILI9341_GREEN,
    .color_off = ILI9341_RED,
    .pos_x = 60,
    .pos_y = 60,
    .shape_r = 50,
    .state = BUTTON_OFF
};

myButton_t Button6 = {
    .color_on = ILI9341_GREEN,
    .color_off = ILI9341_RED,
    .pos_x = 260,
    .pos_y = 60,
    .shape_r = 50,
    .state = BUTTON_OFF
};

myButton_t Button7 = {
    .color_on = ILI9341_GREEN,
    .color_off = ILI9341_RED,
    .pos_x = 60,
    .pos_y = 180,
    .shape_r = 50,
    .state = BUTTON_OFF
};

myButton_t Button8 = {
    .color_on = ILI9341_GREEN,
    .color_off = ILI9341_RED,
    .pos_x = 260,
    .pos_y = 180,
    .shape_r = 50,
    .state = BUTTON_OFF
};

const myButton_t To_Main_Page = {
    .color_on = ILI9341_DARKBLUE,
    .pos_x = 160,
    .pos_y = 120,
    .shape_r = 30,
    .state = BUTTON_ON
};

//-------------------------------------------------------------------------------------------------------------------

const myButton_t Delete = {
    .color_on = ILI9341_RED,
    .pos_x = 25,
    .pos_y = 22,
    .shape_r = 20,
    .state = BUTTON_ON
};

const myButton_t Promt_Confirm = {
    .color_on = ILI9341_RED,
    .pos_x = 110,
    .pos_y = 160,
    .shape_w = 100,
    .shape_h = 30,
    .state = BUTTON_ON
};

//-------------------------------------------------------------------------------------------------------------------

const myButton_t FakeHehe = {
    .color_on = ILI9341_BLACK,
    .pos_x = 25,
    .pos_y = 22,
    .shape_r = 20,
    .state = BUTTON_ON
};


