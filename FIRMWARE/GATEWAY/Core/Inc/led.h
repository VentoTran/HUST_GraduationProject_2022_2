/**
 * @file led.h
 * @author Kyuubi (github.com/Kyuubi0323)
 * @brief 
 * @version 0.1
 * @date 2023-04-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _LED_H_
#define _LED_H_

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define LED_ON 1
#define LED_OFF 0

void led_task(void *param);

#endif
