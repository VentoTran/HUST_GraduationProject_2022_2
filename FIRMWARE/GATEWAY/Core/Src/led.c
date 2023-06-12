
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>

#include "main.h"
#include "cmsis_os.h"

#include "sx1278.h"
#include "common.h"
#include "led.h"
#include "spi.h"
#include "gpio.h"

#include "event_groups.h"
void led_task(void *param)
{
    while (1)
    {
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
