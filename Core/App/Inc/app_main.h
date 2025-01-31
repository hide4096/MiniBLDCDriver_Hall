#pragma once

#include "main.h"
#include <stdio.h>
#include "usart.h"
#include <math.h>
#include <string.h>
#include "tim.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

#ifdef __cplusplus
extern "C"
{
#endif

    void app_main(void);

#ifdef __cplusplus
}
#endif