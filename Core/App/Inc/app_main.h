#pragma once

#include "main.h"
#include <stdio.h>
#include "usart.h"
#include <math.h>
#include <string.h>
#include "tim.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

GETCHAR_PROTOTYPE
{
    uint8_t ch = 0;

    __HAL_UART_CLEAR_OREFLAG(&huart1);
    HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

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