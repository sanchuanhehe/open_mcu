/**
  * @copyright Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2012-2023. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
  * following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
  * disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  * following disclaimer in the documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
  * products derived from this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file    debug.c
  * @author  MCU Driver Team
  * @brief   DEBUG module driver.
  *          This file provides functions to manage the following functionalities of the DEBUG module.
  *          + Initialization and de-initialization functions
  *          + Format string print function
  */

/* Includes ------------------------------------------------------------------*/
#include "debug.h"

/* Macro definitions ---------------------------------------------------------*/
#if (DBG_PRINTF_USE == DBG_USE_CUSTOM_PRINTF)
/* Macro definitions of stdarg.h to prevent using standard library */
#define VA_START(v, l)  __builtin_va_start(v, l)
#define VA_ARG(v, l)    __builtin_va_arg(v, l)
#define VA_END(v)       __builtin_va_end(v)

#define DECIMAL_BASE            10U /**< Cardinality of decimal numbers */
#define HALF_ADJUST_BOUNDARY    5U  /**< The boundary for rounding the floating number */
#define FLOAT_PRECISION         5U  /**< Precision of the decimal part in floating number */
/* FLOAT_SCALE = DECIMAL_BASE ^ (FLOAT_PRECISION + 1) */
#define FLOAT_SCALE DBG_Pow(DECIMAL_BASE, (FLOAT_PRECISION + 1)) /**< Scale of the decimal part in floating number */
#endif

/* Typedef definitions ----------------------------------------------------------------*/
typedef __builtin_va_list va_list;

#if (DBG_PRINTF_USE == DBG_USE_CUSTOM_PRINTF)
/**
  * @brief  Cardinality of binary, octal, decimal, and hexadecimal numbers.
  */
typedef enum {
    BINARY      = 2U,
    OCTAL       = 8U,
    DECIMAL     = 10U,
    HEXADECIMAL = 16U,
} NumBase;
#endif

/* Initialization and de-initialization functions -------------------------------------*/
#if (DBG_PRINTF_USE == DBG_USE_CUSTOM_PRINTF)
/**
 * @brief   Initialize the UART port for DBG_UartPrintf().
 * @param   baudRate The baud rate of UART port.
 * @retval  BSP_StatusType BSP return type: OK, ERROR, BUSY, TIMEOUT.
 */
BSP_StatusType DBG_UartPrintInit(unsigned int baudRate)
{
#ifdef NEED_UART_INIT
    UART_Init(baudRate);
#endif
    return BSP_OK;
}

/**
 * @brief   De-initialize the UART port for DBG_UartPrintf().
 * @retval  BSP_StatusType BSP return type: OK, ERROR, BUSY, TIMEOUT.
 */
BSP_StatusType DBG_UartPrintDeInit(void)
{
    return BSP_OK;
}
#endif

/* Format string print function -------------------------------------------------------*/
#if (DBG_PRINTF_USE == DBG_USE_CUSTOM_PRINTF)
/**
 * @brief   Write a character to the UART port.
 * @param   ch The int promotion of the character to be written.
 * @retval  None.
 */
static void DBG_PrintCh(unsigned int ch)
{
    UART_Out(ch);
}

/**
 * @brief   Print a string through the UART port.
 * @param   str The string to be printed.
 * @retval  int If successed, the total number of characters printed is returned.
 *              If the input paremeter is wrong, a BSP_ERROR is returned.
 */
static int DBG_PrintStr(const char *str)
{
    DEBUG_ASSERT_PARAM(str != NULL);
    int cnt = 0;
    const char *p = str;
    while (*p != '\0') {
        DBG_PrintCh(*p);
        p++;
        cnt++;
    }
    return cnt;
}

/**
 * @brief   Raise base value to the power exponent value.
 * @param   base Base value.
 * @param   exponent Exponent value.
 * @retval  unsigned long The result of raising base to the power exponent.
 */
static unsigned long DBG_Pow(unsigned int base, unsigned int exponent)
{
    unsigned long ret = 1;
    unsigned int exp = exponent;
    while (exp--) {
        ret *= base;
    }
    return ret; /* ret = base ^ exponent */
}

/**
 * @brief   Count the digits of the number.
 * @param   num The number to be counted.
 * @param   base The number base of num.
 * @retval  unsigned int The number of digits.
 */
static unsigned int DBG_CountDigits(int num, NumBase base)
{
    unsigned int cnt = 0;
    int n = num;
    if (base == 0) {
        return 0;
    }
    while (n != 0) {
        cnt++;
        n /= base;
    }
    return cnt;
}

/**
 * @brief   Print unsigned number through UART port.
 * @param   num The unsigned number to be printed.
 * @param   base The number base of num.
 * @param   digits The digits of num.
 */
static void DBG_PutUnsignedNum(unsigned int num, NumBase base, unsigned int digits)
{
    unsigned char ch;
    unsigned int d = digits;
    unsigned int n = num;
    while (d != 0) {
        ch = n / DBG_Pow(base, d - 1);
        n %= DBG_Pow(base, d - 1);
        if (base == DECIMAL) {
            DBG_PrintCh(ch + '0');
        } else if (base == HEXADECIMAL) {
            if (ch < DECIMAL_BASE) {
                DBG_PrintCh(ch + '0');
            } else {
                DBG_PrintCh(ch - DECIMAL_BASE + 'A');
            }
        }
        d--;
    }
}

/**
 * @brief   Print decimal number through UART port.
 * @param   intNum The decimal number to be printed.
 * @retval  unsigned int The total number of characters printed.
 */
static unsigned int DBG_PrintInt(int intNum)
{
    unsigned int cnt;
    int num = intNum;
    if (num == 0) {
        DBG_PrintCh('0');
        return 1;
    }
    if (num < 0) {
        DBG_PrintCh('-');
        num = -num;
    }
    cnt = DBG_CountDigits(num, DECIMAL);
    DBG_PutUnsignedNum(num, DECIMAL, cnt);
    return cnt;
}

/**
 * @brief   Print hexadecimal number through UART port.
 * @param   hexNum The hexadecimal number to be printed.
 * @retval  unsigned int The total number of characters printed.
 */
static unsigned int DBG_PrintHex(unsigned int hexNum)
{
    unsigned int cnt;
    if (hexNum == 0) {
        DBG_PrintCh('0');
        return 1;
    }
    cnt = DBG_CountDigits(hexNum, HEXADECIMAL);
    DBG_PutUnsignedNum(hexNum, HEXADECIMAL, cnt);
    return cnt;
}

/**
 * @brief   Print floating-point number through UART port.
 * @param   fltNum The floating-point number to be printed.
 * @retval  unsigned int The total number of characters printed.
 */
static unsigned int DBG_PrintFlt(float fltNum)
{
    unsigned int cnt = 0;
    float f = fltNum;
    if (f < 0) {
        DBG_PrintCh('-');
        cnt += 1;
        f = -f;
    }
    int integerVal = (int)f;
    int floatVal = (long)(FLOAT_SCALE * (f - integerVal));
    /* Half-adjust: round up or round down */
    if (floatVal % DECIMAL_BASE >= HALF_ADJUST_BOUNDARY) {
        floatVal = floatVal / DECIMAL_BASE + 1;
    } else {
        floatVal = floatVal / DECIMAL_BASE;
    }
    cnt += DBG_PrintInt(integerVal);
    DBG_PrintCh('.');
    cnt += 1;
    /* Pad 0 in float part */
    unsigned int fltCnt = DBG_CountDigits(floatVal, DECIMAL);
    if (FLOAT_PRECISION > fltCnt) {
        for (unsigned int i = 0; i < FLOAT_PRECISION - fltCnt; i++) {
            DBG_PrintCh('0');
        }
    }
    DBG_PutUnsignedNum(floatVal, DECIMAL, fltCnt);
    cnt += FLOAT_PRECISION;
    return cnt;
}

/**
 * @brief   Parse the format specifier and print the parameter by format.
 * @param   ch The format specifier.
 * @param   paramList The pointer of the variable parameter list.
 * @retval  unsigned int The total number of characters printed.
 */
static unsigned int ParseSpecifier(const char ch, const va_list *paramList)
{
    unsigned int cnt = 0;
    unsigned int tmpCnt;
    char chVal = 0;
    const char *strVal = NULL;
    int intVal = 0;
    unsigned int unsignedVal = 0;
    unsigned int hexVal = 0;
    float fltVal = 0;
    switch (ch) {
        case 'c':
            chVal = VA_ARG(*paramList, int); /* Use type int because of byte alignment */
            DBG_PrintCh(chVal);
            cnt += 1;
            break;
        case 's':
            strVal = VA_ARG(*paramList, const char *);
            cnt += DBG_PrintStr(strVal);
            break;
        case 'd':
            intVal = VA_ARG(*paramList, int);
            cnt += DBG_PrintInt(intVal);
            break;
        case 'u':
            unsignedVal = VA_ARG(*paramList, unsigned int);
            tmpCnt = DBG_CountDigits(unsignedVal, DECIMAL);
            DBG_PutUnsignedNum(unsignedVal, DECIMAL, tmpCnt);
            cnt += tmpCnt;
            break;
        case 'x':
        case 'X':
        case 'p':
            hexVal = VA_ARG(*paramList, unsigned int);
            cnt += DBG_PrintHex(hexVal);
            break;
        case 'f':
            fltVal = VA_ARG(*paramList, double);
            cnt += DBG_PrintFlt(fltVal);
            break;
        default:
            DBG_PrintCh(ch);
            cnt += 1;
            break;
    }
    return cnt;
}

/**
 * @brief   Print format string through UART port, supporting %c, %s, %d, %u, %x, %X, %p, %f.
 *          %c      To print a character.
 *          %s      To print a string.
 *          %d      To print a decimal value.
 *          %u      To print an unsigned decimal value.
 *          %x, %X  To print a hexadecimal value using upper case letters.
 *          %p      To print a pointer as a hexadecimal value.
 *          %f      To print a floating-point number with a fixed precision determined by FLOAT_PRECISION.
 * @param   format  A string that contains the text to be printed and the format specifiers.
 * @param   ...     Variable parameter list.
 * @retval  int     If successed, the total number of characters printed is returned.
 *                  If the input paremeter is wrong, return BSP_ERROR.
 */
int DBG_UartPrintf(const char *format, ...)
{
    DEBUG_ASSERT_PARAM(format != NULL);
    int cnt = 0;
    const char *p = format;
    va_list paramList;
    VA_START(paramList, format);

    while (*p != '\0') {
        if (*p != '%') {
            DBG_PrintCh(*p);
            cnt += 1;
        } else {
            p++;
            cnt += ParseSpecifier(*p, &paramList);
        }
        p++;
    }
    VA_END(paramList);
    return cnt;
}
#endif