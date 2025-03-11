/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
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

#include "debug.h"

#if (DBG_PRINTF_USE == DBG_USE_UART_PRINTF)
/* Macro definitions of stdarg.h to prevent using standard library */
#define VA_START(v, l)  __builtin_va_start(v, l)
#define VA_ARG(v, l)    __builtin_va_arg(v, l)
#define VA_END(v)       __builtin_va_end(v)

#define DECIMAL_BASE            10U /* Cardinality of decimal numbers */
#define HALF_ADJUST_BOUNDARY    5U  /* The boundary for rounding the floating number */
#define MAX_DIV_TIMES           31U
/* FLOAT_SCALE = DECIMAL_BASE ^ (FLOAT_PRECISION + 1) */
#endif

typedef __builtin_va_list va_list;

#if (DBG_PRINTF_USE == DBG_USE_UART_PRINTF)
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

#if (DBG_PRINTF_USE == DBG_USE_UART_PRINTF)
UART_Handle g_dbgUart;
/**
 * @brief   Initialize the UART port for DBG_UartPrintf().
 * @param   baudRate The baud rate of UART port.
 * @retval  BASE_StatusType BASE status type: OK, ERROR, BUSY, TIMEOUT.
 */
BASE_StatusType DBG_UartPrintInit(unsigned int baudRate)
{
    g_dbgUart.baseAddress   = DBG_PRINTF_UART_PORT;
    g_dbgUart.baudRate      = baudRate;
    g_dbgUart.dataLength    = UART_DATALENGTH_8BIT;
    g_dbgUart.stopBits      = UART_STOPBITS_ONE;
    g_dbgUart.parity        = UART_PARITY_NONE;
    g_dbgUart.txMode        = UART_MODE_BLOCKING;
    g_dbgUart.rxMode        = UART_MODE_BLOCKING;
    g_dbgUart.fifoMode      = true;
    g_dbgUart.fifoTxThr     = UART_FIFOFULL_ONE_EIGHT;
    g_dbgUart.fifoRxThr     = UART_FIFOFULL_ONE_EIGHT;
    g_dbgUart.hwFlowCtr     = UART_HW_FLOWCTR_DISABLE;
    return HAL_UART_Init(&g_dbgUart);
}

/**
 * @brief   De-initialize the UART port for DBG_UartPrintf().
 * @retval  BASE_StatusType BASE status type: OK, ERROR, BUSY, TIMEOUT.
 */
BASE_StatusType DBG_UartPrintDeInit(void)
{
    return HAL_UART_DeInit(&g_dbgUart);
}
#endif

/* Format string print function */
#if (DBG_PRINTF_USE == DBG_USE_UART_PRINTF)
/**
 * @brief   Write a character to the UART port.
 * @param   ch The int promotion of the character to be written.
 * @retval  None.
 */
static void DBG_PrintCh(unsigned int ch)
{
    while (DBG_PRINTF_UART_PORT->UART_FR.BIT.txff == 1) {
        ;
    }
    DBG_PRINTF_UART_PORT->UART_DR.reg = ch;
}

/**
 * @brief   Print a string through the UART port.
 * @param   str The string to be printed.
 * @retval  int If succeeded, the total number of characters printed is returned.
 *              If the input parameter is wrong, a BASE_STATUS_ERROR is returned.
 */
static int DBG_PrintStr(const char *str)
{
    DEBUG_ASSERT_PARAM(str != NULL);
    int cnt = 0;
    while (*str != '\0') {
        DBG_PrintCh(*str);
        str++;
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
    while (exponent--) {
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
    if (base == 0) {
        return 0;
    }
    while (num != 0) {
        cnt++;
        if (cnt > MAX_DIV_TIMES) {
            break;
        }
        num /= base;
    }
    cnt = (cnt == 0) ? 1 : cnt;
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
    while (digits != 0) {
        ch = num / DBG_Pow(base, digits - 1);
        num %= DBG_Pow(base, digits - 1);
        if (base == DECIMAL) {
            DBG_PrintCh(ch + '0');
        } else if (base == HEXADECIMAL) {
            if (ch < DECIMAL_BASE) {
                DBG_PrintCh(ch + '0');
            } else {
                DBG_PrintCh(ch - DECIMAL_BASE + 'A');
            }
        } else {
            break;
        }
        digits--;
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
    if (intNum == 0) {
        DBG_PrintCh('0');
        return 1;
    }
    if (intNum < 0) {
        DBG_PrintCh('-');
        intNum = -intNum;
    }
    cnt = DBG_CountDigits(intNum, DECIMAL);
    DBG_PutUnsignedNum(intNum, DECIMAL, cnt);
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
 * @param   fltNumber The floating-point number to be printed.
 * @retval  unsigned int The total number of characters printed.
 */
static unsigned int DBG_PrintFlt(float fltNumber, unsigned int precision)
{
    unsigned int cnt = 0;
    unsigned int floatScale;

    if (fltNumber < 0) {
        DBG_PrintCh('-');
        cnt += 1;
        fltNumber = -fltNumber;
    }
    int integerVal = (int)fltNumber;
    floatScale = DBG_Pow(10, (precision + 1)); /* 10: decimal */
    int floatVal = (long)(floatScale * (fltNumber - integerVal));
    /* Half-adjust: round up or round down */
    if (floatVal % DECIMAL_BASE >= HALF_ADJUST_BOUNDARY) {
        floatVal = floatVal / DECIMAL_BASE + 1;
    } else {
        floatVal = floatVal / DECIMAL_BASE;
    }
    unsigned int fltCnt = DBG_CountDigits(floatVal, DECIMAL);
    /* Rounding to the whole part */
    if (fltCnt > precision) {
        integerVal += 1;
        /* The decimal part is all 0s after the carry is carried to the integer part. */
        floatVal = 0;
        /* Decimal significant bits are subtracted by 1. */
        fltCnt -= 1;
    }
    cnt += DBG_PrintInt(integerVal);
    DBG_PrintCh('.');
    cnt += 1;
    /* Pad 0 in float part */
    if (precision > fltCnt) {
        for (unsigned int i = 0; i < precision - fltCnt; i++) {
            DBG_PrintCh('0'); /* add '0' */
        }
    }
    DBG_PutUnsignedNum(floatVal, DECIMAL, fltCnt); /* print unsigned number */
    cnt += precision;
    return cnt;
}

/**
 * @brief   Parse the format specifier and print the parameter by format.
 * @param   ch The format specifier.
 * @param   paramList The pointer of the variable parameter list.
 * @retval  unsigned int The total number of characters printed.
 */
static unsigned int ParseSpecifier(const char ch, va_list *paramList)
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
        case 'c': /* Character format data. */
            chVal = VA_ARG(*paramList, int); /* Use type int because of byte alignment */
            DBG_PrintCh(chVal);
            cnt += 1;
            break;
        case 's': /* String format data. */
            strVal = VA_ARG(*paramList, const char *);
            cnt += DBG_PrintStr(strVal);
            break;
        case 'd': /* Integer decimal data. */
            intVal = VA_ARG(*paramList, int);
            cnt += DBG_PrintInt(intVal);
            break;
        case 'u': /* Unsigned decimal data. */
            unsignedVal = VA_ARG(*paramList, unsigned int);
            tmpCnt = DBG_CountDigits(unsignedVal, DECIMAL);
            DBG_PutUnsignedNum(unsignedVal, DECIMAL, tmpCnt);
            cnt += tmpCnt;
            break;
        case 'x': /* Hexadecimal data. */
        case 'X':
        case 'p': /* Address data. */
            hexVal = VA_ARG(*paramList, unsigned int);
            cnt += DBG_PrintHex(hexVal);
            break;
        case 'f': /* Floating-point data. */
            fltVal = VA_ARG(*paramList, double);
            cnt += DBG_PrintFlt(fltVal, 5); /* default precision: 5 */
            break;
        default:
            DBG_PrintCh(ch);
            cnt += 1;
            break;
    }
    return cnt;
}

/**
 * @brief   Print decimal number with field width.
 * @param   intNum The decimal number to be printed.
 * @param   fieldWidth Field width.
 * @retval  unsigned int The total number of characters printed.
 */
static unsigned int DBG_PrintIntWithField(int intNum, int fieldWidth)
{
    int zeroCnt = 0;
    int digitsCnt = 0;
    unsigned int cnt = 0;

    if (intNum == 0) {
        DBG_PrintCh('0');
        return 1;
    }
    if (intNum < 0) {
        DBG_PrintCh('-'); /* add symbol */
        cnt++;
        intNum = -intNum;
        digitsCnt = DBG_CountDigits(intNum, DECIMAL); /* get int value's width */
        zeroCnt = fieldWidth - digitsCnt;
        for (int i = 0; i < zeroCnt; i++) {
            DBG_PrintCh('0'); /* add '0' */
            cnt++;
        }
        cnt += digitsCnt;
    } else {
        digitsCnt = DBG_CountDigits(intNum, DECIMAL); /* get int value's width */
        cnt = digitsCnt;
        zeroCnt = fieldWidth - digitsCnt;
        for (int i = 0; i < zeroCnt; i++) {
            DBG_PrintCh('0'); /* add '0' */
            cnt++;
        }
    }
    DBG_PutUnsignedNum(intNum, DECIMAL, digitsCnt);
    return cnt;
}

static int DBG_Atoi(const char **s)
{
    int i, c;

    for (i = 0; '0' <= (c = **s) && c <= '9'; ++*s) {
        i = i * 10  + c - '0'; /* 10: decimal */
    }
    return i;
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
 * @retval  int     If succeeded, the total number of characters printed is returned.
 *                  If the input parameter is wrong, return BASE_STATUS_ERROR.
 */
int DBG_UartPrintf(const char *format, ...)
{
    DEBUG_ASSERT_PARAM(format != NULL);
    int cnt = 0;
    int fieldWidth = 0;
    int floatPrecision = 0;
    float fltVal = 0;
    int intVal = 0;
    va_list paramList;
    VA_START(paramList, format);

    while (*format != '\0') {
        if (*format != '%') {
            DBG_PrintCh(*format);
            cnt += 1;
        } else {
            format++;
            if (*format == '0') {
                format++;
                fieldWidth = DBG_Atoi(&format);
                intVal = VA_ARG(paramList, int);
                cnt += DBG_PrintIntWithField(intVal, fieldWidth);
            } else if (*format == '.') {
                format++;
                floatPrecision = DBG_Atoi(&format);
                fltVal = VA_ARG(paramList, double);
                cnt += DBG_PrintFlt(fltVal, floatPrecision);
            } else {
                cnt += ParseSpecifier(*format, &paramList);
            }
        }
        format++;
    }
    VA_END(paramList);
    return cnt;
}

/**
 * @brief   Writes characters to memory of a specified length and returns the length written.
 * @param   chstr The int promotion of the character to be written.
 * @param   destMem point of memory address.
 * @param   destLen point of memory lenth.
 * @retval  unsigned int the length written.
 */
static unsigned int DBG_SnprintfChar(char chstr, char** destMem, unsigned int* destLen)
{
    DEBUG_ASSERT_PARAM(destMem != NULL);
    DEBUG_ASSERT_PARAM(destLen != NULL);
    if (*destLen == 0) {
        return 0;
    }
    **destMem = chstr; /* Value assigned to the address pointed to by the memory pointer */
    *destMem = *destMem + 1; /* Memory pointer address plus 1 */
    *destLen = *destLen - 1; /* The remaining length of the buffer space is reduced by 1. */
    return 1;
}

/**
 * @brief   write a string to memory of a specified length and returns the length written.
 * @param   str Character string to be formatted for output.
 * @param   destMem point of memory address.
 * @param   destLen point of memory lenth.
 * @retval  unsigned int the length written.
 */
static unsigned int DBG_SnprintfString(const char *str, char** destMem, unsigned int* destLen)
{
    DEBUG_ASSERT_PARAM(str != NULL);
    DEBUG_ASSERT_PARAM(destMem != NULL);
    DEBUG_ASSERT_PARAM(destLen != NULL);
    int cnt = 0;
    while (*str != '\0') {
        if (DBG_SnprintfChar(*str, destMem, destLen) == 0) {
            break;  /* Store character strings based on the actual remaining space. */
        }
        str++;
        cnt++; /* Data and Technology Shifting Operations */
    }
    return cnt; /* Returns the actual written length. */
}


/**
 * @brief   Convert unsigned number to string and Stored in the buffer according to the basic data type.
 * @param   ch Characters to be formatted for output.
 * @param   base The number base of num.
 * @param   destMem point of memory address.
 * @param   destLen point of memory lenth.
 * @retval  bool Whether the writing is successful.
 */
static bool DBG_SnprintfBaseTypeNum(unsigned char ch, NumBase base, char** destMem, unsigned int* destLen)
{
    DEBUG_ASSERT_PARAM(destMem != NULL);
    DEBUG_ASSERT_PARAM(destLen != NULL);
    bool ret = true;
    if (base == DECIMAL) { /* Decimal number */
        if (DBG_SnprintfChar((ch + '0'), destMem, destLen) == 0) { /* Number to String Conversion */
            return false;
        }
    } else if (base == HEXADECIMAL) { /* Hexadecimal number */
        if (ch < DECIMAL_BASE) {
            if (DBG_SnprintfChar((ch + '0'), destMem, destLen) == 0) { /* Number to String Conversion */
                return false;
            }
        } else {
            if (DBG_SnprintfChar((ch - DECIMAL_BASE + 'A'), destMem, destLen) == 0) { /* Number to String Conversion */
                return false;
            }
        }
    } else {
        return false;
    }
    return ret;
}

/**
 * @brief   Convert unsigned number to string and write to memory of a specified length and returns the length written.
 * @param   num The unsigned number to be printed.
 * @param   base The number base of num.
 * @param   numLen The digits of num.
 * @param   destMem point of memory address.
 * @param   destLen point of memory lenth.
 * @retval  unsigned int the real length written.
 */
static unsigned int DBG_SnprintfUnsignedNum(unsigned int num, NumBase base, unsigned int numLen, \
                                            char** destMem, unsigned int* destLen)
{
    DEBUG_ASSERT_PARAM(destMem != NULL);
    DEBUG_ASSERT_PARAM(destLen != NULL);
    unsigned char ch = 0;
    unsigned char realCnt = 0;
    while (numLen != 0) {
        ch = num / DBG_Pow(base, numLen - 1); /* Value of the most significant digit */
        num %= DBG_Pow(base, numLen - 1);
        if (DBG_SnprintfBaseTypeNum(ch, base, destMem, destLen) != true) {
            break;
        }
        numLen--;
        realCnt++;
    }
    return realCnt; /* Returns the actual written length. */
}

/**
 * @brief   Convert signed num and write to memory of a specified length and returns the length written.
 * @param   intNum The decimal number to be printed.
 * @param   destMem point of memory address.
 * @param   destLen point of memory lenth.
 * @retval  unsigned int the real length written.
 */
static unsigned int DBG_SnprintfSignedNum(int intNum, char** destMem, unsigned int* destLen)
{
    unsigned int cnt;
    unsigned int realCnt;
    if (intNum == 0) {
        if (DBG_SnprintfChar('0', destMem, destLen) == 0) { /* If the data is 0, store 0. */
            return 0;
        }
        return 1;
    }
    if (intNum < 0) {
        if (DBG_SnprintfChar('-', destMem, destLen) == 0) {  /* If the data is negative, store the minus sign. */
            return 0;
        }
        intNum = -intNum;
    }
    cnt = DBG_CountDigits(intNum, DECIMAL);  /* Obtains the decimal length of data. */
    realCnt = DBG_SnprintfUnsignedNum(intNum, DECIMAL, cnt, destMem, destLen);
    return realCnt; /* Returns the actual written length. */
}

/**
 * @brief   Convert hex num and write to memory of a specified length and returns the length written.
 * @param   hexNum The hex number to be printed.
 * @param   destMem point of memory address.
 * @param   destLen point of memory lenth.
 * @retval  unsigned int the real length written.
 */
static unsigned int DBG_SnprintfHex(unsigned int hexNum, char** destMem, unsigned int* destLen)
{
    unsigned int cnt = 0;
    unsigned int realCnt;
    if (hexNum == 0) {
        if (DBG_SnprintfChar('0', destMem, destLen) == 0) { /* If the data is 0, store 0. */
            return 0;
        }
        return 1;
    }  /* Obtain and store the length of the character converted from the hexadecimal number. */
    cnt += DBG_CountDigits(hexNum, HEXADECIMAL);
    realCnt = DBG_SnprintfUnsignedNum(hexNum, HEXADECIMAL, cnt, destMem, destLen);
    return realCnt; /* Returns the actual written length. */
}

/**
 * @brief   Convert float number and write to memory of a specified length and returns the length written.
 * @param   fltNum The float number to be printed.
 * @param   precision Floating-point number precision.
 * @param   destMem point of memory address.
 * @param   destLen point of memory lenth.
 * @retval  unsigned int the real length written.
 */
static unsigned int DBG_SnprintfFlt(float fltNum, unsigned int precision, char** destMem, unsigned int* destLen)
{
    unsigned int count = 0;
    unsigned int realCnt = 0;
    unsigned int floatScale;

    if (fltNum < 0) {
        if (DBG_SnprintfChar('-', destMem, destLen) == 0) {  /* Stores the minus sign of a negative number */
            return 0;
        }
        count++;
        fltNum = -fltNum;
    }
    int integerVal = (int)fltNum;
    floatScale = DBG_Pow(10, (precision + 1)); /* 10: decimal */
    int floatVal = (long)(floatScale * (fltNum - integerVal));
    /* Half-adjust: round up or round down */
    if (floatVal % DECIMAL_BASE >= HALF_ADJUST_BOUNDARY) {
        floatVal = floatVal / DECIMAL_BASE + 1;
    } else {
        floatVal = floatVal / DECIMAL_BASE;
    }
    unsigned int fltCnt = DBG_CountDigits(floatVal, DECIMAL);
    /* Rounding to the whole part */
    if (fltCnt > precision) {
        /* Decimal significant bits are subtracted by 1. */
        fltCnt -= 1;
        integerVal += 1;
        /* The decimal part is all 0s after the carry is carried to the integer part. */
        floatVal = 0;
    }
    count += DBG_SnprintfSignedNum(integerVal, destMem, destLen); /* Store Integer Part */
    if (DBG_SnprintfChar('.', destMem, destLen) == 0) { /* Stores the decimal point of a floating point number */
        return 0;
    }
    count++;
    /* Pad 0 in float part */
    if (precision > fltCnt) {
        for (unsigned int i = 0; i < precision - fltCnt; i++) {
            if (DBG_SnprintfChar('0', destMem, destLen) == 0) {
                return 0;  /* Fill 0 when the number of decimal places is less than the number of significant digits */
            }
            count++;
        }
    }
    realCnt = DBG_SnprintfUnsignedNum(floatVal, DECIMAL, fltCnt, destMem, destLen);
    realCnt += count;
    return realCnt; /* Returns the actual written length. */
}


/**
 * @brief   Parse the format specifier convert to string and write to memory of a
            specified length and returns the length written.
 * @param   ch The format specifier.
 * @param   paramList The pointer of the variable parameter list.
 * @param   destMem point of memory address.
 * @param   destLen point of memory lenth.
 * @retval  unsigned int the real length written.
 */
static unsigned int DBG_SnprintfParseSpecifier(char** destMem, unsigned int* destLen, \
                                               const char ch, va_list *paramList)
{
    unsigned int countValue = 0, realCnt = 0, tmpCnt = 0, unsignedVal = 0, hexVal = 0;
    char chVal = 0;
    const char *strVal = NULL;
    int intVal = 0;
    float fltVal = 0;
    switch (ch) {
        case 'c': /* character type variable */
            chVal = VA_ARG(*paramList, int); /* Use type int because of byte alignment */
            if (DBG_SnprintfChar(chVal, destMem, destLen) == 0) {
                return 0;
            }
            countValue++;
            break;
        case 's': /* Variable of the string type */
            strVal = VA_ARG(*paramList, const char *);
            countValue += DBG_SnprintfString(strVal, destMem, destLen);
            break;
        case 'd': /* integer type variable */
            intVal = VA_ARG(*paramList, int);
            countValue += DBG_SnprintfSignedNum(intVal, destMem, destLen);
            break;
        case 'u': /* unsigned integer type variable */
            unsignedVal = VA_ARG(*paramList, unsigned int);
            tmpCnt = DBG_CountDigits(unsignedVal, DECIMAL);
            DBG_PutUnsignedNum(unsignedVal, DECIMAL, tmpCnt);
            realCnt = DBG_SnprintfUnsignedNum(unsignedVal, DECIMAL, tmpCnt, destMem, destLen);
            countValue += realCnt;
            break;
        case 'x':
        case 'X':
        case 'p': /* Pointer address type variable or variable address */
            hexVal = VA_ARG(*paramList, unsigned int);
            countValue += DBG_SnprintfString("0x", destMem, destLen);
            countValue += DBG_SnprintfHex(hexVal, destMem, destLen);
            break;
        case 'f': /* Floating-point variable */
            fltVal = VA_ARG(*paramList, double);
            countValue += DBG_SnprintfFlt(fltVal, 6, destMem, destLen); /* default precision: 6 */
            break;
        default:
            if (DBG_SnprintfChar(ch, destMem, destLen) == 0) {
                return 0;
            }
            countValue++;
            break;
    }
    return countValue;
}


/**
 * @brief   Convert decimal number with field width to string and write to memory of a
            specified length and returns the length written.
 * @param   intNumber The decimal number to be printed.
 * @param   fieldWidth Field width.
 * @param   destMem point of memory address.
 * @param   destLen point of memory lenth.
 * @retval  unsigned int the real length written.
 */
static unsigned int DBG_SnprintfIntWithField(int intNumber, int fieldWidth, char** destMem, unsigned int* destLen)
{
    int zeroCnt = 0;
    int digitsCnt = 0;
    unsigned int realCnt = 0;
    if (intNumber == 0) {
        if (DBG_SnprintfChar('0', destMem, destLen) == 0) {  /* If the data is 0, store 0. */
            return 0; /* When the remaining space is insufficient, 0 is returned. */
        }
        return 1;
    }
    if (intNumber < 0) {
        if (DBG_SnprintfChar('-', destMem, destLen) == 0) {  /* If the data is negative, store the minus sign. */
            return 0; /* When the remaining space is insufficient, 0 is returned. */
        }
        intNumber = -intNumber;
        digitsCnt = DBG_CountDigits(intNumber, DECIMAL); /* get int value's width */
        zeroCnt = fieldWidth - digitsCnt;
        for (int i = 0; i < zeroCnt; i++) {
            if (DBG_SnprintfChar('0', destMem, destLen) == 0) { /* Fill 0 when the bit width is less than */
                return 0; /* When the remaining space is insufficient, 0 is returned. */
            }
        }
    } else {
        digitsCnt = DBG_CountDigits(intNumber, DECIMAL); /* get int value's width */
        zeroCnt = fieldWidth - digitsCnt;
        for (int i = 0; i < zeroCnt; i++) {
            if (DBG_SnprintfChar('0', destMem, destLen) == 0) {  /* Fill 0 when the bit width is less than */
                return 0; /* When the remaining space is insufficient, 0 is returned. */
            }
        }
    }
    realCnt = DBG_SnprintfUnsignedNum(intNumber, DECIMAL, digitsCnt, destMem, destLen);
    return realCnt; /* Returns the actual written length. */
}


/**
 * @brief   Convert format string write to memory of a specified length, supporting %c, %s, %d, %u, %x, %X, %p, %f.
 *          %c      To print a character.
 *          %s      To print a string.
 *          %d      To print a decimal value.
 *          %u      To print an unsigned decimal value.
 *          %x, %X  To print a hexadecimal value using upper case letters.
 *          %p      To print a pointer as a hexadecimal value.
 *          %f      To print a floating-point number with a fixed precision determined by FLOAT_PRECISION.
 * @param   format  A string that contains the text to be printed and the format specifiers.
 * @param   ...     Variable parameter list.
 * @param   destMem point of memory address.
 * @param   destLen point of memory lenth.
 * @retval  int the real length written.
 */
int DBG_Snprintf(char* destMem, unsigned int destLen, const char *format, ...)
{
    DEBUG_ASSERT_PARAM(destMem != NULL);
    DEBUG_ASSERT_PARAM(format != NULL);
    char* pDestMem = NULL;
    pDestMem = destMem; /* Memory Pointer Index */
    unsigned int destMemLenth = destLen;  /* Remaining space of the destination memory */
    int cntVal = 0;
    int fieldWidth = 0;
    int floatPrecision = 0;
    float fltVal = 0;
    int intVal = 0;
    va_list paramList;
    VA_START(paramList, format); /* The variable initialization points to the first parameter. */
    while (*format != '\0') {
        if (*format != '%') {
            if (DBG_SnprintfChar(*format, &pDestMem, &destMemLenth) == 0) {
                return 0;
            }
            cntVal++;
        } else {
            format++;
            if (*format == '0') {
                format++;
                fieldWidth = DBG_Atoi(&format); /* Character to Number */
                intVal = VA_ARG(paramList, int);
                cntVal += DBG_SnprintfIntWithField(intVal, fieldWidth, &pDestMem, &destMemLenth);
            } else if (*format == '.') {
                format++;
                floatPrecision = DBG_Atoi(&format); /* Character to Number */
                fltVal = VA_ARG(paramList, double);
                cntVal += DBG_SnprintfFlt(fltVal, floatPrecision, &pDestMem, &destMemLenth);
            } else { /* escape character parsing */
                cntVal += DBG_SnprintfParseSpecifier(&pDestMem, &destMemLenth, *format, &paramList);
            }
        }
        format++;
    }
    VA_END(paramList);
    return cntVal;
}

#endif