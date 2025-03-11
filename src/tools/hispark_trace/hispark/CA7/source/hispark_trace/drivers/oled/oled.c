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
  * @file    oled.c
  * @author  MCU Driver Team
  * @brief   oled process Header
  */

#include <stdbool.h>
#include <string.h>
#include "stm32mp1xx.h"
#include "stm32mp1xx_hal.h"
#include "key.h"
#include "oled.h"

#define LCD_WIDTH      192
#define Y_PAGE_BASE    5            /* Page起始值 */
#define LCD_MAX_PAGE   17
#define LCD_PIXEL_PER_PAGE 8    /* 每个page占用的pixel点数 */
#define PIXEL_32 32             /* 占用pixel个数为32 */
#define PIXEL_16 16             /* 占用pixel个数为16 */
#define PIXEL_8  8              /* 占用pixel个数为8 */

#define BITS_IN_BYTE 8

#define CHARS_OF_CHINESE_WORD 2  /* 每中文字符占两个英文字符 */
#define CHARS_OF_AISCII       1  /* 每ASICII字符占1个英文字符 */

typedef struct {
    uint8_t width;
    uint8_t height;
} FontSize;

static bool g_reverseDisplay = false;  /* global reserved display enable or not */

static uint8_t GetDataFromRom(void);
static void SendCommandToRom(uint8_t data);
static void TransferCommandLcd(uint8_t cmd);
static void TransferDataLcd(uint32_t dataValue, bool reversedDisplay);
static void TransferData(uint32_t data);

/**
 * @brief       get pages number base on row number
 * @param       row: row number
 * @retval      page number
 */
static inline uint8_t PageOf(uint8_t row)
{
    return row / LCD_PIXEL_PER_PAGE;
}

/**
 * @brief       get row number base on page
 * @param       page: page number
 * @retval      row number
 */
static inline uint8_t RowOf(uint8_t page)
{
    return (uint8_t)(page * LCD_PIXEL_PER_PAGE);
}

/**
 * @brief       Set global display mode
 * @param       mode: @see DisplayMode
 * @retval      None
 */
void SetGlobalDisaplyaMode(DisplayMode mode)
{
    g_reverseDisplay = (mode == DISPLAY_REVERSE) ? true : false;
}

/**
 * @brief       Check the two char is GB2312
 * @param       a: the first char
 * @param       b: the seconde char
 * @retval      bool: check result
 */
static inline bool IsWordOfGB2312(char a, char b)
{
    return (a >= 0xb0) && (a <= 0xf7) && (b >= 0xa1);
}

/**
 * @brief       Check the two char is GB2312
 * @param       a: the first char
 * @param       b: the seconde char
 * @retval      bool: check result
 */
static inline bool IsCharOfGB2312(char a, char b)
{
    return (a >= 0xa1) && (a <= 0xa3) && (b >= 0xa1);
}

/**
 * @brief       Check the char is ASIIC
 * @param       c: the  char
 * @retval      bool: check result
 */
static inline bool IsASCII(char c)
{
    return (c >= 0x20) && (c <= 0x7e);
}

/**
 * @brief       Check the char is ASIIC
 * @param       c: the  char
 * @retval      bool: check result
 */
static inline uint32_t GetGB2312Pixel16X16FontAddr(char a, char b)
{
    // 国标简体（GB2312）汉字在晶联讯字库IC中的地址由以下公式来计算：
    // Address = ((MSB - 0xB0) * 94 + (LSB - 0xA1)+ 846)*32+ BaseAdd;BaseAdd=0x2c9d0
    uint32_t fontAddr = ((unsigned char)a - 0xb0) * 0x5e;
    fontAddr += ((unsigned char)b - 0xa1) + 0x34e;
    fontAddr *= 0x20;
    fontAddr += 0x2c9d0;
    return fontAddr;
}

/**
 * @brief       Check word address of 15x16 character of GB2312
 * @param       a: the first char
 * @param       b: the seconde char
 * @retval      the word address
 */
static inline uint32_t GetGB2312Pixel15X16FontAddr(char a, char b)
{
    // 国标简体（GB2312）15x16点的字符在晶联讯字库IC中的地址由以下公式来计算：
    // Address = ((MSB - 0xa1) * 94 + (LSB - 0xA1))*32+ BaseAdd;BaseAdd=0x2c9d0
    uint32_t fontAddr = ((unsigned char)a - 0xa1) * 0x5e;
    fontAddr += ((unsigned char)b - 0xa1);
    fontAddr *= 0x20;
    fontAddr += 0x2c9d0;
    return fontAddr;
}

/**
 * @brief       Check word address of 8x16 character of ASIIC
 * @param       c: the first char
 * @retval      the word address
 */
static inline uint32_t GetAISCIIPixel8X16FontAddr(char c)
{
    // AISCII 8x16点的字符在晶联讯字库IC中的地址由以下公式来计算：
    // Address = ((MSB - 0xa1) * 94 + (LSB - 0xA1))*32+ BaseAdd;BaseAdd=0x1DD780
    uint32_t fontAddr = ((unsigned char)c - 0x20);
    fontAddr *= 0x10;
    fontAddr += 0x1DD780;
    return fontAddr;
}

/**
 * @brief       Check word address of 32x32 character of GB2312
 * @param       a: the first char
 * @param       b: the seconde char
 * @retval      the word address
 */
static inline uint32_t GetGB2312Pixel32X32FontAddr(char a, char b)
{
    // 国标简体（GB2312）汉字在晶联讯字库IC中的地址由以下公式来计算：
    // Address = ((MSB - 0xB0) * 94 + (LSB - 0xA1)+ 846)*128+ BaseAdd;BaseAdd=0xedf00
    uint32_t fontAddr = ((unsigned char)a - 0xb0) * 0x5e;
    fontAddr += ((unsigned char)b - 0xa1) + 0x34e;
    fontAddr *= 0x80;
    fontAddr += 0xedf00;
    return fontAddr;
}

/**
 * @brief       Check word address of 31x32 character of GB2312
 * @param       a: the first char
 * @param       b: the seconde char
 * @retval      the word address
 */
static inline uint32_t GetGB2312Pixel31X32FontAddr(char a, char b)
{
    // 国标简体（GB2312）汉字在晶联讯字库IC中的地址由以下公式来计算：
    // Address = ((MSB - 0xB0) * 94 + (LSB - 0xA1))*128+ BaseAdd;BaseAdd=0xedf00
    uint32_t fontAddr = ((unsigned char)a - 0xb0) * 0x5e;
    fontAddr += ((unsigned char)b - 0xa1);
    fontAddr *= 0x80;
    fontAddr += 0xedf00;
    return fontAddr;
}

/**
 * @brief       Check word address of 16x32 character of ASIIC
 * @param       a: the first char
 * @retval      the word address
 */
static inline uint32_t GetPixel16X32FontAddr(char a)
{
    // 英文字符在晶联讯字库IC中的地址由以下公式来计算：
    // Address = ((MSB - 0x20) * 64 + BaseAdd;BaseAdd=0x1e5a50
    // 由于担心8位单片机有乘法溢出问题，所以分三部取地址
    uint32_t fontAddr = ((unsigned char)a - 0x20);
    fontAddr *= 0x40;
    fontAddr += 0x1e5a50;
    return fontAddr;
}

/**
 * @brief   Check whether line breaks are required.
 */
static inline void ProcWrap(uint8_t beginPage, uint8_t beginColumn,
                            uint8_t pageHeight, uint8_t *page, uint8_t pageTh)
{
    if (beginPage <= pageTh && beginColumn <= LCD_WIDTH) {
        *page += pageHeight;
    }
}

/**
 * @brief   Change Phyiscal page to logic page
 */
static uint8_t PhyPageToLogicPage(uint8_t page)
{
    uint8_t pageHeight = PageOf(PIXEL_16);
    if (pageHeight == 0) {
        return 0; /* can't be here */
    }
    return (uint8_t)((page - Y_PAGE_BASE) / pageHeight);
}

/**
 * @brief  Change logic page to Phyiscal page
 */
static uint8_t LogicPageToPhyPage(uint8_t pageValue)
{
    uint32_t pageHeight = PageOf(PIXEL_16);
    uint32_t page = pageValue * pageHeight + Y_PAGE_BASE;
    if (page > LCD_MAX_PAGE) {
        page = 0; /* Protection, shouldn't be here */
    }
    return page & 0xFF;
}

/**
 * @brief       从指定Pixel清除该行
 * @param       X: 为起始的列地址
 * @param       Y: 为起始的行地址
 * @param       xTotal: 列地址起点到终点的差值
 * @param       yTotal: 行地址的起点到终点的差值
 * @retval      无
 */
static void LcdAddress(uint32_t x, uint32_t y, uint32_t xTotal, uint32_t yTotal)
{
    uint32_t newY = (y > 0) ? (y - 1) : 0;

    TransferCommandLcd(0x15); // Set Column Address
    TransferData(x);
    TransferData(x + xTotal - 1);

    TransferCommandLcd(0x75); // Set Page Address
    TransferData(newY);
    TransferData(newY + yTotal - 1);
    TransferCommandLcd(0x30);
    TransferCommandLcd(0x5c);
}

/*
 * OLED的显存
 * 每个字节表示8个像素, 128,表示有128列, 8表示有64行, 高位表示第行数.
 * 比如:g_oled_gram[0][0],包含了第一列,第1~8行的数据. g_oled_gram[0][0].7,即表示坐标(0,0)
 * 类似的: g_oled_gram[1][0].6,表示坐标(1,1), g_oled_gram[10][1].5,表示坐标(10,10),
 *
 * 存放格式如下(高位表示低行数).
 * [0]0 1 2 3 ... 127
 * [1]0 1 2 3 ... 127
 * [2]0 1 2 3 ... 127
 * [3]0 1 2 3 ... 127
 * [4]0 1 2 3 ... 127
 * [5]0 1 2 3 ... 127
 * [6]0 1 2 3 ... 127
 * [7]0 1 2 3 ... 127
 */
static void GetAndWriteCommon(uint32_t fontaddr, uint8_t column, uint8_t page,
                              uint8_t type, FontSize *fontSize)
{
    uint8_t i;
    uint8_t j;
    uint8_t dispData;

    OledRomCsSet(0);
    SendCommandToRom(0x03);
    SendCommandToRom(((fontaddr & 0xff0000) >> 16) & 0xFF); // Get address[23:16]
    SendCommandToRom(((fontaddr & 0xff00) >> 8) & 0xFF);    // Get address[15:8]
    SendCommandToRom(fontaddr & 0xFF);                      // Get address[7:0]
    uint8_t pageNum = PageOf(fontSize->height);
    LcdAddress(column, page, fontSize->width, pageNum);
    for (i = 0; i < pageNum; i++) {
        for (j = 0; j < fontSize->width; j++) {
            dispData = GetDataFromRom();
            TransferDataLcd(dispData, type); // 写数据到LCD,每写完1字节的数据后列地址自动加1
        }
    }
    OledRomCsSet(1);
}

/**
 * @brief   Write 32 x 32 character of GB2312
 */
static void GetAndWrite32x32(uint32_t fontaddr, uint8_t column, uint8_t page, uint8_t type)
{
    FontSize fontSize = {PIXEL_32, PIXEL_32};
    GetAndWriteCommon(fontaddr, column, page, type, &fontSize);
}

/**
 * @brief   Write 16 x 32 character of ASIIC
 */
static void GetAndWrite16x32(uint32_t fontaddr, uint8_t column, uint8_t page, uint8_t type)
{
    FontSize fontSize = {PIXEL_16, PIXEL_32};
    GetAndWriteCommon(fontaddr, column, page, type, &fontSize);
}

/**
 * @brief   Write 16 x 16 character of GB2312
 */
static void GetAndWrite16x16(uint32_t fontaddr, uint8_t column, uint8_t page, uint8_t type)
{
    FontSize fontSize = {PIXEL_16, PIXEL_16};
    GetAndWriteCommon(fontaddr, column, page, type, &fontSize);
}

/**
 * @brief   Write 8 x 16 character of ASIIC
 */
static void GetAndWrite8x16(uint32_t fontaddr, uint8_t column, uint8_t page, uint8_t type)
{
    FontSize fontSize = {PIXEL_8, PIXEL_16};
    GetAndWriteCommon(fontaddr, column, page, type, &fontSize);
}

/**
 * @brief       向OLED写入一个字节
 * @param       data: 要输出的数据
 * @param       cmd: 命令标志 0,表示命令;1,表示数据;
 * @retval      无
 */
static void TransferCommon(uint8_t data)
{
    uint8_t i;
    uint8_t val = data;
    OledCsSet(0); /* 片选引脚拉低，选中SSD1306 */
    for (i = 0; i < BITS_IN_BYTE; i++) {
        OledSclkSet(0);
        if ((val & 0x80) != 0) { /* 高位在前 */
            OledSdinSet(1);  /* 写1 */
        } else {
            OledSdinSet(0); /* 写0 */
        }
        OledSclkSet(1);
        val <<= 1;
    }
}

// =============transfer command to LCM===============
static void TransferCommandLcd(uint8_t cmd)
{
    OledRsSet(0); /* cmd为0,表示命令;cmd为1,表示数据 */
    TransferCommon(cmd);
}

/**
 * @brief       向OLED写数据
 * @param       data: 要输出的数据
 * @retval      无
 */
static void TransferData(uint32_t data)
{
    OledRsSet(1); /* cmd为0,表示命令;cmd为1,表示数据 */
    TransferCommon(data & 0xFF);
}

/**
 * @brief       向LCD写数据
 * @param       data: 要输出的数据
 * @param       reversedDisplay: 是否需要反显
 * @retval      无
 */
static void TransferDataLcd(uint32_t dataValue, bool reversedDisplay)
{
    uint8_t i;
    uint8_t type = (g_reverseDisplay == true) ? !reversedDisplay : reversedDisplay;
    uint32_t data = dataValue;
    OledRsSet(1); /* cmd为0,表示命令;cmd为1,表示数据 */
    OledCsSet(0); /* 片选引脚拉低，选中SSD1306 */

    for (i = 0; i < BITS_IN_BYTE; i++) {
        OledSclkSet(0);
        if (type != 0) {
            if ((data & 0x80) != 0) { /* 高位在前 */
                OledSdinSet(1);  /* 写1 */
            } else {
                OledSdinSet(0); /* 写0 */
            }
        } else {
            if ((data & 0x80) != 0) { /* 高位在前 */
                OledSdinSet(0);  /* 写1 */
            } else {
                OledSdinSet(1); /* 写0 */
            }
        }
        OledSclkSet(1);
        data <<= 1; /* 左移 */
    }
    OledCsSet(1); /* 关闭片选 */
    OledRsSet(1); /* DC电平恢复至初始态 */
}

static uint8_t OledShowStringBold(uint8_t column, uint8_t pageNum, const char *text, uint8_t size, uint8_t type)
{
    uint32_t fontaddr = 0;
    uint8_t i = 0;
    uint8_t pageHeight = size / LCD_PIXEL_PER_PAGE;
    uint8_t page = pageNum;
    uint8_t showLines = 1;
    uint8_t col = column;

    page = (uint8_t)(page * pageHeight + Y_PAGE_BASE);
    if (page > LCD_MAX_PAGE) {
        return showLines; /* Protection, shouldn't be here */
    }

    while ((text[i] > 0x00)) {
        if (IsWordOfGB2312(text[i], text[i + 1])) {  /* Chinese characters (GB2312) */
            fontaddr = GetGB2312Pixel32X32FontAddr(text[i], text[i + 1]);
            GetAndWrite32x32(fontaddr, col, page, type);
            i += CHARS_OF_CHINESE_WORD;
            col += PIXEL_32;
        } else if (IsCharOfGB2312(text[i], text[i + 1])) { /* Chinese punctuation marks (GB2312) */
            GetGB2312Pixel31X32FontAddr(text[i], text[i + 1]);
            GetAndWrite32x32(fontaddr, col, page, type);
            i += CHARS_OF_CHINESE_WORD;
            col += PIXEL_32;
        } else if ((text[i] >= 0x20) && (text[i] <= 0x7e)) { /* English */
            fontaddr = GetPixel16X32FontAddr(text[i]);
            GetAndWrite16x32(fontaddr, col, page, type);
            i += 1;
            col += PIXEL_16;
        } else { /* Unsupported characters, skip it */
            i++;
        }
    }
    return showLines; /* Large font, which can be used to display only one line. */
}

/**
 * @brief       换行
 * @retval      无
 */
static void OledLineWrap(uint8_t *col, uint8_t *page, uint8_t *line, uint8_t pageHeight)
{
    if (*col >= LCD_WIDTH) {
        /* If the number of characters in a line exceeds the maximum, switch to a new page. */
        *page += pageHeight;
        *col  = 0;
        (*line)++;
    }
}

/**
 * @brief       Obtains the address of the GB2312 character.
 * @retval      font address
 */
static uint32_t GetGB2312FontAddr(int8_t a, int8_t b)
{
    uint32_t fontaddr = 0;
    if (IsWordOfGB2312(a, b) || IsCharOfGB2312(a, b)) {
        /* Obtain the address based on the Chinese and Chinese punctuation marks. */
        if (IsCharOfGB2312(a, b)) {
            fontaddr = GetGB2312Pixel15X16FontAddr(a, b);
        } else {
            fontaddr = GetGB2312Pixel16X16FontAddr(a, b);
        }
    }
    return fontaddr;
}

/**
 * @brief       Display a string of chinese(GB2312) or english in oled
 * @retval      Number of lines occupied by a character string
 */
uint8_t OledShowString(uint8_t startColumn, uint8_t startPage, const char *text, uint8_t size, uint8_t type)
{
    if (size == PIXEL_32) {
        return OledShowStringBold(startColumn, startPage, text, size, type);
    }

    uint8_t pageHeight = size / LCD_PIXEL_PER_PAGE;
    uint8_t pageCharNum = LCD_WIDTH / PIXEL_8;
    uint8_t charIdx = 0;
    uint8_t showLines = 1;
    uint8_t col = startColumn;
    uint8_t page = startPage * pageHeight + Y_PAGE_BASE; /* logic page to phy page */

    if (page > LCD_MAX_PAGE) {
        return showLines; /* don't show, return */
    }

    for (uint8_t i = 0; text[i] > 0x00;) {
        uint32_t fontaddr = 0;
        OledLineWrap(&col, &page, &showLines, pageHeight);

        /* Get Char begin index */
        charIdx = ((col + LCD_PIXEL_PER_PAGE - 1) / LCD_PIXEL_PER_PAGE);
        fontaddr = GetGB2312FontAddr(text[i], text[i + 1]);
        if (fontaddr != 0) {
            if ((col + PIXEL_16) > LCD_WIDTH) { /* The width is insufficient for display. Line feeds. */
                OledClearLine(PhyPageToLogicPage(page), charIdx,  pageCharNum);
                ProcWrap(startPage, startColumn, pageHeight, &page, LCD_MAX_PAGE - PageOf(PIXEL_16));
                col = 0;
                showLines++;
            }
            /* Read the data from the specified address and write it to the specified (page, column)
               coordinates of the LCD. */
            GetAndWrite16x16(fontaddr, col, page, type);
            i += CHARS_OF_CHINESE_WORD;
            col += PIXEL_16;
        } else if (IsASCII(text[i])) {
            fontaddr = GetAISCIIPixel8X16FontAddr(text[i]);
            if ((col + PIXEL_8) > LCD_WIDTH) { /* The width is not enough for display. */
                OledClearLine(PhyPageToLogicPage(page), charIdx,  pageCharNum);
                ProcWrap(startPage, startColumn, pageHeight, &page, LCD_MAX_PAGE - PageOf(PIXEL_8));
                col = 0;
                showLines++;
            }
            GetAndWrite8x16(fontaddr, col, page, type);
            i += CHARS_OF_AISCII;
            col += PIXEL_8;
        } else {
            i++; /* Skip unsupported characters */
        }
    }
    /* set char index(idle) and clear from the char index */
    charIdx = ((col + LCD_PIXEL_PER_PAGE - 1) / LCD_PIXEL_PER_PAGE);
    OledClearLine(PhyPageToLogicPage(page), charIdx,  pageCharNum);
    return showLines;
}

/* ***Send instructions to Jinglianxun font library IC** */
static void SendCommandToRom(uint8_t data)
{
    uint8_t i;
    uint8_t d = data;
    for (i = 0; i < BITS_IN_BYTE; i++) {
        if ((d & 0x80) != 0) {
            OledRomInSet(1);
        } else {
            OledRomInSet(0);
        }
        d = d << 1;
        OledRomSckSet(0);
        OledRomSckSet(1);
    }
}

/* ***Obtain Chinese characters or character data (1 byte) from Jinglianxun font library IC.** */
static uint8_t GetDataFromRom(void)
{
    uint8_t i;
    uint8_t retData = 0;
    OledRomSckSet(1);
    for (i = 0; i < BITS_IN_BYTE; i++) {
        OledRomOutSet(1);
        OledRomSckSet(0);
        retData >>= 1;

        if (OledReadRomOut() != 0) {
            retData += 0x80;
        }
        OledRomSckSet(1);
    }
    return (retData);
}

// ------------------------------------------------------------------------------
/**
 * @brief       Clear screen function, clear screen, the whole screen is black! It's like it's not lit.!!!
 * @param       None
 * @retval      None
 */
void OledClear(void)
{
    OledClearFromStartYPixel(0);
}

/**
 * @brief       Clears the row from the specified Pixel
 * @param       startYPixel the specified Pixel
 * @retval      无
 */
void OledClearFromStartYPixel(uint32_t startYPixel)
{
    int i;
    int j;
    LcdAddress(0, startYPixel, LCD_WIDTH, LCD_MAX_PAGE);
    for (i = 0; i < LCD_MAX_PAGE; i++) {
        for (j = 0; j < LCD_WIDTH; j++) {
            TransferDataLcd(0x00, 1);
        }
    }
}

/**
 * @brief       clear 8*16 points
 * @param       column: colum
 * @param       page: page number
 * @param       xTotal: The difference between the start point and end point of the column address
 * @retval      None
 */
static void Clear8x16(uint8_t column, uint8_t page, bool reverseDisplay)
{
    OledRomCsSet(0);
    LcdAddress(column, page, PIXEL_8, PageOf(PIXEL_16));
    for (uint8_t i = 0; i < PageOf(PIXEL_16); i++) {
        for (uint8_t j = 0; j < PIXEL_8; j++) {
            /* When data is written to the LCD, the column address is automatically increased
               by 1 after one byte of data is written. */
            TransferDataLcd(0, reverseDisplay ? 0 : 1);
        }
    }
    OledRomCsSet(1);
}

/**
 * @brief  Clear Lines
 */
void OledClearLine(uint8_t page, uint8_t begin, uint8_t end)
{
    uint8_t beginRow = RowOf(begin);
    uint8_t endRow = RowOf(end);
    uint8_t phyPage = LogicPageToPhyPage(page);
    for (uint8_t i = beginRow; i < endRow; i += PIXEL_8) {
        Clear8x16(i, phyPage, false);
    }
}

/**
 * @brief  Show Menu items
 */
uint8_t OledShowMenuItem(uint8_t page, const char *context, uint8_t size, uint8_t reversed)
{
    return OledShowString(0, page, context, size, reversed);
}

/**
 * @brief  Show Message common process
 */
static void OledShowMsgCommon(uint8_t page, const char *context, uint8_t size, bool highLight)
{
    size_t len = strlen(context);
    uint8_t begin = 0;
    uint8_t width = (uint8_t)(LCD_WIDTH / (size >> 1)); /* The width is calculated based on English characters.
                                                          The width of English characters is only 1/2 of the size. */
    if (len < width - 1) {
        begin = (uint8_t)((width - len) >> 1); /* Make the message character appear in the middle of the current line */
    }
    (void)OledShowString(((uint8_t)(begin * size) >> 1), page, context, size, highLight ? 0 : 1);
}

/**
 * @brief  Show Message with normal font
 */
void OledShowMsg(uint8_t page, const char *context, uint8_t size)
{
    OledShowMsgCommon(page, context, size, 0);
}

static void TxCmdToLcd(void)
{
    TransferCommandLcd(0x30); // EXT=0
    TransferCommandLcd(0x94); // Sleep out
    TransferCommandLcd(0x31); // EXT=1
    TransferCommandLcd(0xD7); // Autoread disable
    TransferDataLcd(0X9F, 1); //

    TransferCommandLcd(0x32); // Analog SET
    TransferDataLcd(0x00, 1); // OSC Frequency adjustment
    TransferDataLcd(0x01, 1); // Frequency on booster capacitors->6KHz
    TransferDataLcd(0x03, 1); // Bias=1/11

    TransferCommandLcd(0x31); // EXT=1
    TransferCommandLcd(0xf0); // 此指令比较重要,不加此指令升压会慢0.5s
    TransferDataLcd(0x0f, 1);
    TransferDataLcd(0x0f, 1);
    TransferDataLcd(0x0f, 1);
    TransferDataLcd(0x0f, 1);

    TransferCommandLcd(0x20); // Gray Level
    uint32_t data[] = {
        0x01, 0x03, 0x05, 0x07, 0x09, 0x0b, 0x0d, 0x10,
        0x11, 0x13, 0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f
    };
    for (int i = 0; i < sizeof(data) / sizeof(data[0]); ++i) {
        TransferDataLcd(data[i], 1);
    }

    TransferCommandLcd(0x30); // EXT=0
    TransferCommandLcd(0x75); // Page Address setting
    TransferDataLcd(0X00, 1); // XS=0
    TransferDataLcd(0X14, 1); // XE=159 0x28
    TransferCommandLcd(0x15); // Clumn Address setting
    TransferDataLcd(0X00, 1); // XS=0
    TransferDataLcd(0Xff, 1); // XE=256

    TransferCommandLcd(0xBC); // Data scan direction
    TransferDataLcd(0x00, 1); // MX.MY=Normal
    TransferDataLcd(0xA6, 1);

    TransferCommandLcd(0x08); // Data scan direction

    TransferCommandLcd(0xCA); // Display Control
    TransferDataLcd(0X00, 1); //
    TransferDataLcd(0X7F, 1); // Duty=128
    TransferDataLcd(0X20, 1); // Nline=off

    TransferCommandLcd(0xF0); // Display Mode
    TransferDataLcd(0X10, 1); // 10=Monochrome Mode,11=4Gray

    TransferCommandLcd(0x81); // 设置对比度
    TransferDataLcd(0x22, 1); // 微调对比度,范围OX00-OXFF
    TransferDataLcd(0x03, 1); // 粗调对比度,范围OX00-0X07
    TransferCommandLcd(0x20); // Power control

    TransferDataLcd(0x0B, 1); // D0=regulator ; D1=follower ; D3=booste,  on:1 off:0
    HAL_Delay(1);
    TransferCommandLcd(0xAF); // Display on
}

/**
 * @brief       Oled Clock Init
 * @param       无
 * @retval      无
 */
static void OledClkInit(void)
{
    /* 使能GPIOA、GPIOB、GPIOC、GPIOD、GPIOE、GPIOH和GPIOI时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
}

/**
 * @brief       初始化OLED(SSD1306)
 * @param       无
 * @retval      无
 */
void OledInit(void)
{
    GPIO_InitTypeDef gpio_init_struct;

    OledClkInit();
    gpio_init_struct.Pin = OLED_SPI_RST_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;         /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                 /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* 高速 */
    HAL_GPIO_Init(OLED_SPI_RST_PORT, &gpio_init_struct); /* RST引脚模式设置 */

    gpio_init_struct.Pin = OLED_SPI_CS_PIN;
    HAL_GPIO_Init(OLED_SPI_CS_PORT, &gpio_init_struct); /* CS引脚模式设置 */

    gpio_init_struct.Pin = OLED_SPI_RS_PIN;
    HAL_GPIO_Init(OLED_SPI_RS_PORT, &gpio_init_struct); /* RS引脚模式设置 */

    gpio_init_struct.Pin = OLED_SPI_SCLK_PIN;
    HAL_GPIO_Init(OLED_SPI_SCLK_PORT, &gpio_init_struct); /* SCLK引脚模式设置 */

    gpio_init_struct.Pin = OLED_SPI_SDIN_PIN;
    HAL_GPIO_Init(OLED_SPI_SDIN_PORT, &gpio_init_struct); /* SDIN引脚模式设置 */

    OledSdinSet(1); /* 设置SDIN引脚 */
    OledSclkSet(1); /* 设置SCLK引脚 */

    // 字库管脚配置
    gpio_init_struct.Pin = ROM_IN_PIN;
    HAL_GPIO_Init(ROM_IN_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ROM_SCK_PIN;
    HAL_GPIO_Init(ROM_SCK_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ROM_CS_PIN;
    HAL_GPIO_Init(ROM_CS_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ROM_OUT_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT; /* 输入 */
    HAL_GPIO_Init(ROM_OUT_PORT, &gpio_init_struct);

    OledCsSet(1);
    OledRsSet(1);

    OledRstSet(0);
    HAL_Delay(100);  // Delay 100ms
    OledRstSet(1);
    HAL_Delay(100);  // Delay 100ms

    TxCmdToLcd();

    OledClear();
}
