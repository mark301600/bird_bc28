


//BTL001

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "LCDLIB.h"

#define sub_Zone0 7
#define Zone0_Digit_SegNum 14

#define sub_Zone1  3
#define Zone1_Digit_SegNum  8

#define sub_Zone2  1
#define Zone2_Digit_SegNum  20


const ZoneInfo_TypeDef LCD_ZoneInfo[]=
{
    {sub_Zone0, Zone0_Digit_SegNum},
    {sub_Zone1, Zone1_Digit_SegNum},
    {sub_Zone2, Zone2_Digit_SegNum},
};


/**************************************************************************//**
 *
 * Defines each text's segment (alphabet+numeric) in terms of COM and BIT numbers,
 * Using this way that text segment can be consisted of each bit in the
 * following bit pattern:
 * @illustration
 *               A
 *         ----------
 *         |\   |J  /|
 *        F| H  |  K |B
 *         |  \ | /  |
 *         --G-- --M--
 *         |   /| \  |
 *       E |  Q |  N |C
 *         | /  |P  \|
 *         -----------
 *             D
 *
 *         -------0------
 *
 *        |   \7  |8  /9 |
 *        |5   \  |  /   |1
 *
 *         --6---  ---10--
 *
 *        |    /  |  \11 |
 *        |4  /13 |12 \  |2
 *
 *         -------3------
 *
 *
 *
 *****************************************************************************/




const char Zone0[sub_Zone0][Zone0_Digit_SegNum][2] =
{
    {
        // 1
        //{com, seg}
        // A       // B       // C       // D
        {3,  0}, {2,  0}, {1,  0}, {0,  0},
        // E       // F       // G       // H
        {1, 32}, {2, 32}, {2, 33}, {3, 33},
        // J       // K       // M       // N
        {3,  1}, {2,  1}, {1,  1}, {0,  1},
        // P       // Q
        {0, 33}, {1, 33},
    },
    {
        // 2
        //{com, seg}
        // A       // B       // C       // D
        {3,  4}, {2,  4}, {1,  4}, {0,  4},
        // E       // F       // G       // H
        {1,  2}, {2,  2}, {2,  3}, {3,  3},
        // J       // K       // M       // N
        {3,  5}, {2,  5}, {1,  5}, {0, 5},
        // P       // Q
        {0,  3}, {1,  3},
    },
    {
        // 3
        //{com, seg}
        // A          // B     // C     // D
        {3,  8}, {2,  8}, {1,  8}, {0,   8},
        // E          // F       // G       // H
        {1,  6}, {2,  6}, {2,  7}, {3,   7},
        // J          // K       // M       // N
        {3,  9}, {2,  9}, {1,  9}, {0,  9},
        // P          // Q
        {0,  7}, {1,  7},
    },
    {
        // 4
        //{com, seg}
        // A          // B     // C     // D
        {3, 12}, {2, 12}, {1, 12}, {0,  12},
        // E          // F     // G     // H
        {1, 10}, {2, 10}, {2, 11}, {3,  11},
        // J          // K       // M       // N
        {3, 13}, {2, 13}, {1, 13}, {0, 13},
        // P       // Q
        {0, 11}, {1, 11},
    },
    {
        // 5
        //{com, seg}
        // A          // B     // C     // D
        {3, 16}, {2, 16}, {1, 16}, {0,  16},
        // E          // F     // G     // H
        {1, 14}, {2, 14}, {2, 15}, {3,  15},
        // J          // K     // M     // N
        {3, 17}, {2, 17}, {1, 17}, {0, 17},
        // P          // Q
        {0, 15}, {1, 15},
    },
    {
        // 6
        //{com, seg}
        // A          // B     // C     // D
        {3, 20}, {2, 20}, {1, 20}, {0,  20},
        // E          // F     // G     // H
        {1, 18}, {2, 18}, {2, 19}, {3,  19},
        // J          // K       // M       // N
        {3, 21}, {2, 21}, {1, 21}, {0, 21},
        // P          // Q
        {0, 19}, {1, 19},
    },
    {
        // 7
        //{com, seg}
        // A          // B     // C     // D
        {3, 24}, {2, 24}, {1, 24}, {0,  24},
        // E          // F     // G     // H
        {1, 22}, {2, 22}, {2, 23}, {3,  23},
        // J          // K       // M       // N
        {3, 25}, {2, 25}, {1, 25}, {0, 25},
        // P          // Q
        {0, 23}, {1, 23},
    },

};


const char Zone1[sub_Zone1][Zone1_Digit_SegNum][2] =
{
    {
        // 1
        //{com, seg}
        // A          // B     // C     // D
        {0, 30}, {1, 30}, {2, 30}, {3,  31},
        // E          // F     // G     // M
        {2, 31}, {0, 31}, {1, 31}, {1,  31},
    },
    {
        // 2
        //{com, seg}
        // A          // B     // C     // D
        {0, 29}, {0, 28}, {2, 28}, {3,  28},
        // E          // F     // G     // M
        {2, 29}, {1, 29}, {1, 28}, {1,  28},
    },
    {
        // 3
        //{com, seg}
        // A          // B     // C     // D
        {0, 27}, {0, 26}, {2, 26}, {3,  26},
        // E          // F     // G     // M
        {2, 27}, {1, 27}, {1, 26}, {1,  26},
    },

};

//Symbol Zone
const char Zone2[sub_Zone2][Zone2_Digit_SegNum][2] =
{
    {
        {0, 35}, // LOGO          /* 0 */
        {0, 2},  // 1DP           /* 1 */
        {0, 6},  // 2DP           /* 2 */
        {0, 10}, // 3DP           /* 3 */
        {0, 14}, // 4DP           /* 4 */
        {0, 18}, // 5DP           /* 5 */
        {0, 22}, // 6DP           /* 6 */
        {3, 30}, // 10DP          /* 7 */
        {3, 27}, // 11DP          /* 8 */
        {3, 2},  // 2COL          /* 9 */
        {3, 6},  // 3COL          /* 10 */
        {3, 10}, // 4COL          /* 11 */
        {3, 14}, // 5COL          /* 12 */
        {3, 18}, // 6COL          /* 13 */
        {3, 29}, // 10COL         /* 14 */
        {0, 34}, // BRBL          /* 15 */
        {1, 34}, // B2            /* 16 */
        {2, 34}, // B1            /* 17 */
        {3, 34}, // B0            /* 18 */
        {3, 22}, // SB            /* 19 */
    }
};

char *Zone[] =
{
    (char*)(Zone0),
    (char*)(Zone1),
    (char*)(Zone2),
};


/**************************************************************************//**
 *
 * Defines segments for the alphabet
 * Bit pattern below defined for alphabet (text segments)
 *
 *****************************************************************************/
const uint16_t Zone0_TextDisplay[] =
{
    0x0000, /* space */
    0x1100, /* ! */
    0x0280, /* " */
    0x0000, /* # */
    0x0000, /* $ */
    0x0000, /* % */
    0x0000, /* & */
    0x0000, /* ? */
    0x0039, /* ( */
    0x000f, /* ) */
    0x3fc0, /* * */
    0x1540, /* + */
    0x0000, /* , */
    0x0440, /* - */
    0x8000, /* . */
    0x2200, /* / */

    0x003f, /* 0 */
    0x0006, /* 1 */
    0x045b, /* 2 */
    0x044f, /* 3 */
    0x0466, /* 4 */
    0x046d, /* 5 */
    0x047d, /* 6 */
    0x0007, /* 7 */
    0x047f, /* 8 */
    0x046f, /* 9 */

    0x0000, /* : */
    0x0000, /* ; */
    0x0a00, /* < */
    0x0000, /* = */
    0x2080, /* > */
    0x0000, /* ? */
    0xffff, /* @ */

    0x0477, /* A */
    0x0a79, /* B */
    0x0039, /* C */
    0x20b0, /* D */
    0x0079, /* E */
    0x0071, /* F */
    0x047d, /* G */
    0x0476, /* H */
    0x0006, /* I */
    0x000e, /* J */
    0x0a70, /* K */
    0x0038, /* L */
    0x02b6, /* M */
    0x08b6, /* N */
    0x003f, /* O */
    0x0473, /* P */
    0x203f, /* Q */
    0x0c73, /* R */
    0x046d, /* S */
    0x1101, /* T */
    0x003e, /* U */
    0x2230, /* V */
    0x2836, /* W */
    0x2a80, /* X */
    0x046e, /* Y */
    0x2209, /* Z */

    0x0039, /* [ */
    0x0880, /* backslash */
    0x000f, /* ] */
    0x0001, /* ^ */
    0x0008, /* _ */
    0x0000, /* ` */

    0x1058, /* a */
    0x047c, /* b */
    0x0058, /* c */
    0x045e, /* d */
    0x2058, /* e */
    0x0471, /* f */
    0x0c0c, /* g */
    0x0474, /* h */
    0x0004, /* i */
    0x000e, /* j */
    0x0c70, /* k */
    0x0038, /* l */
    0x1454, /* m */
    0x0454, /* n */
    0x045c, /* o */
    0x0473, /* p */
    0x0467, /* q */
    0x0450, /* r */
    0x0c08, /* s */
    0x0078, /* t */
    0x001c, /* u */
    0x2010, /* v */
    0x2814, /* w */
    0x2a80, /* x */
    0x080c, /* y */
    0x2048, /* z */

    0x0000,
};

/**************************************************************************//**
 * Defines segments for the numeric display
 *****************************************************************************/
const uint16_t Zone1_TextDisplay[] =
{
    0x0000, /* space */
    0x0000, /* ! */
    0x0000, /* " */
    0x0000, /* # */
    0x0000, /* $ */
    0x0000, /* % */
    0x0000, /* & */
    0x0000, /* ? */
    0x0000, /* ( */
    0x0000, /* ) */
    0x0000, /* * */
    0x0000, /* + */
    0x0000, /* , */
    0x0000, /* - */
    0x0000, /* . */
    0x0000, /* / */

    0x3f, /* 0 */
    0x06, /* 1 */
    0xdb, /* 2 */
    0xcf, /* 3 */
    0xe6, /* 4 */
    0xed, /* 5 */
    0xfd, /* 6 */
    0x07, /* 7 */
    0xff, /* 8 */
    0xef, /* 9 */
    0xc0, /* - */
};

const uint16_t *Zone_TextDisplay[] =
{
    (uint16_t*)(Zone0_TextDisplay),
    (uint16_t*)(Zone1_TextDisplay),
};

