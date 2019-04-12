/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 15/06/23 1:22p $
 * @brief    Show how to print text on LCD panel of NuTiny-SDK-Nano112-LQFP_TNLCD board.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "Nano1X2Series.h"
#include "lcd.h"
#include "uart.h"
#include "sys.h"
#include "clk.h"

#include "LCDLIB.h"

#define LCD_ALPHABET_NUM        7
#define LCD_DIGIT_NUM           3

long long char_to_int(char c)
{
    if(c=='0') return 0;
    else if(c=='1') return 1;
    else if(c=='2') return 2;
    else if(c=='3') return 3;
    else if(c=='4') return 4;
    else if(c=='5') return 5;
    else if(c=='6') return 6;
    else if(c=='7') return 7;
    else if(c=='8') return 8;
    else if(c=='9') return 9;

    return -1;
}

long long local_atoi(char text[])
{
    int len = strlen(text);
    int len2, negflag=0;
    long long mul=len;
    long long i=0, j=0, mul2=1;
    long long result=0;

    if( text[0] == '-')
    {
        negflag = 1;
        len2 = len - 1;
        for(i=0; i<len2; i++)
        {
            text[i] = text[i+1];
        }
        text[i] = '\0';
        len--;
        mul = len;
    }

    for(i=0; i < len; i++)
    {
        if(mul==1) mul2 = 1;
        else if(mul>1)
            for(j=0; j<(mul-1); j++)
                mul2 *= 10;
        result += mul2*char_to_int(text[i]);
        mul--;
        mul2=1;
    }

    if(negflag==1)
        result = 0 - result;

    return result;
}

uint32_t sysGetNum(void)
{
    uint8_t cInputTemp=0x00, InputString[16]= {0};
    uint32_t nLoop = 0;
    while(cInputTemp != 0x0D)
    {
        cInputTemp = getchar();
        if(cInputTemp == 27)
        {
            return cInputTemp;
        }
        if(cInputTemp == 'x' || cInputTemp == 'X' || cInputTemp == 'f'||
                cInputTemp == 'F' || cInputTemp == 'r' || cInputTemp == 'R')
        {
            return cInputTemp;
        }
        if(cInputTemp == '-')
        {
            InputString[nLoop] = cInputTemp;
            printf("%c",cInputTemp);
            nLoop++;
        }
        else if(cInputTemp >= '0' && cInputTemp <= '9')
        {
            InputString[nLoop] = cInputTemp;
            printf("%c",cInputTemp);
            nLoop++;
        }
    }
    return atoi((const char *)InputString);
}


/**
 * @brief  LCD ISR to handle interrupt event
 * @param  None
 * @retval None
 */
void LCD_IRQHandler(void)
{

    if( LCD->FCSTS & LCD_FCSTS_FCSTS_Msk)
    {
        LCD->FCSTS = LCD_FCSTS_FCSTS_Msk;

        printf("IST: LCD Frame Count interrupt...\n");
    }
    else if( LCD->FCSTS & LCD_FCSTS_PDSTS_Msk)
    {
        LCD->FCSTS = LCD_FCSTS_PDSTS_Msk;

        printf("IST: LCD Power Down interrupt...\n");
    }

}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    //CLK->PWRCTL &= ~CLK_PWRCTL_HXT_EN_Msk;
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_HXT_EN_Pos); // HXT Enabled

    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_LXT_EN_Pos); // LXT Enable

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);
    /* Waiting for 32KHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_LXT_STB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HXT;

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable
    CLK->APBCLK |= CLK_APBCLK_LCD_EN;

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART_S_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD  */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk|SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |=  (SYS_PB_L_MFP_PB0_MFP_UART0_TX|SYS_PB_L_MFP_PB1_MFP_UART0_RX);

    /* Select LCD COMs, SEGs, V1 ~ V3, DH1, DH2 */
    MFP_LCD_TYPEA();

    /* Lock protected registers */
    SYS_LockReg();

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}


static void TestItem (void)
{
    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|                    LCD Sample Program                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Text Display Test                                    |\n");
    printf("+-----------------------------------------------------------+\n");
}

int32_t main(void)
{
    char input;
    char text[LCD_ALPHABET_NUM]="";
    int32_t idx = 0;

    SYS_Init();

    UART0_Init();

    /* LCD Initialize */
    LCD_Open(LCD_C_TYPE, 4, LCD_BIAS_THIRD, LCD_FREQ_DIV64, LCD_CPVOl_3V);

    LCD_EnableDisplay();

    idx = 0;
    strcpy(text, "");   // clear buffer

    TestItem();

    printf("Input text: ");
    while(1)
    {
        input = getchar();
        printf("%c", input);
        if(input == 0xD) break;
        strcat( text, &input);
        idx++;
        if(idx >= LCD_ALPHABET_NUM) break;
    }
    printf("\n");
    printf("%s \n", text);

    LCDLIB_Printf(0, text);

    getchar();

    LCD_DisableDisplay();

    while(1);

}


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/



