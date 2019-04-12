/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/06/23 1:33p $
 * @brief    This sample code display ¡¥NANO¡¦ on LCD
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "Nano1X2Series.h"
#include "lcd.h"
#include "LCDLIB.h"
#include "sys.h"

#ifdef __DEBUG_MSG
#define DEBUG_MSG   printf
#else
#define DEBUG_MSG(...)
#endif

char int_to_char(int32_t i)
{
    if(i==0) return '0';
    else if(i==1) return '1';
    else if(i==2) return '2';
    else if(i==3) return '3';
    else if(i==4) return '4';
    else if(i==5) return '5';
    else if(i==6) return '6';
    else if(i==7) return '7';
    else if(i==8) return '8';
    else if(i==9) return '9';
    else if(i==10) return 'A';
    else if(i==11) return 'B';
    else if(i==12) return 'C';
    else if(i==13) return 'D';
    else if(i==14) return 'E';
    else if(i==15) return 'F';

    return 0xff;
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
    SYS->PB_H_MFP = 0x88888800;   /* seg 32 ~ 31, 28 ~ 23 */
    SYS->PC_L_MFP = 0x88888888;   /* seg 22 ~ 15 */
    SYS->PC_H_MFP = 0x88888888;   /* seg 14 ~ 7 */
    SYS->PD_L_MFP = 0x88888888;   /* seg 6 ~ 0, COM3 */
    SYS->PD_H_MFP = 0x88888888;   /* COM2 ~ 0, DH2 ~ 1, V3 ~1 */
    PB->OFFD |= 0xFC00000;
    PC->OFFD |= 0xFFFF0000;
    PD->OFFD |= 0xFFFF0000;

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


/**
  * @brief  Main routine.
  * @param  None.
  * @return None.
  */
int32_t main(void)
{
    uint32_t i, j;
    char start_str[5];

    SYS_Init();
    UART0_Init();

    DEBUG_MSG("\nNANO112 Nu-LB Demo \n");

    /* Do LCD Initialization */
    LCD_Open(LCD_C_TYPE, 4, LCD_BIAS_THIRD, LCD_FREQ_DIV64, LCD_CPVOl_3V);
    LCD_EnableDisplay();

    DEBUG_MSG("LCD Init. complete!\n");

    /* Start displaying on LCD */
    LCDLIB_Printf(0, "NANO");
    CLK_SysTickDelay(335000);
    CLK_SysTickDelay(335000);

    while(1)
    {
        for(i=0; i<10; i++)
        {
            for(j=0; j<10; j++)
            {
                start_str[0] = ' ';
                start_str[1] = int_to_char(i);
                start_str[2] = ' ';
                start_str[3] = int_to_char(j);
                start_str[4] = ' ';
                LCDLIB_Printf(0, &start_str[0]);
                CLK_SysTickDelay(335000);
            }

            //CLK_SysTickDelay(335000);
            //CLK_SysTickDelay(335000);
        }
    }

}


/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/



