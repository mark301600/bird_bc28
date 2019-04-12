/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 12 $
 * $Date: 15/06/23 1:34p $
 * @brief    Demonstrate how to display RTC time on a LCD panel.
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
#include "LCDLIB.h"
#include "sys.h"
#include "rtc.h"

#ifdef __DEBUG_MSG
#define DEBUG_MSG   printf
#else
#define DEBUG_MSG(...)
#endif

__IO uint32_t g_u32RTC_Count  = 0;

/* External functions */
void textticker(char *string, uint32_t delayus);
void showTime(uint32_t hour, uint32_t minute);
void planNextRTCInterrupt(S_RTC_TIME_DATA_T *sCurTime);

/**
  * @brief  RTC IRQHandler.
  * @param  None.
  * @return None.
  */
void RTC_IRQHandler()
{
    S_RTC_TIME_DATA_T sCurTime;

    DEBUG_MSG("RTC_IRQHandler running...\n");

    /* RTC Tick interrupt */
    if ((RTC->RIER & RTC_RIER_TIER_Msk) && (RTC->RIIR & RTC_RIIR_TIF_Msk))
    {
        DEBUG_MSG("RTC Tick Interrupt.\n");
        RTC->RIIR = RTC_RIIR_TIF_Msk;

        if((g_u32RTC_Count %2 ) == 0)
            LCD_SetPixel(3, 29, 0);
        else
            LCD_SetPixel(3, 29, 1);

        g_u32RTC_Count++;  /* increase RTC tick count */

    }

    /* RTC Alarm interrupt */
    if ((RTC->RIER & RTC_RIER_AIER_Msk) && (RTC->RIIR & RTC_RIIR_AIF_Msk))
    {
        DEBUG_MSG("RTC Alarm Interrupt.\n");
        RTC->RIIR = RTC_RIIR_AIF_Msk;

        RTC_GetDateAndTime(&sCurTime);
        DEBUG_MSG("Current Time:%d/%02d/%02d %02d:%02d:%02d\n",sCurTime.u32Year,sCurTime.u32Month,sCurTime.u32Day,sCurTime.u32Hour,sCurTime.u32Minute,sCurTime.u32Second);
        showTime(sCurTime.u32Hour, sCurTime.u32Minute);

        RTC_DISABLE_TICK_WAKEUP();  /* RTC tick shouldn't wake up CPU */
        planNextRTCInterrupt(&sCurTime);
    }

    if ((RTC->RIER & RTC_RIER_SNOOPIER_Msk) && (RTC->RIIR & RTC_RIIR_SNOOPIF_Msk))   /* snooper interrupt occurred */
    {
        RTC->RIIR = RTC_RIIR_SNOOPIF_Msk;
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
    CLK->APBCLK |= CLK_APBCLK_RTC_EN;

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


/**
  * @brief  Main routine.
  * @param  None.
  * @return None.
  */
int32_t main(void)
{
    S_RTC_TIME_DATA_T sCurTime;

    SYS_Init();
    UART0_Init();

    DEBUG_MSG("\nNANO130 NuTiny EVB Demo Program V1.0.0\n");
    DEBUG_MSG("[LCD Demo]\n");

    /* Time Setting */
    sCurTime.u32Year       = 2013;
    sCurTime.u32Month      = 10;
    sCurTime.u32Day        = 15;
    sCurTime.u32Hour       = 0;
    sCurTime.u32Minute     = 0;
    sCurTime.u32Second     = 0;
    sCurTime.u32DayOfWeek  = RTC_TUESDAY;
    sCurTime.u32TimeScale  = RTC_CLOCK_24;

    RTC_Open(&sCurTime);

    DEBUG_MSG("RTC Init. complete!\n");

    /* Do LCD Initializaton */
    LCD_Open(LCD_C_TYPE, 4, LCD_BIAS_THIRD, LCD_FREQ_DIV64, LCD_CPVOl_3V);
    LCD_EnableDisplay();

    DEBUG_MSG("LCD Init. complete!\n");

    /* Start displaying on LCD */
    LCDLIB_SetSymbol(2, 19, 1);
    CLK_SysTickDelay(335000);

    textticker("*** NUVOTON NANO112 ***", 335000);
    CLK_SysTickDelay(335000);

    LCDLIB_Printf(0, "NUVOTON");
    LCD_EnableBlink(250);
    CLK_SysTickDelay(335000);
    CLK_SysTickDelay(335000);
    CLK_SysTickDelay(335000);
    CLK_SysTickDelay(335000);
    CLK_SysTickDelay(335000);
    LCD_DisableBlink();

    /* Read current RTC time */
    RTC_GetDateAndTime(&sCurTime);
    DEBUG_MSG("Current Time:%d/%02d/%02d %02d:%02d:%02d\n",sCurTime.u32Year,sCurTime.u32Month,sCurTime.u32Day,sCurTime.u3Hour,sCurTime.u32Minute,sCurTime.u32Second);

    /* Display RTC time */
    showTime(sCurTime.u32Hour, sCurTime.u32Minute);

    /* Enable RTC alarm for 1 minute to update RTC time */
    planNextRTCInterrupt(&sCurTime);


    /* Enable RTC timer tick for 1 second to display ":" */
    RTC_SetTickPeriod(RTC_TICK_1_SEC);
    RTC_EnableInt(RTC_RIER_TIER_Msk);
    NVIC_EnableIRQ(RTC_IRQn);

    /* forever loop */
    while (1);

}


#ifdef USE_ASSERT
/**
  * @brief  The function prints the source file name and line number where the assert_param() error
  *         occurs, and then stops in an infinite loop. User can add his own codes here if necessary.
  * @param[in] file Source file name
  * @param[in] line Line number
  * @return None
  */
void assert_error(uint8_t * file, uint32_t line)
{
    MFP_UART0_TO_PORTA();                  /* UART0 TX/RX to PA14/PA15*/
    CLK->APBCLK |= CLK_APBCLK_UART0_EN;    /* Enable UART0 clock */
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_UART_MASK;
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_UART_MASK) | CLK_CLKSEL1_UART_HXT;  /* Select 12 Mhz XTAL */


    /* Set UART to 115200,n,8,1,none */
    UART0->BAUD = 0x67;             /* Baud Rate:115200 for 12MHz */
    UART0->TLCTL = 0x03;            /* Word len is 8 bits */

    printf("[%s] line %d : wrong parameters.\r\n", file, line);

    /* Infinite loop */
    while(1) ;

}
#endif


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/



