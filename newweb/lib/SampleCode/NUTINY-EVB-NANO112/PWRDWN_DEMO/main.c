/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 15 $
 * $Date: 15/06/23 1:35p $
 * @brief    Demonstrate how to keep LCD display while system enters Power-down mode.
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
#include "sys.h"
#include "rtc.h"
#include "LCDLIB.h"

#ifdef __DEBUG_MSG
#define DEBUG_MSG   printf
#else
#define DEBUG_MSG(...)
#endif


/* External functions */
void textticker(char *string, uint32_t delayus);
void showTime(uint32_t hour, uint32_t minute);

/* Global variables */
__IO int32_t   _Wakeup_Flag = 0;    /* 1 indicates system wake up from power down mode */
__IO uint32_t  _Pin_Setting[11];    /* store Px_H_MFP and Px_L_MFP */
__IO uint32_t  _PullUp_Setting[6];  /* store GPIOx_PUEN */



/**
  * @brief  Store original setting of multi-function pin selection.
  * @param  None.
  * @return None.
  */
void SavePinSetting()
{
    /* Save Pin selection setting */
    _Pin_Setting[0] = SYS->PA_L_MFP;
    _Pin_Setting[1] = SYS->PA_H_MFP;
    _Pin_Setting[2] = SYS->PB_L_MFP;
    _Pin_Setting[3] = SYS->PB_H_MFP;
    _Pin_Setting[4] = SYS->PC_L_MFP;
    _Pin_Setting[5] = SYS->PC_H_MFP;
    _Pin_Setting[6] = SYS->PD_L_MFP;
    _Pin_Setting[7] = SYS->PD_H_MFP;
    _Pin_Setting[8] = SYS->PE_L_MFP;
    _Pin_Setting[9] = SYS->PE_H_MFP;
    _Pin_Setting[10] = SYS->PF_L_MFP;

    /* Save Pull-up setting */
    _PullUp_Setting[0] =  PA->PUEN;
    _PullUp_Setting[1] =  PB->PUEN;
    _PullUp_Setting[2] =  PC->PUEN;
    _PullUp_Setting[3] =  PD->PUEN;
    _PullUp_Setting[4] =  PE->PUEN;
    _PullUp_Setting[5] =  PF->PUEN;
}


/**
  * @brief  Restore original setting of multi-function pin selection.
  * @param  None.
  * @return None.
  */
void RestorePinSetting()
{
    /* Restore Pin selection setting */
    SYS->PA_L_MFP = _Pin_Setting[0];
    SYS->PA_H_MFP = _Pin_Setting[1];
    SYS->PB_L_MFP = _Pin_Setting[2];
    SYS->PB_H_MFP = _Pin_Setting[3];
    SYS->PC_L_MFP = _Pin_Setting[4];
    SYS->PC_H_MFP = _Pin_Setting[5];
    SYS->PD_L_MFP = _Pin_Setting[6];
    SYS->PD_H_MFP = _Pin_Setting[7];
    SYS->PE_L_MFP = _Pin_Setting[8];
    SYS->PE_H_MFP = _Pin_Setting[9];
    SYS->PF_L_MFP = _Pin_Setting[10];

    /* Restore Pull-up setting */
    PA->PUEN = _PullUp_Setting[0];
    PB->PUEN = _PullUp_Setting[1];
    PC->PUEN = _PullUp_Setting[2];
    PD->PUEN = _PullUp_Setting[3];
    PE->PUEN = _PullUp_Setting[4];
    PF->PUEN = _PullUp_Setting[5];
}


/**
  * @brief  Save multi-function pin setting and then go to power down.
  * @param  None.
  * @return None.
  */
void Enter_PowerDown()
{
    /* Back up original setting */
    SavePinSetting();

    /* Set function pin to GPIO mode */
    SYS->PA_L_MFP = 0;
    SYS->PA_H_MFP = 0;
    SYS->PB_L_MFP = 0;
    SYS->PB_H_MFP = 0;
    SYS->PC_L_MFP = 0;
    SYS->PC_H_MFP = 0;
    SYS->PD_L_MFP = 0;
    SYS->PD_H_MFP = 0;
    SYS->PE_L_MFP = 0;
    SYS->PE_H_MFP = 0;
    SYS->PF_L_MFP = 0x0000FF00;

    /* Enable GPIO pull up */
    PA->PUEN = 0xFFFF;
    PB->PUEN = 0xFFFF;
    PC->PUEN = 0xFFFF;
    PD->PUEN = 0x07FF;
    PE->PUEN = 0xFFFF;
    PF->PUEN = 0x0033;      /* exclude GPF2 and GPF3 which are HXT OUT/IN */

    SYS_UnlockReg();

    /* Disable LCD clock */
    CLK->APBCLK &= ~CLK_APBCLK_LCD_EN; /* Disable LCD clock */
    CLK->PWRCTL &= ~CLK_PWRCTL_LXT_EN_Msk; /* disable LXT - 32Khz */
    CLK->PWRCTL &= ~CLK_PWRCTL_LIRC_EN_Msk;  /* disable LIRC - 10KHz */

    CLK_PowerDown();
    //SYS_LockReg();

}


/**
  * @brief  This function will not be called.
  * @param  None.
  * @return None.
  */
void Leave_PowerDown()
{

    /* Restore pin setting */
    RestorePinSetting();

    /* Enable LCD clock */
    CLK->PWRCTL |= CLK_PWRCTL_LXT_EN_Msk; /* enable LXT - 32Khz */
    CLK->PWRCTL |= CLK_PWRCTL_LIRC_EN_Msk; /* enable LIRC - 10Khz */
    CLK->APBCLK |= CLK_APBCLK_LCD_EN; /* enable LCD clock */

    SYS_LockReg();
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    // CLK->PWRCTL &= ~CLK_PWRCTL_HXT_EN_Msk;
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
    DEBUG_MSG("[Power Down Demo]\n");

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

    /* Do LCD Initialization */
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

    /* Read curent RTC time */
    RTC_GetDateAndTime(&sCurTime);
    DEBUG_MSG("Current Time:%d/%02d/%02d %02d:%02d:%02d\n",sCurTime.u32Year,sCurTime.u32Month,sCurTime.u32Day,sCurTime.u32Hour,sCurTime.u32Minute,sCurTime.u32Second);

    /* Display RTC time */
    showTime(sCurTime.u32Hour, sCurTime.u32Minute);

    DEBUG_MSG("Start MAIN loop.\n");
    while(1)
    {
        textticker("*** GOING TO POWER DOWN ***", 335000);
        DEBUG_MSG("Going to Power Down...\n");
#ifdef __DEBUG_MSG
        while(!(UART1->FSR & UART_FSR_TE_F_Msk)) ;  /* waits for message send out */
#endif
        /* Enter power down mode */
        Enter_PowerDown();

        DEBUG_MSG("Program resume...\n");

        if (_Wakeup_Flag == 1)
        {
            _Wakeup_Flag = 0;

            textticker("*** WAKE UP ***", 335000);
            CLK_SysTickDelay(335000);
            CLK_SysTickDelay(335000);
            CLK_SysTickDelay(335000);
        }
    };


}


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/



