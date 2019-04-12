/******************************************************************************
 * @file     LDROM_main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/06/23 1:15p $
 * @brief    FMC LDROM IAP sample program for NANO1X2 series MCU
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "Nano1X2Series.h"

typedef void (FUNC_PTR)(void);

/**
 * @brief    Routine to get a char
 * @param    None
 * @returns  Get value from UART debug port or semihost
 * @details  Wait UART debug port or semihost to input a char.
 */
static char GetChar(void)
{
    while(1)
    {
        if ((UART0->FSR & UART_FSR_RX_EMPTY_F_Msk) == 0)
        {
            return (UART0->RBR);
        }
    }
}

extern void SendChar_ToUART(int ch);

static void PutString(char *str)
{
    while (*str != '\0')
    {
        SendChar_ToUART(*str++);
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set HCLK source form HXT and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT,CLK_HCLK_CLK_DIVIDER(1));

    /* Enable external 12MHz HXT, 32KHz LXT and HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk | CLK_PWRCTL_LXT_EN_Msk | CLK_PWRCTL_HIRC_EN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_HXT_STB_Msk | CLK_CLKSTATUS_LXT_STB_Msk | CLK_CLKSTATUS_HIRC_STB_Msk);

    /*  Set HCLK frequency 32MHz */
    CLK_SetCoreClock(32000000);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HIRC, CLK_UART_CLK_DIVIDER(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD  */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_TX | SYS_PB_L_MFP_PB1_MFP_UART0_RX);
}


#ifdef __ARMCC_VERSION
__asm __set_SP(uint32_t _sp)
{
    MSR MSP, r0
    BX lr
}
#endif


int main()
{
    uint32_t    sp;
    FUNC_PTR    *func;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    PutString("\n\n");
    PutString("+----------------------------------------+\n");
    PutString("|     Nano1x2 FMC IAP Sample Code        |\n");
    PutString("|           [LDROM code]                 |\n");
    PutString("+----------------------------------------+\n");

    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    PutString("\n\nPress any key to branch to APROM...\n");
    GetChar();

    PutString("\n\nChange VECMAP and branch to APROM...\n");
    while (!(UART0->FSR & UART_FSR_TX_EMPTY_F_Msk));

    sp = FMC_Read(FMC_APROM_BASE);
    func =  (FUNC_PTR *)FMC_Read(FMC_APROM_BASE+4);

    /*  NOTE!
     *     Before change VECMAP, user MUST disable all interrupts.
     */

    /* FMC_SetVectorPageAddr(FMC_APROM_BASE); */
    FMC->ISPCMD = FMC_ISPCMD_VECMAP;
    FMC->ISPADR = FMC_APROM_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    SYS_LockReg();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) /* for GNU C compiler */
    asm("msr msp, %0" : : "r" (sp));
#else
    __set_SP(sp);
#endif

    func();

    while (1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
