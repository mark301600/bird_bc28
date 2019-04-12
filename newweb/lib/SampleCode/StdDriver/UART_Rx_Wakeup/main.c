/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/06/23 1:30p $
 * @brief    Demonstrate how to wake up system form Power-down mode by UART interrupt.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "Nano1X2Series.h"
#include "sys.h"


/* Global variables */
__IO int32_t   _Wakeup_Flag = 0;    /* 1 indicates system wake up from power down mode */

/*---------------------------------------------------------------------------------------------------------*/
/* PDWU Handle function                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void PDWU_IRQHandler()
{
    printf("PDWU_IRQHandler running...\n");
    CLK->WK_INTSTS = 1; /* clear interrupt status */
    _Wakeup_Flag = 1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* GPABC Wake Up Handle function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void GPABC_IRQHandler(void)
{
    /* To check if PB.5 interrupt occurred */
    if (PB->ISRC & BIT13)
    {
        PB->ISRC = BIT13;
        printf("PB.13 INT occurred. \n");

    }
}


/*---------------------------------------------------------------------------------------------------------*/
/* UART Wake Up Handle function                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint32_t u32IntStatus;

    u32IntStatus = UART0->ISR;

    /* Wake Up */
    if (u32IntStatus & UART_ISR_WAKE_IS_Msk)
    {
        printf("UART_Wakeup. \n");
        UART0->ISR = UART_ISR_WAKE_IS_Msk; //clear status
    }

    while(!(UART0->FSR & UART_FSR_TE_F_Msk)) ;  /* waits for message send out */

}

/**
  * @brief  Save multi-function pin setting and then go to power down.
  * @param  None.
  * @return None.
  */
void Enter_PowerDown()
{
    SYS_UnlockReg();

    UART_EnableInt(UART0, UART_IER_WAKE_IE_Msk);
    UART0->CTL |= UART_CTL_WAKE_DATA_EN_Msk;
    NVIC_EnableIRQ(UART0_IRQn);

#ifdef ENABLE_GPIO_WAKEUP
    NVIC_EnableIRQ(GPABC_IRQn);
#endif

    CLK->PWRCTL |= CLK_PWRCTL_WAKEINT_EN;
    NVIC_EnableIRQ(PDWU_IRQn);

    CLK_PowerDown();
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_HXT_EN_Pos); // HXT Enabled

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HXT;

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART_S_Pos);// Clock source from external 12 MHz or 32 KHz crystal clock

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD  */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk|SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |=  (SYS_PB_L_MFP_PB0_MFP_UART0_TX|SYS_PB_L_MFP_PB1_MFP_UART0_RX);

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
    SYS_Init();

    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Rx WAkeup Function Test                             |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will demo UART1 Rx(PB.13) wakeup from   |\n");
    printf("|    power down mode.                                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Please input any data to uart0 Rx pin to wakeup system.   |\n");
    printf("+-----------------------------------------------------------+\n");

#ifdef ENABLE_GPIO_WAKEUP
    GPIO_EnableInt(PB, 13, GPIO_INT_BOTH_EDGE);
    PB->ISRC = BIT13;
#endif

    printf("Going to Power Down...\n\n");

    while(!(UART0->FSR & UART_FSR_TE_F_Msk)) ;  /* waits for message send out */

    /* Enter power down mode */
    Enter_PowerDown();

    if (_Wakeup_Flag == 1)
    {
        _Wakeup_Flag = 0;

        printf("\n Wakeup OK!!");

        CLK_SysTickDelay(335000);
    }

    printf("\n Wakeup demo end.");

    while(1);
}





/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/





