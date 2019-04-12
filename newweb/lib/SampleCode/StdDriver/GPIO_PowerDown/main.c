/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 3 $
 * $Date: 15/05/14 9:30a $
 * @brief    Demonstrate how to wake system up form Power-down mode by GPIO interrupt.
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "Nano1X2Series.h"


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* To check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;
    CLK->PWRCTL |= (CLK_PWRCTL_PD_EN_Msk | CLK_PWRCTL_WK_DLY_Msk );

    __WFI();
}

/**
 * @brief       PortA/PortB/PortC IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PortA/PortB/PortC default IRQ, declared in startup_nano1x2series.s.
 */
void GPABC_IRQHandler(void)
{
	uint32_t reg;
    /* To check if PA.1 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PA, BIT1))
    {
        GPIO_CLR_INT_FLAG(PA, BIT1);
        printf("PA.1 INT occurred. \n");

    }
    else
    {
        /* Un-expected interrupt. Just clear all PORTA, PORTB, PORTC interrupts */
        reg = PA->ISRC;
        PA->ISRC = reg;
        reg = PB->ISRC;
        PB->ISRC = reg;
        reg = PC->ISRC;
        PC->ISRC = reg;
        printf("Un-expected interrupts. \n");
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
    CLK->PWRCTL &= ~CLK_PWRCTL_HXT_EN_Msk;
    CLK->PWRCTL |= (0x1 << CLK_PWRCTL_HXT_EN_Pos); // HXT Enabled

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_CLKSTATUS_HXT_STB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HXT;

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable

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
    SYS->PB_H_MFP &= ~(SYS_PB_H_MFP_PB13_MFP_Msk | SYS_PB_H_MFP_PB14_MFP_Msk);
    SYS->PB_H_MFP |= (SYS_PB_H_MFP_PB13_MFP_UART0_RX | SYS_PB_H_MFP_PB14_MFP_UART0_TX);
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------+\n");
    printf("|    GPIO Power-Down and Wake-up by PA.1 Sample Code    |\n");
    printf("+-------------------------------------------------------+\n\n");

    /* PA1 is pin 73 */
    GPIO_SetMode(PA, BIT1, GPIO_PMD_INPUT);
    NVIC_EnableIRQ(GPABC_IRQn);

    /* Configure PA.1 as Input mode and enable interrupt by rising edge trigger */
    GPIO_EnableInt(PA, 1, GPIO_INT_RISING);

    /* Waiting for PA.1 rising-edge interrupt event */
    while(1)
    {
        printf("Wait PA.1 to low\n");
        while(1)
            if (PA1 == 0)
                break;
        printf("Enter to Power-Down (rising-edge) ......  %d\n", PA1);
        PowerDownFunction();
        UART_WAIT_TX_EMPTY(UART0);
        printf("System waken-up done. %d\n\n", PA1);
    }
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
