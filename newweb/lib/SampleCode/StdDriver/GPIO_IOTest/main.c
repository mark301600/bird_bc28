/**************************************************************************//**
 * @file     main.c
 * @version  V2.10
 * $Date: 15/06/11 6:12p $
 * @brief    Nano1x2 Series General Purpose I/O Driver Sample Code
 *
 * @note
 * Copyright (C) 2012 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Nano1X2Series.h"

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
    /* To check if PB.5 interrupt occurred */
    if (PB->ISRC & BIT5)
    {
        PB->ISRC = BIT5;
        PD0 = PD0 ^ 1;
        printf("PB.5 INT occurred. \n");

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


/**
 * @brief       PortD/PortE/PortF IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PortD/PortE/PortF default IRQ, declared in startup_nano1x2series.s.
 */
void GPDEF_IRQHandler(void)
{
	uint32_t reg;
    /* To check if PE.2 interrupt occurred */
    if (PE->ISRC & BIT2)
    {
        PE->ISRC = BIT2;
        PD0 = PD0 ^ 1;
        printf("PE.2 INT occurred. \n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PORTD, PORTE and PORTF interrupts */
        reg = PD->ISRC;
        PD->ISRC = reg;
        reg = PE->ISRC;
        PE->ISRC = reg;
        reg = PF->ISRC;
        PF->ISRC = reg;
        printf("Un-expected interrupts. \n");
    }
}


/**
 * @brief       External INT0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT0(PA.2) default IRQ, declared in startup_nano1x2series.s.
 */
void EINT0_IRQHandler(void)
{
    /* For PA.2, clear the INT flag */
    PA->ISRC = BIT2;
    PD0 = PD0 ^ 1;
    printf("PA.2 EINT0 occurred. \n");
}


/**
 * @brief       External INT1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT1(PF.2) default IRQ, declared in startup_Nano1x2series.s.
 */
void EINT1_IRQHandler(void)
{
    /* For PF.2, clear the INT flag */
    PF->ISRC = BIT2;
    PD0 = PD0 ^ 1;
    printf("PF.2 EINT1 occurred. \n");
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
//    SYS->PB_H_MFP &= ~(SYS_PB_H_MFP_PB13_MFP_Msk | SYS_PB_H_MFP_PB14_MFP_Msk);
//    SYS->PB_H_MFP |= (SYS_PB_H_MFP_PB13_MFP_UART0_RX | SYS_PB_H_MFP_PB14_MFP_UART0_TX);
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_TX | SYS_PB_L_MFP_PB1_MFP_UART0_RX);
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
int main (void)
{
    int32_t i32Err;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+--------------------------------------+ \n");
    printf("|    NANO1x2 GPIO Driver Sample Code   | \n");
    printf("+--------------------------------------+ \n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin                              */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("  >> Please connect PB.0 and PD.4 first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();

    /* Configure PB.0 as Output mode and PD.4 as Input mode then close it */
    GPIO_SetMode(PB, BIT0, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PD, BIT4, GPIO_PMD_INPUT);

    i32Err = 0;
    printf("  GPIO Output/Input test ...... \n");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    PB0 = 0;
    CLK_SysTickDelay(10);   /* wait for IO stable */
    if (PD4 != 0)
    {
        i32Err = 1;
    }

    PB0 = 1;
    CLK_SysTickDelay(10);   /* wait for IO stable */
    if (PD4 != 1)
    {
        i32Err = 1;
    }

    if ( i32Err )
    {
        printf("  [FAIL] --- Please make sure PB.0 and PD.4 are connected. \n");
    }
    else
    {
        printf("  [OK] \n");
    }

    /* Configure PB.0 and PD.4 to default Input mode */
    GPIO_SetMode(PB, BIT0, GPIO_PMD_INPUT);
    GPIO_SetMode(PD, BIT4, GPIO_PMD_INPUT);


    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("\n  PB5, PE2, PA2(INT0) and PF2(INT1) are used to test interrupt\n  and control LED(PD0)\n");

    /*Configure PD0 for LED control */
    GPIO_SetMode(PD, BIT0, GPIO_PMD_OUTPUT);

    /* Configure PB5 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PB, BIT5, GPIO_PMD_INPUT);
    GPIO_EnableInt(PB, 5, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPABC_IRQn);


    /*  Configure PE2 as Input mode pull-up and enable interrupt by falling edge trigger */
    GPIO_SetMode(PE, BIT2, GPIO_PMD_INPUT);
    GPIO_ENABLE_PULL_UP(PE, BIT2);
    GPIO_EnableInt(PE, 2, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPDEF_IRQn);

    /* Configure PA2 as EINT0 pin and enable interrupt by falling edge trigger */
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~0xf00) | 0x100;
    GPIO_SetMode(PA, BIT2, GPIO_PMD_INPUT);
    GPIO_EnableEINT0(PA, 2, GPIO_INT_FALLING);
    NVIC_EnableIRQ(EINT0_IRQn);

    /* Configure PF2 as EINT1 pin and enable interrupt by rising and falling edge trigger */
    SYS->PF_L_MFP = (SYS->PF_L_MFP & ~0xf00) | 0x100;
    GPIO_SetMode(PF, BIT2, GPIO_PMD_INPUT);
    GPIO_EnableEINT1(PF, 2, GPIO_INT_BOTH_EDGE);
    NVIC_EnableIRQ(EINT1_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time */
    GPIO_SET_DEBOUNCE_TIME(GPIO_DBCLKSRC_HCLK, GPIO_DBCLKSEL_1);
    GPIO_ENABLE_DEBOUNCE(PB, BIT5);
    GPIO_ENABLE_DEBOUNCE(PE, BIT2);
    GPIO_ENABLE_DEBOUNCE(PA, BIT2);
    GPIO_ENABLE_DEBOUNCE(PF, BIT2);

    /* Waiting for interrupts */
    while (1);

}


