/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 4 $
 * $Date: 14/11/27 7:10p $
 * @brief    Demonstrate how to wake up system form Power-down mode by I2C interrupt.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Nano1X2Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void PDWU_IRQHandler(void)
{
    /* Clear I2C wake up flag */
    I2C_CLEAR_WAKEUP_FLAG(I2C0);

    /* Clear wake up flag */
    CLK->WK_INTSTS = 1;
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
    CLK_SetModuleClock(I2C0_MODULE, 0, 0);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD  */
    SYS->PB_L_MFP &= ~(SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_TX | SYS_PB_L_MFP_PB1_MFP_UART0_RX);

    /* Set multi function pin for I2C0 */
    SYS->PC_L_MFP = (SYS_PC_L_MFP_PC0_MFP_I2C0_SCL | SYS_PC_L_MFP_PC1_MFP_I2C0_SDA);

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Init Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C0 Slave Addresses */
    I2C_SetSlaveAddr(I2C0, 0, 0x25, I2C_GCMODE_DISABLE);   /* Set Slave Address */

    NVIC_EnableIRQ(PDWU_IRQn);

    /* Enable I2C0 wakeup function */
    I2C_EnableWakeup(I2C0);

    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{
    volatile uint8_t u8RxData[3];
    volatile uint8_t u8ReadWrite;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+-------------------------------------------------------+\n");
    printf("|        Nano1x2 Series I2C Wake Up Sample Code         |\n");
    printf("+-------------------------------------------------------+\n");
    printf("Note: Master need to send the I2C signal likes the following :\n");
    printf("    S | SLA+W | DATA | DATA | DATA | P \n");
    printf(" or S | SLA+R | DATA | DATA | DATA | P \n\n");

    /* Init I2C0 as Slave */
    I2C0_Init();

    printf("Enter power down mode !!!\n");
    // Wait 'til UART FIFO empty to get a cleaner console out
    while(!UART_IS_TX_EMPTY(UART0));

    /* Enter power down mode and enable wake up interrupt */
    SYS_UnlockReg();
    CLK->PWRCTL |= CLK_PWRCTL_PD_WK_IE_Msk;
    CLK_PowerDown();

    /* System wakes up, SCK pin will be held down until software clears wake up done flag */
    /* Need to make sure I2C sla is done(after the 9th bit) */
    while(!I2C_GET_WAKEUP_ACK_DONE_FLAG(I2C0));

    /* Remember master wants to read or write data */
    u8ReadWrite = I2C_GET_WAKEUP_RW_FLAG(I2C0);

    /* If master wants to read, we need to prepare Tx output data here before release SCK pin */
    if(u8ReadWrite)
        I2C_SET_DATA(I2C0, 0x11);

    /* I2C can release SCK pin and enter I2C normal operation */
    I2C_CLEAR_WAKEUP_ACK_DONE_FLAG(I2C0);

    if(u8ReadWrite)     // master wants to read
    {
        /* 1st data */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA);

        /* 2nd data */
        I2C_WAIT_READY(I2C0);                       // wait if interrupt happens
        I2C_SET_DATA(I2C0, 0x55);                   // prepare tx data
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA); // trigger it

        /* 3rd data */
        I2C_WAIT_READY(I2C0);                       // wait if interrupt happens
        I2C_SET_DATA(I2C0, 0xAA);                   // prepare tx data
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA); // trigger it

        /* stop */
        I2C_WAIT_READY(I2C0);                       // wait if interrupt happens
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_STO);// trigger it
        while(I2C0->CON & I2C_CON_STOP_Msk);        // wait until STOP happens
    }
    else            // master wants to write
    {
        /* 1st data */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA); // trigger it
        I2C_WAIT_READY(I2C0);                       // wait if interrupt happens
        u8RxData[0] = I2C_GET_DATA(I2C0);           // receive data

        /* 2nd data */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA); // trigger it
        I2C_WAIT_READY(I2C0);                       // wait if interrupt happens
        u8RxData[1] = I2C_GET_DATA(I2C0);           // receive data

        /* 3rd data */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA); // trigger it
        I2C_WAIT_READY(I2C0);                       // wait if interrupt happens
        u8RxData[2] = I2C_GET_DATA(I2C0);           // receive data

        /* stop */
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_STO);// trigger it
        while(I2C0->CON & I2C_CON_STOP_Msk);        // wait until STOP happens
    }

    printf("Wake up !!\n");

    while(1);
}
