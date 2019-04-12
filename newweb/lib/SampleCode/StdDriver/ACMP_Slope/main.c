/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 13 $
 * $Date: 14/11/27 7:10p $
 * @brief    Use ACMP slop mode to measure capacitor discharge time.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Nano1X2Series.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
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

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART_S_HXT,CLK_UART_CLK_DIVIDER(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->PB_L_MFP &= ~( SYS_PB_L_MFP_PB0_MFP_Msk | SYS_PB_L_MFP_PB1_MFP_Msk);
    SYS->PB_L_MFP |= (SYS_PB_L_MFP_PB0_MFP_UART0_TX | SYS_PB_L_MFP_PB1_MFP_UART0_RX );

    /* Set PB multi-function pins for Clock Output */
    SYS->PB_H_MFP = ( SYS->PB_H_MFP & ~SYS_PB_H_MFP_PB12_MFP_Msk ) |  SYS_PB_H_MFP_PB12_MFP_CKO0;

    /* Set PA multi-function pins for ACMP */
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA6_MFP_Msk ) | SYS_PA_L_MFP_PA6_MFP_ACMP0_O;  /* ACMP CPO0  */
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA5_MFP_Msk ) | SYS_PA_L_MFP_PA5_MFP_ACMP0_N;  /* ACMP CPN0  */
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA4_MFP_Msk ) | SYS_PA_L_MFP_PA4_MFP_ACMP0_P;  /* ACMP CPP0  */
    SYS->PA_H_MFP = (SYS->PA_H_MFP & ~SYS_PA_H_MFP_PA14_MFP_Msk) | SYS_PA_H_MFP_PA14_MFP_ACMP0_C; /* ACMP charge pin  */
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~SYS_PA_L_MFP_PA3_MFP_Msk ) | SYS_PA_L_MFP_PA3_MFP_ACMP0_C;  /* ACMP charge pin  */


    /* Lock protected registers */
    SYS_LockReg();
}


void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

volatile int32_t tmr0_complete=0;
volatile int32_t tmr0_cap_complete=0;
void TMR0_IRQHandler(void)
{
    int status;
    status=TIMER0->ISR ;
    if(status & 0x1) tmr0_complete=1;
    if(status & 0x2) tmr0_cap_complete=1;
    TIMER0->ISR = status;
}

#define PRESCALE 1
#define CHARGE_NUM 2
int32_t main (void)
{
    uint32_t i,j;
    float val[CHARGE_NUM];
    uint32_t tcap[CHARGE_NUM];
    uint32_t charge[CHARGE_NUM]= {ACMP_MODCR0_CH_DIS_PINSEL_PA14,ACMP_MODCR0_CH_DIS_PINSEL_PA3};

    SYS_Init();

    UART0_Init();

    //ACMP CPO0 : PA6
    //ACMP CPN0 : PA5
    //ACMP CPP0 : PA4
    //Charge/Discharge pin : PA14 / PA3

    CLK_EnableModuleClock(ACMP_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR0_MODULE,CLK_CLKSEL1_TMR0_S_HXT,CLK_TMR0_CLK_DIVIDER(1));
    CLK_SetModuleClock(TMR1_MODULE,CLK_CLKSEL1_TMR1_S_HXT,CLK_TMR1_CLK_DIVIDER(1));

    TIMER0->PRECNT = PRESCALE;        // Set timer 0 pre-scale to 1, optional
    TIMER0->CMPR   = 0x00FFFFFF;      // Set timer 0 comparator value to maximum
    TIMER0->IER    = 0x00000002;      // Set timer 0 interrupt enable
    TIMER0->ECTL   = 0x00010000;      // Enable timer 0 event generator function, choose event generator reference source from ACMP and enable check for timer counter equals to half of comparator
    TIMER0->CTL    = 0x001D0001;      // Enable timer 0 ACMP enable timer function

    NVIC_EnableIRQ(TMR0_IRQn);
    NVIC_EnableIRQ(TMR1_IRQn);
    ACMP_Open(ACMP,0,ACMP_CR_CN_CRV|1,ACMP_CR_ACMP_HYSTERSIS_DISABLE);
    ACMP_SetSlopeConv(ACMP_TIMER01,ACMP_MODCR0_TMR_TRI_LV_FALLING,ACMP_CR_CPP0SEL_PA4,ACMP_MODCR0_CH_DIS_PINSEL_PA14);

    while(1)
    {
        for(j=0; j<CHARGE_NUM; j++)
        {
            ACMP_SetSlopeConv(ACMP_TIMER01,ACMP_MODCR0_TMR_TRI_LV_FALLING,ACMP_CR_CPP0SEL_PA4,charge[j]);
            ACMP_ENABLE(ACMP,0);
            for(i=0; i<0x50000; i++);
            ACMP_START_CONV(ACMP);
            while(!tmr0_cap_complete);
            tmr0_cap_complete=0;
            ACMP_DISABLE(ACMP,0);
            tcap[j]=TIMER0->TCAP;
            val[j]=(double)(tcap[j]*83*(PRESCALE+1))/1000;  /* timer src 12Mhz,1 clock is 83ns */
        }
        for(j=0; j<CHARGE_NUM; j++)
            printf("[%d]TCAP=%4d,time=%4.2f(us) ",j,tcap[j],val[j]);
        printf("\n");
    }
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
