/**************************************************************************//**
 * @file     main.c
 * @version  V2.10
 * $Date: 15/06/11 6:12p $
 * @brief    Calculate the CRC-CCITT checksum value by CRC DMA mode.
 *
 * @note
 * Copyright (C) 2013~2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "Nano1X2Series.h"

uint8_t volatile g_u8IsTargetAbortINTFlag = 0, g_u8IsBlockTransferDoneINTFlag = 0;

/**
 * @brief       DMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The DMA default IRQ, declared in startup_nano1x2series.s.
 */
void PDMA_IRQHandler(void)
{
    uint32_t status = CRC_GET_INT_FLAG();
    if (status & DMA_CRC_DMAISR_BLKD_IF_Msk)
    {
        /* Clear Block Transfer Done Interrupt Flag */
        CRC_CLR_INT_FLAG(DMA_CRC_DMAISR_BLKD_IF_Msk);

        g_u8IsBlockTransferDoneINTFlag++;
    }
    else if (status & DMA_CRC_DMAISR_TABORT_IF_Msk)
    {
        /* Clear Target Abort Interrupt Flag */
        CRC_CLR_INT_FLAG(DMA_CRC_DMAISR_TABORT_IF_Msk);

        g_u8IsTargetAbortINTFlag++;
    }
    else
    {
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
    CLK->AHBCLK |= CLK_AHBCLK_DMA_EN;
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
    //SYS->PB_H_MFP &= ~(SYS_PB_H_MFP_PB13_MFP_Msk | SYS_PB_H_MFP_PB14_MFP_Msk);
    //SYS->PB_H_MFP |= (SYS_PB_H_MFP_PB13_MFP_UART0_RX | SYS_PB_H_MFP_PB14_MFP_UART0_TX);
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
/*  CRC-CCITT Polynomial Mode Test                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void CRC_CCITTPolyModeTest(uint32_t u32SrcAddr, uint32_t u32TransByteCount)
{
    uint32_t u32TargetChecksum = 0x29B1, u32CalChecksum = 0;

    printf("# Calculate string \"123456789\" CRC-CCITT checksum value by CRC DMA mode. \n");
    printf("    - Seed value is 0xFFFF          \n");
    printf("    - Checksum Complement disable   \n");
    printf("    - Checksum Reverse disable      \n");
    printf("    - Write Data Complement disable \n");
    printf("    - Write Data Reverse disable    \n");
    printf("    - Checksum should be 0x%X       \n", u32TargetChecksum);
    printf("... \n\n");

    g_u8IsTargetAbortINTFlag = g_u8IsBlockTransferDoneINTFlag = 0;

    /* Enable CRC channel clock */
    /* Configure CRC Operation Settings for CRC DMA mode */
    CRC_Open(CRC_CCITT, 0, 0xFFFF, 0);

    /* Enable DMA Target Abort and Block Transfer Done Interrupt */
    CRC_ENABLE_INT(DMA_CRC_DMAIER_TABORT_IE_Msk|DMA_CRC_DMAIER_BLKD_IE_Msk);

    /* Enable PDMA and CRC NVIC */
    NVIC_EnableIRQ(PDMA_IRQn);

    /* Trigger CRC DMA transfer */
    CRC_StartDMATransfer(u32SrcAddr, u32TransByteCount);

    /* Wait CRC Interrupt Flag occurred */
    while (1)
    {
        if (g_u8IsTargetAbortINTFlag == 1)
        {
            printf("DMA Target Abort Interrupt occurred. \n");
            break;
        }
        if (g_u8IsBlockTransferDoneINTFlag == 1)
        {
            break;
        }
    }

    /* Disable PDMA and CRC NVIC */
    NVIC_DisableIRQ(PDMA_IRQn);

    /* Get CRC Checksum value */
    u32CalChecksum = CRC_GetChecksum();
    if (g_u8IsBlockTransferDoneINTFlag == 1)
    {
        printf("CRC checksum is 0x%X ... %s.\n", u32CalChecksum, (u32CalChecksum==u32TargetChecksum)?"PASS":"FAIL");
    }

    printf("\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  CRC-8 Polynomial Mode Test                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CRC_CRC8PolyModeTest(uint32_t u32SrcAddr, uint32_t u32TransByteCount)
{
    uint32_t i = 0;
    uint32_t u32TargetChecksum = 0x58, u32CalChecksum = 0;
    uint8_t *p8SrcAddr;

    printf("# Calculate string \"123456789\" CRC-8 checksum value by CRC CPU mode. \n");
    printf("    - Seed value is 0x5A            \n");
    printf("    - CPU Write Length is 8-bit     \n");
    printf("    - Checksum Complement disable   \n");
    printf("    - Checksum Reverse disable      \n");
    printf("    - Write Data Complement disable \n");
    printf("    - Write Data Reverse disable    \n");
    printf("    - Checksum should be 0x%X       \n", u32TargetChecksum);
    printf("... \n\n");

    p8SrcAddr = (uint8_t *)u32SrcAddr;

    /* Enable CRC channel clock */
    /* Configure CRC Operation Settings for CRC DMA mode */
    CRC_Open(CRC_8, 0, 0x5A, CRC_CPU_WDATA_8);

    for (i=0; i<u32TransByteCount; i++)
    {
        CRC_WRITE_DATA((p8SrcAddr[i]&0xFF));
    }

    /* Get CRC Checksum value */
    u32CalChecksum = CRC_GetChecksum();
    printf("CRC checksum is 0x%X ... %s.\n", u32CalChecksum, (u32CalChecksum==u32TargetChecksum)?"PASS":"FAIL");

    printf("\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main (void)
{
    const uint8_t acCRCSrcPattern[] = "123456789";

    /* Init System, IP clock and multi-function I/O */
    SYS_Init(); //In the end of SYS_Init() will issue SYS_LockReg() to lock protected register. If user want to write protected register, please issue SYS_UnlockReg() to unlock protected register.

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz \n", SystemCoreClock);

    printf("+--------------------------------------+ \n");
    printf("|    NANO1x2 CRC Driver Sample Code    | \n");
    printf("+--------------------------------------+ \n");

    printf(" CRC-CCITT Polynomial mode test \n");
    CRC_CCITTPolyModeTest((uint32_t )acCRCSrcPattern, strlen((char *)acCRCSrcPattern));

    printf(" CRC-8 Polynomial mode test \n");
    CRC_CRC8PolyModeTest((uint32_t )acCRCSrcPattern, strlen((char *)acCRCSrcPattern));

    printf("\nExit CRC Sample Code. \n");
    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
