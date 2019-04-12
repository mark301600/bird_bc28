;/**************************************************************************//**
; * @file     ap_images.s
; * @version  V1.00
; * $Revision: 1 $
; * $Date: 18/06/21 4:41p $
; * @brief    Embedded LDROM_code.bin into fmc_ap_main.bin.
; *
; * @note
; * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/


    AREA _image, DATA, READONLY

    EXPORT  loaderImage1Base
    EXPORT  loaderImage1Limit

    ALIGN   4

loaderImage1Base
    INCBIN ./obj/LDROM_code.bin
loaderImage1Limit

    END