/*******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co., Ltd. All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co., Ltd. ("HDSC").
 *
 * BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
 * BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
 *
 * This software contains source code for use with HDSC
 * components. This software is licensed by HDSC to be adapted only
 * for use in systems utilizing HDSC components. HDSC shall not be
 * responsible for misuse or illegal use of this software for devices not
 * supported herein. HDSC is providing this software "AS IS" and will
 * not be responsible for issues arising from incorrect user implementation
 * of the software.
 *
 * Disclaimer:
 * HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
 * REGARDING THE SOFTWARE (INCLUDING ANY ACCOMPANYING WRITTEN MATERIALS),
 * ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
 * WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
 * WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
 * WARRANTY OF NONINFRINGEMENT.
 * HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
 * NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
 * LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
 * LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
 * INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
 * SAVINGS OR PROFITS,
 * EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
 * INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
 * FROM, THE SOFTWARE.
 *
 * This software may be replicated in part or whole for the licensed use,
 * with the restriction that this Disclaimer and Copyright notice must be
 * included with each copy of this software, whether used in part or whole,
 * at all times.
 */
/******************************************************************************/
/** \file main.c
 **
 ** \brief efm sample
 **
 **   - 2018-11-01  1.0  chengy First version for Device Driver Library of
 **     efm
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define EFM_IRQn                        Int129_IRQn

/* LED0 Port/Pin definition */
#define  LED0_PORT                      PortE
#define  LED0_PIN                       Pin06
/* LED0~1 definition */
#define LED0_ON()                       PORT_SetBits(LED0_PORT, LED0_PIN)
#define LED0_OFF()                      PORT_ResetBits(LED0_PORT, LED0_PIN)

#define FLASH_SECTOR62_ADRR             0x0007C000

#define FLASH_WIN_START_ADDR            0x0007D000
#define FLASH_WIN_END_ADDR              0x0007E000

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
void EfmPageEraseErr_IrqHandler(void)
{
    LED0_ON();

    EFM_Unlock();
    EFM_ClearFlag(EFM_FLAG_PEPRTERR);
    EFM_Lock();
}
/**
 *******************************************************************************
 ** \brief  Led init.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Led_Init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;

    LED0_OFF();

    /* LED0 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief  Main function of template project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    uint32_t u32Addr;
    stc_efm_win_protect_addr_t stcWinAddr;
    const uint32_t u32TestData = 0x5A5A5A5A;

    /* Init Led. */
    Led_Init();

    /* Unlock EFM. */
    EFM_Unlock();

    /* Enable flash. */
    EFM_FlashCmd(Enable);
    /* Wait flash ready. */
    while(Set != EFM_GetFlagStatus(EFM_FLAG_RDY));

    /* Erase sector 62. */
    EFM_SectorErase(FLASH_SECTOR62_ADRR);

    /* Set windows protect address. */
    stcWinAddr.StartAddr = FLASH_WIN_START_ADDR;
    stcWinAddr.EndAddr = FLASH_WIN_END_ADDR;
    EFM_SetWinProtectAddr(stcWinAddr);

    /* Enable program & erase err interrupt. */
    EFM_InterruptCmd(PgmErsErrInt, Enable);

    /* Set EFM_PEERR interrupt. */
    enShareIrqEnable(INT_EFM_PEERR);

    /* Enable interrupt. */
    NVIC_ClearPendingIRQ(EFM_IRQn);
    NVIC_SetPriority(EFM_IRQn,DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(EFM_IRQn);

    /* program between windows address. */
    u32Addr = FLASH_WIN_START_ADDR + 4;
    EFM_SingleProgram(u32Addr,u32TestData);

    /* SW2 */
    while(0 != PORT_GetBit(PortD, Pin03));

    /* program out of windows address. */
    u32Addr = FLASH_WIN_START_ADDR - 4;
    EFM_SingleProgram(u32Addr,u32TestData);

    EFM_ClearFlag(EFM_FLAG_PEPRTERR);

    /* Lock EFM. */
    EFM_Lock();

    while(1);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
