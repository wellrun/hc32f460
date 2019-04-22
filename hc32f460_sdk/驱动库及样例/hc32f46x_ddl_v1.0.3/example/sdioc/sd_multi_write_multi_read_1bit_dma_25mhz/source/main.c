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
 ** \brief This example demonstrates how to use SDIOC to read/write SDCard by
 **        normal-speed(25MHz) && 1 bit && multiple blocks mode && DMA.
 **
 **   - 2018-11-08  1.0  Hongjh First version for Device Driver Library of SDIOC
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "sd_card.h"

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* LED0(D23: red color) Port/Pin definition */
#define LED0_PORT                       PortE
#define LED0_PIN                        Pin06

/* LED1(D26: green color) Port/Pin definition */
#define LED1_PORT                       PortA
#define LED1_PIN                        Pin07

/* LED0 & LED1 */
#define LED0_ON()                       PORT_SetBits(LED0_PORT, LED0_PIN)
#define LED0_OFF()                      PORT_ResetBits(LED0_PORT, LED0_PIN)
#define LED1_ON()                       PORT_SetBits(LED1_PORT, LED1_PIN)
#define LED1_OFF()                      PORT_ResetBits(LED1_PORT, LED1_PIN)

/* SDIOC Port/Pin definition */
#define SDIOC_CD_PORT                   PortE
#define SDIOC_CD_PIN                    Pin14

#define SDIOC_CK_PORT                   PortC
#define SDIOC_CK_PIN                    Pin12

#define SDIOC_CMD_PORT                  PortD
#define SDIOC_CMD_PIN                   Pin02

#define SDIOC_D0_PORT                   PortC
#define SDIOC_D0_PIN                    Pin08

#define SDIOC_D1_PORT                   PortC
#define SDIOC_D1_PIN                    Pin09

#define SDIOC_D2_PORT                   PortC
#define SDIOC_D2_PIN                    Pin10

#define SDIOC_D3_PORT                   PortC
#define SDIOC_D3_PIN                    Pin11

/* SD sector && count */
#define SD_SECTOR_START                 (0u)
#define SD_SECTOR_COUNT                 (4u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void LedInit(void);
static void ClkInit(void);
static en_result_t SdiocInitPins(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static stc_sdcard_init_t m_stcCardInitCfg =
{
    SdiocBusWidth1Bit,
    SdiocClk25M,
    SdiocNormalSpeedMode,
};

static stc_sdcard_dma_init_t m_stcDmaInitCfg =
{
    M4_DMA1,
    DmaCh0,
};

static stc_sd_handle_t m_stcSdhandle =
{
    M4_SDIOC1,
    SdCardDmaMode,
    &m_stcDmaInitCfg,
};

static uint32_t m_u32WriteBlocks[512];
static uint32_t m_u32ReadBlocks[ARRAY_SZ(m_u32WriteBlocks)];

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Initialize LED.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void LedInit(void)
{
    stc_port_init_t stcPortInit;

    LED0_OFF();
    LED1_OFF();

    /* LED0&LED1 Port/Pin initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief Initialize Clock.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void ClkInit(void)
{
    stc_clk_xtal_cfg_t   stcXtalCfg;
    stc_clk_mpll_cfg_t   stcMpllCfg;
    en_clk_sys_source_t  enSysClkSrc;
    stc_clk_sysclk_cfg_t stcSysClkCfg;

    MEM_ZERO_STRUCT(enSysClkSrc);
    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv  = ClkSysclkDiv1;  /* Max 168MHz */
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;  /* Max 84MHz */
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;  /* Max 168MHz */
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;  /* Max 84MHz */
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;  /* Max 60MHz */
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;  /* Max 42MHz */
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;  /* Max 84MHz */
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Switch system clock source to MPLL. */
    /* Use Xtal as MPLL source. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* MPLL config. */
    stcMpllCfg.pllmDiv = 1;
    stcMpllCfg.plln = 50;
    stcMpllCfg.PllpDiv = 4;
    stcMpllCfg.PllqDiv = 4;
    stcMpllCfg.PllrDiv = 4;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_5);
    EFM_Lock();

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while (Set != CLK_GetFlagStatus(ClkFlagMPLLRdy))
    {
    }

    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);
}

/**
 ******************************************************************************
 ** \brief Initialize SDIO pins
 **
 ** \param [in] None
 **
 ** \retval Ok  SDIO pins initialized successfully
 **
 ******************************************************************************/
static en_result_t SdiocInitPins(void)
{
    PORT_SetFunc(SDIOC_D0_PORT, SDIOC_D0_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D1_PORT, SDIOC_D1_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D2_PORT, SDIOC_D2_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_D3_PORT, SDIOC_D3_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CD_PORT, SDIOC_CD_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CK_PORT, SDIOC_CK_PIN, Func_Sdio, Disable);
    PORT_SetFunc(SDIOC_CMD_PORT, SDIOC_CMD_PIN, Func_Sdio, Disable);

    return Ok;
}

/**
 *******************************************************************************
 ** \brief  Main function of project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    uint32_t i;
    en_result_t enTestResult = Ok;

    /* Write buffer data */
    for (i = 0; i < ARRAY_SZ(m_u32WriteBlocks); i++)
    {
        m_u32WriteBlocks[i] = i;
    }

    MEM_ZERO_STRUCT(m_u32ReadBlocks);

    /* Initialize Clock */
    ClkInit();

    /* Initialize LED */
    LedInit();

    /* Initialize SDIOC pin */
    SdiocInitPins();

    /* Initialize SD card */
    if (Ok != SDCARD_Init(&m_stcSdhandle, &m_stcCardInitCfg))
    {
        enTestResult = Error;
    }
    else
    {
    }

    /* Erase SD card */
    if (Ok != SDCARD_Erase(&m_stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, 20000))
    {
        enTestResult = Error;
    }
    else
    {
    }

    /* Read SD card */
    if (Ok != SDCARD_ReadBlocks(&m_stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, (uint8_t *)m_u32ReadBlocks, 2000))
    {
        enTestResult = Error;
    }
    else
    {
    }

    /* Check whether data value is OxFFFFFFFF or 0x00000000 after erase SD card */
    for (i = 0; i < ARRAY_SZ(m_u32WriteBlocks); i++)
    {
        if ((m_u32ReadBlocks[i] != 0xFFFFFFFFul) &&
            (m_u32ReadBlocks[i] != 0x00000000ul))
        {
            enTestResult = Error;
            break;
        }
        else
        {
        }
    }

    /* Write SD card */
    if (Ok != SDCARD_WriteBlocks(&m_stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, (uint8_t *)m_u32WriteBlocks, 2000))
    {
        enTestResult = Error;
    }
    else
    {
    }

    /* Read SD card */
    if (Ok != SDCARD_ReadBlocks(&m_stcSdhandle, SD_SECTOR_START, SD_SECTOR_COUNT, (uint8_t *)m_u32ReadBlocks, 20000))
    {
        enTestResult = Error;
    }
    else
    {
    }

    /* Compare read/write data */
    if (0 != memcmp(m_u32WriteBlocks, m_u32ReadBlocks, sizeof(m_u32ReadBlocks)))
    {
        enTestResult = Error;
    }
    else
    {
    }

    if (Ok == enTestResult)
    {
        LED1_ON();    /* Test pass && meet the expected */
    }
    else
    {
        LED0_ON();    /* Test fail && don't meet the expected */
    }

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
