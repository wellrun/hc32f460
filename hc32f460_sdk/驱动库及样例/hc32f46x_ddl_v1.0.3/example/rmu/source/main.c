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
 ** \brief ADC sample
 **
 **   - 2018-11-18  1.0  Wuze First version for Device Driver Library of
 **     ADC
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
/* Reset mode definition. */
#define POWER_ON_RESET              (0u)
#define NRST_PIN_RESET              (1u)
#define UNDERVOLTAGE_RESET          (2u)
#define PVD1_RESET                  (3u)
#define PVD2_RESET                  (4u)
#define WDT_RESET                   (5U)
#define SWDT_RESET                  (6u)
#define POWER_DOWN_RESET            (7u)
#define SOFTWARE_RESET              (8u)
#define MPU_ERR_RESET               (9u)
#define RAM_PARITY_ERR_RESET        (10u)
#define RAM_ECC_RESET               (11u)
#define CLK_FREQ_ERR_RESET          (12u)
#define XTAL_ERR_RESET              (13u)
#define NORMAL_RESET               (0xFFu)

#define RESET_MODE                  NORMAL_RESET

/* Indicate led definition. */
#define LED0_PORT                   PortE
#define LED0_PIN                    Pin06
#define LED0_ON()                   PORT_SetBits(LED0_PORT, LED0_PIN)
#define LED0_OFF()                  PORT_ResetBits(LED0_PORT, LED0_PIN)
#define LED0_TOGGLE()               PORT_Toggle(LED0_PORT, LED0_PIN)

#if (RESET_MODE == MPU_ERR_RESET)
#define MPU_TYPE                    SMPU1Region
#define MPU_RIGION_NUM              MpuRegionNum0
#define MPU_RIGION_SIZE             MpuRegionSize32Byte

#define DMA_UNIT                    M4_DMA1
#define DMA_CH                      DmaCh0
#define DMA_TRG_SEL                 EVT_AOS_STRG
#define DMA_TRNCNT                  (2u)
#define DMA_BLKSIZE                 (64u)
#define DMA_RPTSIZE                 DMA_BLKSIZE
#endif

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void LedConfig(void);
static void LedIndicate(void);
static void SetResetMode(void);
static void MakeReset(void);
static void PrintResetMode(stc_rmu_rstcause_t stcRst);

#if (RESET_MODE == MPU_ERR_RESET)
static void DmaInit(void);
static void DmaBufferInit(void);
#endif

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
#if (RESET_MODE == MPU_ERR_RESET)

#if defined (__ICCARM__)                /* IAR Compiler */
#pragma data_alignment=DMA_BLKSIZE
static uint8_t m_au8SrcBuf[DMA_BLKSIZE];

#pragma data_alignment=DMA_BLKSIZE
static uint8_t m_au8DstBuf[DMA_BLKSIZE] = {0};

#elif defined (__CC_ARM)                /* ARM Compiler */
__align(DMA_BLKSIZE)
static uint8_t m_au8SrcBuf[DMA_BLKSIZE];

__align(DMA_BLKSIZE)
static uint8_t m_au8DstBuf[DMA_BLKSIZE] = {0};
#endif

#endif // #if (RESET_MODE == MPU_ERR_RESET)

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief  Main function.
 **
 ** \param  None.
 **
 ** \retval int32_t return value, if needed.
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_rmu_rstcause_t stcResetFlag;
    Ddl_UartInit();
    LedConfig();
    Ddl_Delay1ms(10);

    RMU_GetResetCause(&stcResetFlag);
    RMU_ClrResetFlag();
    PrintResetMode(stcResetFlag);

    SetResetMode();

    Ddl_Delay1ms(100u);
    MakeReset();
    while (1u)
    {
        //: YOUR CODE
    }
}

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief Led configuration.
 **
 ******************************************************************************/
static void LedConfig(void)
{
    stc_port_init_t stcPortInit;
    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);
    LED0_OFF();
}

/**
 *******************************************************************************
 ** \brief Led indicate.
 **
 ******************************************************************************/
static void LedIndicate(void)
{
    LED0_ON();
    Ddl_Delay1ms(500u);
    LED0_OFF();
}

/**
 *******************************************************************************
 ** \brief Set reset mode.
 **
 ******************************************************************************/
static void SetResetMode(void)
{
#if (RESET_MODE == UNDERVOLTAGE_RESET)

#elif (RESET_MODE == PVD1_RESET)
    stc_pwc_pvd_cfg_t stcPvdCfg;
    MEM_ZERO_STRUCT(stcPvdCfg);

    stcPvdCfg.stcPvd1Ctl.enPvdIREn = Enable;
    stcPvdCfg.stcPvd1Ctl.enPvdMode = PvdReset;
    stcPvdCfg.stcPvd1Ctl.enPvdCmpOutEn = Enable;
    stcPvdCfg.enPvd1Level = Pvd1Level21;
    PWC_PvdCfg(&stcPvdCfg);
    PWC_Pvd1Cmd(Enable);

#elif (RESET_MODE == PVD2_RESET)
    stc_pwc_pvd_cfg_t stcPvdCfg;
    MEM_ZERO_STRUCT(stcPvdCfg);

    stcPvdCfg.stcPvd2Ctl.enPvdIREn = Enable;
    stcPvdCfg.stcPvd2Ctl.enPvdMode = PvdReset;
    stcPvdCfg.stcPvd2Ctl.enPvdCmpOutEn = Enable;
    stcPvdCfg.enPvd2Level = Pvd2Level26;
    PWC_PvdCfg(&stcPvdCfg);
    PWC_Pvd2Cmd(Enable);

#elif (RESET_MODE == WDT_RESET)
    stc_wdt_init_t stcWdtInit;
    MEM_ZERO_STRUCT(stcWdtInit);

    stcWdtInit.enClkDiv = WdtPclk3Div512;
    stcWdtInit.enCountCycle = WdtCountCycle16384;
    stcWdtInit.enRefreshRange = WdtRefresh0To25Pct;
    stcWdtInit.enSleepModeCountEn = Disable;
    stcWdtInit.enRequsetType = WdtTriggerResetRequest;
    WDT_Init(&stcWdtInit);
    WDT_RefreshCounter();

#elif (RESET_MODE == SWDT_RESET)

#elif (RESET_MODE == POWER_DOWN_RESET)

#elif (RESET_MODE == SOFTWARE_RESET)

#elif (RESET_MODE == MPU_ERR_RESET)
    stc_mpu_prot_region_init_t stcProtRegionInit;
    /* Disable SMPU region */
    MPU_WriteProtCmd(Disable);
    MPU_RegionTypeCmd(MPU_TYPE, Disable);
    /* Initialize SMPU */
    MEM_ZERO_STRUCT(stcProtRegionInit);
    stcProtRegionInit.u32RegionBaseAddress = (uint32_t)(&m_au8DstBuf[0]);
    stcProtRegionInit.enRegionSize = MPU_RIGION_SIZE;
    stcProtRegionInit.stcSMPU1Permission.enRegionEnable = Enable;
    stcProtRegionInit.stcSMPU1Permission.enWriteEnable = Disable;
    stcProtRegionInit.stcSMPU1Permission.enReadEnable = Enable;
    MPU_ProtRegionInit(MPU_RIGION_NUM, &stcProtRegionInit);
    MPU_SetNoPermissionAcessAction(MPU_TYPE, MpuTrigReset);
    MPU_RegionTypeCmd(MPU_TYPE, Enable);

    DmaBufferInit();
    DmaInit();

#elif (RESET_MODE == RAM_PARITY_ERR_RESET)
    stc_sram_config_t stcSramConfig;
    MEM_ZERO_STRUCT(stcSramConfig);

    stcSramConfig.enSramEccMode = EccMode3;
    stcSramConfig.enSramPyOp    = SramReset;
    SRAM_Init(&stcSramConfig);
    SRAM_WT_Enable();
    SRAM_CK_Enable();

#elif (RESET_MODE == RAM_ECC_RESET)
    stc_sram_config_t stcSramConfig;
    MEM_ZERO_STRUCT(stcSramConfig);

    stcSramConfig.enSramEccMode = EccMode3;
    stcSramConfig.enSramEccOp   = SramReset;
    SRAM_Init(&stcSramConfig);
    SRAM_WT_Enable();
    SRAM_CK_Enable();

#elif (RESET_MODE == CLK_FREQ_ERR_RESET)

#elif (RESET_MODE == XTAL_ERR_RESET)
    stc_clk_xtal_cfg_t stcXtalCfg;
    stc_clk_xtal_stp_cfg_t stcXtalStpCfg;
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcXtalStpCfg);

    stcXtalStpCfg.enDetect = Disable;
    stcXtalStpCfg.enMode = ClkXtalStpModeReset;
    stcXtalStpCfg.enModeReset = Enable;
    stcXtalStpCfg.enModeInt = Disable;
    CLK_XtalStpConfig(&stcXtalStpCfg);

    stcXtalCfg.enFastStartup = Enable;
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    CLK_XtalConfig(&stcXtalCfg);

    CLK_XtalCmd(Enable);
    Ddl_Delay1ms(100u);

    stcXtalStpCfg.enDetect = Enable;
    CLK_XtalStpConfig(&stcXtalStpCfg);

#else
#endif
}

/**
 *******************************************************************************
 ** \brief  Make an reset.
 **
 ******************************************************************************/
static void MakeReset(void)
{
#if (RESET_MODE == UNDERVOLTAGE_RESET)

#elif (RESET_MODE == PVD1_RESET)

#elif (RESET_MODE == PVD2_RESET)

#elif (RESET_MODE == WDT_RESET)

#elif (RESET_MODE == SWDT_RESET)

#elif (RESET_MODE == POWER_DOWN_RESET)

#elif (RESET_MODE == SOFTWARE_RESET)
    __NVIC_SystemReset();

#elif (RESET_MODE == MPU_ERR_RESET)
    /* Trigger DMA to write m_au8DstBuf */
    AOS_SW_Trigger();

#elif (RESET_MODE == RAM_PARITY_ERR_RESET)

#elif (RESET_MODE == RAM_ECC_RESET)

#elif (RESET_MODE == CLK_FREQ_ERR_RESET)

#elif (RESET_MODE == XTAL_ERR_RESET)

#else
#endif
}

/**
 *******************************************************************************
 ** \brief  Print reset information.
 **
 ******************************************************************************/
static void PrintResetMode(stc_rmu_rstcause_t stcRst)
{
    if (Set == stcRst.enMultiRst)
    {
        printf("\nMultiple reset");
    }
    if (Set == stcRst.enPowerOn)
    {
        printf("\nPower on reset.");
    }
    if (Set == stcRst.enRstPin)
    {
        LedIndicate();
        printf("\nNRST pin reset.");
    }
    if (Set == stcRst.enBrownOut)
    {
        printf("\nUndervoltage reset.");
    }
    if (Set == stcRst.enPvd1)
    {
        printf("\nPvd1 reset.");
    }
    if (Set == stcRst.enPvd2)
    {
        printf("\nPvd2 reset.");
    }
    if (Set == stcRst.enWdt)
    {
#if (RESET_MODE == WDT_RESET)
        LED0_ON();
        Ddl_Delay1ms(500u);
        LED0_OFF();
#endif
        printf("\nWDT reset.");
    }
    if (Set == stcRst.enSwdt)
    {
        printf("\nSpecial WDT reset.");
    }
    if (Set == stcRst.enPowerDown)
    {
        printf("\nPower down reset.");
    }
    if (Set == stcRst.enSoftware)
    {
        printf("\nSoftware reset.");
    }
    if (Set == stcRst.enMpuErr)
    {
        printf("\nMPU error reset.");
    }
    if (Set == stcRst.enRamParityErr)
    {
        printf("\nRAM parity error reset.");
    }
    if (Set == stcRst.enRamEcc)
    {
        printf("\nRAM ECC reset.");
    }
    if (Set == stcRst.enClkFreqErr)
    {
        printf("\nClock frequence error reset.");
    }
    if (Set == stcRst.enXtalErr)
    {
        printf("\nXTAL error reset.");
    }
}

#if (RESET_MODE == MPU_ERR_RESET)
/**
 *******************************************************************************
 ** \brief Initialize DMA.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void DmaInit(void)
{
    stc_dma_config_t stcDmaInit;

    /* Enable peripheral clock */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA1 | PWC_FCG0_PERIPH_DMA2,Enable);

    /* Enable DMA. */
    DMA_Cmd(DMA_UNIT,Enable);

    /* Initialize DMA. */
    MEM_ZERO_STRUCT(stcDmaInit);
    stcDmaInit.u16BlockSize = DMA_BLKSIZE;    /* Set data block size. */
    stcDmaInit.u16TransferCnt = DMA_TRNCNT;   /* Set transfer count. */
    stcDmaInit.u32SrcAddr = (uint32_t)(&m_au8SrcBuf[0]);  /* Set source address. */
    stcDmaInit.u32DesAddr = (uint32_t)(&m_au8DstBuf[0]);  /* Set destination address. */
    stcDmaInit.u16SrcRptSize = DMA_RPTSIZE;      /* Set repeat size. */
    stcDmaInit.u16DesRptSize = DMA_RPTSIZE;      /* Set repeat size. */
    stcDmaInit.stcDmaChCfg.enSrcRptEn = Enable;  /* Enable repeat. */
    stcDmaInit.stcDmaChCfg.enDesRptEn = Enable;  /* Enable repeat. */
    stcDmaInit.stcDmaChCfg.enSrcInc = AddressIncrease;  /* Set source address mode. */
    stcDmaInit.stcDmaChCfg.enDesInc = AddressIncrease;  /* Set destination address mode. */
    stcDmaInit.stcDmaChCfg.enIntEn = Enable;      /* Enable interrupt. */
    stcDmaInit.stcDmaChCfg.enTrnWidth = Dma8Bit;  /* Set data width 8bit. */
    DMA_InitChannel(DMA_UNIT, DMA_CH, &stcDmaInit);

    /* Enable the specified DMA channel. */
    DMA_ChannelCmd(DMA_UNIT, DMA_CH, Enable);

    /* Clear DMA flag. */
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq);

    /* Enable peripheral circuit trigger function. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS,Enable);

    /* Set DMA trigger source. */
    DMA_SetTriggerSrc(DMA_UNIT, DMA_CH, DMA_TRG_SEL);
}

/**
 *******************************************************************************
 ** \brief Initialize DMA buffer.
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
static void DmaBufferInit(void)
{
    /* Initialize source buffer */
    for(uint16_t i = 0; i < DMA_BLKSIZE; i++)
    {
        m_au8SrcBuf[i] = i;
        m_au8DstBuf[i] = 0;
    }
}
#endif // #if (RESET_MODE == MPU_ERR_RESET)

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
