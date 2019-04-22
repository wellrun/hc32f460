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
 ** \brief This example demonstrates how to use read protection of SMPU2
 **        region2 1KByte range.
 **
 **   - 2018-10-20  1.0  Hongjh First version for Device Driver Library of MPU
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
/* MPU */
#define MPU_TYPE                        SMPU2Region
#define MPU_RIGION_NUM                  MpuRegionNum2
#define MPU_RIGION_SIZE                 MpuRegionSize1KByte

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

/* DMAC */
#define DMA_UNIT                        M4_DMA2
#define DMA_CH                          DmaCh0
#define DMA_TRG_SEL                     EVT_AOS_STRG
#define DMA_TRNCNT                      (2u)
#define DMA_BLKSIZE                     (1024)
#define DMA_RPTSIZE                     DMA_BLKSIZE

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void LedInit(void);
static void DmaInit(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
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
    stcDmaInit.u16BlockSize = (DMA_BLKSIZE == 1024) ? 0: DMA_BLKSIZE;// 0 == 1024Byte
    stcDmaInit.u16TransferCnt = DMA_TRNCNT;   /* Set transfer count. */
    stcDmaInit.u32SrcAddr = (uint32_t)(&m_au8SrcBuf[0]);  /* Set source address. */
    stcDmaInit.u32DesAddr = (uint32_t)(&m_au8DstBuf[0]);  /* Set destination address. */
    stcDmaInit.u16SrcRptSize = (DMA_RPTSIZE == 1024) ? 0: DMA_RPTSIZE;  /* Set repeat size. */
    stcDmaInit.u16DesRptSize = (DMA_RPTSIZE == 1024) ? 0: DMA_RPTSIZE;  /* Set repeat size. */
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
    for (uint16_t i = 0; i < DMA_BLKSIZE; i++)
    {
        m_au8SrcBuf[i] = i;
        m_au8DstBuf[i] = 0;
    }
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
    en_result_t enTestResult = Ok;
    stc_mpu_prot_region_init_t stcProtRegionInit;

    /* Disable SMPU region */
    MPU_WriteProtCmd(Disable);
    MPU_RegionTypeCmd(MPU_TYPE, Disable);

    /* Initialize buffer && LED && DMA */
    DmaBufferInit();
    LedInit();
    DmaInit();

    /* Triggger DMA */
    AOS_SW_Trigger();

    while (Reset == DMA_GetIrqFlag(DMA_UNIT, DMA_CH, BlkTrnCpltIrq)) /* Wait DMA block transfer complete */
    {
    }

    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, BlkTrnCpltIrq);

    if (0 != memcmp(m_au8SrcBuf, m_au8DstBuf, sizeof(m_au8DstBuf))) /* Verify DMA function */
    {
        enTestResult = Error;
    }
    else
    {
    }

    /* Initialize buffer */
    DmaBufferInit();

    /* Initialize SMPU */
    MEM_ZERO_STRUCT(stcProtRegionInit);
    stcProtRegionInit.u32RegionBaseAddress = (uint32_t)m_au8SrcBuf;
    stcProtRegionInit.enRegionSize = MPU_RIGION_SIZE;
    stcProtRegionInit.stcSMPU2Permission.enRegionEnable = Enable;
    stcProtRegionInit.stcSMPU2Permission.enWriteEnable = Enable;
    stcProtRegionInit.stcSMPU2Permission.enReadEnable = Disable;
    if (Ok != MPU_ProtRegionInit(MPU_RIGION_NUM, &stcProtRegionInit))
    {
        enTestResult = Error;
    }
    else
    {
    }

    /* Enable SMPU region */
    MPU_RegionTypeCmd(MPU_TYPE, Enable);

    /* Triggger DMA */
    AOS_SW_Trigger();

    /* Wait DMA transfer complete */
    while ((Set != DMA_GetIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq)) ||
           (Set != DMA_GetIrqFlag(DMA_UNIT, DMA_CH, BlkTrnCpltIrq)))
    {
    }

    /* DMA source buffer is protected(forbid read) by MPU, so DMA can't read source buffer */
    if (0 == memcmp(m_au8SrcBuf, m_au8DstBuf, sizeof(m_au8DstBuf)))
    {
        enTestResult = Error;
    }
    else
    {
    }

    /* Clear DMA flag */
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq);
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, BlkTrnCpltIrq);
    MPU_ClearStatus(MPU_TYPE);

    if (Ok == enTestResult)
    {
        LED1_ON();  /* Test pass && meet the expected */
    }
    else
    {
        LED0_ON();  /* Test fail && don't meet the expected */
    }

    while (1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
