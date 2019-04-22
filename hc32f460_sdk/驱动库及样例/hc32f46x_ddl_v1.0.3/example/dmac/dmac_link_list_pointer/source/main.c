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
 ** \brief dmac sample
 **
 **   - 2018-10-20  1.0  Chengy First version for Device Driver Library of
 **     dmac
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
/* SW4 */
#define KEY1_PORT               PortD
#define KEY1_PIN                Pin04

/* LED */
#define LED0_PORT               PortE
#define LED0_PIN                Pin06
#define LED1_PORT               PortA
#define LED1_PIN                Pin07
#define LED0_ON()               PORT_SetBits(LED0_PORT, LED0_PIN)
#define LED0_OFF()              PORT_ResetBits(LED0_PORT, LED0_PIN)
#define LED1_ON()               PORT_SetBits(LED1_PORT, LED1_PIN)
#define LED1_OFF()              PORT_ResetBits(LED1_PORT, LED1_PIN)

/* DMAC */
#define DMA_UNIT                (M4_DMA1)
#define DMA_CH                  (DmaCh3)
#define DMA_TRG_SEL             (INT_PORT_EIRQ4)  /* External Int Ch.4 trigger request number */
#define DMA_TRNCNT              (1)
#define DMA_BLKSIZE             (ARRAY_SZ(u16SrcBuf))
#define DMA_LLP_MODE            (LlpRunNow)
#define DMA_INC_MODE            (AddressIncrease)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static const uint8_t u8SrcBuf[10] = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
static uint8_t u8DstBuf[10] = {0};

static const uint16_t u16SrcBuf[10] = {21, 22, 23, 24, 25, 26, 27, 28, 29, 30};
static uint16_t u16DstBuf[10] = {0};

static const uint32_t u32SrcBuf[10] = {31, 32, 33, 34, 35, 36, 37, 38, 39, 40};
static uint32_t u32DstBuf[10] = {0};

static stc_dma_llp_descriptor_t stcLlpDesc[2] = {0};
/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief LED initialization function
 **

 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
static void Led_Init(void)
{
    stc_port_init_t stcPortInit;

    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;

    LED0_OFF();
    LED1_OFF();

    /* LED0 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* LED1 Port/Pin initialization */
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);
}
/**
 *******************************************************************************
 ** \brief  SW4 interrupt callback function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt04_Callback(void)
{
}
/**
 *******************************************************************************
 ** \brief KEY1(SW4) init function
 **
 ** \param [in]  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SW4_Init(void)
{
    stc_port_init_t stcPortInit;
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /**************************************************************************/
    /* External Int Ch.4                                                     */
    /**************************************************************************/
    /* Set PD04 as External Int Ch.4 input */
    stcPortInit.enExInt = Enable;
    PORT_Init(KEY1_PORT, KEY1_PIN, &stcPortInit);

    stcExtiConfig.enExitCh = ExtiCh04;
    /* Filter setting */
    stcExtiConfig.enFilterEn = Disable;
    /* Rising edge */
    stcExtiConfig.enExtiLvl = ExIntFallingEdge;
    EXINT_Init(&stcExtiConfig);

    /* Select External Int Ch.4 */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ4;
    /* Register External Int to Vect.No.007 */
    stcIrqRegiConf.enIRQn = Int007_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = ExtInt04_Callback;
    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);
    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);
    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
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
    uint32_t u32CmpRet = 0;
    stc_dma_config_t stcDmaCfg;

    MEM_ZERO_STRUCT(stcDmaCfg);

    Led_Init();
    SW4_Init();

    /* descriptor 0 */
    stcLlpDesc[0].SARx = (uint32_t)(&u16SrcBuf[0]);
    stcLlpDesc[0].DARx = (uint32_t)(&u16DstBuf[0]);
    stcLlpDesc[0].DTCTLx_f.CNT = DMA_TRNCNT;
    stcLlpDesc[0].DTCTLx_f.BLKSIZE = DMA_BLKSIZE;
    stcLlpDesc[0].LLPx = (uint32_t)(&stcLlpDesc[1]);
    stcLlpDesc[0].CHxCTL_f.SINC = DMA_INC_MODE;
    stcLlpDesc[0].CHxCTL_f.DINC = DMA_INC_MODE;
    stcLlpDesc[0].CHxCTL_f.HSIZE = Dma16Bit;
    stcLlpDesc[0].CHxCTL_f.LLPEN = Enable;
    stcLlpDesc[0].CHxCTL_f.LLPRUN = DMA_LLP_MODE;

    /* descriptor 1 */
    stcLlpDesc[1].SARx = (uint32_t)(&u32SrcBuf[0]);
    stcLlpDesc[1].DARx = (uint32_t)(&u32DstBuf[0]);
    stcLlpDesc[1].DTCTLx_f.CNT = DMA_TRNCNT;
    stcLlpDesc[1].DTCTLx_f.BLKSIZE = DMA_BLKSIZE;
    stcLlpDesc[1].CHxCTL_f.SINC = DMA_INC_MODE;
    stcLlpDesc[1].CHxCTL_f.DINC = DMA_INC_MODE;
    stcLlpDesc[1].CHxCTL_f.HSIZE = Dma32Bit;
    stcLlpDesc[1].CHxCTL_f.LLPEN = Disable;

    /* Set data block size. */
    stcDmaCfg.u16BlockSize = DMA_BLKSIZE;
    /* Set transfer count. */
    stcDmaCfg.u16TransferCnt = DMA_TRNCNT;
    /* Set source & destination address. */
    stcDmaCfg.u32SrcAddr = (uint32_t)(&u8SrcBuf[0]);
    stcDmaCfg.u32DesAddr = (uint32_t)(&u8DstBuf[0]);

    /* Config dma channel. */
    /* Enable linked list transfer. */
    stcDmaCfg.stcDmaChCfg.enLlpEn = Enable;
    stcDmaCfg.stcDmaChCfg.enLlpMd = DMA_LLP_MODE;
    stcDmaCfg.u32DmaLlp = (uint32_t)(&stcLlpDesc[0]);
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressIncrease;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma8Bit;

    /* Enable DMA clock. */
    if(DMA_UNIT == M4_DMA1)
    {
        PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA1,Enable);
    }
    else if(DMA_UNIT == M4_DMA2)
    {
        PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA2,Enable);
    }

    /* Init Dma channel. */
    DMA_InitChannel(DMA_UNIT, DMA_CH, &stcDmaCfg);

    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS,Enable);
    /* Set dma trigger source. */
    DMA_SetTriggerSrc(DMA_UNIT, DMA_CH, EVT_PORT_EIRQ4);

    /*  Clear interrupt flag  */
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq);
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, BlkTrnCpltIrq);

    /* Enable DMA1 channel0. */
    DMA_ChannelCmd(DMA_UNIT, DMA_CH,Enable);

    /*  Wait transfter completion  */
    while(Set != DMA_GetIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq));

    /* Verify destination buffer data && expeted data */
    if(0 != memcmp(u8DstBuf, u8SrcBuf, sizeof(u8SrcBuf)))
    {
        u32CmpRet += 1;
    }

    if(0 != memcmp(u16DstBuf, u16SrcBuf, sizeof(u16SrcBuf)))
    {
        u32CmpRet += 1;
    }

    if(0 != memcmp(u32DstBuf, u32SrcBuf, sizeof(u32SrcBuf)))
    {
        u32CmpRet += 1;
    }

    if(u32CmpRet == 0)
    {
        LED1_ON();      /* Meet the expected */
    }
    else
    {
        LED0_ON();      /* Don't meet the expected */
    }

    while(1);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
