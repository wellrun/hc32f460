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
/* DMAC */
#define DMA_UNIT                M4_DMA2
#define DMA_CH                  DmaCh0
#define DMA_TRNCNT              (20u)
#define DMA_BLKSIZE             (1u)
#define DMA_RPTB_SIZE           (5u)

/* LED0 Port/Pin definition */
#define  LED0_PORT              PortE
#define  LED0_PIN               Pin06

/* LED1 Port/Pin definition */
#define  LED1_PORT              PortA
#define  LED1_PIN               Pin07

/* LED0~1 definition */
#define LED0_ON()               PORT_SetBits(LED0_PORT, LED0_PIN)
#define LED0_OFF()              PORT_ResetBits(LED0_PORT, LED0_PIN)

#define LED1_ON()               PORT_SetBits(LED1_PORT, LED1_PIN)
#define LED1_OFF()              PORT_ResetBits(LED1_PORT, LED1_PIN)

#define KEY0_PORT               PortD
#define KEY0_PIN                Pin03

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint8_t u8CmpRet = 1;
/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static const uint32_t u32SrcBuf[22] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
                                       11, 12, 13, 14, 15, 16, 17, 18,
                                       19, 20, 21, 22};
static uint32_t u32DstBuf[22] = {0};
static uint32_t u32ExpectDstBufData[22] = {1, 2, 3, 4, 5, 1, 2, 3, 4, 5};
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

    PORT_Unlock();
    LED0_OFF();
    LED1_OFF();
    PORT_Lock();

    /* LED0 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* LED1 Port/Pin initialization */
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);
}
/**
 *******************************************************************************
 ** \brief  SW2 interrupt callback function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt03_Callback(void)
{
    EXINT_IrqFlgClr(ExtiCh03);
    AOS_SW_Trigger();

    while(Set != DMA_GetIrqFlag(DMA_UNIT,DMA_CH, TrnCpltIrq))
    {
        AOS_SW_Trigger();
    }
    u8CmpRet = memcmp(u32DstBuf, u32ExpectDstBufData, sizeof(u32DstBuf));
}
/**
 *******************************************************************************
 ** \brief KEY0(SW2) init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Sw2_Init(void)
{
    stc_port_init_t stcPortInit;
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /**************************************************************************/
    /* External Int Ch.3                                                      */
    /**************************************************************************/
    /* Set PD03 as External Int Ch.3 input */
    stcPortInit.enExInt = Enable;
    PORT_Init(KEY0_PORT, KEY0_PIN, &stcPortInit);

    stcExtiConfig.enExitCh = ExtiCh03;
    /* Filter setting */
    stcExtiConfig.enFilterEn = Disable;
    /* Falling edge */
    stcExtiConfig.enExtiLvl = ExIntFallingEdge;
    EXINT_Init(&stcExtiConfig);

    /* Select External Int Ch.3 */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ3;
    /* Register External Int to Vect.No.007 */
    stcIrqRegiConf.enIRQn = Int007_IRQn;
    /* Callback function */
    stcIrqRegiConf.pfnCallback = ExtInt03_Callback;
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
 ** \brief DMA init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Dma_Init(void)
{
    stc_dma_config_t    stcDmaCfg;

    MEM_ZERO_STRUCT(stcDmaCfg);

    /* Set data block size. */
    stcDmaCfg.u16BlockSize = DMA_BLKSIZE;
    /* Set transfer count. */
    stcDmaCfg.u16TransferCnt = DMA_TRNCNT;
    /* Set source & destination address. */
    stcDmaCfg.u32SrcAddr = (uint32_t)(&u32SrcBuf[0]);
    stcDmaCfg.u32DesAddr = (uint32_t)(&u32DstBuf[0]);

    /* Disable linked list transfer. */
    stcDmaCfg.stcDmaChCfg.enLlpEn = Disable;
    /* Enable repeat function. */
    stcDmaCfg.stcDmaChCfg.enSrcRptEn = Enable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn = Enable;
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressIncrease;
    /* Disable interrup. */
    stcDmaCfg.stcDmaChCfg.enIntEn = Disable;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma32Bit;

    /* Initialize DMA. */
    DMA_InitChannel(DMA_UNIT, DMA_CH, &stcDmaCfg);
}

/**
 *******************************************************************************
 ** \brief DMA Re_Config init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Dma_ReCfgInit(void)
{
    stc_dma_recfg_ctl_t stcDmaReCfg;

    MEM_ZERO_STRUCT(stcDmaReCfg);

    /* set remain transfer count after re_config :
    same with transfer count whitch DMA_SNSEQCTLBx.SNSCNTB / DMA_RPTBx.SRPTB set.*/
    stcDmaReCfg.enCntMd = CntSrcAddr;
    /* destination address update DMA_DARx after re_config */
    stcDmaReCfg.enDaddrMd = DaddrRep;
    /* source address update DMA_SARx after re_config */
    stcDmaReCfg.enSaddrMd = SaddrRep;
    /* re_config channel */
    stcDmaReCfg.enReCfgCh = DMA_CH;
    /* Disable LLP */
    stcDmaReCfg.enReCfgLlp = Disable;
    /* re_config destination repeat size */
    stcDmaReCfg.u16DesRptBSize = DMA_RPTB_SIZE + 5;
    /* re_config source repeat size */
    stcDmaReCfg.u16SrcRptBSize = DMA_RPTB_SIZE;

    /* Initialize DMA re_config */
    DMA_InitReConfig(DMA_UNIT, DMA_CH, &stcDmaReCfg);
    /* Enable DMA re_config */
    DMA_ReCfgCmd(DMA_UNIT, Enable);
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
    Led_Init();
    Sw2_Init();

    /* Enable DMA clock. */
    if(DMA_UNIT == M4_DMA1)
    {
        PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA1,Enable);
    }
    else if(DMA_UNIT == M4_DMA2)
    {
        PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA2,Enable);
    }

    /* Enable DMA. */
    DMA_Cmd(DMA_UNIT,Enable);

    /* Init Dma channel */
    Dma_Init();
    /* Init Dma re_config */
    Dma_ReCfgInit();

    /* Enable DMA channel. */
    DMA_ChannelCmd(DMA_UNIT, DMA_CH,Enable);

    /* Clear DMA transfer complete interrupt flag. */
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH,TrnCpltIrq);

    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS,Enable);

    DMA_SetTriggerSrc(DMA_UNIT, DMA_CH, EVT_AOS_STRG);
    DMA_SetReConfigTriggerSrc(EVT_PORT_EIRQ3);

    /* SW2 */
    while(0 != PORT_GetBit(KEY0_PORT, KEY0_PIN));

    while(Set != DMA_GetIrqFlag(DMA_UNIT,DMA_CH, TrnCpltIrq))
    {
        AOS_SW_Trigger();
    }


    if(0 == u8CmpRet)
    {
        LED1_ON();    /* Meet the expected */
    }
    else
    {
        LED0_ON();    /* Don't meet the expected */
    }

    while(1);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
