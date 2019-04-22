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
#define DMA_UNIT                (M4_DMA2)
#define DMA_CH                  (DmaCh0)
#define DMA_TRG_SEL             (INT_PORT_EIRQ4)  /* External Int Ch.4 trigger request number */
#define DMA_TRNCNT              (4)
#define DMA_BLKSIZE             (5)
#define DMA_SNSEQ_CNT           (4)
#define DMA_SNSEQ_OFFSET        (6)
#define DMA_DNSEQ_CNT           (5)
#define DMA_DNSEQ_OFFSET        (6)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static const uint32_t u32SrcBuf[40] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 
                                   11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                                   21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
                                   31, 32, 33, 34, 35, 36, 37, 38, 39, 40};
static uint32_t u32DstBuf[40] = {0};

/* Note: if change DMAC configure, please modify the buffer data */
static uint32_t u32ExpectDstBufData[40] = { 1,  2,  3,  4, 10, 0, 0, 0, 0, 0, 
                                              11, 12, 13, 19, 20, 0, 0, 0, 0, 0,
                                              21, 22, 28, 29, 30, 0, 0, 0, 0, 0,
                                              31, 37, 38, 39, 40, 0, 0, 0, 0, 0};

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
    
    /* Set data block size. */
    stcDmaCfg.u16BlockSize = DMA_BLKSIZE;
    /* Set transfer count. */
    stcDmaCfg.u16TransferCnt = DMA_TRNCNT;
    /* Set source & destination address. */
    stcDmaCfg.u32SrcAddr = (uint32_t)(&u32SrcBuf[0]);
    stcDmaCfg.u32DesAddr = (uint32_t)(&u32DstBuf[0]);
    
    /* Enable non_sequence transfer. */
    stcDmaCfg.stcDmaChCfg.enSrcNseqEn = Enable;
    stcDmaCfg.stcDmaChCfg.enDesNseqEn = Enable;
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressIncrease;    
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma32Bit;
    
    /* Set source/destination no_sequence count & offset*/
    stcDmaCfg.stcSrcNseqCfg.u16Cnt = DMA_SNSEQ_CNT;
    stcDmaCfg.stcSrcNseqCfg.u32Offset = DMA_SNSEQ_OFFSET;
    stcDmaCfg.stcDesNseqCfg.u16Cnt = DMA_DNSEQ_CNT;
    stcDmaCfg.stcDesNseqCfg.u32Offset = DMA_DNSEQ_OFFSET;
    
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
    DMA_SetTriggerSrc(DMA_UNIT, DMA_CH, EVT_AOS_STRG);
    
    /*  Clear interrupt flag  */
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq);
    
    /* Enable DMA channel. */
    DMA_ChannelCmd(DMA_UNIT, DMA_CH,Enable);
    
    /* wait SW2 pressed. */
    while(0 != PORT_GetBit(PortD, Pin03)); 
    
    /*  Wait transfter completion  */
    while(Set != DMA_GetIrqFlag(DMA_UNIT, DMA_CH, TrnCpltIrq))
    {
        DMA_ClearIrqFlag(DMA_UNIT, DMA_CH,TrnCpltIrq);
        AOS_SW_Trigger();
        while(Set != DMA_GetIrqFlag(DMA_UNIT, DMA_CH, BlkTrnCpltIrq));
    }
    
    /* Verify destination buffer data && expeted data */
    u32CmpRet = memcmp(u32DstBuf, u32ExpectDstBufData, sizeof(u32DstBuf));
    if(0 == u32CmpRet)
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
