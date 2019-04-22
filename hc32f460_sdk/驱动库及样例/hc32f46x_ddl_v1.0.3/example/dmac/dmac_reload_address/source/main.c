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
#define DMA_UNIT                (M4_DMA2)
#define DMA_CH                  (DmaCh0)
#define DMA_TRNCNT              (5u)
#define DMA_BLKSIZE             (4u)
#define DMA_SRPT_SIZE           (30u)
#define DMA_DRPT_SIZE           (15u)

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
static uint32_t u32ExpectDstBufData[22] = {16, 17, 18, 19, 20,
                                            6,  7,  8,  9, 10,
                                           11, 12, 13, 14, 15};
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
 ** \brief  Main function of template project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
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
    /* Set repeat size. */
    stcDmaCfg.u16SrcRptSize = DMA_SRPT_SIZE;
    stcDmaCfg.u16DesRptSize = DMA_DRPT_SIZE;

    /* Disable linked list transfer. */
    stcDmaCfg.stcDmaChCfg.enLlpEn = Disable;
    /* Enable repeat function. */
    stcDmaCfg.stcDmaChCfg.enSrcRptEn = Enable;
    stcDmaCfg.stcDmaChCfg.enDesRptEn = Enable;
    /* Set source & destination address mode. */
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressIncrease;
    /* Enable interrup. */
    stcDmaCfg.stcDmaChCfg.enIntEn = Enable;
    /* Set data width 32bit. */
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma32Bit;

    /* Enable DMA clock. */
    if(DMA_UNIT == M4_DMA1)
    {
        PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA1,Enable);
    }
    else if(DMA_UNIT == M4_DMA2)
    {
        PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA2,Enable);
    }

    /* Enable DMA1. */
    DMA_Cmd(DMA_UNIT,Enable);
    /* Initialize DMA. */
    DMA_InitChannel(DMA_UNIT, DMA_CH, &stcDmaCfg);
    /* Enable DMA1 channel0. */
    DMA_ChannelCmd(DMA_UNIT, DMA_CH,Enable);
    /* Clear DMA transfer complete interrupt flag. */
    DMA_ClearIrqFlag(DMA_UNIT, DMA_CH,TrnCpltIrq);

    /* Enable PTDIS(AOS) clock*/
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_PTDIS,Enable);

    DMA_SetTriggerSrc(DMA_UNIT, DMA_CH, EVT_AOS_STRG);

    AOS_SW_Trigger();

    while(Set != DMA_GetIrqFlag(DMA_UNIT,DMA_CH, TrnCpltIrq))
    {
        AOS_SW_Trigger();
    }

    u8CmpRet = memcmp(u32DstBuf, u32ExpectDstBufData, sizeof(u32DstBuf));
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
