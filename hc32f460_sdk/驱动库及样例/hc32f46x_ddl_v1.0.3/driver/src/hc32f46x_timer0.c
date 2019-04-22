/*******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co.,Ltd All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co.,Ltd ("HDSC").
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
/** \file hc32f46x_timer0.c
 **
 ** A detailed description is available at
 ** @link Timer0Group description @endlink
 **
 **   - 2018-10-11  1.0  Wangmin  First version for Device Driver Library of TIMER0.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32f46x_timer0.h"

#if (DDL_TIMER0_ENABLE == DDL_ON)

/**
 *******************************************************************************
 ** \addtogroup Timer0Group
 ******************************************************************************/
//@{

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* Parameter valid check for peripheral Instances. */
#define IS_VALID_POINTER(x)           (NULL != (x))

/* Parameter validity check for unit. */
#define IS_VALID_UNIT(x)                                                       \
(   ((x) == M4_TMR01)                              ||                          \
    ((x) == M4_TMR02))

/* Parameter validity check for channel. */
#define IS_VALID_CHANNEL(x)                                                    \
(   ((x) == Tim0_ChannelA)                         ||                          \
    ((x) == Tim0_ChannelB))

/* Parameter validity check for command. */
#define IS_VALID_COMMAND(x)                                                    \
(   ((x) == Disable)                               ||                          \
    ((x) == Enable))

/* Parameter validity check for timer0 function mode. */
#define IS_VALID_FUNCTION(x)                                                   \
(   ((x) == Tim0_OutputCapare)                     ||                          \
    ((x) == Tim0_InputCaptrue))

/* Parameter validity check for clock division. */
#define IS_VALID_CLK_DIVISION(x)                                               \
(   ((x) == Tim0_ClkDiv0)                          ||                          \
    ((x) == Tim0_ClkDiv2)                          ||                          \
    ((x) == Tim0_ClkDiv4)                          ||                          \
    ((x) == Tim0_ClkDiv8)                          ||                          \
    ((x) == Tim0_ClkDiv16)                         ||                          \
    ((x) == Tim0_ClkDiv32)                         ||                          \
    ((x) == Tim0_ClkDiv64)                         ||                          \
    ((x) == Tim0_ClkDiv128)                        ||                          \
    ((x) == Tim0_ClkDiv256)                        ||                          \
    ((x) == Tim0_ClkDiv512)                        ||                          \
    ((x) == Tim0_ClkDiv1024))

/* Parameter validity check for synchronous clock source. */
#define IS_VALID_CLK_SYN_SRC(x)                                                \
(   ((x) == Tim0_Pclk1)                            ||                          \
    ((x) == Tim0_InsideHardTrig))

/* Parameter validity check for asynchronous clock source. */
#define IS_VALID_CLK_ASYN_SRC(x)                                               \
(   ((x) == Tim0_LRC)                              ||                          \
    ((x) == Tim0_XTAL32))

/* Parameter validity check for counter clock mode. */
#define IS_VALID_CLK_MODE(x)                                                   \
(   ((x) == Tim0_Sync)                             ||                          \
    ((x) == Tim0_Async))

/* Parameter validity check for external trigger event. */
#define IS_VALID_TRIG_SRC_EVENT(x)                                             \
(   (((x) >= EVT_PORT_EIRQ0) && ((x) <= EVT_PORT_EIRQ15))              ||      \
    (((x) >= EVT_DMA1_TC0) && ((x) <= EVT_DMA2_BTC3))                  ||      \
    (((x) >= EVT_EFM_OPTEND) && ((x) <= EVT_USBFS_SOF))                ||      \
    (((x) >= EVT_DCU1) && ((x) <= EVT_DCU4))                           ||      \
    (((x) >= EVT_TMR01_GCMA) && ((x) <= EVT_TMR02_GCMB))               ||      \
    (((x) >= EVT_RTC_ALM) && ((x) <= EVT_RTC_PRD))                     ||      \
    (((x) >= EVT_TMR61_GCMA) && ((x) <= EVT_TMR61_GUDF))               ||      \
    (((x) >= EVT_TMR61_SCMA) && ((x) <= EVT_TMR61_SCMB))               ||      \
    (((x) >= EVT_TMR62_GCMA) && ((x) <= EVT_TMR62_GUDF))               ||      \
    (((x) >= EVT_TMR62_SCMA) && ((x) <= EVT_TMR62_SCMB))               ||      \
    (((x) >= EVT_TMR63_GCMA) && ((x) <= EVT_TMR63_GUDF))               ||      \
    (((x) >= EVT_TMR63_SCMA) && ((x) <= EVT_TMR63_SCMB))               ||      \
    (((x) >= EVT_TMRA1_OVF) && ((x) <= EVT_TMRA5_CMP))                 ||      \
    (((x) >= EVT_TMRA6_OVF) && ((x) <= EVT_TMRA6_CMP))                 ||      \
    (((x) >= EVT_USART1_EI) && ((x) <= EVT_USART4_RTO))                ||      \
    (((x) >= EVT_SPI1_SRRI) && ((x) <= EVT_AOS_STRG))                  ||      \
    (((x) >= EVT_TMR41_SCMUH) && ((x) <= EVT_TMR42_SCMWL))             ||      \
    (((x) >= EVT_TMR43_SCMUH) && ((x) <= EVT_TMR43_SCMWL))             ||      \
    (((x) >= EVT_EVENT_PORT1)  && ((x) <= EVT_EVENT_PORT4))            ||      \
    (((x) >= EVT_I2S1_TXIRQOUT)  && ((x) <= EVT_I2S1_RXIRQOUT))        ||      \
    (((x) >= EVT_I2S2_TXIRQOUT)  && ((x) <= EVT_I2S2_RXIRQOUT))        ||      \
    (((x) >= EVT_I2S3_TXIRQOUT)  && ((x) <= EVT_I2S3_RXIRQOUT))        ||      \
    (((x) >= EVT_I2S4_TXIRQOUT)  && ((x) <= EVT_I2S4_RXIRQOUT))        ||      \
    (((x) >= EVT_ACMP1)  && ((x) <= EVT_ACMP3))                        ||      \
    (((x) >= EVT_I2C1_RXI) && ((x) <= EVT_I2C3_EE1))                   ||      \
    (((x) >= EVT_PVD_PVD1) && ((x) <= EVT_OTS))                        ||      \
    ((x) == EVT_WDT_REFUDF)                                            ||      \
    (((x) >= EVT_ADC1_EOCA) && ((x) <= EVT_TRNG_END))                  ||      \
    (((x) >= EVT_SDIOC1_DMAR) && ((x) <= EVT_SDIOC1_DMAW))             ||      \
    (((x) >= EVT_SDIOC2_DMAR) && ((x) <= EVT_SDIOC2_DMAW)))

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
/**
 *******************************************************************************
 ** \brief Get Timer0 status flag
 **
 ** \param [in] pstcTim0Reg       Pointer to Timer0 register
 **
 ** \param [in] enCh              channel, Tim0_ChannelA or Tim0_ChannelB
 **
 ** \retval Set        Flag is set
 **         Reset      Flag is reset
 **
 ******************************************************************************/
en_flag_status_t TIMER0_GetFlag(M4_TMR0_TypeDef* pstcTim0Reg, en_tim0_channel_t enCh)
{
    en_flag_status_t enFlag;
    DDL_ASSERT(IS_VALID_UNIT(pstcTim0Reg));
    DDL_ASSERT(IS_VALID_CHANNEL(enCh));

    switch(enCh)

    {
        case Tim0_ChannelA:
            enFlag = (en_flag_status_t)pstcTim0Reg->STFLR_f.CMAF;
            break;
        case Tim0_ChannelB:
            enFlag = (en_flag_status_t)pstcTim0Reg->STFLR_f.CMBF;
            break;
        default:
            break;
    }
    return enFlag;
}

/**
 *******************************************************************************
 ** \brief Clear Timer0 status flag
 **
 ** \param [in] pstcTim0Reg       Pointer to Timer0 register
 **
 ** \param [in] enCh              Timer0 channel, Timer0_ChA or Timer0_ChB
 **
 ** \retval None
 **
 ******************************************************************************/
void TIMER0_ClearFlag(M4_TMR0_TypeDef* pstcTim0Reg, en_tim0_channel_t enCh)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcTim0Reg));
    DDL_ASSERT(IS_VALID_CHANNEL(enCh));

    if(Tim0_ChannelA == enCh)
    {
        pstcTim0Reg->STFLR_f.CMAF =0u;
        while(0u != pstcTim0Reg->STFLR_f.CMAF);
    }
    else if(Tim0_ChannelB == enCh)
    {
        pstcTim0Reg->STFLR_f.CMBF = 0u;
        while(0u != pstcTim0Reg->STFLR_f.CMBF);
    }
}

/**
 *******************************************************************************
 ** \brief Command the timer0 function
 **
 ** \param [in] pstcTim0Reg       Pointer to Timer0 register
 **
 ** \param [in] enCh              Timer0 channel, Timer0_ChA or Timer0_ChB
 **
 ** \param [in] enCmd             Disable or Enable the function
 **
 ** \retval None
 **
 ******************************************************************************/
void TIMER0_Cmd(M4_TMR0_TypeDef* pstcTim0Reg, en_tim0_channel_t enCh,
                en_functional_state_t enCmd)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcTim0Reg));
    DDL_ASSERT(IS_VALID_CHANNEL(enCh));
    DDL_ASSERT(IS_VALID_COMMAND(enCmd));

    switch (enCh)
    {
        case Tim0_ChannelA:
            pstcTim0Reg->BCONR_f.CSTA = enCmd;
            while(enCmd != pstcTim0Reg->BCONR_f.CSTA);
            break;
        case Tim0_ChannelB:
            pstcTim0Reg->BCONR_f.CSTB = enCmd;
            while(enCmd != pstcTim0Reg->BCONR_f.CSTB);
            break;
        default:
            break;
    }
}

/**
 *******************************************************************************
 ** \brief Select the timer0 function mode
 **
 ** \param [in] pstcTim0Reg       Pointer to Timer0 register
 **
 ** \param [in] enCh              Timer0 channel, Tim0_ChannelA or Tim0_ChannelB
 **
 ** \param [in] enFunc            Timer0 function,Tim0_OutputCapare or Tim0_InputCapture
 **
 ** \retval None
 **
 ******************************************************************************/
void TIMER0_SetFunc(M4_TMR0_TypeDef* pstcTim0Reg, en_tim0_channel_t enCh,
                       en_tim0_function_t enFunc)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcTim0Reg));
    DDL_ASSERT(IS_VALID_CHANNEL(enCh));
    DDL_ASSERT(IS_VALID_FUNCTION(enFunc));

    switch (enCh)
    {
        case Tim0_ChannelA:
            pstcTim0Reg->BCONR_f.CAPMDA = enFunc;
            break;
        case Tim0_ChannelB:
            pstcTim0Reg->BCONR_f.CAPMDB = enFunc;
            break;
        default:
            break;
    }
}

/**
 *******************************************************************************
 ** \brief Timer0 interrupt function command
 **
 ** \param [in] pstcTim0Reg       Pointer to Timer0 register
 **
 ** \param [in] enCh              Timer0 channel, Tim0_ChannelA or Tim0_ChannelB
 **
 ** \param [in] enCmd             Disable or Enable the function
 **
 ** \retval None
 **
 ******************************************************************************/
void TIMER0_IntCmd(M4_TMR0_TypeDef* pstcTim0Reg, en_tim0_channel_t enCh,
                       en_functional_state_t enCmd)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcTim0Reg));
    DDL_ASSERT(IS_VALID_CHANNEL(enCh));
    DDL_ASSERT(IS_VALID_COMMAND(enCmd));

    switch (enCh)
    {
        case Tim0_ChannelA:
            pstcTim0Reg->BCONR_f.INTENA = enCmd;
            break;
        case Tim0_ChannelB:
            pstcTim0Reg->BCONR_f.INTENB = enCmd;
            break;
        default:
            break;
    }
}

/**
 *******************************************************************************
 ** \brief Get Timer0 counter register
 **
 ** \param [in] pstcTim0Reg       Pointer to Timer0 register
 **
 ** \param [in] enCh              Timer0 channel, Tim0_ChannelA or Tim0_ChannelB
 **
 ** \retval uint16_t              Count register
 **
 ******************************************************************************/
uint16_t TIMER0_GetCntReg(M4_TMR0_TypeDef* pstcTim0Reg,en_tim0_channel_t enCh)
{
    uint16_t u16Value = 0;
    DDL_ASSERT(IS_VALID_UNIT(pstcTim0Reg));
    DDL_ASSERT(IS_VALID_CHANNEL(enCh));

    if(Tim0_ChannelA == enCh)
    {
        u16Value = pstcTim0Reg->CNTAR&0xFFFF;
    }
    else if(Tim0_ChannelB == enCh)
    {
        u16Value = pstcTim0Reg->CNTBR&0xFFFF;
    }
    return u16Value;
}

/**
 *******************************************************************************
 ** \brief Write Timer0 counter register
 **
 ** \param [in] pstcTim0Reg       Pointer to Timer0 register
 **
 ** \param [in] enCh              Timer0 channel, Tim0_ChannelA or Tim0_ChannelB
 **
 ** \param [in] u16Cnt            Data to write
 **
 ** \retval uint16_t              None
 **
 ******************************************************************************/
void TIMER0_WriteCntReg(M4_TMR0_TypeDef* pstcTim0Reg,en_tim0_channel_t enCh,
                        uint16_t u16Cnt)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcTim0Reg));
    DDL_ASSERT(IS_VALID_CHANNEL(enCh));

    if(Tim0_ChannelA == enCh)
    {
        pstcTim0Reg->CNTAR = (uint32_t)u16Cnt;
        while(u16Cnt != (uint16_t)pstcTim0Reg->CNTAR);
    }
    else if(Tim0_ChannelB == enCh)
    {
        pstcTim0Reg->CNTBR = (uint32_t)u16Cnt;
        while(u16Cnt != (uint16_t)pstcTim0Reg->CNTBR);
    }
}

/**
 *******************************************************************************
 ** \brief Get Timer0 base compare count register
 **
 ** \param [in] pstcTim0Reg       Pointer to Timer0 register
 **
 ** \param [in] enCh              Timer0 channel, Tim0_ChannelA or Tim0_ChannelB
 **
 ** \retval uint16_t              Base compare count register
 **
 ******************************************************************************/
uint16_t TIMER0_GetCmpReg(M4_TMR0_TypeDef* pstcTim0Reg,en_tim0_channel_t enCh)
{
    uint16_t u16Value = 0;
    DDL_ASSERT(IS_VALID_UNIT(pstcTim0Reg));
    DDL_ASSERT(IS_VALID_CHANNEL(enCh));

    if(Tim0_ChannelA == enCh)
    {
        u16Value = pstcTim0Reg->CMPAR&0xFFFF;
    }
    else if(Tim0_ChannelB == enCh)
    {
        u16Value = pstcTim0Reg->CMPBR&0xFFFF;
    }
    return u16Value;
}

/**
 *******************************************************************************
 ** \brief Wirte Timer0 base compare count register
 **
 ** \param [in] pstcTim0Reg       Pointer to Timer0 register
 **
 ** \param [in] enCh              Timer0 channel, Tim0_ChannelA or Tim0_ChannelB
 **
 ** \param [in] u16Cnt            Data to write
 **
 ** \retval uint16_t              None
 **
 ******************************************************************************/
void TIMER0_WriteCmpReg(M4_TMR0_TypeDef* pstcTim0Reg, en_tim0_channel_t enCh,
                        uint16_t u16Cnt)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcTim0Reg));
    DDL_ASSERT(IS_VALID_CHANNEL(enCh));

    if(Tim0_ChannelA == enCh)
    {
        pstcTim0Reg->CMPAR = (uint32_t)u16Cnt;
        while(u16Cnt != (uint16_t)pstcTim0Reg->CMPAR);
    }
    else if(Tim0_ChannelB == enCh)
    {
        pstcTim0Reg->CMPBR = (uint32_t)u16Cnt;
        while(u16Cnt != (uint16_t)pstcTim0Reg->CMPBR);
    }
}

/**
 *******************************************************************************
 ** \brief  Timer0 peripheral base function initalize
 **
 ** \param [in] pstcTim0Reg     Pointer to Timer0 register
 **
 ** \param [in] enCh            Timer0 channel, Tim0_ChannelA or Tim0_ChannelB
 **
 ** \param [in] pstcBaseInit    Timer0 function base parameter structure
 **
 ** \retval None
 **
 ******************************************************************************/
void TIMER0_BaseInit(M4_TMR0_TypeDef* pstcTim0Reg,en_tim0_channel_t enCh,
                       const stc_tim0_base_init_t* pstcBaseInit)
{
    stc_tmr0_bconr_field_t stcBconrTmp;

    DDL_ASSERT(IS_VALID_UNIT(pstcTim0Reg));
    DDL_ASSERT(IS_VALID_CHANNEL(enCh));
    DDL_ASSERT(IS_VALID_POINTER(pstcBaseInit));
    DDL_ASSERT(IS_VALID_CLK_DIVISION(pstcBaseInit->Tim0_ClockDivision));
    DDL_ASSERT(IS_VALID_CLK_SYN_SRC(pstcBaseInit->Tim0_SyncClockSource));
    DDL_ASSERT(IS_VALID_CLK_ASYN_SRC(pstcBaseInit->Tim0_AsyncClockSource));
    DDL_ASSERT(IS_VALID_CLK_MODE(pstcBaseInit->Tim0_CounterMode));

    /*Read current BCONR register */
    stcBconrTmp = pstcTim0Reg->BCONR_f;

    switch(enCh)
    {
        case Tim0_ChannelA:
            /*set timer counter mode*/
            stcBconrTmp.SYNSA = pstcBaseInit->Tim0_CounterMode;
            switch(pstcBaseInit->Tim0_CounterMode)
            {
                case Tim0_Sync:
                    stcBconrTmp.SYNCLKA = pstcBaseInit->Tim0_SyncClockSource;
                    break;
                case Tim0_Async:
                    stcBconrTmp.ASYNCLKA = pstcBaseInit->Tim0_AsyncClockSource;
                    break;
                default:
                    break;
            }
            /*set clock division*/
            stcBconrTmp.CKDIVA = pstcBaseInit->Tim0_ClockDivision;
            /* Write BCONR register */
            pstcTim0Reg->BCONR_f = stcBconrTmp;
            /*set timer compare value*/
            pstcTim0Reg->CMPAR = pstcBaseInit->Tim0_CmpValue;
            while(pstcBaseInit->Tim0_CmpValue != (uint16_t)pstcTim0Reg->CMPAR);

            break;

        case Tim0_ChannelB:
            /*set timer counter mode*/
            stcBconrTmp.SYNSB = pstcBaseInit->Tim0_CounterMode;
            switch(pstcBaseInit->Tim0_CounterMode)
            {
                case Tim0_Sync:
                    stcBconrTmp.SYNCLKB = pstcBaseInit->Tim0_SyncClockSource;
                    break;
                case Tim0_Async:
                    stcBconrTmp.ASYNCLKB = pstcBaseInit->Tim0_AsyncClockSource;
                    break;
                default:
                    break;
            }
            /*set clock division*/
            stcBconrTmp.CKDIVB = pstcBaseInit->Tim0_ClockDivision;
            /* Write BCONR register */
            pstcTim0Reg->BCONR_f = stcBconrTmp;
            /*set timer compare value*/
            pstcTim0Reg->CMPBR = pstcBaseInit->Tim0_CmpValue;
            while(pstcBaseInit->Tim0_CmpValue != (uint16_t)pstcTim0Reg->CMPBR);

            break;
        default:
            break;
    }
}

/**
 *******************************************************************************
 ** \brief  Timer0 peripheral base function initalize
 **
 ** \param [in] pstcTim0Reg     Pointer to Timer0 register
 **
 ** \param [in] enCh            Timer0 channel, Tim0_ChannelA or Tim0_ChannelB
 **
 ** \retval None
 **
 ******************************************************************************/
void TIMER0_DeInit(M4_TMR0_TypeDef* pstcTim0Reg,en_tim0_channel_t enCh)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcTim0Reg));
    DDL_ASSERT(IS_VALID_CHANNEL(enCh));

    switch(enCh)
    {
        case Tim0_ChannelA:
            pstcTim0Reg->BCONR &= 0xFFFF0000;
            pstcTim0Reg->CMPAR = 0x0000FFFF;
            pstcTim0Reg->CNTAR = 0x00000000;
            pstcTim0Reg->STFLR_f.CMAF =0u;
            break;

        case Tim0_ChannelB:
            pstcTim0Reg->BCONR &= 0x0000FFFF;
            pstcTim0Reg->CMPBR = 0x0000FFFF;
            pstcTim0Reg->CNTBR = 0x00000000;
            pstcTim0Reg->STFLR_f.CMBF =0u;
            break;
        default:
            break;
    }
}

/**
 *******************************************************************************
 ** \brief  Set external trigger source for Timer0
 **
 ** \param [in] pstcTim0Reg     Pointer to Timer0 register
 **
 ** \param [in] enEvent         External event source
 **
 ** \retval None
 **
 ******************************************************************************/
void TIMER0_SetTriggerSrc(M4_TMR0_TypeDef* pstcTim0Reg, en_event_src_t enEvent)
{
    DDL_ASSERT(IS_VALID_UNIT(pstcTim0Reg));
    DDL_ASSERT(IS_VALID_TRIG_SRC_EVENT(enEvent));

    M4_AOS->TMR0_HTSSR_f.TRGSEL = enEvent;
}

/**
 *******************************************************************************
 ** \brief  Timer0 hardware trigger function initalize
 **
 ** \param [in] pstcTim0Reg     Pointer to Timer0 register
 **
 ** \param [in] enCh            Timer0 channel, Tim0_ChannelA or Tim0_ChannelB
 **
 ** \param [in] pStcInit        Timer0 hareware trigger function structure
 **
 ** \retval None
 **
 ******************************************************************************/
void TIMER0_HardTriggerInit(M4_TMR0_TypeDef* pstcTim0Reg,en_tim0_channel_t enCh,
                                const stc_tim0_trigger_init_t* pStcInit)
{
    stc_tmr0_bconr_field_t stcBconrTmp;
    DDL_ASSERT(IS_VALID_UNIT(pstcTim0Reg));
    DDL_ASSERT(IS_VALID_CHANNEL(enCh));
    DDL_ASSERT(IS_VALID_POINTER(pStcInit));
    DDL_ASSERT(IS_VALID_FUNCTION(pStcInit->Tim0_OCMode));
    DDL_ASSERT(IS_VALID_TRIG_SRC_EVENT(pStcInit->Tim0_SelTrigSrc));

    /*Read current BCONR register */
    stcBconrTmp = pstcTim0Reg->BCONR_f;

    switch(enCh)
    {
        case Tim0_ChannelA:
            /*set work on input captrue or output capare*/
            stcBconrTmp.CAPMDA = pStcInit->Tim0_OCMode;
            /*enable input capture*/
            stcBconrTmp.HICPA = pStcInit->Tim0_InTrigEnable;
            /*enable trigger clear counter*/
            stcBconrTmp.HCLEA = pStcInit->Tim0_InTrigClear;
            /*enable trigger start counter*/
            stcBconrTmp.HSTAA = pStcInit->Tim0_InTrigStart;
            /*enable trigger stop counter*/
            stcBconrTmp.HSTPA = pStcInit->Tim0_InTrigStop;

            /* Write BCONR register */
            pstcTim0Reg->BCONR_f = stcBconrTmp;
            break;
        case Tim0_ChannelB:
            /*set work on input captrue or output capare*/
            stcBconrTmp.CAPMDB = pStcInit->Tim0_OCMode;
            /*enable input capture*/
            stcBconrTmp.HICPB = pStcInit->Tim0_InTrigEnable;
            /*enable trigger clear counter*/
            stcBconrTmp.HCLEB = pStcInit->Tim0_InTrigClear;
            /*enable trigger start counter*/
            stcBconrTmp.HSTAB = pStcInit->Tim0_InTrigStart;
            /*enable trigger stop counter*/
            stcBconrTmp.HSTPB = pStcInit->Tim0_InTrigStop;

            /* Write BCONR register */
            pstcTim0Reg->BCONR_f = stcBconrTmp;
            break;
        default:
            break;
    }
    /* Set trigger source*/
    M4_AOS->TMR0_HTSSR_f.TRGSEL = pStcInit->Tim0_SelTrigSrc;

}

//@} // Timer0Group

#endif /* DDL_TIMER0_ENABLE */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
