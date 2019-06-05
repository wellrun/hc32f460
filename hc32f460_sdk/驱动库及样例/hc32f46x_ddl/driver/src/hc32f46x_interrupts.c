/******************************************************************************
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
/*****************************************************************************/
/** \file hc32f46x_interrupts.c
 **
 ** A detailed description is available at
 ** @link InterruptGroup Interrupt description @endlink
 **
 **   - 2018-10-12  1.0  Zhangxl First version for Device Driver Library of
 **     interrupt.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32f46x_interrupts.h"
#include "hc32f46x_utility.h"

#if (DDL_INTERRUPTS_ENABLE == DDL_ON)

/**
 *******************************************************************************
 ** \addtogroup InterruptGroup
 ******************************************************************************/
//@{

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/*! Parameter validity check for null pointer. */
#define     IS_NULL_POINT(x)        (NULL != (x))

/*! Parameter validity check for valid wakeup source from stop mode. */
#define     IS_VALID_WKUP_SRC(x)                                                \
(   ((x) == INT_USART1_WUPI)                    ||                              \
    ((x) == INT_TMR01_GCMA)                     ||                              \
    ((x) == INT_RTC_ALM)                        ||                              \
    ((x) == INT_RTC_PRD)                        ||                              \
    ((x) == INT_WKTM_PRD)                       ||                              \
    ((x) == INT_ACMP1)                          ||                              \
    ((x) == INT_PVD_PVD1)                       ||                              \
    ((x) == INT_PVD_PVD2)                       ||                              \
    ((x) == INT_SWDT_REFUDF)                    ||                              \
    ((x) == INT_PORT_EIRQ0)                     ||                              \
    ((x) == INT_PORT_EIRQ1)                     ||                              \
    ((x) == INT_PORT_EIRQ2)                     ||                              \
    ((x) == INT_PORT_EIRQ3)                     ||                              \
    ((x) == INT_PORT_EIRQ4)                     ||                              \
    ((x) == INT_PORT_EIRQ5)                     ||                              \
    ((x) == INT_PORT_EIRQ6)                     ||                              \
    ((x) == INT_PORT_EIRQ7)                     ||                              \
    ((x) == INT_PORT_EIRQ8)                     ||                              \
    ((x) == INT_PORT_EIRQ9)                     ||                              \
    ((x) == INT_PORT_EIRQ10)                    ||                              \
    ((x) == INT_PORT_EIRQ11)                    ||                              \
    ((x) == INT_PORT_EIRQ12)                    ||                              \
    ((x) == INT_PORT_EIRQ13)                    ||                              \
    ((x) == INT_PORT_EIRQ14)                    ||                              \
    ((x) == INT_PORT_EIRQ15))

/*! Max IRQ Handler. */
#define     IRQ_NUM_MAX             128u

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
uint32_t NVIC_ISER_BAK[5];

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
func_ptr_t IrqHandler[IRQ_NUM_MAX] = {NULL};

/**
 *******************************************************************************
 ** \brief NVIC backup and disable before entry from stop mode
 **
 ** param  none
 **
 ** retval Ok                           Ok, Backup and disable sucessfully.
 **
 *****************************************************************************/
en_result_t enNvicBackup(void)
{
    uint8_t u8Cnt;
    stc_intc_sel_field_t *stcIntSel;

    /* Backup NVIC set enable register for IRQ0~143*/
    for (u8Cnt = 0; u8Cnt < 5; u8Cnt++)
    {
        NVIC_ISER_BAK[u8Cnt] = NVIC->ISER[u8Cnt];
    }

    /* Disable share vector */
    for (u8Cnt = 128; u8Cnt < 144; u8Cnt++)
    {
        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
    }

    for (u8Cnt = 0; u8Cnt < 128; u8Cnt++)
    {
        stcIntSel = (stc_intc_sel_field_t *)((uint32_t)(&M4_INTC->SEL0) + (4 * u8Cnt));
        /* Disable NVIC if it is the wakeup-able source from stop mode */
        if (IS_VALID_WKUP_SRC(stcIntSel->INTSEL))
        {
            switch (stcIntSel->INTSEL)
            {
                case INT_USART1_WUPI:
                    if (false == M4_INTC->WUPEN_f.SCIWEN)
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_TMR01_GCMA:
                    if (false == M4_INTC->WUPEN_f.TMR0WUEN)
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_RTC_ALM:
                    if (false == M4_INTC->WUPEN_f.RTCALMWUEN)
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_RTC_PRD:
                    if (false == M4_INTC->WUPEN_f.RTCPRDWUEN)
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_WKTM_PRD:
                    if (false == M4_INTC->WUPEN_f.WKTMWUEN)
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_ACMP1:
                    if (false == M4_INTC->WUPEN_f.CMPI0WUEN)
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PVD_PVD1:
                    if (false == M4_INTC->WUPEN_f.PVD1WUEN)
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PVD_PVD2:
                    if (false == M4_INTC->WUPEN_f.PVD2WUEN)
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_SWDT_REFUDF:
                    if (false == M4_INTC->WUPEN_f.SWDTWUEN)
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ0:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x0001u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ1:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x0002u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ2:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x0004u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ3:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x0008u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ4:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x0010u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ5:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x0020u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ6:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x0040u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ7:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x0080u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ8:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x0100u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ9:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x0200u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ10:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x0400u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ11:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x0800u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ12:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x1000u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ13:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x2000u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ14:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x4000u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                case INT_PORT_EIRQ15:
                    if (false == !!(M4_INTC->WUPEN_f.EIRQWUEN & 0x8000u))
                    {
                        NVIC_DisableIRQ((IRQn_Type)u8Cnt);
                    }
                    break;
                default:
                    break;
            }
        }
        /* Disable NVIC for all none-wakeup source */
        else if (INT_MAX != stcIntSel->INTSEL)
        {
            NVIC_DisableIRQ((IRQn_Type)u8Cnt);
        }
        else
        {
        }
    }
    return Ok;
}

/**
 *******************************************************************************
 ** \brief NVIC recover after wakeup from stop mode
 **
 ** param  none
 **
 ** retval Ok                           Ok, NVIC set enable recover sucessfully.
 **
 *****************************************************************************/
en_result_t enNvicRecover(void)
{
    uint8_t u8Cnt;

    for (u8Cnt = 0; u8Cnt < 8; u8Cnt++)
    {
        NVIC->ISER[u8Cnt] = NVIC_ISER_BAK[u8Cnt];
    }
    return Ok;
}

/**
 *******************************************************************************
 ** \brief IRQ Registration
 **
 ** param [in]                          pstcIrqRegiConf, IRQ registration
 **                                     configure structure
 **
 ** retval                              Ok, IRQ register successfully.
 **                                     ErrorInvalidParameter, IRQ No. and
 **                                     Vector No. are not match.
 **                                     ErrorUninitialized, Make sure the
 **                                     Interrupt select register (INTSEL) is
 **                                     default value (0x1FFu) before setting.
 **
 *****************************************************************************/
en_result_t enIrqRegistration(const stc_irq_regi_conf_t *pstcIrqRegiConf)
{
    // todo, assert ...
    stc_intc_sel_field_t *stcIntSel;

    //DDL_ASSERT(NULL != pstcIrqRegiConf->pfnCallback);
    DDL_ASSERT(IS_NULL_POINT(pstcIrqRegiConf->pfnCallback));

    /* IRQ032~128 whether out of range */
    if (((((pstcIrqRegiConf->enIntSrc/32)*6 + 32) > pstcIrqRegiConf->enIRQn) || \
        (((pstcIrqRegiConf->enIntSrc/32)*6 + 37) < pstcIrqRegiConf->enIRQn)) && \
        pstcIrqRegiConf->enIRQn >= 32)
    {
        return ErrorInvalidParameter;
    }

    stcIntSel = (stc_intc_sel_field_t *)((uint32_t)(&M4_INTC->SEL0)         +   \
                                         (4 * pstcIrqRegiConf->enIRQn));
    if (0x1FFu == stcIntSel->INTSEL)
    {
        stcIntSel->INTSEL = pstcIrqRegiConf->enIntSrc;
        IrqHandler[pstcIrqRegiConf->enIRQn] = pstcIrqRegiConf->pfnCallback;
    }
    else
    {
        return ErrorUninitialized;
    }
    return Ok;
}

/**
 *******************************************************************************
 ** \brief IRQ Resign
 **
 ** param [in]                          enIRQn, IRQ enumunation (Int000_IRQn ~
 **                                     Int127_IRQn
 **
 ** retval                              Ok, IRQ resign sucessfully.
 **                                     ErrorInvalidParameter, IRQ No. is out
 **                                     of range
 **
 *****************************************************************************/
en_result_t enIrqResign(IRQn_Type enIRQn)
{
    stc_intc_sel_field_t *stcIntSel;

    if (enIRQn < Int000_IRQn || enIRQn > Int127_IRQn)
    {
        return ErrorInvalidParameter;
    }

    stcIntSel = (stc_intc_sel_field_t *)((uint32_t)(&M4_INTC->SEL0) + (4 * enIRQn));
    stcIntSel->INTSEL = 0x1FFu;
    IrqHandler[enIRQn] = NULL;
    return Ok;
}

/**
 *******************************************************************************
 ** \brief Share IRQ handler enable
 **
 ** param [in]                          enIntSrc, interrupt souce, This parameter
 **                                     can be any value of @ref en_int_src_t
 **
 ** retval                              Ok
 **
 ******************************************************************************/
en_result_t enShareIrqEnable(en_int_src_t enIntSrc)
{
    uint32_t *VSSELx;

    //todo assert

    VSSELx = (uint32_t *)(((uint32_t)&M4_INTC->VSSEL128) + (4 * (enIntSrc/32)));
    *VSSELx |= (1u << (enIntSrc%32));

    return Ok;
}

/**
 *******************************************************************************
 ** \brief Share IRQ handler disable
 **
 ** param [in]                          enIntSrc, interrupt souce, This parameter
 **                                     can be any value of @ref en_int_src_t
 **
 ** retval                              Ok
 **
 ******************************************************************************/
en_result_t enShareIrqDisable(en_int_src_t enIntSrc)
{
    uint32_t *VSSELx;

    //todo assert

    VSSELx = (uint32_t *)(((uint32_t)&M4_INTC->VSSEL128) + (4 * (enIntSrc/32)));
    *VSSELx &= ~(1u << (enIntSrc%32));

    return Ok;
}

/**
 *******************************************************************************
 ** \brief Enable stop mode wakeup source
 **
 ** param [in]                          u32WakeupSrc, This parameter can be any
 **                                     composed value of @ref en_int_wkup_src_t
 **
 ** retval                              Ok, corresponding wakeup source be enabled
 **                                     ErrorInvalidParameter, parameter with
 **                                     non-definition bits
 **
 ******************************************************************************/
en_result_t enIntWakeupEnable(uint32_t u32WakeupSrc)
{
    if (0 != (u32WakeupSrc & 0xFD000000u))
    {
        return ErrorInvalidParameter;
    }
    M4_INTC->WUPEN |= u32WakeupSrc;
    return Ok;
}

/**
 *******************************************************************************
 ** \brief Disable stop mode wakeup source
 **
 ** param [in]                          u32WakeupSrc, This parameter can be any
 **                                     composed value of @ref en_int_wkup_src_t
 **
 ** retval                              Ok, corresponding wakeup source be disabled
 **                                     ErrorInvalidParameter, parameter with
 **                                     non-definition bits
 **
 ******************************************************************************/
en_result_t enIntWakeupDisable(uint32_t u32WakeupSrc)
{
    if (0 != (u32WakeupSrc & 0xFD000000u))
    {
        return ErrorInvalidParameter;
    }
    M4_INTC->WUPEN &= ~u32WakeupSrc;
    return Ok;
}

/**
 *******************************************************************************
 ** \brief Event enable
 **
 ** param [in]                          u32Event, This parameter can be
 **                                     any composed value of @ref en_evt_t
 **
 ** retval                              Ok, corresponding event Ch. be enabled
 **
 ******************************************************************************/
en_result_t enEventEnable(uint32_t u32Event)
{
    M4_INTC->EVTER |= u32Event;
    return Ok;
}

/**
 *******************************************************************************
 ** \brief Event enable
 **
 ** param [in]                          u32Event, This parameter can be
 **                                     any composed value of @ref en_evt_t
 **
 ** retval                              Ok, corresponding event Ch. be disabled
 **
 ******************************************************************************/
en_result_t enEventDisable(uint32_t u32Event)
{
    M4_INTC->EVTER &= ~u32Event;
    return Ok;
}

/**
 *******************************************************************************
 ** \brief Interrupt enable
 **
 *  param [in]                          u32Int, This parameter can be any composed
 *                                      value of @ref en_int_t
 **
 ** retval                              Ok, corresponding interrupt vector be enabled
 **
 ******************************************************************************/
en_result_t enIntEnable(uint32_t u32Int)
{
    M4_INTC->IER |= u32Int;
    return Ok;
}

/**
 *******************************************************************************
 ** \brief Interrupt disable
 **
 *  param [in]                          u32Int, This parameter can be any composed
 *                                      value of @ref en_int_t
 **
 ** retval                              Ok, corresponding interrupt vector be disabled
 **
 ******************************************************************************/
en_result_t enIntDisable(uint32_t u32Int)
{
    M4_INTC->IER &= ~u32Int;
    return Ok;
}

/**
 *******************************************************************************
 ** \brief NMI IRQ handler
 **
 ******************************************************************************/
void NMI_Handler(void)
{
    NMI_IrqHandler();
}

/**
 *******************************************************************************
 ** \brief Hard Fault IRQ handler
 **
 ******************************************************************************/
void HardFault_Handler(void)
{
    HardFault_IrqHandler();
}

/**
 *******************************************************************************
 ** \brief MPU Fault IRQ handler
 **
 ******************************************************************************/
void MemManage_Handler(void)
{
    MemManage_IrqHandler();
}

/**
 *******************************************************************************
 ** \brief Bus Fault IRQ handler
 **
 ******************************************************************************/
void BusFault_Handler(void)
{
    BusFault_IrqHandler();
}

/**
 *******************************************************************************
 ** \brief Usage Fault IRQ handler
 **
 ******************************************************************************/
void UsageFault_Handler(void)
{
    UsageFault_IrqHandler();
}

/**
 *******************************************************************************
 ** \brief SVCall IRQ handler
 **
 ******************************************************************************/
void SVC_Handler(void)
{
    SVC_IrqHandler();
}

/**
 *******************************************************************************
 ** \brief DebugMon IRQ handler
 **
 ******************************************************************************/
void DebugMon_Handler(void)
{
    DebugMon_IrqHandler();
}

/**
 *******************************************************************************
 ** \brief PendSV IRQ handler
 **
 ******************************************************************************/
void PendSV_Handler(void)
{
    PendSV_IrqHandler();
}

/**
 *******************************************************************************
 ** \brief Systick IRQ handler
 **
 ******************************************************************************/
void SysTick_Handler(void)
{
    SysTick_IrqHandler();
}

/**
 *******************************************************************************
 ** \brief Int No.000 IRQ handler
 **
 ******************************************************************************/
void IRQ000_Handler(void)
{
    if (NULL != IrqHandler[Int000_IRQn])
    {
        IrqHandler[Int000_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.001 IRQ handler
 **
 ******************************************************************************/
void IRQ001_Handler(void)
{
    if (NULL != IrqHandler[Int001_IRQn])
    {
        IrqHandler[Int001_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.002 IRQ handler
 **
 ******************************************************************************/
void IRQ002_Handler(void)
{
    if (NULL != IrqHandler[Int002_IRQn])
    {
        IrqHandler[Int002_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.003 IRQ handler
 **
 ******************************************************************************/
void IRQ003_Handler(void)
{
    if (NULL != IrqHandler[Int003_IRQn])
    {
        IrqHandler[Int003_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.004 IRQ handler
 **
 ******************************************************************************/
void IRQ004_Handler(void)
{
    if (NULL != IrqHandler[Int004_IRQn])
    {
        IrqHandler[Int004_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.005 IRQ handler
 **
 ******************************************************************************/
void IRQ005_Handler(void)
{
    if (NULL != IrqHandler[Int005_IRQn])
    {
        IrqHandler[Int005_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.006 IRQ handler
 **
 ******************************************************************************/
void IRQ006_Handler(void)
{
    if (NULL != IrqHandler[Int006_IRQn])
    {
        IrqHandler[Int006_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.007 IRQ handler
 **
 ******************************************************************************/
void IRQ007_Handler(void)
{
    if (NULL != IrqHandler[Int007_IRQn])
    {
        IrqHandler[Int007_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.008 IRQ handler
 **
 ******************************************************************************/
void IRQ008_Handler(void)
{
    if (NULL != IrqHandler[Int008_IRQn])
    {
        IrqHandler[Int008_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.009 IRQ handler
 **
 ******************************************************************************/
void IRQ009_Handler(void)
{
    if (NULL != IrqHandler[Int009_IRQn])
    {
        IrqHandler[Int009_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.010 IRQ handler
 **
 ******************************************************************************/
void IRQ010_Handler(void)
{
    if (NULL != IrqHandler[Int010_IRQn])
    {
        IrqHandler[Int010_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.011 IRQ handler
 **
 ******************************************************************************/
void IRQ011_Handler(void)
{
    if (NULL != IrqHandler[Int011_IRQn])
    {
        IrqHandler[Int011_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.012 IRQ handler
 **
 ******************************************************************************/
void IRQ012_Handler(void)
{
    if (NULL != IrqHandler[Int012_IRQn])
    {
        IrqHandler[Int012_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.013 IRQ handler
 **
 ******************************************************************************/
void IRQ013_Handler(void)
{
    if (NULL != IrqHandler[Int013_IRQn])
    {
        IrqHandler[Int013_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.014 IRQ handler
 **
 ******************************************************************************/
void IRQ014_Handler(void)
{
    if (NULL != IrqHandler[Int014_IRQn])
    {
        IrqHandler[Int014_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.015 IRQ handler
 **
 ******************************************************************************/
void IRQ015_Handler(void)
{
    if (NULL != IrqHandler[Int015_IRQn])
    {
        IrqHandler[Int015_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.016 IRQ handler
 **
 ******************************************************************************/
void IRQ016_Handler(void)
{
    if (NULL != IrqHandler[Int016_IRQn])
    {
        IrqHandler[Int016_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.017 IRQ handler
 **
 ******************************************************************************/
void IRQ017_Handler(void)
{
    if (NULL != IrqHandler[Int017_IRQn])
    {
        IrqHandler[Int017_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.018 IRQ handler
 **
 ******************************************************************************/
void IRQ018_Handler(void)
{
    if (NULL != IrqHandler[Int018_IRQn])
    {
        IrqHandler[Int018_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.019 IRQ handler
 **
 ******************************************************************************/
void IRQ019_Handler(void)
{
    if (NULL != IrqHandler[Int019_IRQn])
    {
        IrqHandler[Int019_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.020 IRQ handler
 **
 ******************************************************************************/
void IRQ020_Handler(void)
{
    if (NULL != IrqHandler[Int020_IRQn])
    {
        IrqHandler[Int020_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.021 IRQ handler
 **
 ******************************************************************************/
void IRQ021_Handler(void)
{
    if (NULL != IrqHandler[Int021_IRQn])
    {
        IrqHandler[Int021_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.022 IRQ handler
 **
 ******************************************************************************/
void IRQ022_Handler(void)
{
    if (NULL != IrqHandler[Int022_IRQn])
    {
        IrqHandler[Int022_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.023 IRQ handler
 **
 ******************************************************************************/
void IRQ023_Handler(void)
{
    if (NULL != IrqHandler[Int023_IRQn])
    {
        IrqHandler[Int023_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.024 IRQ handler
 **
 ******************************************************************************/
void IRQ024_Handler(void)
{
    if (NULL != IrqHandler[Int024_IRQn])
    {
        IrqHandler[Int024_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.025 IRQ handler
 **
 ******************************************************************************/
void IRQ025_Handler(void)
{
    if (NULL != IrqHandler[Int025_IRQn])
    {
        IrqHandler[Int025_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.026 IRQ handler
 **
 ******************************************************************************/
void IRQ026_Handler(void)
{
    if (NULL != IrqHandler[Int026_IRQn])
    {
        IrqHandler[Int026_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.027 IRQ handler
 **
 ******************************************************************************/
void IRQ027_Handler(void)
{
    if (NULL != IrqHandler[Int027_IRQn])
    {
        IrqHandler[Int027_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.028 IRQ handler
 **
 ******************************************************************************/
void IRQ028_Handler(void)
{
    if (NULL != IrqHandler[Int028_IRQn])
    {
        IrqHandler[Int028_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.029 IRQ handler
 **
 ******************************************************************************/
void IRQ029_Handler(void)
{
    if (NULL != IrqHandler[Int029_IRQn])
    {
        IrqHandler[Int029_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.030 IRQ handler
 **
 ******************************************************************************/
void IRQ030_Handler(void)
{
    if (NULL != IrqHandler[Int030_IRQn])
    {
        IrqHandler[Int030_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.031 IRQ handler
 **
 ******************************************************************************/
void IRQ031_Handler(void)
{
    if (NULL != IrqHandler[Int031_IRQn])
    {
        IrqHandler[Int031_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.032 IRQ handler
 **
 ******************************************************************************/
void IRQ032_Handler(void)
{
    if (NULL != IrqHandler[Int032_IRQn])
    {
        IrqHandler[Int032_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.033 IRQ handler
 **
 ******************************************************************************/
void IRQ033_Handler(void)
{
    if (NULL != IrqHandler[Int033_IRQn])
    {
        IrqHandler[Int033_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.034 IRQ handler
 **
 ******************************************************************************/
void IRQ034_Handler(void)
{
    if (NULL != IrqHandler[Int034_IRQn])
    {
        IrqHandler[Int034_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.035 IRQ handler
 **
 ******************************************************************************/
void IRQ035_Handler(void)
{
    if (NULL != IrqHandler[Int035_IRQn])
    {
        IrqHandler[Int035_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.036 IRQ handler
 **
 ******************************************************************************/
void IRQ036_Handler(void)
{
    if (NULL != IrqHandler[Int036_IRQn])
    {
        IrqHandler[Int036_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.037 IRQ handler
 **
 ******************************************************************************/
void IRQ037_Handler(void)
{
    if (NULL != IrqHandler[Int037_IRQn])
    {
        IrqHandler[Int037_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.038 IRQ handler
 **
 ******************************************************************************/
void IRQ038_Handler(void)
{
    if (NULL != IrqHandler[Int038_IRQn])
    {
        IrqHandler[Int038_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.039 IRQ handler
 **
 ******************************************************************************/
void IRQ039_Handler(void)
{
    if (NULL != IrqHandler[Int039_IRQn])
    {
        IrqHandler[Int039_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.040 IRQ handler
 **
 ******************************************************************************/
void IRQ040_Handler(void)
{
    if (NULL != IrqHandler[Int040_IRQn])
    {
        IrqHandler[Int040_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.041 IRQ handler
 **
 ******************************************************************************/
void IRQ041_Handler(void)
{
    if (NULL != IrqHandler[Int041_IRQn])
    {
        IrqHandler[Int041_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.042 IRQ handler
 **
 ******************************************************************************/
void IRQ042_Handler(void)
{
    if (NULL != IrqHandler[Int042_IRQn])
    {
        IrqHandler[Int042_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.043 IRQ handler
 **
 ******************************************************************************/
void IRQ043_Handler(void)
{
    if (NULL != IrqHandler[Int043_IRQn])
    {
        IrqHandler[Int043_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.044 IRQ handler
 **
 ******************************************************************************/
void IRQ044_Handler(void)
{
    if (NULL != IrqHandler[Int044_IRQn])
    {
        IrqHandler[Int044_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.045 IRQ handler
 **
 ******************************************************************************/
void IRQ045_Handler(void)
{
    if (NULL != IrqHandler[Int045_IRQn])
    {
        IrqHandler[Int045_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.046 IRQ handler
 **
 ******************************************************************************/
void IRQ046_Handler(void)
{
    if (NULL != IrqHandler[Int046_IRQn])
    {
        IrqHandler[Int046_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.047 IRQ handler
 **
 ******************************************************************************/
void IRQ047_Handler(void)
{
    if (NULL != IrqHandler[Int047_IRQn])
    {
        IrqHandler[Int047_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.048 IRQ handler
 **
 ******************************************************************************/
void IRQ048_Handler(void)
{
    if (NULL != IrqHandler[Int048_IRQn])
    {
        IrqHandler[Int048_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.049 IRQ handler
 **
 ******************************************************************************/
void IRQ049_Handler(void)
{
    if (NULL != IrqHandler[Int049_IRQn])
    {
        IrqHandler[Int049_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.050 IRQ handler
 **
 ******************************************************************************/
void IRQ050_Handler(void)
{
    if (NULL != IrqHandler[Int050_IRQn])
    {
        IrqHandler[Int050_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.051 IRQ handler
 **
 ******************************************************************************/
void IRQ051_Handler(void)
{
    if (NULL != IrqHandler[Int051_IRQn])
    {
        IrqHandler[Int051_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.052 IRQ handler
 **
 ******************************************************************************/
void IRQ052_Handler(void)
{
    if (NULL != IrqHandler[Int052_IRQn])
    {
        IrqHandler[Int052_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.053 IRQ handler
 **
 ******************************************************************************/
void IRQ053_Handler(void)
{
    if (NULL != IrqHandler[Int053_IRQn])
    {
        IrqHandler[Int053_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.054 IRQ handler
 **
 ******************************************************************************/
void IRQ054_Handler(void)
{
    if (NULL != IrqHandler[Int054_IRQn])
    {
        IrqHandler[Int054_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.055 IRQ handler
 **
 ******************************************************************************/
void IRQ055_Handler(void)
{
    if (NULL != IrqHandler[Int055_IRQn])
    {
        IrqHandler[Int055_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.056 IRQ handler
 **
 ******************************************************************************/
void IRQ056_Handler(void)
{
    if (NULL != IrqHandler[Int056_IRQn])
    {
        IrqHandler[Int056_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.057 IRQ handler
 **
 ******************************************************************************/
void IRQ057_Handler(void)
{
    if (NULL != IrqHandler[Int057_IRQn])
    {
        IrqHandler[Int057_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.058 IRQ handler
 **
 ******************************************************************************/
void IRQ058_Handler(void)
{
    if (NULL != IrqHandler[Int058_IRQn])
    {
        IrqHandler[Int058_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.059 IRQ handler
 **
 ******************************************************************************/
void IRQ059_Handler(void)
{
    if (NULL != IrqHandler[Int059_IRQn])
    {
        IrqHandler[Int059_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.060 IRQ handler
 **
 ******************************************************************************/
void IRQ060_Handler(void)
{
    if (NULL != IrqHandler[Int060_IRQn])
    {
        IrqHandler[Int060_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.061 IRQ handler
 **
 ******************************************************************************/
void IRQ061_Handler(void)
{
    if (NULL != IrqHandler[Int061_IRQn])
    {
        IrqHandler[Int061_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.062 IRQ handler
 **
 ******************************************************************************/
void IRQ062_Handler(void)
{
    if (NULL != IrqHandler[Int062_IRQn])
    {
        IrqHandler[Int062_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.063 IRQ handler
 **
 ******************************************************************************/
void IRQ063_Handler(void)
{
    if (NULL != IrqHandler[Int063_IRQn])
    {
        IrqHandler[Int063_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.064 IRQ handler
 **
 ******************************************************************************/
void IRQ064_Handler(void)
{
    if (NULL != IrqHandler[Int064_IRQn])
    {
        IrqHandler[Int064_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.065 IRQ handler
 **
 ******************************************************************************/
void IRQ065_Handler(void)
{
    if (NULL != IrqHandler[Int065_IRQn])
    {
        IrqHandler[Int065_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.066 IRQ handler
 **
 ******************************************************************************/
void IRQ066_Handler(void)
{
    if (NULL != IrqHandler[Int066_IRQn])
    {
        IrqHandler[Int066_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.067 IRQ handler
 **
 ******************************************************************************/
void IRQ067_Handler(void)
{
    if (NULL != IrqHandler[Int067_IRQn])
    {
        IrqHandler[Int067_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.068 IRQ handler
 **
 ******************************************************************************/
void IRQ068_Handler(void)
{
    if (NULL != IrqHandler[Int068_IRQn])
    {
        IrqHandler[Int068_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.069 IRQ handler
 **
 ******************************************************************************/
void IRQ069_Handler(void)
{
    if (NULL != IrqHandler[Int069_IRQn])
    {
        IrqHandler[Int069_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.070 IRQ handler
 **
 ******************************************************************************/
void IRQ070_Handler(void)
{
    if (NULL != IrqHandler[Int070_IRQn])
    {
        IrqHandler[Int070_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.071 IRQ handler
 **
 ******************************************************************************/
void IRQ071_Handler(void)
{
    if (NULL != IrqHandler[Int071_IRQn])
    {
        IrqHandler[Int071_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.072 IRQ handler
 **
 ******************************************************************************/
void IRQ072_Handler(void)
{
    if (NULL != IrqHandler[Int072_IRQn])
    {
        IrqHandler[Int072_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.073 IRQ handler
 **
 ******************************************************************************/
void IRQ073_Handler(void)
{
    if (NULL != IrqHandler[Int073_IRQn])
    {
        IrqHandler[Int073_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.074 IRQ handler
 **
 ******************************************************************************/
void IRQ074_Handler(void)
{
    if (NULL != IrqHandler[Int074_IRQn])
    {
        IrqHandler[Int074_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.075 IRQ handler
 **
 ******************************************************************************/
void IRQ075_Handler(void)
{
    if (NULL != IrqHandler[Int075_IRQn])
    {
        IrqHandler[Int075_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.076 IRQ handler
 **
 ******************************************************************************/
void IRQ076_Handler(void)
{
    if (NULL != IrqHandler[Int076_IRQn])
    {
        IrqHandler[Int076_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.077 IRQ handler
 **
 ******************************************************************************/
void IRQ077_Handler(void)
{
    if (NULL != IrqHandler[Int077_IRQn])
    {
        IrqHandler[Int077_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.078 IRQ handler
 **
 ******************************************************************************/
void IRQ078_Handler(void)
{
    if (NULL != IrqHandler[Int078_IRQn])
    {
        IrqHandler[Int078_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.079 IRQ handler
 **
 ******************************************************************************/
void IRQ079_Handler(void)
{
    if (NULL != IrqHandler[Int079_IRQn])
    {
        IrqHandler[Int079_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.080 IRQ handler
 **
 ******************************************************************************/
void IRQ080_Handler(void)
{
    if (NULL != IrqHandler[Int080_IRQn])
    {
        IrqHandler[Int080_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.081 IRQ handler
 **
 ******************************************************************************/
void IRQ081_Handler(void)
{
    if (NULL != IrqHandler[Int081_IRQn])
    {
        IrqHandler[Int081_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.082 IRQ handler
 **
 ******************************************************************************/
void IRQ082_Handler(void)
{
    if (NULL != IrqHandler[Int082_IRQn])
    {
        IrqHandler[Int082_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.083 IRQ handler
 **
 ******************************************************************************/
void IRQ083_Handler(void)
{
    if (NULL != IrqHandler[Int083_IRQn])
    {
        IrqHandler[Int083_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.084 IRQ handler
 **
 ******************************************************************************/
void IRQ084_Handler(void)
{
    if (NULL != IrqHandler[Int084_IRQn])
    {
        IrqHandler[Int084_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.085 IRQ handler
 **
 ******************************************************************************/
void IRQ085_Handler(void)
{
    if (NULL != IrqHandler[Int085_IRQn])
    {
        IrqHandler[Int085_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.086 IRQ handler
 **
 ******************************************************************************/
void IRQ086_Handler(void)
{
    if (NULL != IrqHandler[Int086_IRQn])
    {
        IrqHandler[Int086_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.087 IRQ handler
 **
 ******************************************************************************/
void IRQ087_Handler(void)
{
    if (NULL != IrqHandler[Int087_IRQn])
    {
        IrqHandler[Int087_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.088 IRQ handler
 **
 ******************************************************************************/
void IRQ088_Handler(void)
{
    if (NULL != IrqHandler[Int088_IRQn])
    {
        IrqHandler[Int088_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.089 IRQ handler
 **
 ******************************************************************************/
void IRQ089_Handler(void)
{
    if (NULL != IrqHandler[Int089_IRQn])
    {
        IrqHandler[Int089_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.090 IRQ handler
 **
 ******************************************************************************/
void IRQ090_Handler(void)
{
    if (NULL != IrqHandler[Int090_IRQn])
    {
        IrqHandler[Int090_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.091 IRQ handler
 **
 ******************************************************************************/
void IRQ091_Handler(void)
{
    if (NULL != IrqHandler[Int091_IRQn])
    {
        IrqHandler[Int091_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.092 IRQ handler
 **
 ******************************************************************************/
void IRQ092_Handler(void)
{
    if (NULL != IrqHandler[Int092_IRQn])
    {
        IrqHandler[Int092_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.093 IRQ handler
 **
 ******************************************************************************/
void IRQ093_Handler(void)
{
    if (NULL != IrqHandler[Int093_IRQn])
    {
        IrqHandler[Int093_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.094 IRQ handler
 **
 ******************************************************************************/
void IRQ094_Handler(void)
{
    if (NULL != IrqHandler[Int094_IRQn])
    {
        IrqHandler[Int094_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.095 IRQ handler
 **
 ******************************************************************************/
void IRQ095_Handler(void)
{
    if (NULL != IrqHandler[Int095_IRQn])
    {
        IrqHandler[Int095_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.096 IRQ handler
 **
 ******************************************************************************/
void IRQ096_Handler(void)
{
    if (NULL != IrqHandler[Int096_IRQn])
    {
        IrqHandler[Int096_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.097 IRQ handler
 **
 ******************************************************************************/
void IRQ097_Handler(void)
{
    if (NULL != IrqHandler[Int097_IRQn])
    {
        IrqHandler[Int097_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.098 IRQ handler
 **
 ******************************************************************************/
void IRQ098_Handler(void)
{
    if (NULL != IrqHandler[Int098_IRQn])
    {
        IrqHandler[Int098_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.099 IRQ handler
 **
 ******************************************************************************/
void IRQ099_Handler(void)
{
    if (NULL != IrqHandler[Int099_IRQn])
    {
        IrqHandler[Int099_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.100 IRQ handler
 **
 ******************************************************************************/
void IRQ100_Handler(void)
{
    if (NULL != IrqHandler[Int100_IRQn])
    {
        IrqHandler[Int100_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.101 IRQ handler
 **
 ******************************************************************************/
void IRQ101_Handler(void)
{
    if (NULL != IrqHandler[Int101_IRQn])
    {
        IrqHandler[Int101_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.102 IRQ handler
 **
 ******************************************************************************/
void IRQ102_Handler(void)
{
    if (NULL != IrqHandler[Int102_IRQn])
    {
        IrqHandler[Int102_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.103 IRQ handler
 **
 ******************************************************************************/
void IRQ103_Handler(void)
{
    if (NULL != IrqHandler[Int103_IRQn])
    {
        IrqHandler[Int103_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.104 IRQ handler
 **
 ******************************************************************************/
void IRQ104_Handler(void)
{
    if (NULL != IrqHandler[Int104_IRQn])
    {
        IrqHandler[Int104_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.105 IRQ handler
 **
 ******************************************************************************/
void IRQ105_Handler(void)
{
    if (NULL != IrqHandler[Int105_IRQn])
    {
        IrqHandler[Int105_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.106 IRQ handler
 **
 ******************************************************************************/
void IRQ106_Handler(void)
{
    if (NULL != IrqHandler[Int106_IRQn])
    {
        IrqHandler[Int106_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.107 IRQ handler
 **
 ******************************************************************************/
void IRQ107_Handler(void)
{
    if (NULL != IrqHandler[Int107_IRQn])
    {
        IrqHandler[Int107_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.108 IRQ handler
 **
 ******************************************************************************/
void IRQ108_Handler(void)
{
    if (NULL != IrqHandler[Int108_IRQn])
    {
        IrqHandler[Int108_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.109 IRQ handler
 **
 ******************************************************************************/
void IRQ109_Handler(void)
{
    if (NULL != IrqHandler[Int109_IRQn])
    {
        IrqHandler[Int109_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.110 IRQ handler
 **
 ******************************************************************************/
void IRQ110_Handler(void)
{
    if (NULL != IrqHandler[Int110_IRQn])
    {
        IrqHandler[Int110_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.111 IRQ handler
 **
 ******************************************************************************/
void IRQ111_Handler(void)
{
    if (NULL != IrqHandler[Int111_IRQn])
    {
        IrqHandler[Int111_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.112 IRQ handler
 **
 ******************************************************************************/
void IRQ112_Handler(void)
{
    if (NULL != IrqHandler[Int112_IRQn])
    {
        IrqHandler[Int112_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.113 IRQ handler
 **
 ******************************************************************************/
void IRQ113_Handler(void)
{
    if (NULL != IrqHandler[Int113_IRQn])
    {
        IrqHandler[Int113_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.114 IRQ handler
 **
 ******************************************************************************/
void IRQ114_Handler(void)
{
    if (NULL != IrqHandler[Int114_IRQn])
    {
        IrqHandler[Int114_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.115 IRQ handler
 **
 ******************************************************************************/
void IRQ115_Handler(void)
{
    if (NULL != IrqHandler[Int115_IRQn])
    {
        IrqHandler[Int115_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.116 IRQ handler
 **
 ******************************************************************************/
void IRQ116_Handler(void)
{
    if (NULL != IrqHandler[Int116_IRQn])
    {
        IrqHandler[Int116_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.117 IRQ handler
 **
 ******************************************************************************/
void IRQ117_Handler(void)
{
    if (NULL != IrqHandler[Int117_IRQn])
    {
        IrqHandler[Int117_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.118 IRQ handler
 **
 ******************************************************************************/
void IRQ118_Handler(void)
{
    if (NULL != IrqHandler[Int118_IRQn])
    {
        IrqHandler[Int118_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.119 IRQ handler
 **
 ******************************************************************************/
void IRQ119_Handler(void)
{
    if (NULL != IrqHandler[Int119_IRQn])
    {
        IrqHandler[Int119_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.120 IRQ handler
 **
 ******************************************************************************/
void IRQ120_Handler(void)
{
    if (NULL != IrqHandler[Int120_IRQn])
    {
        IrqHandler[Int120_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.121 IRQ handler
 **
 ******************************************************************************/
void IRQ121_Handler(void)
{
    if (NULL != IrqHandler[Int121_IRQn])
    {
        IrqHandler[Int121_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.122 IRQ handler
 **
 ******************************************************************************/
void IRQ122_Handler(void)
{
    if (NULL != IrqHandler[Int122_IRQn])
    {
        IrqHandler[Int122_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.123 IRQ handler
 **
 ******************************************************************************/
void IRQ123_Handler(void)
{
    if (NULL != IrqHandler[Int123_IRQn])
    {
        IrqHandler[Int123_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.124 IRQ handler
 **
 ******************************************************************************/
void IRQ124_Handler(void)
{
    if (NULL != IrqHandler[Int124_IRQn])
    {
        IrqHandler[Int124_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.125 IRQ handler
 **
 ******************************************************************************/
void IRQ125_Handler(void)
{
    if (NULL != IrqHandler[Int125_IRQn])
    {
        IrqHandler[Int125_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.126 IRQ handler
 **
 ******************************************************************************/
void IRQ126_Handler(void)
{
    if (NULL != IrqHandler[Int126_IRQn])
    {
        IrqHandler[Int126_IRQn]();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.127 IRQ handler
 **
 ******************************************************************************/
void IRQ127_Handler(void)
{
    if (NULL != IrqHandler[Int127_IRQn])
    {
        IrqHandler[Int127_IRQn]();
    }
}


/**
 *******************************************************************************
 ** \brief Int No.128 share IRQ handler
 **
 ******************************************************************************/
void IRQ128_Handler(void)
{
    /* external interrupt 00 */
    if ((true == M4_INTC->VSSEL128_f.VSEL0) && (true == M4_INTC->EIFR_f.EIFR0))
    {
        Extint00_IrqHandler();
    }
    /* external interrupt 01 */
    if ((true == M4_INTC->VSSEL128_f.VSEL1) && (true == M4_INTC->EIFR_f.EIFR1))
    {
        Extint01_IrqHandler();
    }
    /* external interrupt 02 */
    if ((true == M4_INTC->VSSEL128_f.VSEL2) && (true == M4_INTC->EIFR_f.EIFR2))
    {
        Extint02_IrqHandler();
    }
    /* external interrupt 03 */
    if ((true == M4_INTC->VSSEL128_f.VSEL3) && (true == M4_INTC->EIFR_f.EIFR3))
    {
        Extint03_IrqHandler();
    }
    /* external interrupt 04 */
    if ((true == M4_INTC->VSSEL128_f.VSEL4) && (true == M4_INTC->EIFR_f.EIFR4))
    {
        Extint04_IrqHandler();
    }
    /* external interrupt 05 */
    if ((true == M4_INTC->VSSEL128_f.VSEL5) && (true == M4_INTC->EIFR_f.EIFR5))
    {
        Extint05_IrqHandler();
    }
    /* external interrupt 06 */
    if ((true == M4_INTC->VSSEL128_f.VSEL6) && (true == M4_INTC->EIFR_f.EIFR6))
    {
        Extint06_IrqHandler();
    }
    /* external interrupt 07 */
    if ((true == M4_INTC->VSSEL128_f.VSEL7) && (true == M4_INTC->EIFR_f.EIFR7))
    {
        Extint07_IrqHandler();
    }
    /* external interrupt 08 */
    if ((true == M4_INTC->VSSEL128_f.VSEL8) && (true == M4_INTC->EIFR_f.EIFR8))
    {
        Extint08_IrqHandler();
    }
    /* external interrupt 09 */
    if ((true == M4_INTC->VSSEL128_f.VSEL9) && (true == M4_INTC->EIFR_f.EIFR9))
    {
        Extint09_IrqHandler();
    }
    /* external interrupt 10 */
    if ((true == M4_INTC->VSSEL128_f.VSEL10) && (true == M4_INTC->EIFR_f.EIFR10))
    {
        Extint10_IrqHandler();
    }
    /* external interrupt 11 */
    if ((true == M4_INTC->VSSEL128_f.VSEL11) && (true == M4_INTC->EIFR_f.EIFR11))
    {
        Extint11_IrqHandler();
    }
    /* external interrupt 12 */
    if ((true == M4_INTC->VSSEL128_f.VSEL12) && (true == M4_INTC->EIFR_f.EIFR12))
    {
        Extint12_IrqHandler();
    }
    /* external interrupt 13 */
    if ((true == M4_INTC->VSSEL128_f.VSEL13) && (true == M4_INTC->EIFR_f.EIFR13))
    {
        Extint13_IrqHandler();
    }
    /* external interrupt 14 */
    if ((true == M4_INTC->VSSEL128_f.VSEL14) && (true == M4_INTC->EIFR_f.EIFR14))
    {
        Extint14_IrqHandler();
    }
    /* external interrupt 15 */
    if ((true == M4_INTC->VSSEL128_f.VSEL15) && (true == M4_INTC->EIFR_f.EIFR15))
    {
        Extint15_IrqHandler();
    }
}


/**
 *******************************************************************************
 ** \brief Int No.129 share IRQ handler
 **
 ******************************************************************************/
void IRQ129_Handler(void)
{
    uint8_t u8Tmp = 0;
    /* DMA1 ch.0 Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL0) && (true == !!(M4_DMA1->INTSTAT1_f.TC & 0x01u)))
    {
        Dma1Tc0_IrqHandler();
    }
    /* DMA1 ch.1 Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL1) && (true == !!(M4_DMA1->INTSTAT1_f.TC & 0x02u)))
    {
        Dma1Tc1_IrqHandler();
    }
    /* DMA1 ch.2 Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL2) && (true == !!(M4_DMA1->INTSTAT1_f.TC & 0x04u)))
    {
        Dma1Tc2_IrqHandler();
    }
    /* DMA1 ch.3 Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL3) && (true == !!(M4_DMA1->INTSTAT1_f.TC & 0x08u)))
    {
        Dma1Tc3_IrqHandler();
    }
    /* DMA2 ch.0 Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL4) && (Set == !!(M4_DMA2->INTSTAT1_f.TC & 0x01u)))
    {
        Dma2Tc0_IrqHandler();
    }
    /* DMA2 ch.1 Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL5) && (true == !!(M4_DMA2->INTSTAT1_f.TC & 0x02u)))
    {
        Dma2Tc1_IrqHandler();
    }
    /* DMA2 ch.2 Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL6) && (true == !!(M4_DMA2->INTSTAT1_f.TC & 0x04u)))
    {
        Dma2Tc2_IrqHandler();
    }
    /* DMA2 ch.3 Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL7) && (true == !!(M4_DMA2->INTSTAT1_f.TC & 0x08u)))
    {
        Dma2Tc3_IrqHandler();
    }

    /* DMA1 ch.0 Block Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL8) && (true == !!(M4_DMA1->INTSTAT1_f.BTC & 0x01u)))
    {
        Dma1Btc0_IrqHandler();
    }
    /* DMA1 ch.1 Block Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL9) && (true == !!(M4_DMA1->INTSTAT1_f.BTC & 0x02u)))
    {
        Dma1Btc1_IrqHandler();
    }
    /* DMA1 ch.2 Block Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL10) && (true == !!(M4_DMA1->INTSTAT1_f.BTC & 0x04u)))
    {
        Dma1Btc2_IrqHandler();
    }
    /* DMA1 ch.3 Block Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL11) && (true == !!(M4_DMA1->INTSTAT1_f.BTC & 0x08u)))
    {
        Dma1Btc3_IrqHandler();
    }
    /* DMA2 ch.0 Block Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL12) && (true == !!(M4_DMA2->INTSTAT1_f.BTC & 0x01u)))
    {
        Dma2Btc0_IrqHandler();
    }
    /* DMA2 ch.1 Block Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL13) && (true == !!(M4_DMA2->INTSTAT1_f.BTC & 0x02u)))
    {
        Dma2Btc1_IrqHandler();
    }
    /* DMA2 ch.2 Block Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL14) && (true == !!(M4_DMA2->INTSTAT1_f.BTC & 0x04u)))
    {
        Dma2Btc2_IrqHandler();
    }
    /* DMA2 ch.3 Block Tx completed */
    if ((true == M4_INTC->VSSEL129_f.VSEL15) && (true == !!(M4_DMA2->INTSTAT1_f.BTC & 0x08u)))
    {
        Dma2Btc3_IrqHandler();
    }
    /* DMA1 Error */
    if ((true == M4_INTC->VSSEL129_f.VSEL16) && (true == !!(M4_DMA1->INTSTAT0 & 0x000F000Fu)))
    {
        Dma1Err_IrqHandler();
    }
    /* DMA2 Error */
    if ((true == M4_INTC->VSSEL129_f.VSEL17) && (true == !!(M4_DMA2->INTSTAT0 & 0x000F000Fu)))
    {
        Dma2Err_IrqHandler();
    }
    /* EFM program/erase Error */
    if ((true == M4_INTC->VSSEL129_f.VSEL18) && (true == !!(M4_EFM->FSR & 0x0Fu)))
    {
        EfmPgmEraseErr_IrqHandler();
    }
    /* EFM collision Error */
    if ((true == M4_INTC->VSSEL129_f.VSEL19) && (true == M4_EFM->FSR_f.RDCOLERR))
    {
        EfmColErr_IrqHandler();
    }
    /* EFM operate end */
    if ((true == M4_INTC->VSSEL129_f.VSEL20) && (true == M4_EFM->FSR_f.OPTEND))
    {
        EfmOpEnd_IrqHandler();
    }
    /* USB SOF detected */
    //if ((true == M4_INTC->VSSEL129_f.VSEL21) && (true == !!(xx)))
    //{
    //    UsbSOF_IrqHandler();
    //}
    /* QSPI interrupt */
    if ((true == M4_INTC->VSSEL129_f.VSEL22) && (true == M4_QSPI->SR_f.RAER))
    {
        QspiInt_IrqHandler();
    }
    /* DCU ch.1 */
    u8Tmp = M4_DCU1->INTSEL;
    if ((true == M4_INTC->VSSEL129_f.VSEL23) && (true == !!((M4_DCU1->FLAG & u8Tmp) & 0x7Fu)))
    {
        Dcu1_IrqHandler();
    }
    /* DCU ch.2 */
    u8Tmp = M4_DCU2->INTSEL;
    if ((true == M4_INTC->VSSEL129_f.VSEL24) && (true == !!((M4_DCU2->FLAG & u8Tmp) & 0x7Fu)))
    {
        Dcu2_IrqHandler();
    }
    /* DCU ch.3 */
    u8Tmp = M4_DCU3->INTSEL;
    if ((true == M4_INTC->VSSEL129_f.VSEL25) && (true == !!((M4_DCU3->FLAG & u8Tmp) & 0x7Fu)))
    {
        Dcu3_IrqHandler();
    }
    /* DCU ch.4 */
    u8Tmp = M4_DCU4->INTSEL;
    if ((true == M4_INTC->VSSEL129_f.VSEL26) && (true == !!((M4_DCU4->FLAG & u8Tmp) & 0x7Fu)))
    {
        Dcu4_IrqHandler();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.130 share IRQ handler
 **
 ******************************************************************************/
void IRQ130_Handler(void)
{
    /* Timer0 Ch. 1 A compare match */
    if ((true == M4_INTC->VSSEL130_f.VSEL0) && (true == M4_TMR01->STFLR_f.CMAF))
    {
        Timer01GCMA_IrqHandler();
    }
    /* Timer0 Ch. 1 B compare match */
    if ((true == M4_INTC->VSSEL130_f.VSEL1) && (true == M4_TMR01->STFLR_f.CMBF))
    {
        Timer01GCMB_IrqHandler();
    }
    /* Timer0 Ch. 2 A compare match */
    if ((true == M4_INTC->VSSEL130_f.VSEL2) && (true == M4_TMR02->STFLR_f.CMAF))
    {
        Timer02GCMA_IrqHandler();
    }
    /* Timer0 Ch. 2 B compare match */
    if ((true == M4_INTC->VSSEL130_f.VSEL3) && (true == M4_TMR02->STFLR_f.CMBF))
    {
        Timer02GCMB_IrqHandler();
    }
    /* Main-OSC stop */
    if ((true == M4_INTC->VSSEL130_f.VSEL21) && (true == M4_SYSREG->CMU_XTALSTDSR_f.XTALSTDF))
    {
        MainOscStop_IrqHandler();
    }
    /* Wakeup timer */
    if ((true == M4_INTC->VSSEL130_f.VSEL22) && (true == M4_WKTM->CR_f.WKOVF))
    {
        WakeupTimer_IrqHandler();
    }
    /* SWDT */
    if ((true == M4_INTC->VSSEL130_f.VSEL23) && (true == !!(M4_SWDT->SR & 0x00030000u)))
    {
        Swdt_IrqHandler();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.131 share IRQ handler
 **
 ******************************************************************************/
void IRQ131_Handler(void)
{
    /* Timer6 Ch.1 A compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL0) && (true == M4_TMR61->STFLR_f.CMAF))
    {
        Timer61GCMA_IrqHandler();
    }
    /* Timer6 Ch.1 B compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL1) && (true == M4_TMR61->STFLR_f.CMBF))
    {
        Timer61GCMB_IrqHandler();
    }
    /* Timer6 Ch.1 C compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL2) && (true == M4_TMR61->STFLR_f.CMCF))
    {
        Timer61GCMC_IrqHandler();
    }
    /* Timer6 Ch.1 D compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL3) && (true == M4_TMR61->STFLR_f.CMDF))
    {
        Timer61GCMD_IrqHandler();
    }
    /* Timer6 Ch.1 E compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL4) && (true == M4_TMR61->STFLR_f.CMEF))
    {
        Timer61GCME_IrqHandler();
    }
    /* Timer6 Ch.1 F compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL5) && (true == M4_TMR61->STFLR_f.CMFF))
    {
        Timer61GCMF_IrqHandler();
    }
    /* Timer6 Ch.1 overflow */
    if ((true == M4_INTC->VSSEL131_f.VSEL6) && (true == M4_TMR61->STFLR_f.OVFF))
    {
        Timer61GOV_IrqHandler();
    }
    /* Timer6 Ch.1 underflow */
    if ((true == M4_INTC->VSSEL131_f.VSEL7) && (true == M4_TMR61->STFLR_f.UDFF))
    {
        Timer61GUD_IrqHandler();
    }
    /* Timer6 Ch.1 dead time */
    if ((true == M4_INTC->VSSEL131_f.VSEL8) && (true == M4_TMR61->STFLR_f.DTEF))
    {
        Timer61GDT_IrqHandler();
    }
    /* Timer6 Ch.1 A up-down compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL11) && (true == !!(M4_TMR61->STFLR & 0x00000600u)))
    {
        Timer61SCMA_IrqHandler();
    }
    /* Timer6 Ch.1 B up-down compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL12) && (true == !!(M4_TMR61->STFLR & 0x00001800u)))
    {
        Timer61SCMB_IrqHandler();
    }
    /* Timer6 Ch.2 A compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL16) && (true == M4_TMR62->STFLR_f.CMAF))
    {
        Timer62GCMA_IrqHandler();
    }
    /* Timer6 Ch.2 B compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL17) && (true == M4_TMR62->STFLR_f.CMBF))
    {
        Timer62GCMB_IrqHandler();
    }
    /* Timer6 Ch.2 C compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL18) && (true == M4_TMR62->STFLR_f.CMCF))
    {
        Timer62GCMC_IrqHandler();
    }
    /* Timer6 Ch.2 D compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL19) && (true == M4_TMR62->STFLR_f.CMDF))
    {
        Timer62GCMD_IrqHandler();
    }
    /* Timer6 Ch.2 E compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL20) && (true == M4_TMR62->STFLR_f.CMEF))
    {
        Timer62GCME_IrqHandler();
    }
    /* Timer6 Ch.2 F compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL21) && (true == M4_TMR62->STFLR_f.CMFF))
    {
        Timer62GCMF_IrqHandler();
    }
    /* Timer6 Ch.2 overflow */
    if ((true == M4_INTC->VSSEL131_f.VSEL22) && (true == M4_TMR62->STFLR_f.OVFF))
    {
        Timer62GOV_IrqHandler();
    }
    /* Timer6 Ch.2 underflow */
    if ((true == M4_INTC->VSSEL131_f.VSEL23) && (true == M4_TMR62->STFLR_f.UDFF))
    {
        Timer62GUD_IrqHandler();
    }
    /* Timer6 Ch.2 dead time */
    if ((true == M4_INTC->VSSEL131_f.VSEL24) && (true == M4_TMR62->STFLR_f.DTEF))
    {
        Timer62GDT_IrqHandler();
    }
    /* Timer6 Ch.2 A up-down compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL27) && (true == !!(M4_TMR62->STFLR & 0x00000600u)))
    {
        Timer62SCMA_IrqHandler();
    }
    /* Timer6 Ch.2 B up-down compare match */
    if ((true == M4_INTC->VSSEL131_f.VSEL28) && (true == !!(M4_TMR62->STFLR & 0x00001800u)))
    {
        Timer62SCMB_IrqHandler();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.132 share IRQ handler
 **
 ******************************************************************************/
void IRQ132_Handler(void)
{
    /* Timer6 Ch.3 A compare match */
    if ((true == M4_INTC->VSSEL132_f.VSEL0) && (true == M4_TMR63->STFLR_f.CMAF))
    {
        Timer63GCMA_IrqHandler();
    }
    /* Timer6 Ch.3 B compare match */
    if ((true == M4_INTC->VSSEL132_f.VSEL1) && (true == M4_TMR63->STFLR_f.CMBF))
    {
        Timer63GCMB_IrqHandler();
    }
    /* Timer6 Ch.3 C compare match */
    if ((true == M4_INTC->VSSEL132_f.VSEL2) && (true == M4_TMR63->STFLR_f.CMCF))
    {
        Timer63GCMC_IrqHandler();
    }
    /* Timer6 Ch.3 D compare match */
    if ((true == M4_INTC->VSSEL132_f.VSEL3) && (true == M4_TMR63->STFLR_f.CMDF))
    {
        Timer63GCMD_IrqHandler();
    }
    /* Timer6 Ch.3 E compare match */
    if ((true == M4_INTC->VSSEL132_f.VSEL4) && (true == M4_TMR63->STFLR_f.CMEF))
    {
        Timer63GCME_IrqHandler();
    }
    /* Timer6 Ch.3 F compare match */
    if ((true == M4_INTC->VSSEL132_f.VSEL5) && (true == M4_TMR63->STFLR_f.CMFF))
    {
        Timer63GCMF_IrqHandler();
    }
    /* Timer6 Ch.3 overflow */
    if ((true == M4_INTC->VSSEL132_f.VSEL6) && (true == M4_TMR63->STFLR_f.OVFF))
    {
        Timer63GOV_IrqHandler();
    }
    /* Timer6 Ch.3 underflow */
    if ((true == M4_INTC->VSSEL132_f.VSEL7) && (true == M4_TMR63->STFLR_f.UDFF))
    {
        Timer63GUD_IrqHandler();
    }
    /* Timer6 Ch.3 dead time */
    if ((true == M4_INTC->VSSEL132_f.VSEL8) && (true == M4_TMR63->STFLR_f.DTEF))
    {
        Timer63GDT_IrqHandler();
    }
    /* Timer6 Ch.3 A up-down compare match */
    if ((true == M4_INTC->VSSEL132_f.VSEL11) && (true == !!(M4_TMR63->STFLR & 0x00000600u)))
    {
        Timer63SCMA_IrqHandler();
    }
    /* Timer6 Ch.3 B up-down compare match */
    if ((true == M4_INTC->VSSEL132_f.VSEL12) && (true == !!(M4_TMR63->STFLR & 0x00001800u)))
    {
        Timer63SCMB_IrqHandler();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.136 share IRQ handler
 **
 ******************************************************************************/
void IRQ136_Handler(void)
{
    /* TiemrA Ch.1 overflow */
    if ((true == M4_INTC->VSSEL136_f.VSEL0) && (true == M4_TMRA1->BCSTR_f.OVFF))
    {
        TimerA1OV_IrqHandler();
    }
    /* TiemrA Ch.1 underflow */
    if ((true == M4_INTC->VSSEL136_f.VSEL1) && (true == M4_TMRA1->BCSTR_f.UDFF))
    {
        TimerA1OV_IrqHandler();
    }
    /* TiemrA Ch.1 compare match */
    if ((true == M4_INTC->VSSEL136_f.VSEL2) && (true == !!(M4_TMRA1->STFLR & 0xFFu)))
    {
        TimerA1CMP_IrqHandler();
    }
    /* TiemrA Ch.2 overflow */
    if ((true == M4_INTC->VSSEL136_f.VSEL3) && (true == M4_TMRA2->BCSTR_f.OVFF))
    {
        TimerA2OV_IrqHandler();
    }
    /* TiemrA Ch.2 underflow */
    if ((true == M4_INTC->VSSEL136_f.VSEL4) && (true == M4_TMRA2->BCSTR_f.UDFF))
    {
        TimerA2OV_IrqHandler();
    }
    /* TiemrA Ch.2 compare match */
    if ((true == M4_INTC->VSSEL136_f.VSEL5) && (true == !!(M4_TMRA2->STFLR & 0xFFu)))
    {
        TimerA2CMP_IrqHandler();
    }
    /* TiemrA Ch.3 overflow */
    if ((true == M4_INTC->VSSEL136_f.VSEL6) && (true == M4_TMRA3->BCSTR_f.OVFF))
    {
        TimerA3OV_IrqHandler();
    }
    /* TiemrA Ch.3 underflow */
    if ((true == M4_INTC->VSSEL136_f.VSEL7) && (true == M4_TMRA3->BCSTR_f.UDFF))
    {
        TimerA3OV_IrqHandler();
    }
    /* TiemrA Ch.3 compare match */
    if ((true == M4_INTC->VSSEL136_f.VSEL8) && (true == !!(M4_TMRA3->STFLR & 0xFFu)))
    {
        TimerA3CMP_IrqHandler();
    }
    /* TiemrA Ch.4 overflow */
    if ((true == M4_INTC->VSSEL136_f.VSEL9) && (true == M4_TMRA4->BCSTR_f.OVFF))
    {
        TimerA4OV_IrqHandler();
    }
    /* TiemrA Ch.4 underflow */
    if ((true == M4_INTC->VSSEL136_f.VSEL10) && (true == M4_TMRA4->BCSTR_f.UDFF))
    {
        TimerA4OV_IrqHandler();
    }
    /* TiemrA Ch.4 compare match */
    if ((true == M4_INTC->VSSEL136_f.VSEL11) && (true == !!(M4_TMRA4->STFLR & 0xFFu)))
    {
        TimerA4CMP_IrqHandler();
    }
    /* TiemrA Ch.5 overflow */
    if ((true == M4_INTC->VSSEL136_f.VSEL12) && (true == M4_TMRA5->BCSTR_f.OVFF))
    {
        TimerA5OV_IrqHandler();
    }
    /* TiemrA Ch.5 underflow */
    if ((true == M4_INTC->VSSEL136_f.VSEL13) && (true == M4_TMRA5->BCSTR_f.UDFF))
    {
        TimerA5OV_IrqHandler();
    }
    /* TiemrA Ch.5 compare match */
    if ((true == M4_INTC->VSSEL136_f.VSEL14) && (true == !!(M4_TMRA5->STFLR & 0xFFu)))
    {
        TimerA5CMP_IrqHandler();
    }
    /* TiemrA Ch.6 overflow */
    if ((true == M4_INTC->VSSEL136_f.VSEL16) && (true == M4_TMRA6->BCSTR_f.OVFF))
    {
        TimerA6OV_IrqHandler();
    }
    /* TiemrA Ch.6 underflow */
    if ((true == M4_INTC->VSSEL136_f.VSEL17) && (true == M4_TMRA6->BCSTR_f.UDFF))
    {
        TimerA6OV_IrqHandler();
    }
    /* TiemrA Ch.6 compare match */
    if ((true == M4_INTC->VSSEL136_f.VSEL18) && (true == !!(M4_TMRA6->STFLR & 0xFFu)))
    {
        TimerA6CMP_IrqHandler();
    }
    /* USBFS global interrupt */
    if ((true == M4_INTC->VSSEL136_f.VSEL19) && (true == !!(M4_USBFS->GINTSTS & 0xF77CFCFBu)))
    {
        UsbGlobal_IrqHandler();
    }
    /* USART Ch.1 Receive error */
    if ((true == M4_INTC->VSSEL136_f.VSEL22) && (true == !!(M4_USART1->SR & 0x0Bu)))
    {
        Usart1RxErr_IrqHandler();
    }
    /* USART Ch.1 Receive completed */
    if ((true == M4_INTC->VSSEL136_f.VSEL23) && (true == M4_USART1->SR_f.RXNE))
    {
        Usart1RxEnd_IrqHandler();
    }
    /* USART Ch.1 Transmit data empty */
    if ((true == M4_INTC->VSSEL136_f.VSEL24) && (true == M4_USART1->SR_f.TXE))
    {
        Usart1TxEmpty_IrqHandler();
    }
    /* USART Ch.1 Transmit completed */
    if ((true == M4_INTC->VSSEL136_f.VSEL25) && (true == M4_USART1->SR_f.TC))
    {
        Usart1TxEnd_IrqHandler();
    }
    /* USART Ch.1 Receive timeout */
    if ((true == M4_INTC->VSSEL136_f.VSEL26) && (true == M4_USART1->SR_f.RTOF))
    {
        Usart1RxTO_IrqHandler();
    }
    /* USART Ch.2 Receive error */
    if ((true == M4_INTC->VSSEL136_f.VSEL27) && (true == !!(M4_USART2->SR & 0x0Bu)))
    {
        Usart2RxErr_IrqHandler();
    }
    /* USART Ch.2 Receive completed */
    if ((true == M4_INTC->VSSEL136_f.VSEL28) && (true == M4_USART2->SR_f.RXNE))
    {
        Usart2RxEnd_IrqHandler();
    }
    /* USART Ch.2 Transmit data empty */
    if ((true == M4_INTC->VSSEL136_f.VSEL29) && (true == M4_USART2->SR_f.TXE))
    {
        Usart2TxEmpty_IrqHandler();
    }
    /* USART Ch.2 Transmit completed */
    if ((true == M4_INTC->VSSEL136_f.VSEL30) && (true == M4_USART2->SR_f.TC))
    {
        Usart2TxEnd_IrqHandler();
    }
    /* USART Ch.2 Receive timeout */
    if ((true == M4_INTC->VSSEL136_f.VSEL31) && (true == M4_USART2->SR_f.RTOF))
    {
        Usart2RxTO_IrqHandler();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.137 share IRQ handler
 **
 ******************************************************************************/
void IRQ137_Handler(void)
{
    /* USART Ch.3 Receive error */
    if ((true == M4_INTC->VSSEL137_f.VSEL0) && (true == !!(M4_USART3->SR & 0x0Bu)))
    {
        Usart3RxErr_IrqHandler();
    }
    /* USART Ch.3 Receive completed */
    if ((true == M4_INTC->VSSEL137_f.VSEL1) && (true == M4_USART3->SR_f.RXNE))
    {
        Usart3RxEnd_IrqHandler();
    }
    /* USART Ch.3 Transmit data empty */
    if ((true == M4_INTC->VSSEL137_f.VSEL2) && (true == M4_USART3->SR_f.TXE))
    {
        Usart3TxEmpty_IrqHandler();
    }
    /* USART Ch.3 Transmit completed */
    if ((true == M4_INTC->VSSEL137_f.VSEL3) && (true == M4_USART3->SR_f.TC))
    {
        Usart3TxEnd_IrqHandler();
    }
    /* USART Ch.3 Receive timeout */
    if ((true == M4_INTC->VSSEL137_f.VSEL4) && (true == M4_USART3->SR_f.RTOF))
    {
        Usart3RxTO_IrqHandler();
    }
    /* USART Ch.4 Receive error */
    if ((true == M4_INTC->VSSEL137_f.VSEL5) && (true == !!(M4_USART4->SR & 0x0Bu)))
    {
        Usart4RxErr_IrqHandler();
    }
    /* USART Ch.4 Receive completed */
    if ((true == M4_INTC->VSSEL137_f.VSEL6) && (true == M4_USART4->SR_f.RXNE))
    {
        Usart4RxEnd_IrqHandler();
    }
    /* USART Ch.4 Transmit data empty */
    if ((true == M4_INTC->VSSEL137_f.VSEL7) && (true == M4_USART4->SR_f.TXE))
    {
        Usart4TxEmpty_IrqHandler();
    }
    /* USART Ch.4 Transmit completed */
    if ((true == M4_INTC->VSSEL137_f.VSEL8) && (true == M4_USART4->SR_f.TC))
    {
        Usart4TxEnd_IrqHandler();
    }
    /* USART Ch.4 Receive timeout */
    if ((true == M4_INTC->VSSEL137_f.VSEL9) && (true == M4_USART4->SR_f.RTOF))
    {
        Usart4RxTO_IrqHandler();
    }
    /* SPI Ch.1 Receive completed */
    if ((true == M4_INTC->VSSEL137_f.VSEL11) && (true == M4_SPI1->SR_f.RDFF))
    {
        Spi1RxEnd_IrqHandler();
    }
    /* SPI Ch.1 Receive completed */
    if ((true == M4_INTC->VSSEL137_f.VSEL12) && (true == M4_SPI1->SR_f.TDEF))
    {
        Spi1TxEmpty_IrqHandler();
    }
    /* SPI Ch.1 bus idle */
    if ((true == M4_INTC->VSSEL137_f.VSEL13) && (true == M4_SPI1->SR_f.IDLNF))
    {
        Spi1Idle_IrqHandler();
    }
    /* SPI Ch.1 parity/overflow/underflow/mode error */
    if ((true == M4_INTC->VSSEL137_f.VSEL14) && (true == !!(M4_SPI1->SR & 0x1Du)))
    {
        Spi1Err_IrqHandler();
    }
    /* SPI Ch.2 Receive completed */
    if ((true == M4_INTC->VSSEL137_f.VSEL16) && (true == M4_SPI2->SR_f.RDFF))
    {
        Spi2RxEnd_IrqHandler();
    }
    /* SPI Ch.2 Transmit completed */
    if ((true == M4_INTC->VSSEL137_f.VSEL17) && (true == M4_SPI2->SR_f.TDEF))
    {
        Spi2TxEmpty_IrqHandler();
    }
    /* SPI Ch.2 bus idle */
    if ((true == M4_INTC->VSSEL137_f.VSEL18) && (true == M4_SPI2->SR_f.IDLNF))
    {
        Spi2Idle_IrqHandler();
    }
    /* SPI Ch.2 parity/overflow/underflow/mode error */
    if ((true == M4_INTC->VSSEL137_f.VSEL19) && (true == !!(M4_SPI2->SR & 0x1Du)))
    {
        Spi2Err_IrqHandler();
    }
    /* SPI Ch.3 Receive completed */
    if ((true == M4_INTC->VSSEL137_f.VSEL21) && (true == M4_SPI3->SR_f.RDFF))
    {
        Spi3RxEnd_IrqHandler();
    }
    /* SPI Ch.3 Transmit completed */
    if ((true == M4_INTC->VSSEL137_f.VSEL22) && (true == M4_SPI3->SR_f.TDEF))
    {
        Spi3TxEmpty_IrqHandler();
    }
    /* SPI Ch.3 bus idle */
    if ((true == M4_INTC->VSSEL137_f.VSEL23) && (true == M4_SPI3->SR_f.IDLNF))
    {
        Spi3Idle_IrqHandler();
    }
    /* SPI Ch.3 parity/overflow/underflow/mode error */
    if ((true == M4_INTC->VSSEL137_f.VSEL24) && (true == !!(M4_SPI3->SR & 0x1Du)))
    {
        Spi3Err_IrqHandler();
    }
    /* SPI Ch.4 Receive completed */
    if ((true == M4_INTC->VSSEL137_f.VSEL26) && (true == M4_SPI4->SR_f.RDFF))
    {
        Spi4RxEnd_IrqHandler();
    }
    /* SPI Ch.4 Transmit completed */
    if ((true == M4_INTC->VSSEL137_f.VSEL27) && (true == M4_SPI4->SR_f.TDEF))
    {
        Spi4TxEmpty_IrqHandler();
    }
    /* SPI Ch.4 bus idle */
    if ((true == M4_INTC->VSSEL137_f.VSEL28) && (true == M4_SPI4->SR_f.IDLNF))
    {
        Spi4Idle_IrqHandler();
    }
    /* SPI Ch.4 parity/overflow/underflow/mode error */
    if ((true == M4_INTC->VSSEL137_f.VSEL29) && (true == !!(M4_SPI4->SR & 0x1Du)))
    {
        Spi4Err_IrqHandler();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.138 share IRQ handler
 **
 ******************************************************************************/
void IRQ138_Handler(void)
{
    /* Timer4 Ch.1 U phase higher compare match */
    if ((true == M4_INTC->VSSEL138_f.VSEL0) && (true == M4_TMR41->OCSRU_f.OCFHU))
    {
        Timer41GCMUH_IrqHandler();
    }
    /* Timer4 Ch.1 U phase lower compare match */
    if ((true == M4_INTC->VSSEL138_f.VSEL1) && (true == M4_TMR41->OCSRU_f.OCFLU))
    {
        Timer41GCMUL_IrqHandler();
    }
    /* Timer4 Ch.1 V phase higher compare match */
    if ((true == M4_INTC->VSSEL138_f.VSEL2) && (true == M4_TMR41->OCSRV_f.OCFHV))
    {
        Timer41GCMVH_IrqHandler();
    }
    /* Timer4 Ch.1 V phase lower compare match */
    if ((true == M4_INTC->VSSEL138_f.VSEL3) && (true == M4_TMR41->OCSRV_f.OCFLV))
    {
        Timer41GCMVL_IrqHandler();
    }
    /* Timer4 Ch.1 W phase higher compare match */
    if ((true == M4_INTC->VSSEL138_f.VSEL4) && (true == M4_TMR41->OCSRW_f.OCFHW))
    {
        Timer41GCMWH_IrqHandler();
    }
    /* Timer4 Ch.1 W phase lower compare match */
    if ((true == M4_INTC->VSSEL138_f.VSEL5) && (true == M4_TMR41->OCSRW_f.OCFLW))
    {
        Timer41GCMWL_IrqHandler();
    }
    /* Timer4 Ch.1 overflow */
    if ((true == M4_INTC->VSSEL138_f.VSEL6) && (true == M4_TMR41->CCSR_f.IRQPF))
    {
        Timer41GOV_IrqHandler();
    }
    /* Timer4 Ch.1 underflow */
    if ((true == M4_INTC->VSSEL138_f.VSEL7) && (true == M4_TMR41->CCSR_f.IRQZF))
    {
        Timer41GUD_IrqHandler();
    }
    /* Timer4 Ch.1 U phase reload */
    if ((true == M4_INTC->VSSEL138_f.VSEL8) && (true == M4_TMR41->RCSR_f.RTIFU))
    {
        Timer41ReloadU_IrqHandler();
    }
    /* Timer4 Ch.1 V phase reload */
    if ((true == M4_INTC->VSSEL138_f.VSEL9) && (true == M4_TMR41->RCSR_f.RTIFV))
    {
        Timer41ReloadV_IrqHandler();
    }
    /* Timer4 Ch.1 W phase reload */
    if ((true == M4_INTC->VSSEL138_f.VSEL10) && (true == M4_TMR41->RCSR_f.RTIFW))
    {
        Timer41ReloadW_IrqHandler();
    }
    /* Timer4 Ch.2 U phase higher compare match */
    if ((true == M4_INTC->VSSEL138_f.VSEL16) && (true == M4_TMR42->OCSRU_f.OCFHU))
    {
        Timer42GCMUH_IrqHandler();
    }
    /* Timer4 Ch.2 U phase lower compare match */
    if ((true == M4_INTC->VSSEL138_f.VSEL17) && (true == M4_TMR42->OCSRU_f.OCFLU))
    {
        Timer42GCMUL_IrqHandler();
    }
    /* Timer4 Ch.2 V phase higher compare match */
    if ((true == M4_INTC->VSSEL138_f.VSEL18) && (true == M4_TMR42->OCSRV_f.OCFHV))
    {
        Timer42GCMVH_IrqHandler();
    }
    /* Timer4 Ch.2 V phase lower compare match */
    if ((true == M4_INTC->VSSEL138_f.VSEL19) && (true == M4_TMR42->OCSRV_f.OCFLV))
    {
        Timer42GCMVL_IrqHandler();
    }
    /* Timer4 Ch.2 W phase higher compare match */
    if ((true == M4_INTC->VSSEL138_f.VSEL20) && (true == M4_TMR42->OCSRW_f.OCFHW))
    {
        Timer42GCMWH_IrqHandler();
    }
    /* Timer4 Ch.2 W phase lower compare match */
    if ((true == M4_INTC->VSSEL138_f.VSEL21) && (true == M4_TMR42->OCSRW_f.OCFLW))
    {
        Timer42GCMWL_IrqHandler();
    }
    /* Timer4 Ch.2 overflow */
    if ((true == M4_INTC->VSSEL138_f.VSEL22) && (true == M4_TMR42->CCSR_f.IRQPF))
    {
        Timer42GOV_IrqHandler();
    }
    /* Timer4 Ch.2 underflow */
    if ((true == M4_INTC->VSSEL138_f.VSEL23) && (true == M4_TMR42->CCSR_f.IRQZF))
    {
        Timer42GUD_IrqHandler();
    }
    /* Timer4 Ch.2 U phase reload */
    if ((true == M4_INTC->VSSEL138_f.VSEL24) && (true == M4_TMR42->RCSR_f.RTIFU))
    {
        Timer42ReloadU_IrqHandler();
    }
    /* Timer4 Ch.2 V phase reload */
    if ((true == M4_INTC->VSSEL138_f.VSEL25) && (true == M4_TMR42->RCSR_f.RTIFV))
    {
        Timer42ReloadV_IrqHandler();
    }
    /* Timer4 Ch.2 W phase reload */
    if ((true == M4_INTC->VSSEL138_f.VSEL26) && (true == M4_TMR42->RCSR_f.RTIFW))
    {
        Timer42ReloadW_IrqHandler();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.139 share IRQ handler
 **
 ******************************************************************************/
void IRQ139_Handler(void)
{
    /* Timer4 Ch.3 U phase higher compare match */
    if ((true == M4_INTC->VSSEL139_f.VSEL0) && (true == M4_TMR43->OCSRU_f.OCFHU))
    {
        Timer43GCMUH_IrqHandler();
    }
    /* Timer4 Ch.3 U phase lower compare match */
    if ((true == M4_INTC->VSSEL139_f.VSEL1) && (true == M4_TMR43->OCSRU_f.OCFLU))
    {
        Timer43GCMUL_IrqHandler();
    }
    /* Timer4 Ch.3 V phase higher compare match */
    if ((true == M4_INTC->VSSEL139_f.VSEL2) && (true == M4_TMR43->OCSRV_f.OCFHV))
    {
        Timer43GCMVH_IrqHandler();
    }
    /* Timer4 Ch.3 V phase lower compare match */
    if ((true == M4_INTC->VSSEL139_f.VSEL3) && (true == M4_TMR43->OCSRV_f.OCFLV))
    {
        Timer43GCMVL_IrqHandler();
    }
    /* Timer4 Ch.3 W phase higher compare match */
    if ((true == M4_INTC->VSSEL139_f.VSEL4) && (true == M4_TMR43->OCSRW_f.OCFHW))
    {
        Timer43GCMWH_IrqHandler();
    }
    /* Timer4 Ch.3 W phase lower compare match */
    if ((true == M4_INTC->VSSEL139_f.VSEL5) && (true == M4_TMR43->OCSRW_f.OCFLW))
    {
        Timer43GCMWL_IrqHandler();
    }
    /* Timer4 Ch.3 overflow */
    if ((true == M4_INTC->VSSEL139_f.VSEL6) && (true == M4_TMR43->CCSR_f.IRQPF))
    {
        Timer43GOV_IrqHandler();
    }
    /* Timer4 Ch.3 underflow */
    if ((true == M4_INTC->VSSEL139_f.VSEL7) && (true == M4_TMR43->CCSR_f.IRQZF))
    {
        Timer43GUD_IrqHandler();
    }
    /* Timer4 Ch.3 U phase reload */
    if ((true == M4_INTC->VSSEL139_f.VSEL8) && (true == M4_TMR43->RCSR_f.RTIFU))
    {
        Timer43ReloadU_IrqHandler();
    }
    /* Timer4 Ch.3 V phase reload */
    if ((true == M4_INTC->VSSEL139_f.VSEL9) && (true == M4_TMR43->RCSR_f.RTIFV))
    {
        Timer43ReloadV_IrqHandler();
    }
    /* Timer4 Ch.3 W phase reload */
    if ((true == M4_INTC->VSSEL139_f.VSEL10) && (true == M4_TMR43->RCSR_f.RTIFW))
    {
        Timer43ReloadW_IrqHandler();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.140 share IRQ handler
 **
 ******************************************************************************/
void IRQ140_Handler(void)
{
    /* EMB0 */
    if ((true == M4_INTC->VSSEL140_f.VSEL6) && (true == !!(M4_EMB1->STAT & 0x0Fu)))
    {
        Emb0_IrqHandler();
    }
    /* EMB1 */
    if ((true == M4_INTC->VSSEL140_f.VSEL7) && (true == !!(M4_EMB2->STAT & 0x0Fu)))
    {
        Emb1_IrqHandler();
    }
    /* EMB2 */
    if ((true == M4_INTC->VSSEL140_f.VSEL8) && (true == !!(M4_EMB3->STAT & 0x0Fu)))
    {
        Emb2_IrqHandler();
    }
    /* EMB3 */
    if ((true == M4_INTC->VSSEL140_f.VSEL9) && (true == !!(M4_EMB4->STAT & 0x0Fu)))
    {
        Emb3_IrqHandler();
    }
    /* I2S Ch.1 Transmit */
    if ((true == M4_INTC->VSSEL140_f.VSEL16) && (true == M4_I2S1->SR_f.TXBA))
    {
        I2s1Tx_IrqHandler();
    }
    /* I2S Ch.1 Receive */
    if ((true == M4_INTC->VSSEL140_f.VSEL17) && (true == M4_I2S1->SR_f.RXBA))
    {
        I2s1Rx_IrqHandler();
    }
    /* I2S Ch.1 Error */
    if ((true == M4_INTC->VSSEL140_f.VSEL18) && (true == !!(M4_I2S1->ER & 0x3u)))
    {
        I2s1Err_IrqHandler();
    }
    /* I2S Ch.2 Transmit */
    if ((true == M4_INTC->VSSEL140_f.VSEL19) && (true == M4_I2S2->SR_f.TXBA))
    {
        I2s2Tx_IrqHandler();
    }
    /* I2S Ch.2 Receive */
    if ((true == M4_INTC->VSSEL140_f.VSEL20) && (true == M4_I2S2->SR_f.RXBA))
    {
        I2s2Rx_IrqHandler();
    }
    /* I2S Ch.2 Error */
    if ((true == M4_INTC->VSSEL140_f.VSEL21) && (true == !!(M4_I2S2->ER & 0x3u)))
    {
        I2s2Err_IrqHandler();
    }
    /* I2S Ch.3 Transmit */
    if ((true == M4_INTC->VSSEL140_f.VSEL22) && (true == M4_I2S3->SR_f.TXBA))
    {
        I2s3Tx_IrqHandler();
    }
    /* I2S Ch.3 Receive */
    if ((true == M4_INTC->VSSEL140_f.VSEL23) && (true == M4_I2S3->SR_f.RXBA))
    {
        I2s3Rx_IrqHandler();
    }
    /* I2S Ch.3 Error */
    if ((true == M4_INTC->VSSEL140_f.VSEL24) && (true == !!(M4_I2S3->ER & 0x3u)))
    {
        I2s3Err_IrqHandler();
    }
    /* I2S Ch.4 Transmit */
    if ((true == M4_INTC->VSSEL140_f.VSEL25) && (true == M4_I2S4->SR_f.TXBA))
    {
        I2s4Tx_IrqHandler();
    }
    /* I2S Ch.4 Receive */
    if ((true == M4_INTC->VSSEL140_f.VSEL26) && (true == M4_I2S4->SR_f.RXBA))
    {
        I2s4Rx_IrqHandler();
    }
    /* I2S Ch.4 Error */
    if ((true == M4_INTC->VSSEL140_f.VSEL27) && (true == !!(M4_I2S4->ER & 0x3u)))
    {
        I2s4Err_IrqHandler();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.141 share IRQ handler
 **
 ******************************************************************************/
void IRQ141_Handler(void)
{
    /* I2C Ch.1 Receive completed */
    if ((true == M4_INTC->VSSEL141_f.VSEL4) && (true == !!(M4_I2C1->SR_f.RFULLF)))
    {
        I2c1RxEnd_IrqHandler();
    }
    /* I2C Ch.1 Transmit completed */
    if ((true == M4_INTC->VSSEL141_f.VSEL5) && (true == !!(M4_I2C1->SR_f.TENDF)))
    {
        I2c1TxEnd_IrqHandler();
    }
    /* I2C Ch.1 Transmit data empty */
    if ((true == M4_INTC->VSSEL141_f.VSEL6) && (true == !!(M4_I2C1->SR_f.TEMPTYF)))
    {
        I2c1TxEmpty_IrqHandler();
    }
    /* I2C Ch.1 Error */
    if ((true == M4_INTC->VSSEL141_f.VSEL7) && (true == !!(M4_I2C1->SR & 0x00F05217u)))
    {
        I2c1Err_IrqHandler();
    }
    /* I2C Ch.2 Receive completed */
    if ((true == M4_INTC->VSSEL141_f.VSEL8) && (true == !!(M4_I2C2->SR_f.RFULLF)))
    {
        I2c2RxEnd_IrqHandler();
    }
    /* I2C Ch.2 Transmit completed */
    if ((true == M4_INTC->VSSEL141_f.VSEL9) && (true == !!(M4_I2C2->SR_f.TENDF)))
    {
        I2c2TxEnd_IrqHandler();
    }
    /* I2C Ch.2 Transmit data empty */
    if ((true == M4_INTC->VSSEL141_f.VSEL10) && (true == !!(M4_I2C2->SR_f.TEMPTYF)))
    {
        I2c2TxEmpty_IrqHandler();
    }
    /* I2C Ch.2 Error */
    if ((true == M4_INTC->VSSEL141_f.VSEL11) && (true == !!(M4_I2C2->SR & 0x00F05217u)))
    {
        I2c2Err_IrqHandler();
    }
    /* I2C Ch.3 Receive completed */
    if ((true == M4_INTC->VSSEL141_f.VSEL12) && (true == !!(M4_I2C3->SR_f.RFULLF)))
    {
        I2c3RxEnd_IrqHandler();
    }
    /* I2C Ch.3 Transmit completed */
    if ((true == M4_INTC->VSSEL141_f.VSEL13) && (true == !!(M4_I2C3->SR_f.TENDF)))
    {
        I2c3TxEnd_IrqHandler();
    }
    /* I2C Ch.3 Transmit data empty */
    if ((true == M4_INTC->VSSEL141_f.VSEL14) && (true == !!(M4_I2C3->SR_f.TEMPTYF)))
    {
        I2c3TxEmpty_IrqHandler();
    }
    /* I2C Ch.3 Error */
    if ((true == M4_INTC->VSSEL141_f.VSEL15) && (true == !!(M4_I2C3->SR & 0x00F05217u)))
    {
        I2c3Err_IrqHandler();
    }

    /* I2C Ch.1 detected */
    if ((true == M4_INTC->VSSEL141_f.VSEL17) && (true == M4_SYSREG->PWR_PVDDSR_f.PVD1DETFLG))
    {
        Lvd1_IrqHandler();
    }
    /* LVD Ch.2 detected */
    if ((true == M4_INTC->VSSEL141_f.VSEL18) && (true == M4_SYSREG->PWR_PVDDSR_f.PVD1DETFLG))
    {
        Lvd2_IrqHandler();
    }

    /* Freq. calculate error detected */
    if ((true == M4_INTC->VSSEL141_f.VSEL20) && (true == M4_FCM->SR_f.ERRF))
    {
        FcmErr_IrqHandler();
    }
    /* Freq. calculate completed */
    if ((true == M4_INTC->VSSEL141_f.VSEL21) && (true == M4_FCM->SR_f.MENDF))
    {
        FcmEnd_IrqHandler();
    }
    /* Freq. calculate overflow */
    if ((true == M4_INTC->VSSEL141_f.VSEL22) && (true == M4_FCM->SR_f.OVF))
    {
        FcmOV_IrqHandler();
    }
    /* WDT */
    if ((true == M4_INTC->VSSEL141_f.VSEL23) && (true == !!(M4_WDT->SR & 0x00030000u)))
    {
        Wdt_IrqHandler();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.142 share IRQ handler
 **
 ******************************************************************************/
void IRQ142_Handler(void)
{
    /* ADC unit.1 seq. A */
    if ((true == M4_INTC->VSSEL142_f.VSEL0) && (true == M4_ADC1->ISR_f.EOCAF))
    {
        ADC1A_IrqHandler();
    }
    /* ADC unit.1 seq. B */
    if ((true == M4_INTC->VSSEL142_f.VSEL1) && (true == M4_ADC1->ISR_f.EOCBF))
    {
        ADC1B_IrqHandler();
    }
    /* ADC unit.1 seq. A */
    if ((true == M4_INTC->VSSEL142_f.VSEL2)             &&                     \
        ((true == !!(M4_ADC1->AWDSR0 & 0xFFFFu))        ||                     \
         (true == M4_ADC1->AWDSR1_f.AWDF16)))
    {
        ADC1ChCmp_IrqHandler();
    }
    /* ADC unit.1 seq. cmp */
    if ((true == M4_INTC->VSSEL142_f.VSEL3)             &&                     \
        ((true == !!(M4_ADC1->AWDSR0 & 0xFFFFu))        ||                     \
         (true == M4_ADC1->AWDSR1_f.AWDF16)))
    {
        ADC1SeqCmp_IrqHandler();
    }

    /* ADC unit.2 seq. A */
    if ((true == M4_INTC->VSSEL142_f.VSEL4) && (true == M4_ADC2->ISR_f.EOCAF))
    {
        ADC2A_IrqHandler();
    }
    /* ADC unit.2 seq. B */
    if ((true == M4_INTC->VSSEL142_f.VSEL5) && (true == M4_ADC2->ISR_f.EOCBF))
    {
        ADC2B_IrqHandler();
    }
    /* ADC unit.2 seq. A */
    if ((true == M4_INTC->VSSEL142_f.VSEL6) && (true == !!(M4_ADC2->AWDSR0 & 0x1FFu)))
    {
        ADC2ChCmp_IrqHandler();
    }
    /* ADC unit.2 seq. cmp */
    if ((true == M4_INTC->VSSEL142_f.VSEL7) && (true == !!(M4_ADC2->AWDSR0 & 0x1FFu)))
    {
        ADC2SeqCmp_IrqHandler();
    }
}

/**
 *******************************************************************************
 ** \brief Int No.143 share IRQ handler
 **
 ******************************************************************************/
void IRQ143_Handler(void)
{
    /* SDIO Ch.1 */
    if ((true == M4_INTC->VSSEL143_f.VSEL2)             &&                     \
        ((true == !!(M4_SDIOC1->NORINTST & 0x81F7u))    ||                     \
         (true == !!(M4_SDIOC1->ERRINTST & 0x017Fu))))
    {
        Sdio1_IrqHandler();
    }
    /* SDIO Ch.2 */
    if ((true == M4_INTC->VSSEL143_f.VSEL5)             &&                     \
        ((true == !!(M4_SDIOC2->NORINTST & 0x81F7u))    ||                     \
         (true == !!(M4_SDIOC2->ERRINTST & 0x017Fu))))
    {
        Sdio2_IrqHandler();
    }
    /* CAN */
    if ((true == M4_INTC->VSSEL143_f.VSEL6)             &&                     \
        ((true == !!(M4_CAN->RTIF & 0xFFu))             ||                     \
         (true == !!(M4_CAN->ERRINT & 0x15u))           ||                     \
         (true == !!(M4_CAN->TTCFG & 0x68u))))
    {
        Can_IrqHandler();
    }
}

//@} // InterruptGroup

#endif /* DDL_INTERRUPTS_ENABLE */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
