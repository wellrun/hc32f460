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
 ** \brief power voltage detected interrupt sample
 **
 **   - 2018-11-06  1.0  Chengy First version for Device Driver Library of PWC.
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
#define PVD_IRQn                Int141_IRQn

/* LED0 Port/Pin definition */
#define  LED0_PORT              PortE
#define  LED0_PIN               Pin06

/* LED2 Port/Pin definition */
#define  LED2_PORT        PortB
#define  LED2_PIN         Pin05

/* LED0~1 definition */
#define LED0_ON()               PORT_SetBits(LED0_PORT, LED0_PIN)
#define LED0_OFF()              PORT_ResetBits(LED0_PORT, LED0_PIN)

/* LED2 definition */
#define LED2_ON()               PORT_SetBits(LED2_PORT, LED2_PIN)
#define LED2_OFF()              PORT_ResetBits(LED2_PORT, LED2_PIN)

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

    PORT_Unlock();
    LED0_OFF();
    LED2_OFF();
    PORT_Lock();

    /* LED0 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* LED2 Port/Pin initialization */
    PORT_Init(LED2_PORT, LED2_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief  PVD1 interrupt function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Lvd1_IrqHandler(void)
{
    PORT_Unlock();
    LED0_ON();
    PORT_Lock();
}
/**
 *******************************************************************************
 ** \brief  PVD2 interrupt function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Pvd2_IrqHandler(void)
{
    PORT_Unlock();
    LED0_OFF();
    LED2_ON();
    PORT_Lock();
}
/**
 *******************************************************************************
 ** \brief  PVD1 & PVD2 interrupt.
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_pwc_pvd_cfg_t   stcPwcPvdCfg;
    stc_nmi_config_t    stcNmiCfd;

    MEM_ZERO_STRUCT(stcPwcPvdCfg);

    Led_Init();

    /* Config PVD1. */
    /* Disable filter. */
    stcPwcPvdCfg.enPvd1FilterEn = Disable;
    /* Msk interrupt. */
    stcPwcPvdCfg.enPvd1Int = MskInt;
    /* Interrupt. */
    stcPwcPvdCfg.stcPvd1Ctl.enPvdMode = PvdInt;
    /* Enable Pvd1 interrupt. */
    stcPwcPvdCfg.stcPvd1Ctl.enPvdIREn = Enable;
    /* Enable output compared result. */
    stcPwcPvdCfg.stcPvd1Ctl.enPvdCmpOutEn = Enable;
    /* PVD1 Threshold Voltage 2.8V. */
    stcPwcPvdCfg.enPvd1Level = Pvd1Level28;

    /* Config PVD2.*/
    /* Disable filter. */
    stcPwcPvdCfg.enPvd2FilterEn = Disable;
    /* Non-Msk interrupt. */
    stcPwcPvdCfg.enPvd2Int = NonMskInt;
    /* Interrupt. */
    stcPwcPvdCfg.stcPvd2Ctl.enPvdMode = PvdInt;
    /* Enable Pvd2 interrupt. */
    stcPwcPvdCfg.stcPvd2Ctl.enPvdIREn = Enable;
    /* Enable output compared result. */
    stcPwcPvdCfg.stcPvd2Ctl.enPvdCmpOutEn = Enable;
    /* PVD2 Threshold Voltage 2.3V. */
    stcPwcPvdCfg.enPvd2Level = Pvd2Level23;

    PWC_PvdCfg(&stcPwcPvdCfg);

    /* Config NMI.*/
    /* Set PVD2 as NMI source. */
    stcNmiCfd.u16NmiSrc = NmiSrcVdu2;
    /* Disbale filter. */
    stcNmiCfd.enFilterEn = Disable;
    /* Set Pvd2 interrupt callback. */
    stcNmiCfd.pfnNmiCallback = Pvd2_IrqHandler;

    NMI_Init(&stcNmiCfd);

    /* Set PVD1 interrupt. */
    enShareIrqEnable(INT_PVD_PVD1);

    /* Enable interrupt. */
    NVIC_ClearPendingIRQ(PVD_IRQn);
    NVIC_SetPriority(PVD_IRQn,DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(PVD_IRQn);

    /* Enable PVD1. */
    PWC_Pvd1Cmd(Enable);
    /* Enable PVD2. */
    PWC_Pvd2Cmd(Enable);

    while(1);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
