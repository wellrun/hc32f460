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
 ** \brief clock switch system clock sample
 **
 **   - 2018-10-02  1.0  Chengy First version for Device Driver Library of
 **     clock
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
#define FCM_IRQn                Int141_IRQn

/* FCM windows lower/upper limitition */
#define FCM_WINDOWS_LOWER       0x0
#define FCM_WINDOWS_UPPER       0xFFFF

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

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
/* u16ExpectCnt = stcFcmMeasCfg.enSrc
                  / stcFcmMeasCfg.enSrcDiv
                  * stcFcmRefCfg.enIntRefDiv
                  / stcFcmRefCfg.enIntRefSrc   */
uint16_t u16ExpectCnt = 1953;
/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
 /**
 *******************************************************************************
 ** \brief  Fcm end interrupt function.
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void FcmEnd_IrqHandler(void)
{
    uint16_t u16Cnt = 0;

    u16Cnt = CLK_GetFcmCounter();

    if((u16Cnt < (u16ExpectCnt+20)) &&(u16Cnt > (u16ExpectCnt-20)))
    {
        LED1_ON();
    }
    else
    {
        LED0_ON();
    }

    CLK_ClearFcmFlag(ClkFcmFlagMendf);
}
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

    LED0_OFF();
    LED1_OFF();

    /* LED0 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* LED1 Port/Pin initialization */
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);
}
/**
 *******************************************************************************
 ** \brief  Main function of switch system clock source to MPLL project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_clk_sysclk_cfg_t        stcSysClkCfg;
    stc_clk_fcm_cfg_t           stcFcmCfg;
    stc_clk_fcm_window_cfg_t    stcFcmWinCfg;
    stc_clk_fcm_measure_cfg_t   stcFcmMeasCfg;
    stc_clk_fcm_reference_cfg_t stcFcmRefCfg;
    stc_clk_fcm_interrupt_cfg_t stcFcmIntCfg;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcFcmCfg);
    MEM_ZERO_STRUCT(stcFcmWinCfg);
    MEM_ZERO_STRUCT(stcFcmMeasCfg);
    MEM_ZERO_STRUCT(stcFcmRefCfg);
    MEM_ZERO_STRUCT(stcFcmIntCfg);

    Led_Init();

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv = ClkSysclkDiv1;   // Max 168MHz
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;  // Max 168MHz
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;  // Max 60MHz
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;  // Max 42MHz
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;  // Max 84MHz
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Enable HRC. */
    CLK_HrcCmd(Enable);
    /* Enable XTAL32. */
    CLK_Xtal32Cmd(Enable);

    /* Switch system clock source to HRC. */
    CLK_SetSysClkSource(ClkSysSrcHRC);

    /* Enable Fcm Clk. */
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_FCM,Enable);

    /* Fcm measurement config. */
    stcFcmMeasCfg.enSrc = ClkFcmSrcPclk1;
    stcFcmMeasCfg.enSrcDiv = ClkFcmMeaDiv4;
    /* Fmc reference config. */
    stcFcmRefCfg.enEdge = ClkFcmEdgeRising;
    stcFcmRefCfg.enExtRef = Disable;
    stcFcmRefCfg.enFilterClk = ClkFcmFilterClkNone;
    stcFcmRefCfg.enIntRefSrc = ClkFcmSrcXtal32;
    stcFcmRefCfg.enIntRefDiv = ClkFcmIntrefDiv32;
    stcFcmRefCfg.enRefSel = ClkFcmInterRef;
    /* Fcm windows config. */
    stcFcmWinCfg.windowLower = FCM_WINDOWS_LOWER;
    stcFcmWinCfg.windowUpper = FCM_WINDOWS_UPPER;
    /* Fcm interrupt config. */
    stcFcmIntCfg.enHandleSel = ClkFcmHandleInterrupt;
    stcFcmIntCfg.enHandleInterrupt = Disable;
    stcFcmIntCfg.enEndInterrupt = Enable;
    stcFcmIntCfg.enHandleReset = Disable;
    stcFcmIntCfg.enOvfInterrupt = Disable;
    /* Fcm Config. */
    stcFcmCfg.pstcFcmWindowCfg = &stcFcmWinCfg;
    stcFcmCfg.pstcFcmMeaCfg = &stcFcmMeasCfg;
    stcFcmCfg.pstcFcmRefCfg = &stcFcmRefCfg;
    stcFcmCfg.pstcFcmIntCfg = &stcFcmIntCfg;
    CLK_FcmConfig(&stcFcmCfg);

    /* Set Fcm interrupt. */
    enShareIrqEnable(INT_FCMMENDI);

    /* Enable interrupt. */
    NVIC_ClearPendingIRQ(FCM_IRQn);
    NVIC_SetPriority(FCM_IRQn,DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(FCM_IRQn);

    /* Enable Fcm. */
    CLK_FcmCmd(Enable);

    while(1);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
