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

#define EFM_FLASH_TRIMLOAD()    *((volatile unsigned int*)(0x40010490UL)) = 2
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
    LED1_OFF();
    PORT_Lock();

    /* LED0 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* LED1 Port/Pin initialization */
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief  Main function of dectect xtal break down.
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_clk_xtal_cfg_t      stcXtalCfg;
    stc_clk_xtal_stp_cfg_t  stcXtalStpCfg;
    stc_clk_output_cfg_t    stcOutputClkCfg;
    stc_rmu_rstcause_t      stcRstCause;

    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcXtalStpCfg);
    MEM_ZERO_STRUCT(stcOutputClkCfg);

    Led_Init();

    RMU_GetResetCause(&stcRstCause);
    if(Set == stcRstCause.enXtalErr)
    {
        RMU_ClrResetFlag();
        PORT_Unlock();
        while(1)
        {
            LED0_ON();
            LED1_OFF();
        }
    }

    /* Xtal config. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* Enable xtal fault dectect and occur reset. */
    stcXtalStpCfg.enDetect = Enable;
    stcXtalStpCfg.enMode = ClkXtalStpModeReset;
    stcXtalStpCfg.enModeInt = Disable;
    stcXtalStpCfg.enModeReset = Enable;
    CLK_XtalStpConfig(&stcXtalStpCfg);

    /* Clk output.*/
    stcOutputClkCfg.enOutputSrc = ClkOutputSrcXtal;
    stcOutputClkCfg.enOutputDiv = ClkOutputDiv8;
    CLK_OutputClkConfig(ClkOutputCh1,&stcOutputClkCfg);
    CLK_OutputClkCmd(ClkOutputCh1,Enable);

    PORT_SetFunc(PortA, Pin08, Func_Mclkout, Disable);

    PORT_Unlock();
    while(1)
    {
        LED1_ON();
        LED0_OFF();
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
