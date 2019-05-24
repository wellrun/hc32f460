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
/** \file main.c
 **
 ** \brief This sample demonstrates how to use CMP by single mode.
 **
 **   - 2018-10-22  1.0  Pangw first version for Device Driver Library of CMP.
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

/* LED Port/Pin definition */
#define LED_PORT                        PortA
#define LED_PIN                         Pin07

/* LED toggle definition */
#define LED_TOGGLE()                    PORT_Toggle(LED_PORT, LED_PIN)

/* CMP definition */
#define CMP_UNIT                        M4_CMP2
#define CMP_INT_NUM                     INT_ACMP2
#define CMP_INT_IRQn                    Int002_IRQn

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void LedInit(void);
static void CmpCallback(void);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

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

    PORT_ResetBits(LED_PORT, LED_PIN);

    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;

    /* LED Port/Pin initialization */
    PORT_Init(LED_PORT, LED_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief CMP irq callback function.
 **
 ******************************************************************************/
static void CmpCallback(void)
{
     LED_TOGGLE();
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
    stc_cmp_init_t stcCmpConfig;
    stc_port_init_t stcPortInit;
    stc_cmp_input_sel_t stcCmpInput;
    stc_cmp_dac_init_t stcDacInitCfg;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* Initialize structure */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcCmpInput);
    MEM_ZERO_STRUCT(stcCmpConfig);
    MEM_ZERO_STRUCT(stcDacInitCfg);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Initialize LED */
    LedInit();

    stcPortInit.enPinMode = Pin_Mode_Ana;
    /* Set PA4 as CMP2_INP1 input */
    PORT_Init(PortA, Pin04, &stcPortInit);

    /* Set PB13 as Vcout output */
    PORT_SetFunc(PortB, Pin13, Func_Vcout, Disable);

    /* Enable peripheral clock */
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_CMP, Enable);
    PWC_Fcg3PeriphClockCmd(PWC_FCG3_PERIPH_DAC, Enable);

    /* Set DAC */
    stcDacInitCfg.u8DacData = 0x80;
    stcDacInitCfg.enCmpDacEN = Enable;
    CMP_DAC_Init(CmpDac1, &stcDacInitCfg);
    CMP_DAC_Init(CmpDac2, &stcDacInitCfg);

    /* Set CMP mode */
    stcCmpConfig.enCmpIntEN = Enable;         /* Interrupt enable */
    stcCmpConfig.enCmpInvEn = Disable;
    stcCmpConfig.enCmpOutputEn = Enable;
    stcCmpConfig.enCmpVcoutOutputEn = Enable; /* Out enable */
    stcCmpConfig.enEdgeSel = CmpBothEdge;
    stcCmpConfig.enFltClkDiv = CmpFltPclk3Div64;
    CMP_Init(CMP_UNIT, &stcCmpConfig);

    /* Set CMP input */
    stcCmpInput.enInpSel = CmpInp1;
    stcCmpInput.enInmSel = CmpInm3;
    CMP_InputSel(CMP_UNIT, &stcCmpInput);

    /* Registration IRQ : CMP */
    stcIrqRegiConf.enIntSrc = CMP_INT_NUM;
    stcIrqRegiConf.enIRQn = CMP_INT_IRQn;
    stcIrqRegiConf.pfnCallback = CmpCallback;
    enIrqRegistration(&stcIrqRegiConf);
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn); /* Clear pending */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15); /* Set priority */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);       /* Enable NVIC */

    /* Enable CMP */
    CMP_Cmd(CMP_UNIT, Enable);

    while(1)
    {
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
