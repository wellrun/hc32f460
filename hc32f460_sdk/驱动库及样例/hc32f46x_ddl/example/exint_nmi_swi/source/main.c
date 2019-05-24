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
 ** \brief The example for EXTI, NMI and SWI function demonstration
 **
 **   - 2018-10-17  1.0  Zhangxl First version for sample of exint_nmi_swi module.
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
/* KEY0 */
#define  SW2_PORT         PortD
#define  SW2_PIN          Pin03
/* KEY1 */
#define  SW3_PORT         PortD
#define  SW3_PIN          Pin04
/* KEY2 */
#define  SW4_PORT         PortD
#define  SW4_PIN          Pin05
/* KEY3 */
#define  SW5_PORT         PortD
#define  SW5_PIN          Pin06

/* LED0 Port/Pin definition */
#define  LED0_PORT        PortE
#define  LED0_PIN         Pin06

/* LED1 Port/Pin definition */
#define  LED1_PORT        PortA
#define  LED1_PIN         Pin07

/* LED2 Port/Pin definition */
#define  LED2_PORT        PortB
#define  LED2_PIN         Pin05

/* LED3 Port/Pin definition */
#define  LED3_PORT        PortB
#define  LED3_PIN         Pin09

/* LED0~3 toggle definition */
#define  LED0_TOGGLE()    PORT_Toggle(LED0_PORT, LED0_PIN)
#define  LED1_TOGGLE()    PORT_Toggle(LED1_PORT, LED1_PIN)
#define  LED2_TOGGLE()    PORT_Toggle(LED2_PORT, LED2_PIN)
#define  LED3_TOGGLE()    PORT_Toggle(LED3_PORT, LED3_PIN)

/* uncomment this line if wants to print information to Terminal I/O window */
//#define  __PRINT_TO_TERMINAL

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
uint32_t u32NmiCount = 0;
uint32_t u32ExtInt00Count = 0;
uint32_t u32ExtInt01Count = 0;
uint32_t u32ExtInt02Count = 0;
uint32_t u32ExtInt03Count = 0;
uint32_t u32ExtInt04Count = 0;
uint32_t u32ExtInt05Count = 0;
uint32_t u32ExtInt06Count = 0;
uint32_t u32ExtInt07Count = 0;
uint32_t u32ExtInt08Count = 0;
uint32_t u32ExtInt09Count = 0;
uint32_t u32ExtInt10Count = 0;
uint32_t u32ExtInt11Count = 0;
uint32_t u32ExtInt12Count = 0;
uint32_t u32ExtInt13Count = 0;
uint32_t u32ExtInt14Count = 0;
uint32_t u32ExtInt15Count = 0;
uint32_t u32SWI00Count = 0;
uint32_t u32SWI01Count = 0;
uint32_t u32SWI02Count = 0;
uint32_t u32SWI03Count = 0;
uint32_t u32SWI04Count = 0;
uint32_t u32SWI05Count = 0;
uint32_t u32SWI06Count = 0;
uint32_t u32SWI07Count = 0;
uint32_t u32SWI08Count = 0;
uint32_t u32SWI09Count = 0;
uint32_t u32SWI10Count = 0;
uint32_t u32SWI11Count = 0;
uint32_t u32SWI12Count = 0;
uint32_t u32SWI13Count = 0;
uint32_t u32SWI14Count = 0;
uint32_t u32SWI15Count = 0;
uint32_t u32SWI16Count = 0;
uint32_t u32SWI17Count = 0;
uint32_t u32SWI18Count = 0;
uint32_t u32SWI19Count = 0;
uint32_t u32SWI20Count = 0;
uint32_t u32SWI21Count = 0;
uint32_t u32SWI22Count = 0;
uint32_t u32SWI23Count = 0;
uint32_t u32SWI24Count = 0;
uint32_t u32SWI25Count = 0;
uint32_t u32SWI26Count = 0;
uint32_t u32SWI27Count = 0;
uint32_t u32SWI28Count = 0;
uint32_t u32SWI29Count = 0;
uint32_t u32SWI30Count = 0;
uint32_t u32SWI31Count = 0;
/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief ExtInt00 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt00_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh00))
    {
        u32ExtInt00Count++;

        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh00);
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt01 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt01_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh01))
    {
        u32ExtInt01Count++;

        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh01);
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt02 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt02_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh02))
    {
        u32ExtInt02Count++;

        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh02);
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt03 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt03_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh03))
    {
        u32ExtInt03Count++;

        LED0_TOGGLE();
#ifdef  __PRINT_TO_TERMINAL
        printf("External interrupt 03(SW2) interrupt occurrence: %d\n", u32ExtInt03Count);
#endif
        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh03);
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt04 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt04_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh04))
    {
        u32ExtInt04Count++;

        LED1_TOGGLE();
#ifdef  __PRINT_TO_TERMINAL
        printf("External interrupt 04(SW4) interrupt occurrence: %d\n", u32ExtInt04Count);
#endif
        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh04);
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt05 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt05_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh05))
    {
        u32ExtInt05Count++;

        LED2_TOGGLE();
#ifdef  __PRINT_TO_TERMINAL
        printf("External interrupt 05(SW3) interrupt occurrence: %d\n", u32ExtInt05Count);
#endif
        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh05);
    }

}

/**
 *******************************************************************************
 ** \brief ExtInt06 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt06_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh06))
    {
        u32ExtInt06Count++;

        LED3_TOGGLE();
        Ddl_Delay1ms(100);
        LED3_TOGGLE();
        Ddl_Delay1ms(100);
#ifdef  __PRINT_TO_TERMINAL
        printf("External interrupt 06(SW5) interrupt occurrence: %d\n", u32ExtInt06Count);
#endif
        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh06);
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt07 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt07_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh07))
    {
        u32ExtInt07Count++;

        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh07);
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt08 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt08_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh08))
    {
        u32ExtInt08Count++;

        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh08);
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt09 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt09_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh09))
    {
        u32ExtInt09Count++;

        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh09);
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt10 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt10_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh10))
    {
        u32ExtInt10Count++;

        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh10);
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt11 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt11_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh11))
    {
        u32ExtInt11Count++;

        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh11);
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt12 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt12_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh12))
    {
        u32ExtInt12Count++;

        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh12);
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt13 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt13_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh13))
    {
        u32ExtInt13Count++;

        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh13);
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt14 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt14_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh14))
    {
        u32ExtInt14Count++;

        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh14);
    }
}

/**
 *******************************************************************************
 ** \brief ExtInt15 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt15_Callback(void)
{

    if (Set == EXINT_IrqFlgGet(ExtiCh15))
    {
        u32ExtInt15Count++;

        /* clear int request flag */
        EXINT_IrqFlgClr(ExtiCh15);
    }
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.00 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI00_Callback(void)
{
    u32SWI00Count++;

    SWI_Disable(SwiCh00);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.01 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI01_Callback(void)
{
    u32SWI01Count++;

    SWI_Disable(SwiCh01);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.02 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI02_Callback(void)
{
    u32SWI02Count++;

    SWI_Disable(SwiCh02);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.03 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI03_Callback(void)
{
    u32SWI03Count++;

    SWI_Disable(SwiCh03);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.04 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI04_Callback(void)
{
    u32SWI04Count++;

    SWI_Disable(SwiCh04);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.05 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI05_Callback(void)
{
    u32SWI05Count++;

    SWI_Disable(SwiCh05);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.06 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI06_Callback(void)
{
    u32SWI06Count++;

    SWI_Disable(SwiCh06);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.07 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI07_Callback(void)
{
    u32SWI07Count++;

    SWI_Disable(SwiCh07);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.08 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI08_Callback(void)
{
    u32SWI08Count++;

    SWI_Disable(SwiCh08);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.09 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI09_Callback(void)
{
    u32SWI09Count++;

    SWI_Disable(SwiCh09);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.10 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI10_Callback(void)
{
    u32SWI10Count++;

    SWI_Disable(SwiCh10);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.11 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI11_Callback(void)
{
    u32SWI11Count++;

    SWI_Disable(SwiCh11);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.12 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI12_Callback(void)
{
    u32SWI12Count++;

    SWI_Disable(SwiCh12);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.13 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI13_Callback(void)
{
    u32SWI13Count++;

    SWI_Disable(SwiCh13);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.14 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI14_Callback(void)
{
    u32SWI14Count++;

    SWI_Disable(SwiCh14);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.15 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI15_Callback(void)
{
    u32SWI15Count++;

    SWI_Disable(SwiCh15);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.16 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI16_Callback(void)
{
    u32SWI16Count++;

    SWI_Disable(SwiCh16);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.17 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI17_Callback(void)
{
    u32SWI17Count++;

    SWI_Disable(SwiCh17);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.18 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI18_Callback(void)
{
    u32SWI18Count++;

    SWI_Disable(SwiCh18);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.19 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI19_Callback(void)
{
    u32SWI19Count++;

    SWI_Disable(SwiCh19);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.20 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI20_Callback(void)
{
    u32SWI20Count++;

    SWI_Disable(SwiCh20);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.21 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI21_Callback(void)
{
    u32SWI21Count++;

    SWI_Disable(SwiCh21);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.22 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI22_Callback(void)
{
    u32SWI22Count++;

    SWI_Disable(SwiCh22);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.23 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI23_Callback(void)
{
    u32SWI23Count++;

    SWI_Disable(SwiCh23);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.24 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI24_Callback(void)
{
    u32SWI24Count++;

    SWI_Disable(SwiCh24);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.25 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI25_Callback(void)
{
    u32SWI25Count++;

    SWI_Disable(SwiCh25);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.26 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI26_Callback(void)
{
    u32SWI26Count++;

    SWI_Disable(SwiCh26);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.27 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI27_Callback(void)
{
    u32SWI27Count++;

    SWI_Disable(SwiCh27);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.28 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI28_Callback(void)
{
    u32SWI28Count++;

    SWI_Disable(SwiCh28);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.29 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI29_Callback(void)
{
    u32SWI29Count++;

    SWI_Disable(SwiCh29);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.30 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI30_Callback(void)
{
    u32SWI30Count++;

    SWI_Disable(SwiCh30);
}

/**
 *******************************************************************************
 ** \brief Software interrupt Ch.31 callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI31_Callback(void)
{
    u32SWI31Count++;

#ifdef  __PRINT_TO_TERMINAL
    printf("Software 31 interrupt occurrence: %d\n", u32SWI31Count);
#endif
    SWI_Disable(SwiCh31);
}

/**
 *******************************************************************************
 ** \brief NMI ISR callback function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Nmi_IrqCallback(void)
{
    u32NmiCount++;

    /* NMI Pin */
    if (Set == NMI_IrqFlgGet(NmiSrcNmi))
    {
        NMI_IrqFlgClr(NmiSrcNmi);
        LED0_TOGGLE();
        LED1_TOGGLE();
        LED2_TOGGLE();
        LED3_TOGGLE();
    }

    /* SWDT underflow or refresh */
    if (Set == NMI_IrqFlgGet(NmiSrcSwdt))
    {
        NMI_IrqFlgClr(NmiSrcSwdt);
    }

    /* LVD1 detected */
    if (Set == NMI_IrqFlgGet(NmiSrcVdu1))
    {
        NMI_IrqFlgClr(NmiSrcVdu1);
    }

    /* LVD2 detected */
    if (Set == NMI_IrqFlgGet(NmiSrcVdu2))
    {
        NMI_IrqFlgClr(NmiSrcVdu2);
    }

    /* XTAL stop */
    if (Set == NMI_IrqFlgGet(NmiSrcXtalStop))
    {
        NMI_IrqFlgClr(NmiSrcXtalStop);
    }

    /* SRAM1/2/HS/Ret Parity error */
    if (Set == NMI_IrqFlgGet(NmiSrcSramPE))
    {
        NMI_IrqFlgClr(NmiSrcSramPE);
    }

    /* SRAM3 ECC error */
    if (Set == NMI_IrqFlgGet(NmiSrcSramDE))
    {
        NMI_IrqFlgClr(NmiSrcSramDE);
    }

    /* MPU bus error */
    if (Set == NMI_IrqFlgGet(NmiSrcMpu))
    {
        NMI_IrqFlgClr(NmiSrcMpu);
    }

    /* WDT underflow or refresh */
    if (Set == NMI_IrqFlgGet(NmiSrcWdt))
    {
        NMI_IrqFlgClr(NmiSrcWdt);
    }
}

/**
 *******************************************************************************
 ** \brief SW2 init function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Sw2_Init(void)
{
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    /**************************************************************************/
    /* External Int Ch.3                                                      */
    /**************************************************************************/
    stcExtiConfig.enExitCh = ExtiCh03;

    /* Filter setting */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk = Pclk3Div8;
    /* Both edge */
    stcExtiConfig.enExtiLvl = ExIntBothEdge;
    EXINT_Init(&stcExtiConfig);

    /* Set PD03 as External Int Ch.3 input */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enExInt = Enable;
    PORT_Init(SW2_PORT, SW2_PIN, &stcPortInit);

    /* Select External Int Ch.3 */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ3;

    /* Register External Int to Vect.No.000 */
    stcIrqRegiConf.enIRQn = Int000_IRQn;

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
 ** \brief SW3 init function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Sw3_Init(void)
{
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    /**************************************************************************/
    /* External Int Ch.5                                                      */
    /**************************************************************************/
    stcExtiConfig.enExitCh = ExtiCh05;

    /* Filter setting */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk = Pclk3Div8;
    /* Rising edge */
    stcExtiConfig.enExtiLvl = ExIntFallingEdge;
    EXINT_Init(&stcExtiConfig);

    /* Set PD05 as External Int Ch.5 input */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enExInt = Enable;
    PORT_Init(SW4_PORT, SW4_PIN, &stcPortInit);

    /* Select External Int Ch.5 */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ5;

    /* Register External Int to Vect.No.001 */
    stcIrqRegiConf.enIRQn = Int001_IRQn;

    /* Callback function */
    stcIrqRegiConf.pfnCallback = ExtInt05_Callback;

    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);

    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);

    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

/**
 *******************************************************************************
 ** \brief SW4 init function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Sw4_Init(void)
{
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    /**************************************************************************/
    /* External Int Ch.4                                                      */
    /**************************************************************************/
    stcExtiConfig.enExitCh = ExtiCh04;

    /* Filter setting */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk = Pclk3Div8;
    /* Rising edge */
    stcExtiConfig.enExtiLvl = ExIntRisingEdge;
    EXINT_Init(&stcExtiConfig);

    /* Set PD04 as External Int Ch.4 input */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enExInt = Enable;
    PORT_Init(SW3_PORT, SW3_PIN, &stcPortInit);

    /* Select External Int Ch.4 */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ4;

    /* Register External Int to Vect.No.025 */
    stcIrqRegiConf.enIRQn = Int025_IRQn;

    /* Callback function */
    stcIrqRegiConf.pfnCallback = ExtInt04_Callback;

    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);

    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);

    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

/**
 *******************************************************************************
 ** \brief SW5 init function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Sw5_Init(void)
{
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;
    stc_port_init_t stcPortInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);
    MEM_ZERO_STRUCT(stcPortInit);

    /**************************************************************************/
    /* External Int Ch.6                                                      */
    /**************************************************************************/
    stcExtiConfig.enExitCh = ExtiCh06;

    /* Filter setting */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk = Pclk3Div8;
    /* Rising edge */
    stcExtiConfig.enExtiLvl = ExIntLowLevel;
    EXINT_Init(&stcExtiConfig);

    /* Set PD06 as External Int Ch.6 input */
    MEM_ZERO_STRUCT(stcPortInit);
    stcPortInit.enExInt = Enable;
    PORT_Init(SW5_PORT, SW5_PIN, &stcPortInit);

    /* Select External Int Ch.6 */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ6;

    /* Register External Int to Vect.No.002 */
    stcIrqRegiConf.enIRQn = Int002_IRQn;

    /* Callback function */
    stcIrqRegiConf.pfnCallback = ExtInt06_Callback;

    /* Registration IRQ */
    enIrqRegistration(&stcIrqRegiConf);

    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);

    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);

    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);
}

/**
 *******************************************************************************
 ** \brief Software interrupt init function
 **
 ** \param [in]  pstcSwiConfig, software configure structure
 **
 ** \retval None
 **
 ******************************************************************************/
void SWI_Init(stc_swi_config_t *pstcSwiConfig)
{
    uint8_t u8SWI_IRQ, i;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    for (i = 0; i< 32; i++)
    {
        if ((1u << i) == pstcSwiConfig->enSwiCh)
        {
            u8SWI_IRQ = i;
            break;
        }
    }

    /* Select software Int Ch.8 */
    stcIrqRegiConf.enIntSrc = (en_int_src_t)(u8SWI_IRQ);

    /* Register software Int to Vect.No.008 */
    stcIrqRegiConf.enIRQn = (IRQn_Type)(u8SWI_IRQ);

    /* Callback function */
    stcIrqRegiConf.pfnCallback = pstcSwiConfig->pfnSwiCallback;

    enIrqRegistration(&stcIrqRegiConf);

    /* Clear Pending */
    NVIC_ClearPendingIRQ(stcIrqRegiConf.enIRQn);

    /* Set priority */
    NVIC_SetPriority(stcIrqRegiConf.enIRQn, DDL_IRQ_PRIORITY_15);

    /* Enable NVIC */
    NVIC_EnableIRQ(stcIrqRegiConf.enIRQn);

    /* Enable Software interrupt */
    SWI_Enable(pstcSwiConfig->enSwiCh);
}

/**
 *******************************************************************************
 ** \brief LED 0~3 init function
 **
 ** \param  None
 **
 ** \retval None
 **
 ******************************************************************************/
void Led_Init(void)
{
    stc_port_init_t stcPortInit;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;

    /* LED0 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* LED1 Port/Pin initialization */
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);

    /* LED2 Port/Pin initialization */
    PORT_Init(LED2_PORT, LED2_PIN, &stcPortInit);

    /* LED3 Port/Pin initialization */
    PORT_Init(LED3_PORT, LED3_PIN, &stcPortInit);
}

/**
 *******************************************************************************
 ** \brief  main function for EXINT, NMI, SWI function
 **
 ** \param  None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    stc_nmi_config_t stcNmiConfig;
    stc_swi_config_t stcSwiConfig;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcNmiConfig);
    MEM_ZERO_STRUCT(stcSwiConfig);

    /* Init LED 0~3 */
    Led_Init();

    /**************************************************************************/
    /* NMI Pin (PB11)                                                         */
    /**************************************************************************/
    /* Filter setting */
    stcNmiConfig.enFilterEn = Enable;
    stcNmiConfig.enFilterClk = Pclk3Div8;

    /* Falling edge */
    stcNmiConfig.enNmiLvl = NmiFallingEdge;

    /* Callback function */
    stcNmiConfig.pfnNmiCallback = Nmi_IrqCallback;
    stcNmiConfig.u16NmiSrc = NmiSrcNmi;

    NMI_Init(&stcNmiConfig);

    /**************************************************************************/
    /* SWI 31 configuration                                                   */
    /**************************************************************************/
    /* SWI Ch.31 */
    stcSwiConfig.enSwiCh = SwiCh31;

    /* Software interrupt */
    stcSwiConfig.enSwiType = SwInt;

    /* Software interrupt callback function */
    stcSwiConfig.pfnSwiCallback = SWI31_Callback;

    SWI_Init(&stcSwiConfig);

    /**************************************************************************/
    /* Switch2~5 (ext. interrupt) configuration                               */
    /**************************************************************************/
    Sw2_Init();
    Sw3_Init();
    Sw4_Init();
    Sw5_Init();

    while(1);
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
