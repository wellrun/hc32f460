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
/** \file hc32f46x_clk.c
 **
 ** A detailed description is available at
 ** @link CmuGroup Clock description @endlink
 **
 **   - 2018-10-13  1.0  Chengy First version for Device Driver Library of CMU.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32f46x_clk.h"
#include "hc32f46x_utility.h"

#if (DDL_CLK_ENABLE == DDL_ON)

/**
 *******************************************************************************
 ** \addtogroup CmuGroup
 ******************************************************************************/
//@{

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define CLK_XTAL_TIMEOUT                    ((uint16_t)0x1000)
#define CLK_XTAL32_TIMEOUT                  ((uint8_t)0x05)
#define CLK_HRC_TIMEOUT                     ((uint16_t)0x1000)
#define CLK_MRC_TIMEOUT                     ((uint8_t)0x05)
#define CLK_LRC_TIMEOUT                     ((uint8_t)0x05)
#define CLK_MPLL_TIMEOUT                    ((uint16_t)0x1000)
#define CLK_UPLL_TIMEOUT                    ((uint16_t)0x1000)
#define CLK_FCG_STABLE                      ((uint16_t)0x1000)
#define CLK_SYSCLK_STABLE                   ((uint16_t)0x1000)
#define CLK_USBCLK_STABLE                   ((uint16_t)0x1000)

#define CLK_PLL_DIV_MIN                     2u
#define CLK_PLL_DIV_MAX                     16u

#define CLK_PLLQ_DIV_MIN                    1u
#define CLK_PLLQ_DIV_MAX                    16u

#define CLK_PLLN_MIN                        20u
#define CLK_PLLN_MAX                        480u

#define CLK_PLLM_MIN                        1u
#define CLK_PLLM_MAX                        24u

#define CLK_UPLLM_MIN                       2u
#define CLK_UPLLM_MAX                       24u

#define CLK_PLL_VCO_IN_MIN                  1*1000*1000
#define CLK_PLL_VCO_IN_MAX                  24*1000*1000

#define CLK_PLL_VCO_OUT_MIN                 240*1000*1000
#define CLK_PLL_VCO_OUT_MAX                 480*1000*1000

#define ENABLE_CLOCK_REG_WRITE()            (M4_SYSREG->PWR_FPRC = 0xa501)
#define DISABLE_CLOCK_REG_WRITE()           (M4_SYSREG->PWR_FPRC = 0xa500)

#define ENABLE_CLOCK1_REG_WRITE()           (M4_SYSREG->PWR_FPRC = 0xa502)
#define DISABLE_CLOCK1_REG_WRITE()          (M4_SYSREG->PWR_FPRC = 0xa500)


#define DEFAULT_FCG0                        0xFFFFFAEE
#define DEFAULT_FCG1                        0xFFFFFFFF
#define DEFAULT_FCG2                        0xFFFFFFFF
#define DEFAULT_FCG3                        0xFFFFFFFF

#define FCG0_OFFSET_FCM                     16u
#define FCG1_OFFSET_CAN                     0u
#define FCG1_OFFSET_QSPI                    3u
#define FCG1_OFFSET_USBFS                   8u
#define FCG1_OFFSET_SPI1                    16u
#define FCG1_OFFSET_SPI2                    17u
#define FCG1_OFFSET_SPI3                    18u
#define FCG1_OFFSET_SPI4                    19u
#define FCG3_OFFSET_ADC1                    0u
#define FCG3_OFFSET_ADC2                    1u
#define FCG3_OFFSET_DAC                     4u

/*! Parameter validity check for XTAL stablization time \a stb. */
#define IS_XTAL_STB_VALID(stb)                                                 \
(   (ClkXtalStbCycle35          <= (stb))  &&                                  \
    (ClkXtalStbCycle8163        >=  (stb)))

/*! Parameter validity check for stop XTAL \a syssrc \a pllsrc. */
#define IS_XTAL_STOP_VALID(syssrc,pllsrc)                                      \
(   (ClkSysSrcXTAL      != (syssrc)) &&                                        \
    (!((ClkPllSrcXTAL ==  (pllsrc)) && (M4_SYSREG->CMU_PLLCR_f.MPLLOFF == 0))))

/*! Parameter validity check for stop XTAL32 \a syssrc. */
#define IS_XTAL32_STOP_VALID(syssrc)        (ClkSysSrcXTAL32 != (syssrc))

/*! Parameter validity check for stop HRC \a syssrc \a pllsrc. */
#define IS_HRC_STOP_VALID(syssrc,pllsrc)                                       \
(   (ClkSysSrcHRC           !=  (syssrc))   &&                                 \
    (!((ClkPllSrcHRC ==  (pllsrc)) && (M4_SYSREG->CMU_PLLCR_f.MPLLOFF == 0))))

/*! Parameter validity check for stop MRC \a syssrc. */
#define IS_MRC_STOP_VALID(syssrc)           (ClkSysSrcMRC != syssrc)

/*! Parameter validity check for stop LRC \a syssrc. */
#define IS_LRC_STOP_VALID(syssrc)           (ClkSysSrcLRC != syssrc)

/*! Parameter validity check for stop MPLL \a syssrc. */
#define IS_MPLL_STOP_VALID(syssrc)          (CLKSysSrcMPLL != syssrc)

/*! Parameter validity check for pll source \a src. */
#define IS_PLL_SOURCE(src)                                                     \
(   (ClkPllSrcXTAL          ==  (src))      ||                                 \
    (ClkPllSrcHRC           ==  (src)))

/*! Parameter validity check for mpll div \a pllp, pllr, upll div \a pllp, pllq, pllr*/
#define IS_PLL_DIV_VALID(pllx)                                                 \
(   (CLK_PLL_DIV_MIN        <=  (pllx))     &&                                 \
    (CLK_PLL_DIV_MAX        >=  (pllx)))

/*! Parameter validity check for pll div \a pllq. */
#define IS_PLLQ_DIV_VALID(pllx)                                                \
(   (CLK_PLLQ_DIV_MIN        <=  (pllx))    &&                                 \
    (CLK_PLLQ_DIV_MAX        >=  (pllx)))

/*! Parameter validity check for plln \a plln. */
#define IS_PLLN_VALID(plln)                                                    \
(   (CLK_PLLN_MIN           <=  (plln))     &&                                 \
    (CLK_PLLN_MAX           >=  (plln)))

/*! Parameter validity check for pllm \a pllm. */
#define IS_PLLM_VALID(pllm)                                                    \
(   (CLK_PLLM_MIN           <=  (pllm))     &&                                 \
    (CLK_PLLM_MAX           >=  (pllm)))

/*! Parameter validity check for pllm \a pllm. */
#define IS_UPLLM_VALID(pllm)                                                   \
(   (CLK_UPLLM_MIN           <=  (pllm))     &&                                \
    (CLK_UPLLM_MAX           >=  (pllm)))

/*! Parameter validity check for pllsource/pllm \a vco_in. */
#define IS_PLL_VCO_IN_VALID(vco_in)                                            \
(   (CLK_PLL_VCO_IN_MIN     <=  (vco_in))   &&                                 \
    (CLK_PLL_VCO_IN_MAX     >=  (vco_in)))

/*! Parameter validity check for pllsource/pllm*plln \a vco_out. */
#define IS_PLL_VCO_OUT_VALID(vco_out)                                          \
(   (CLK_PLL_VCO_OUT_MIN    <=  (vco_out))  &&                                 \
    (CLK_PLL_VCO_OUT_MAX    >=  (vco_out)))

/*! Parameter validity check for system clock source \a syssrc. */
#define IS_SYSCLK_SOURCE(syssrc)                                               \
(   (ClkSysSrcHRC           ==  (syssrc))   ||                                 \
    (ClkSysSrcMRC           <=  (syssrc))   &&                                 \
    (CLKSysSrcMPLL          >=  (syssrc)))

/*! Parameter validity check for usb clock source \a usbsrc. */
#define IS_USBCLK_SOURCE(usbsrc)                                               \
(   (ClkUsbSrcSysDiv2       <=  (usbsrc))   &&                                 \
    (ClkUsbSrcSysDiv4       >=  (usbsrc))   ||                                 \
    (ClkUsbSrcMpllp         <=  (usbsrc))   &&                                 \
    (ClkUsbSrcUpllr         >=  (usbsrc)))

/*! Parameter validity check for peripheral(adc/trng/I2S) clock source \a adcsrc. */
#define IS_PERICLK_SOURCE(adcsrc)                                              \
(   (ClkPeriSrcPclk         ==  (adcsrc))   ||                                 \
    (ClkPeriSrcMpllp        <=  (adcsrc))   &&                                 \
    (ClkPeriSrcUpllr        >=  (adcsrc)))

/*! Parameter validity check for output clock source \a outsrc. */
#define IS_OUTPUTCLK_SOURCE(outsrc)                                            \
(   (ClkOutputSrcHrc        ==  (outsrc))   ||                                 \
    (ClkOutputSrcMrc        ==  (outsrc))   ||                                 \
    (ClkOutputSrcLrc        ==  (outsrc))   ||                                 \
    (ClkOutputSrcXtal       ==  (outsrc))   ||                                 \
    (ClkOutputSrcXtal32     ==  (outsrc))   ||                                 \
    (ClkOutputSrcMpllp      ==  (outsrc))   ||                                 \
    (ClkOutputSrcUpllp      ==  (outsrc))   ||                                 \
    (ClkOutputSrcMpllq      ==  (outsrc))   ||                                 \
    (ClkOutputSrcUpllq      ==  (outsrc))   ||                                 \
    (ClkOutputSrcSysclk     ==  (outsrc)))

/*! Parameter validity check for fcm source \a fcmsrc. */
#define IS_FCM_SOURCE(fcmsrc)                                                  \
(   (ClkFcmSrcXtal          ==  (fcmsrc))   ||                                 \
    (ClkFcmSrcXtal32        <=  (fcmsrc))   &&                                 \
    (ClkFcmSrcRtcLrc        >=  (fcmsrc)))

/*! Parameter validity check for output clock channel \a outch. */
#define IS_OUTPUTCLK_CHANNEL(outch)                                            \
(   (ClkOutputCh1           ==  (outch))   ||                                  \
    (ClkOutputCh2           ==  (outch)))

/*! Parameter validity check for fcm reference \a ref. */
#define IS_FCM_REF(ref)                                                        \
(   (ClkFcmExtRef           ==  (ref))      ||                                 \
    (ClkFcmInterRef         ==  (ref)))

/*! Parameter validity check for fcm edge \a edge. */
#define IS_FCM_EDGE(edge)                                                      \
(   (ClkFcmEdgeRising       ==  (edge))     ||                                 \
    (ClkFcmEdgeFalling      ==  (edge))     ||                                 \
    (ClkFcmEdgeBoth         ==  (edge)))

/*! Parameter validity check for fcm filter clock \a clk. */
#define IS_FCM_FILTER_CLK(clk)                                                 \
(   (ClkFcmFilterClkNone    ==  (clk))      ||                                 \
    (ClkFcmFilterClkFcmSrc  ==  (clk))      ||                                 \
    (ClkFcmFilterClkFcmSrcDiv4  ==  (clk))  ||                                 \
    (ClkFcmFilterClkFcmSrcDiv16 ==  (clk)))

/*! Parameter validity check for fcm abnormal handle \a handle. */
#define IS_FCM_HANDLE(handle)                                                  \
(   (ClkFcmHandleInterrupt  ==  (handle))   ||                                 \
    (ClkFcmHandleReset      ==  (handle)))

/*! Parameter validity check for debug clock division \a div. */
#define IS_TPIUCLK_DIV_VALID(div)                                              \
(   (ClkTpiuclkDiv1         ==  (div))      ||                                 \
    (ClkTpiuclkDiv2         ==  (div))      ||                                 \
    (ClkTpiuclkDiv4         ==  (div)))

/*! Parameter validity check for output clock division \a div. */
#define IS_OUTPUTCLK_DIV_VALID(div)                                            \
(   (ClkOutputDiv1          ==  (div))      ||                                 \
    (ClkOutputDiv2          <=  (div))      &&                                 \
    (ClkOutputDiv128        >=  (div)))

/*! Parameter validity check for fcm measurement source division \a div. */
#define IS_FCM_MEASRC_DIV_VALID(div)                                           \
(   (ClkFcmMeaDiv1          ==  (div))      ||                                 \
    (ClkFcmMeaDiv4          ==  (div))      ||                                 \
    (ClkFcmMeaDiv8          ==  (div))      ||                                 \
    (ClkFcmMeaDiv32         ==  (div)))

/*! Parameter validity check for internal reference source division \a div. */
#define IS_FCM_INTREF_DIV_VALID(div)                                           \
(   (ClkFcmIntrefDiv32      ==  (div))      ||                                 \
    (ClkFcmIntrefDiv128     ==  (div))      ||                                 \
    (ClkFcmIntrefDiv1024    ==  (div))      ||                                 \
    (ClkFcmIntrefDiv8192    ==  (div)))

/*! Parameter validity check for system clock config \a cfg. */
#define IS_SYSCLK_CONFIG_VALID(cfg)                                            \
(   (cfg->enHclkDiv           <=  (cfg->enPclk1Div))    &&                     \
    (cfg->enHclkDiv           <=  (cfg->enPclk3Div))    &&                     \
    (cfg->enHclkDiv           <=  (cfg->enPclk4Div))    &&                     \
    (cfg->enPclk0Div          <=  (cfg->enPclk1Div))    &&                     \
    (cfg->enPclk0Div          <=  (cfg->enPclk3Div))    &&                     \
    ((cfg->enPclk2Div-cfg->enPclk4Div   ==  3)          ||                     \
    (cfg->enPclk2Div-cfg->enPclk4Div    ==  2)          ||                     \
    (cfg->enPclk2Div-cfg->enPclk4Div    ==  1)          ||                     \
    (cfg->enPclk2Div-cfg->enPclk4Div    ==  0)          ||                     \
    (cfg->enPclk4Div-cfg->enPclk2Div    ==  1)          ||                     \
    (cfg->enPclk4Div-cfg->enPclk2Div    ==  2)          ||                     \
    (cfg->enPclk4Div-cfg->enPclk2Div    ==  3)))


/*! Parameter validity check for clock status \a flag. */
#define IS_CLK_FLAG(flag)                                                      \
(   (ClkFlagHRCRdy        ==  (flag))       ||                                 \
    (ClkFlagXTALRdy       ==  (flag))       ||                                 \
    (ClkFlagMPLLRdy       ==  (flag))       ||                                 \
    (ClkFlagUPLLRdy       ==  (flag))       ||                                 \
    (ClkFlagXTALStoppage  ==  (flag)))
/*! Parameter validity check for fcm status \a flag. */
#define IS_FCM_FLAG(flag)                                                      \
(   (ClkFcmFlagOvf          ==  (flag))     ||                                 \
    (ClkFcmFlagMendf        ==  (flag))     ||                                 \
    (ClkFcmFlagErrf         ==  (flag)))


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
 ** \brief  Configures the external high speed oscillator(XTAL).
 **
 ** \param  [in] pstcXtalCfg            The XTAL configures struct.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_XtalConfig(const stc_clk_xtal_cfg_t *pstcXtalCfg)
{
    DDL_ASSERT(NULL != pstcXtalCfg);

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_XTALCFGR_f.SUPDRV = pstcXtalCfg->enFastStartup;
    M4_SYSREG->CMU_XTALCFGR_f.XTALMS = pstcXtalCfg->enMode;
    M4_SYSREG->CMU_XTALCFGR_f.XTALDRV = pstcXtalCfg->enDrv;

    DISABLE_CLOCK_REG_WRITE();
}

/**
 *******************************************************************************
 ** \brief  Configures the XTAL stable time.
 **
 ** \param  [in] enXtalStb              The XTAL stable time.
 **
 ** \retval None
 **
 ** \note   One of the stable clock is 1/8 LRC clock.
 **
 ******************************************************************************/
void CLK_XtalStbConfig(const en_clk_xtal_stb_cycle_t enXtalStb)
{
    DDL_ASSERT(IS_XTAL_STB_VALID(enXtalStb));

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_XTALSTBCR_f.XTALSTB = enXtalStb;

    DISABLE_CLOCK_REG_WRITE();
}

/**
 *******************************************************************************
 ** \brief  Configures the XTAL stoppage.
 **
 ** \param  [in] pstcXtalStpCfg         The XTAL stoppage configures struct.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_XtalStpConfig(const stc_clk_xtal_stp_cfg_t *pstcXtalStpCfg)
{
    DDL_ASSERT(NULL != pstcXtalStpCfg);

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_XTALSTDCR_f.XTALSTDE = pstcXtalStpCfg->enDetect;
    M4_SYSREG->CMU_XTALSTDCR_f.XTALSTDRIS = pstcXtalStpCfg->enMode;
    M4_SYSREG->CMU_XTALSTDCR_f.XTALSTDRE = pstcXtalStpCfg->enModeReset;
    M4_SYSREG->CMU_XTALSTDCR_f.XTALSTDIE = pstcXtalStpCfg->enModeInt;

    DISABLE_CLOCK_REG_WRITE();
}

/**
 *******************************************************************************
 ** \brief  Enable or disable the XTAL.
 **
 ** \param  [in] enNewState             The new state of the XTAL.
 ** \arg    Enable                      Enable XTAL.
 ** \arg    Disable                     Disable XTAL.
 **
 ** \retval None
 **
 ** \note   XTAL can not be stopped if it is used as system clock source or pll
 **         clock source.
 **
 ******************************************************************************/
void CLK_XtalCmd(en_functional_state_t enNewState)
{
    __IO uint32_t timeout = 0;
    en_flag_status_t status;

    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));
    DDL_ASSERT((Enable == enNewState) ?
                1 : IS_XTAL_STOP_VALID(M4_SYSREG->CMU_CKSWR_f.CKSW,
                M4_SYSREG->CMU_PLLCFGR_f.PLLSRC));

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_XTALCR_f.XTALSTP = ((Enable == enNewState) ? 0 : 1);

    DISABLE_CLOCK_REG_WRITE();

    do
    {
        status = CLK_GetFlagStatus(ClkFlagXTALRdy);
        timeout++;
    }while((timeout < CLK_XTAL_TIMEOUT) && (status != ((Enable == enNewState) ? Set : Reset)));
}

/**
 *******************************************************************************
 ** \brief  Configures the external low speed oscillator(XTAL32).
 **
 ** \param  [in] pstcXtal32Cfg          The XTAL32 configures struct.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_Xtal32Config(const stc_clk_xtal32_cfg_t *pstcXtal32Cfg)
{
    DDL_ASSERT(NULL != pstcXtal32Cfg);

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_XTAL32CFGR_f.XTAL32SUPDRV = pstcXtal32Cfg->enFastStartup;
    M4_SYSREG->CMU_XTAL32CFGR_f.XTAL32DRV = pstcXtal32Cfg->enDrv;
    M4_SYSREG->CMU_XTAL32NFR_f.XTAL32NF = pstcXtal32Cfg->enFilterMode;

    DISABLE_CLOCK_REG_WRITE();
}

/**
 *******************************************************************************
 ** \brief  Enable or disable the XTAL32.
 **
 ** \param  [in] enNewState             The new state of the XTAL32.
 ** \arg    Enable                      Enable XTAL32.
 ** \arg    Disable                     Disable XTAL32.
 **
 ** \retval None
 **
 ** \note   XTAL32 can not be stopped if it is used as system clock source.
 **
 ******************************************************************************/
void CLK_Xtal32Cmd(en_functional_state_t enNewState)
{
    __IO uint32_t timeout = 0;

    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));
    DDL_ASSERT(((Enable == enNewState) ?
                1 : IS_XTAL32_STOP_VALID(M4_SYSREG->CMU_CKSWR_f.CKSW)));

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_XTAL32CR_f.XTAL32STP = ((Enable == enNewState) ? 0 : 1);

    DISABLE_CLOCK_REG_WRITE();

    do
    {
        timeout++;
    }while(timeout < CLK_XTAL32_TIMEOUT);
}

/**
 *******************************************************************************
 ** \brief  Trim the internal high speed oscillator(HRC).
 **
 ** \param  [in] trimValue              The trim value.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_HrcTrim(int8_t trimValue)
{
    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_HRCTRM = trimValue;

    DISABLE_CLOCK_REG_WRITE();
}

/**
 *******************************************************************************
 ** \brief  Enable or disable the HRC.
 **
 ** \param  [in] enNewState             The new state of the HRC.
 ** \arg    Enable                      Enable HRC.
 ** \arg    Disable                     Disable HRC.
 **
 ** \retval None
 **
 ** \note   HRC can not be stopped if it is used as system clock source or pll
 **         clock source.
 **
 ******************************************************************************/
void CLK_HrcCmd(en_functional_state_t enNewState)
{
    __IO uint32_t timeout = 0;
    en_flag_status_t status;

    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));
    DDL_ASSERT((Enable == enNewState) ?
                1 : IS_HRC_STOP_VALID(M4_SYSREG->CMU_CKSWR_f.CKSW,
                M4_SYSREG->CMU_PLLCFGR_f.PLLSRC));

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_HRCCR_f.HRCSTP = ((Enable == enNewState) ? 0 : 1);

    DISABLE_CLOCK_REG_WRITE();

    do
    {
        status = CLK_GetFlagStatus(ClkFlagHRCRdy);
        timeout++;
    }while((timeout < CLK_HRC_TIMEOUT) && (status != ((Enable == enNewState) ? Set : Reset)));
}

/**
 *******************************************************************************
 ** \brief  Trim the internal middle speed oscillator(MRC).
 **
 ** \param  [in] trimValue              The trim value.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_MrcTrim(int8_t trimValue)
{
    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_MRCTRM = trimValue;

    DISABLE_CLOCK_REG_WRITE();
}

/**
 *******************************************************************************
 ** \brief  Enable or disable the MRC.
 **
 ** \param  [in] enNewState             The new state of the MRC.
 ** \arg    Enable                      Enable MRC.
 ** \arg    Disable                     Disable MRC.
 **
 ** \retval None
 **
 ** \note   MRC can not be stopped if it is used as system clock source.
 **
 ******************************************************************************/
void CLK_MrcCmd(en_functional_state_t enNewState)
{
    __IO uint32_t timeout = 0;

    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));
    DDL_ASSERT((Enable == enNewState) ?
                1 : IS_MRC_STOP_VALID(M4_SYSREG->CMU_CKSWR_f.CKSW));

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_MRCCR_f.MRCSTP = ((Enable == enNewState) ? 0 : 1);

    DISABLE_CLOCK_REG_WRITE();

    do
    {
        timeout++;
    }while(timeout < CLK_MRC_TIMEOUT);
}

/**
 *******************************************************************************
 ** \brief  Trim the internal low speed oscillator(LRC).
 **
 ** \param  [in] trimValue              The trim value.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_LrcTrim(int8_t trimValue)
{
    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_LRCTRM = trimValue;

    DISABLE_CLOCK_REG_WRITE();
}

/**
 *******************************************************************************
 ** \brief  Enable or disable the LRC.
 **
 ** \param  [in] enNewState             The new state of the LRC.
 ** \arg    Enable                      Enable LRC.
 ** \arg    Disable                     Disable LRC.
 **
 ** \retval None
 **
 ** \note   LRC can not be stopped if it is used as system clock source.
 **
 ******************************************************************************/
void CLK_LrcCmd(en_functional_state_t enNewState)
{
    __IO uint32_t timeout = 0;

    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));
    DDL_ASSERT((Enable == enNewState) ?
                1 : IS_LRC_STOP_VALID(M4_SYSREG->CMU_CKSWR_f.CKSW));

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_LRCCR_f.LRCSTP = ((Enable == enNewState) ? 0 : 1);

    DISABLE_CLOCK_REG_WRITE();

    do
    {
        timeout++;
    }while(timeout < CLK_LRC_TIMEOUT);
}

/**
 *******************************************************************************
 ** \brief  Select pll clock source.
 **
 ** \param  [in] enPllSrc               The pll clock source.
 ** \arg    ClkPllSrcXTAL               Select XTAL as pll clock source.
 ** \arg    ClkPllSrcHRC                Select HRC as pll clock source.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_SetPllSource(en_clk_pll_source_t enPllSrc)
{
    DDL_ASSERT(IS_PLL_SOURCE(enPllSrc));

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_PLLCFGR_f.PLLSRC = enPllSrc;

    DISABLE_CLOCK_REG_WRITE();
}

/**
 *******************************************************************************
 ** \brief  Configures the MPLL.
 **
 ** \param  [in] pstcMpllCfg            The MPLL configures struct.
 **
 ** \retval None
 **
 ** \note   The pllsource/pllm is between 1MHz and 24MHz.
 **         The pllsource/pllm*plln is between 240MHz and 480MHz.
 **         The maximum of pllsource/pllm*plln/pllp is 200MHz.
 **
 ******************************************************************************/
void CLK_MpllConfig(const stc_clk_mpll_cfg_t *pstcMpllCfg)
{
    uint32_t vcoIn = 0;
    uint32_t vcoOut = 0;

    DDL_ASSERT(NULL != pstcMpllCfg);
    DDL_ASSERT(IS_PLL_DIV_VALID(pstcMpllCfg->PllpDiv));
    DDL_ASSERT(IS_PLLQ_DIV_VALID(pstcMpllCfg->PllqDiv));
    DDL_ASSERT(IS_PLL_DIV_VALID(pstcMpllCfg->PllrDiv));
    DDL_ASSERT(IS_PLLN_VALID(pstcMpllCfg->plln));
    DDL_ASSERT(IS_PLLM_VALID(pstcMpllCfg->pllmDiv));

    vcoIn = ((ClkPllSrcXTAL == M4_SYSREG->CMU_PLLCFGR_f.PLLSRC ?
              XTAL_VALUE : HRC_VALUE) / pstcMpllCfg->pllmDiv);
    vcoOut = vcoIn * pstcMpllCfg->plln;

    DDL_ASSERT(IS_PLL_VCO_IN_VALID(vcoIn));
    DDL_ASSERT(IS_PLL_VCO_OUT_VALID(vcoOut));

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_PLLCFGR_f.MPLLP = pstcMpllCfg->PllpDiv - 1;
    M4_SYSREG->CMU_PLLCFGR_f.MPLLQ = pstcMpllCfg->PllqDiv - 1;
    M4_SYSREG->CMU_PLLCFGR_f.MPLLR = pstcMpllCfg->PllrDiv - 1;
    M4_SYSREG->CMU_PLLCFGR_f.MPLLN = pstcMpllCfg->plln - 1;
    M4_SYSREG->CMU_PLLCFGR_f.MPLLM = pstcMpllCfg->pllmDiv - 1;

    DISABLE_CLOCK_REG_WRITE();
}

/**
 *******************************************************************************
 ** \brief  Enable or disable the MPLL.
 **
 ** \param  [in] enNewState             The new state of the MPLL.
 ** \arg    Enable                      Enable MPLL.
 ** \arg    Disable                     Disable MPLL.
 **
 ** \retval None
 **
 ** \note   MPLL can not be stopped if it is used as system clock source.
 **
 ******************************************************************************/
void CLK_MpllCmd(en_functional_state_t enNewState)
{
    __IO uint32_t timeout = 0;
    en_flag_status_t status;

    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));
    DDL_ASSERT((Enable == enNewState) ?
                1 : IS_MPLL_STOP_VALID(M4_SYSREG->CMU_CKSWR_f.CKSW));

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_PLLCR_f.MPLLOFF = ((Enable == enNewState) ? 0 : 1);

    DISABLE_CLOCK_REG_WRITE();

    do
    {
        status = CLK_GetFlagStatus(ClkFlagMPLLRdy);
        timeout++;
    }while((timeout < CLK_MPLL_TIMEOUT) && (status != ((Enable == enNewState) ? Set : Reset)));
}

/**
 *******************************************************************************
 ** \brief  Configures the UPLL.
 **
 ** \param  [in] pstcUpllCfg            The UPLL configures struct.
 **
 ** \retval None
 **
 ** \note   The pllsource/pllm is between 1MHz and 24MHz.
 **         The pllsource/pllm*plln is between 240MHz and 480MHz.
 **         The maximum of pllsource/pllm*plln/pllp is 200MHz.
 **
 ******************************************************************************/
void CLK_UpllConfig(const stc_clk_upll_cfg_t *pstcUpllCfg)
{
    uint32_t vcoIn = 0;
    uint32_t vcoOut = 0;

    DDL_ASSERT(NULL != pstcUpllCfg);
    DDL_ASSERT(IS_PLL_DIV_VALID(pstcUpllCfg->PllpDiv));
    DDL_ASSERT(IS_PLL_DIV_VALID(pstcUpllCfg->PllqDiv));
    DDL_ASSERT(IS_PLL_DIV_VALID(pstcUpllCfg->PllrDiv));
    DDL_ASSERT(IS_PLLN_VALID(pstcUpllCfg->plln));
    DDL_ASSERT(IS_UPLLM_VALID(pstcUpllCfg->pllmDiv));


    vcoIn = ((ClkPllSrcXTAL == M4_SYSREG->CMU_PLLCFGR_f.PLLSRC ?
            XTAL_VALUE : HRC_VALUE) / pstcUpllCfg->pllmDiv);
    vcoOut = vcoIn * pstcUpllCfg->plln;

    DDL_ASSERT(IS_PLL_VCO_IN_VALID(vcoIn));
    DDL_ASSERT(IS_PLL_VCO_OUT_VALID(vcoOut));

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_UPLLCFGR_f.UPLLP = pstcUpllCfg->PllpDiv - 1;
    M4_SYSREG->CMU_UPLLCFGR_f.UPLLQ = pstcUpllCfg->PllqDiv - 1;
    M4_SYSREG->CMU_UPLLCFGR_f.UPLLR = pstcUpllCfg->PllrDiv - 1;
    M4_SYSREG->CMU_UPLLCFGR_f.UPLLN = pstcUpllCfg->plln - 1;
    M4_SYSREG->CMU_UPLLCFGR_f.UPLLM = pstcUpllCfg->pllmDiv - 1;

    DISABLE_CLOCK_REG_WRITE();
}

/**
 *******************************************************************************
 ** \brief  Enable or disable the UPLL.
 **
 ** \param  [in] enNewState             The new state of the UPLL.
 ** \arg    Enable                      Enable UPLL.
 ** \arg    Disable                     Disable UPLL.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_UpllCmd(en_functional_state_t enNewState)
{
    __IO uint32_t timeout = 0;
    en_flag_status_t status;

    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_UPLLCR_f.UPLLOFF = ((Enable == enNewState) ? 0 : 1);

    DISABLE_CLOCK_REG_WRITE();

    do
    {
        status = CLK_GetFlagStatus(ClkFlagUPLLRdy);
        timeout++;
    }while((timeout < CLK_UPLL_TIMEOUT) && (status != ((Enable == enNewState) ? Set : Reset)));
}


/**
 *******************************************************************************
 ** \brief  Select system clock source.
 **
 ** \param  [in] enTargetSysSrc         The system clock source.
 ** \arg    ClkSysSrcHRC                Select HRC as system clock source.
 ** \arg    ClkSysSrcMRC                Select MRC as system clock source.
 ** \arg    ClkSysSrcLRC                Select LRC as system clock source.
 ** \arg    ClkSysSrcXTAL               Select XTAL as system clock source.
 ** \arg    ClkSysSrcXTAL32             Select XTAL32 as system clock source.
 ** \arg    CLKSysSrcMPLL               Select MPLL as system clock source.
 **
 ** \retval None
 **
 ** \note   Must close all of the fcg register before switch system clock source.
 **
 ******************************************************************************/
void CLK_SetSysClkSource(en_clk_sys_source_t enTargetSysSrc)
{
    __IO uint32_t timeout = 0;
    __IO uint32_t fcg0 = M4_MSTP->FCG0;
    __IO uint32_t fcg1 = M4_MSTP->FCG1;
    __IO uint32_t fcg2 = M4_MSTP->FCG2;
    __IO uint32_t fcg3 = M4_MSTP->FCG3;

    DDL_ASSERT(IS_SYSCLK_SOURCE(enTargetSysSrc));

    /* Only current system clock source or target system clock source is MPLL
    need to close fcg0~fcg3 and open fcg0~fcg3 during switch system clock source.
    We need to backup fcg0~fcg3 before close them. */
    if(CLKSysSrcMPLL == M4_SYSREG->CMU_CKSWR_f.CKSW ||
        CLKSysSrcMPLL == enTargetSysSrc)
    {
        /* Close fcg0~fcg3. */
        M4_MSTP->FCG0 = DEFAULT_FCG0;
        M4_MSTP->FCG1 = DEFAULT_FCG1;
        M4_MSTP->FCG2 = DEFAULT_FCG2;
        M4_MSTP->FCG3 = DEFAULT_FCG3;

        /* Wait stable after close fcg. */
        do
        {
            timeout++;
        }while(timeout < CLK_FCG_STABLE);
    }

    /* Switch to target system clock source. */
    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_CKSWR_f.CKSW = enTargetSysSrc;

    DISABLE_CLOCK_REG_WRITE();

    timeout = 0;
    do
    {
        timeout++;
    }while(timeout < CLK_SYSCLK_STABLE);

    /* Open fcg0~fcg3. */
    M4_MSTP->FCG0 = fcg0;
    M4_MSTP->FCG1 = fcg1;
    M4_MSTP->FCG2 = fcg2;
    M4_MSTP->FCG3 = fcg3;

    /* Wait stable after open fcg. */
    timeout = 0;
    do
    {
        timeout++;
    }while(timeout < CLK_FCG_STABLE);

    SystemCoreClockUpdate();
}

/**
 *******************************************************************************
 ** \brief  Get system clock source.
 **
 ** \param  None
 **
 ** \retval The system clock source.
 **
 ** \note   None
 **
 ******************************************************************************/
en_clk_sys_source_t CLK_GetSysClkSource(void)
{
    return (en_clk_sys_source_t)M4_SYSREG->CMU_CKSWR_f.CKSW;
}


/**
 *******************************************************************************
 ** \brief  Configures the division factor for hclk,exck,pclk0,pclk1,pclk2,pclk3,
 **         pclk4 from system clock.
 **
 ** \param  [in] pstcSysclkCfg          The system clock configures struct.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_SysClkConfig(const stc_clk_sysclk_cfg_t *pstcSysclkCfg)
{
    __IO uint32_t timeout = 0;
    __IO uint32_t fcg0 = M4_MSTP->FCG0;
    __IO uint32_t fcg1 = M4_MSTP->FCG1;
    __IO uint32_t fcg2 = M4_MSTP->FCG2;
    __IO uint32_t fcg3 = M4_MSTP->FCG3;

    DDL_ASSERT(NULL != pstcSysclkCfg);
    DDL_ASSERT(IS_SYSCLK_CONFIG_VALID(pstcSysclkCfg));

    /* Only current system clock source is MPLL need to close fcg0~fcg3 and
    open fcg0~fcg3 during switch system clock division.
    We need to backup fcg0~fcg3 before close them. */
    if(CLKSysSrcMPLL == M4_SYSREG->CMU_CKSWR_f.CKSW)
    {
        /* Close fcg0~fcg3. */
        M4_MSTP->FCG0 = DEFAULT_FCG0;
        M4_MSTP->FCG1 = DEFAULT_FCG1;
        M4_MSTP->FCG2 = DEFAULT_FCG2;
        M4_MSTP->FCG3 = DEFAULT_FCG3;

        /* Wait stable after close fcg. */
        do
        {
            timeout++;
        }while(timeout < CLK_FCG_STABLE);
    }

    /* Switch to target system clock division. */
    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_SCFGR = ( pstcSysclkCfg->enPclk0Div                         |
                            (pstcSysclkCfg->enPclk1Div << 4)                   |
                            (pstcSysclkCfg->enPclk2Div << 8)                   |
                            (pstcSysclkCfg->enPclk3Div << 12)                  |
                            (pstcSysclkCfg->enPclk4Div << 16)                  |
                            (pstcSysclkCfg->enExclkDiv << 20)                  |
                            (pstcSysclkCfg->enHclkDiv << 24));

    DISABLE_CLOCK_REG_WRITE();

    timeout = 0;
    do
    {
        timeout++;
    }while(timeout < CLK_SYSCLK_STABLE);

    /* Open fcg0~fcg3. */
    M4_MSTP->FCG0 = fcg0;
    M4_MSTP->FCG1 = fcg1;
    M4_MSTP->FCG2 = fcg2;
    M4_MSTP->FCG3 = fcg3;

    /* Wait stable after open fcg. */
    timeout = 0;
    do
    {
        timeout++;
    }while(timeout < CLK_FCG_STABLE);

}


/**
 *******************************************************************************
 ** \brief  Get clock frequency.
 **
 ** \param  [in] pstcClkFreq            The clock source struct.
 **
 ** \retval The clock frequency include system clock,hclk,exck,pclk0,pclk1,pclk2
 **         pclk3,pclk4.
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_GetClockFreq(stc_clk_freq_t *pstcClkFreq)
{
    uint8_t plln = 0, pllp = 0, pllm = 0, pllsource = 0;

    DDL_ASSERT(NULL != pstcClkFreq);

    /* Get system clock. */
    switch(M4_SYSREG->CMU_CKSWR_f.CKSW)
    {
        case ClkSysSrcHRC:
            /* HRC used as system clock. */
            pstcClkFreq->sysclkFreq = HRC_VALUE;
            break;
        case ClkSysSrcMRC:
            /* MRC used as system clock. */
            pstcClkFreq->sysclkFreq = MRC_VALUE;
            break;
        case ClkSysSrcLRC:
            /* LRC used as system clock. */
            pstcClkFreq->sysclkFreq = LRC_VALUE;
            break;
        case ClkSysSrcXTAL:
            /* XTAL used as system clock. */
            pstcClkFreq->sysclkFreq = XTAL_VALUE;
            break;
        case ClkSysSrcXTAL32:
            /* XTAL32 used as system clock. */
            pstcClkFreq->sysclkFreq = XTAL32_VALUE;
            break;
        default:
            /* MPLLP used as system clock. */
            pllsource = M4_SYSREG->CMU_PLLCFGR_f.PLLSRC;
            pllp = M4_SYSREG->CMU_PLLCFGR_f.MPLLP;
            plln = M4_SYSREG->CMU_PLLCFGR_f.MPLLN;
            pllm = M4_SYSREG->CMU_PLLCFGR_f.MPLLM;

            /* PLLCLK = ((pllsrc / pllm) * plln) / pllp */
            if (ClkPllSrcXTAL == pllsource)
            {
                pstcClkFreq->sysclkFreq = (XTAL_VALUE)/(pllm+1)*(plln+1)/(pllp+1);
            }
            else if (ClkPllSrcHRC == pllsource)
            {
                pstcClkFreq->sysclkFreq = (HRC_VALUE)/(pllm+1)*(plln+1)/(pllp+1);
            }
            break;
    }

    /* Get hclk. */
    pstcClkFreq->hclkFreq = pstcClkFreq->sysclkFreq >> M4_SYSREG->CMU_SCFGR_f.HCLKS;

    /* Get exck. */
    pstcClkFreq->exckFreq = pstcClkFreq->sysclkFreq >> M4_SYSREG->CMU_SCFGR_f.EXCKS;

    /* Get pclk0. */
    pstcClkFreq->pclk0Freq = pstcClkFreq->sysclkFreq >> M4_SYSREG->CMU_SCFGR_f.PCLK0S;

    /* Get pclk1. */
    pstcClkFreq->pclk1Freq = pstcClkFreq->sysclkFreq >> M4_SYSREG->CMU_SCFGR_f.PCLK1S;

    /* Get pclk2. */
    pstcClkFreq->pclk2Freq = pstcClkFreq->sysclkFreq >> M4_SYSREG->CMU_SCFGR_f.PCLK2S;

    /* Get pclk3. */
    pstcClkFreq->pclk3Freq = pstcClkFreq->sysclkFreq >> M4_SYSREG->CMU_SCFGR_f.PCLK3S;

    /* Get pclk4. */
    pstcClkFreq->pclk4Freq = pstcClkFreq->sysclkFreq >> M4_SYSREG->CMU_SCFGR_f.PCLK4S;
}

/**
 *******************************************************************************
 ** \brief  Get PLL clock frequency.
 **
 ** \param  [in] pstcPllClkFreq        The PLL clock source struct.
 **
 ** \retval The clock frequency include mpllp, mpllq, mpllr, upllp, upllq, upllr.
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_GetPllClockFreq(stc_pll_clk_freq_t *pstcPllClkFreq)
{
    uint8_t pllsource;
    uint8_t mplln = 0, mpllp = 0, mpllq = 0, mpllr = 0, mpllm = 0;
    uint8_t uplln = 0, upllp = 0, upllq = 0, upllr = 0, upllm = 0;

    /* Get pll clock source */
    pllsource = M4_SYSREG->CMU_PLLCFGR_f.PLLSRC;

    /* Get Mpll parameter value */
    mpllp = M4_SYSREG->CMU_PLLCFGR_f.MPLLP;
    mpllq = M4_SYSREG->CMU_PLLCFGR_f.MPLLQ;
    mpllr = M4_SYSREG->CMU_PLLCFGR_f.MPLLR;
    mplln = M4_SYSREG->CMU_PLLCFGR_f.MPLLN;
    mpllm = M4_SYSREG->CMU_PLLCFGR_f.MPLLM;

    /* Get Upll paramter value */
    upllp = M4_SYSREG->CMU_UPLLCFGR_f.UPLLP;
    upllq = M4_SYSREG->CMU_UPLLCFGR_f.UPLLQ;
    upllr = M4_SYSREG->CMU_UPLLCFGR_f.UPLLR;
    uplln = M4_SYSREG->CMU_UPLLCFGR_f.UPLLN;
    upllm = M4_SYSREG->CMU_UPLLCFGR_f.UPLLM;

    /* Get mpllp ,mpllr, mpllq, upllp, upllq, upllr clock frequency */
    if (ClkPllSrcXTAL == pllsource)
    {
        pstcPllClkFreq->mpllp = (XTAL_VALUE)/(mpllm+1)*(mplln+1)/(mpllp+1);
        pstcPllClkFreq->mpllq = (XTAL_VALUE)/(mpllm+1)*(mplln+1)/(mpllq+1);
        pstcPllClkFreq->mpllr = (XTAL_VALUE)/(mpllm+1)*(mplln+1)/(mpllr+1);
        pstcPllClkFreq->upllp = (XTAL_VALUE)/(upllm+1)*(uplln+1)/(upllp+1);
        pstcPllClkFreq->upllq = (XTAL_VALUE)/(upllm+1)*(uplln+1)/(upllq+1);
        pstcPllClkFreq->upllr = (XTAL_VALUE)/(upllm+1)*(uplln+1)/(upllr+1);
    }
    else if (ClkPllSrcHRC == pllsource)
    {
        pstcPllClkFreq->mpllp = (HRC_VALUE)/(mpllm+1)*(mplln+1)/(mpllp+1);
        pstcPllClkFreq->mpllq = (HRC_VALUE)/(mpllm+1)*(mplln+1)/(mpllq+1);
        pstcPllClkFreq->mpllr = (HRC_VALUE)/(mpllm+1)*(mplln+1)/(mpllr+1);
        pstcPllClkFreq->upllp = (HRC_VALUE)/(upllm+1)*(uplln+1)/(upllp+1);
        pstcPllClkFreq->upllq = (HRC_VALUE)/(upllm+1)*(uplln+1)/(upllq+1);
        pstcPllClkFreq->upllr = (HRC_VALUE)/(upllm+1)*(uplln+1)/(upllr+1);
    }
}

/**
 *******************************************************************************
 ** \brief  Select usb clock source.
 **
 ** \param  [in] enTargetUsbSrc         The usb clock source.
 ** \arg    ClkUsbSrcSysDiv2            Select 1/2 system clock as usb clock source.
 ** \arg    ClkUsbSrcSysDiv3            Select 1/3 system clock as usb clock source.
 ** \arg    ClkUsbSrcSysDiv4            Select 1/4 system clock as usb clock source.
 ** \arg    ClkUsbSrcMpllp              Select MPLLP as usb clock source.
 ** \arg    ClkUsbSrcMpllq              Select MPLLQ as usb clock source.
 ** \arg    ClkUsbSrcMpllr              Select MPLLR as usb clock source.
 ** \arg    ClkUsbSrcUpllp              Select UPLLP as usb clock source.
 ** \arg    ClkUsbSrcUpllq              Select UPLLQ as usb clock source.
 ** \arg    ClkUsbSrcUpllr              Select UPLLR as usb clock source.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_SetUsbClkSource(en_clk_usb_source_t enTargetUsbSrc)
{
    __IO uint32_t timeout = 0;
    __IO uint32_t fcg0 = M4_MSTP->FCG0;
    __IO uint32_t fcg1 = M4_MSTP->FCG1;
    __IO uint32_t fcg2 = M4_MSTP->FCG2;
    __IO uint32_t fcg3 = M4_MSTP->FCG3;

    DDL_ASSERT(IS_USBCLK_SOURCE(enTargetUsbSrc));

    /* Only current system clock source is MPLL need to close fcg0~fcg3 and
    open fcg0~fcg3 during switch USB clock source.
    We need to backup fcg0~fcg3 before close them. */
    if(CLKSysSrcMPLL == M4_SYSREG->CMU_CKSWR_f.CKSW)
    {
        /* Close fcg0~fcg3.Set bit to close it. */
        BIT_SET(M4_MSTP->FCG0,  BIT_VALUE(FCG0_OFFSET_FCM));
        BIT_SET(M4_MSTP->FCG1,  BIT_VALUE(FCG1_OFFSET_CAN)  |
                                BIT_VALUE(FCG1_OFFSET_QSPI) |
                                BIT_VALUE(FCG1_OFFSET_USBFS)|
                                BIT_VALUE(FCG1_OFFSET_SPI1) |
                                BIT_VALUE(FCG1_OFFSET_SPI2) |
                                BIT_VALUE(FCG1_OFFSET_SPI3) |
                                BIT_VALUE(FCG1_OFFSET_SPI4));

        M4_MSTP->FCG2 = DEFAULT_FCG2;

        BIT_SET(M4_MSTP->FCG3,  BIT_VALUE(FCG3_OFFSET_ADC1) |
                                BIT_VALUE(FCG3_OFFSET_ADC2) |
                                BIT_VALUE(FCG3_OFFSET_DAC));

        /* Wait stable after close fcg. */
        do
        {
            timeout++;
        }while(timeout < CLK_FCG_STABLE);
    }

    /* Switch to target usb clock source. */
    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_UFSCKCFGR_f.USBCKS = enTargetUsbSrc;

    DISABLE_CLOCK_REG_WRITE();

    timeout = 0;
    do
    {
        timeout++;
    }while(timeout < CLK_USBCLK_STABLE);

    /* Open fcg0~fcg3. Write the backup value. */
    M4_MSTP->FCG0 = fcg0;
    M4_MSTP->FCG1 = fcg1;
    M4_MSTP->FCG2 = fcg2;
    M4_MSTP->FCG3 = fcg3;

    /* Wait stable after open fcg. */
    timeout = 0;
    do
    {
        timeout++;
    }while(timeout < CLK_FCG_STABLE);

    if(M4_SYSREG->PWR_STPMCR_f.CKSMRC == 1u)
    {
        M4_SYSREG->CMU_UFSCKCFGR_f.USBCKS = ClkUsbSrcSysDiv4;
    }
}


/**
 *******************************************************************************
 ** \brief  Select peripheral(adc/trng) clock source.
 **
 ** \param  [in] enTargetPeriSrc        The  peripheral(adc/trng) clock source.
 ** \arg    ClkPeriSrcPclk              Select PCLK2 as adc analog clok, PCLK4 as adc digital clock. Select PCLK4 as trng clock.
 ** \arg    ClkPeriSrcMpllp             Select MPLLP as peripheral(adc/trng) clock source.
 ** \arg    ClkPeriSrcMpllq             Select MPLLQ as peripheral(adc/trng) clock source.
 ** \arg    ClkPeriSrcMpllr             Select MPLLR as peripheral(adc/trng) clock source.
 ** \arg    ClkPeriSrcUpllp             Select UPLLP as peripheral(adc/trng) clock source.
 ** \arg    ClkPeriSrcUpllq             Select UPLLQ as peripheral(adc/trng) clock source.
 ** \arg    ClkPeriSrcUpllr             Select UPLLR as peripheral(adc/trng) clock source.
 **
 ** \retval None
 **
 ** \note   Must close usb,ether,exbus,can,qspi,spi,timer,fcm,adc and dac
 **         before switch adc clock source.
 **
 ******************************************************************************/
void CLK_SetPeriClkSource(en_clk_peri_source_t enTargetPeriSrc)
{
    __IO uint32_t timeout = 0;
    DDL_ASSERT(IS_PERICLK_SOURCE(enTargetPeriSrc));

    ENABLE_CLOCK1_REG_WRITE();

    /* Switch to target adc clock source. */
    M4_SYSREG->CMU_PERICKSEL_f.PERICKSEL = enTargetPeriSrc;

    DISABLE_CLOCK1_REG_WRITE();

    if(CLKSysSrcMPLL == M4_SYSREG->CMU_CKSWR_f.CKSW)
    {
        /* Wait stable after switch adc clock source.
        Only current system clock source or target system clock source is MPLL
        need to wait stable. */
        timeout = 0;
        do
        {
            timeout++;
        }while(timeout < CLK_USBCLK_STABLE);
    }
}

/**
 *******************************************************************************
 ** \brief  Select I2S clock source.
 **
 ** \param  [in] pstcI2sReg             Pointer to I2S register
 ** \arg    M4_I2S1                     I2s channel 1
 ** \arg    M4_I2S2                     I2s channel 2
 ** \arg    M4_I2S3                     I2s channel 3
 ** \arg    M4_I2S4                     I2s channel 4
 ** \param  [in] enTargetPeriSrc        The  I2S clock source.
 ** \arg    ClkPeriSrcPclk              Select PCLK3 as I2S clock source.
 ** \arg    ClkPeriSrcMpllp             Select MPLLP as I2S clock source.
 ** \arg    ClkPeriSrcMpllq             Select MPLLQ as I2S clock source.
 ** \arg    ClkPeriSrcMpllr             Select MPLLR as I2S clock source.
 ** \arg    ClkPeriSrcUpllp             Select UPLLP as I2S clock source.
 ** \arg    ClkPeriSrcUpllq             Select UPLLQ as I2S clock source.
 ** \arg    ClkPeriSrcUpllr             Select UPLLR as I2S clock source.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_SetI2sClkSource(M4_I2S_TypeDef* pstcI2sReg, en_clk_peri_source_t enTargetPeriSrc)
{
    DDL_ASSERT(IS_PERICLK_SOURCE(enTargetPeriSrc));

    ENABLE_CLOCK1_REG_WRITE();

    if(M4_I2S1 == pstcI2sReg)
    {
        M4_SYSREG->CMU_I2SCKSEL_f.I2S1CKSEL = enTargetPeriSrc;
    }
    else if(M4_I2S2 == pstcI2sReg)
    {
        M4_SYSREG->CMU_I2SCKSEL_f.I2S2CKSEL = enTargetPeriSrc;
    }
    else if(M4_I2S3 == pstcI2sReg)
    {
        M4_SYSREG->CMU_I2SCKSEL_f.I2S3CKSEL = enTargetPeriSrc;
    }
    else if(M4_I2S4 == pstcI2sReg)
    {
        M4_SYSREG->CMU_I2SCKSEL_f.I2S4CKSEL = enTargetPeriSrc;
    }

    DISABLE_CLOCK1_REG_WRITE();
}

/**
 *******************************************************************************
 ** \brief  Get I2S clock source.
 **
 ** \param  [in] pstcI2sReg             Pointer to I2S register
 ** \arg    M4_I2S1                     I2s channel 1
 ** \arg    M4_I2S2                     I2s channel 2
 ** \arg    M4_I2S3                     I2s channel 3
 ** \arg    M4_I2S4                     I2s channel 4
 **
 ** \retval en_clk_peri_source_t        The  I2S clock source.
 **
 ** \note   None
 **
 ******************************************************************************/
en_clk_peri_source_t CLK_GetI2sClkSource(M4_I2S_TypeDef* pstcI2sReg)
{
    en_clk_peri_source_t enI2sClkSource;

    if(M4_I2S1 == pstcI2sReg)
    {
        enI2sClkSource = (en_clk_peri_source_t)M4_SYSREG->CMU_I2SCKSEL_f.I2S1CKSEL;
    }
    else if(M4_I2S2 == pstcI2sReg)
    {
        enI2sClkSource = (en_clk_peri_source_t)M4_SYSREG->CMU_I2SCKSEL_f.I2S2CKSEL;
    }
    else if(M4_I2S3 == pstcI2sReg)
    {
        enI2sClkSource = (en_clk_peri_source_t)M4_SYSREG->CMU_I2SCKSEL_f.I2S3CKSEL;
    }
    else if(M4_I2S4 == pstcI2sReg)
    {
        enI2sClkSource = (en_clk_peri_source_t)M4_SYSREG->CMU_I2SCKSEL_f.I2S4CKSEL;
    }

    return enI2sClkSource;
}

/**
 *******************************************************************************
 ** \brief  Configures the debug clock.
 **
 ** \param  [in] enTpiuDiv              The division of debug clock from system
 **                                     clock.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_TpiuClkConfig(const en_clk_tpiuclk_div_factor_t enTpiuDiv)
{
    DDL_ASSERT(IS_TPIUCLK_DIV_VALID(enTpiuDiv));

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_TPIUCKCFGR_f.TPIUCKS = enTpiuDiv;

    DISABLE_CLOCK_REG_WRITE();
}

/**
 *******************************************************************************
 ** \brief  Enable or disable the debug clock.
 **
 ** \param  [in] enNewState             The new state of the debug clock.
 ** \arg    Enable                      Enable debug clock.
 ** \arg    Disable                     Disable debug clock.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_TpiuClkCmd(en_functional_state_t enNewState)
{
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    ENABLE_CLOCK_REG_WRITE();

    M4_SYSREG->CMU_TPIUCKCFGR_f.TPIUCKOE = enNewState;

    DISABLE_CLOCK_REG_WRITE();
}

/**
 *******************************************************************************
 ** \brief  Configures the output clock.
 **
 ** \param  [in] enCh                   The clock output channel.
 ** \param  [in] pstcOutputCfg          The clock output configures struct.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_OutputClkConfig(en_clk_output_ch_t enCh, const stc_clk_output_cfg_t *pstcOutputCfg)
{
    DDL_ASSERT(NULL != pstcOutputCfg);
    DDL_ASSERT(IS_OUTPUTCLK_CHANNEL(enCh));
    DDL_ASSERT(IS_OUTPUTCLK_SOURCE(pstcOutputCfg->enOutputSrc));
    DDL_ASSERT(IS_OUTPUTCLK_DIV_VALID(pstcOutputCfg->enOutputDiv));

    ENABLE_CLOCK_REG_WRITE();

    switch(enCh)
    {
        case ClkOutputCh1:
            M4_SYSREG->CMU_MCO1CFGR_f.MCO1SEL = pstcOutputCfg->enOutputSrc;
            M4_SYSREG->CMU_MCO1CFGR_f.MCO1DIV = pstcOutputCfg->enOutputDiv;
            break;
        case ClkOutputCh2:
            M4_SYSREG->CMU_MCO2CFGR_f.MCO2SEL = pstcOutputCfg->enOutputSrc;
            M4_SYSREG->CMU_MCO2CFGR_f.MCO2DIV = pstcOutputCfg->enOutputDiv;
            break;
    }

    DISABLE_CLOCK_REG_WRITE();
}

/**
 *******************************************************************************
 ** \brief  Enable or disable the clock output.
 **
 ** \param  [in] enCh                   The clock output channel.
 ** \param  [in] enNewState             The new state of the clock output.
 ** \arg    Enable                      Enable clock output.
 ** \arg    Disable                     Disable clock output.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_OutputClkCmd(en_clk_output_ch_t enCh, en_functional_state_t enNewState)
{
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    ENABLE_CLOCK_REG_WRITE();

    switch(enCh)
    {
        case ClkOutputCh1:
            M4_SYSREG->CMU_MCO1CFGR_f.MCO1EN = enNewState;
            break;
        case ClkOutputCh2:
            M4_SYSREG->CMU_MCO2CFGR_f.MCO2EN = enNewState;
            break;
    }

    DISABLE_CLOCK_REG_WRITE();
}


/**
 *******************************************************************************
 ** \brief  Get the specified clock flag status.
 **
 ** \param  [in] enClkFlag              The specified clock flag.
 ** \arg    ClkFlagHRCRdy               HRC is ready or not.
 ** \arg    ClkFlagXTALRdy              XTAL is ready or not.
 ** \arg    ClkFlagMPLLRdy              MPLL is ready or not.
 ** \arg    ClkFlagUPLLRdy              UPLL is ready or not.
 ** \arg    ClkFlagXTALStoppage         XTAL is detected stoppage or not.
 **
 ** \retval en_flag_status_t
 **
 ** \note   None
 **
 ******************************************************************************/
en_flag_status_t CLK_GetFlagStatus(en_clk_flag_t enClkFlag)
{
    en_flag_status_t status;

    DDL_ASSERT(IS_CLK_FLAG(enClkFlag));

    switch(enClkFlag)
    {
        case ClkFlagHRCRdy:
            status = ((1 == M4_SYSREG->CMU_OSCSTBSR_f.HRCSTBF) ? Set : Reset);
            break;
        case ClkFlagXTALRdy:
            status = ((1 == M4_SYSREG->CMU_OSCSTBSR_f.XTALSTBF) ? Set : Reset);
            break;
        case ClkFlagMPLLRdy:
            status = ((1 == M4_SYSREG->CMU_OSCSTBSR_f.MPLLSTBF) ? Set : Reset);
            break;
        case ClkFlagUPLLRdy:
            status = ((1 == M4_SYSREG->CMU_OSCSTBSR_f.UPLLSTBF) ? Set : Reset);
            break;
        default:
            status = ((1 == M4_SYSREG->CMU_XTALSTDSR_f.XTALSTDF) ? Set : Reset);
            break;
    }

    return status;
}


/**
 *******************************************************************************
 ** \brief  Configures the clock frequency measurement.
 **
 ** \param  [in] pstcClkFcmCfg          The clock frequency measurement configures
 **                                     struct.
 **
 ** \retval None
 **
 ** \note   Configures the window,measurement,reference and interrupt independently.
 **
 ******************************************************************************/
void CLK_FcmConfig(const stc_clk_fcm_cfg_t *pstcClkFcmCfg)
{
    DDL_ASSERT(NULL != pstcClkFcmCfg);

    /* Window config. */
    if(pstcClkFcmCfg->pstcFcmWindowCfg)
    {
        /* Set window lower. */
        M4_FCM->LVR = pstcClkFcmCfg->pstcFcmWindowCfg->windowLower;
        /* Set window upper. */
        M4_FCM->UVR = pstcClkFcmCfg->pstcFcmWindowCfg->windowUpper;
    }

    /* Measure config. */
    if(pstcClkFcmCfg->pstcFcmMeaCfg)
    {
        DDL_ASSERT(IS_FCM_SOURCE(pstcClkFcmCfg->pstcFcmMeaCfg->enSrc));
        DDL_ASSERT(IS_FCM_MEASRC_DIV_VALID(pstcClkFcmCfg->pstcFcmMeaCfg->enSrcDiv));

        /* Measure source. */
        M4_FCM->MCCR_f.MCKS = pstcClkFcmCfg->pstcFcmMeaCfg->enSrc;
        /* Measure source division. */
        M4_FCM->MCCR_f.MDIVS = pstcClkFcmCfg->pstcFcmMeaCfg->enSrcDiv;
    }

    /* Reference config. */
    if(pstcClkFcmCfg->pstcFcmRefCfg)
    {
        DDL_ASSERT(IS_FCM_REF(pstcClkFcmCfg->pstcFcmRefCfg->enRefSel));
        DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcClkFcmCfg->pstcFcmRefCfg->enExtRef));
        DDL_ASSERT(IS_FCM_SOURCE(pstcClkFcmCfg->pstcFcmRefCfg->enIntRefSrc));
        DDL_ASSERT(IS_FCM_INTREF_DIV_VALID(pstcClkFcmCfg->pstcFcmRefCfg->enIntRefDiv));
        DDL_ASSERT(IS_FCM_EDGE(pstcClkFcmCfg->pstcFcmRefCfg->enEdge));
        DDL_ASSERT(IS_FCM_FILTER_CLK(pstcClkFcmCfg->pstcFcmRefCfg->enFilterClk));

        M4_FCM->RCCR_f.INEXS = pstcClkFcmCfg->pstcFcmRefCfg->enRefSel;
        M4_FCM->RCCR_f.EXREFE = pstcClkFcmCfg->pstcFcmRefCfg->enExtRef;
        M4_FCM->RCCR_f.RCKS = pstcClkFcmCfg->pstcFcmRefCfg->enIntRefSrc;
        M4_FCM->RCCR_f.RDIVS = pstcClkFcmCfg->pstcFcmRefCfg->enIntRefDiv;
        M4_FCM->RCCR_f.EDGES = pstcClkFcmCfg->pstcFcmRefCfg->enEdge;
        M4_FCM->RCCR_f.DNFS = pstcClkFcmCfg->pstcFcmRefCfg->enFilterClk;
    }

    /* Interrupt config. */
    if(pstcClkFcmCfg->pstcFcmIntCfg)
    {
        DDL_ASSERT(IS_FCM_HANDLE(pstcClkFcmCfg->pstcFcmIntCfg->enHandleSel));
        DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcClkFcmCfg->pstcFcmIntCfg->enHandleReset));
        DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcClkFcmCfg->pstcFcmIntCfg->enHandleInterrupt));
        DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcClkFcmCfg->pstcFcmIntCfg->enOvfInterrupt));
        DDL_ASSERT(IS_FUNCTIONAL_STATE(pstcClkFcmCfg->pstcFcmIntCfg->enEndInterrupt));

        M4_FCM->RIER_f.ERRINTRS = pstcClkFcmCfg->pstcFcmIntCfg->enHandleSel;
        M4_FCM->RIER_f.ERRE = pstcClkFcmCfg->pstcFcmIntCfg->enHandleReset;
        M4_FCM->RIER_f.ERRIE = pstcClkFcmCfg->pstcFcmIntCfg->enHandleInterrupt;
        M4_FCM->RIER_f.MENDIE = pstcClkFcmCfg->pstcFcmIntCfg->enEndInterrupt;
        M4_FCM->RIER_f.OVFIE = pstcClkFcmCfg->pstcFcmIntCfg->enOvfInterrupt;
    }
}

/**
 *******************************************************************************
 ** \brief  Enable or disable the clock frequency measurement.
 **
 ** \param  [in] enNewState             The new state of the clock frequency
 **                                     measurement.
 ** \arg    Enable                      Enable clock frequency measurement.
 ** \arg    Disable                     Disable clock frequency measurement.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_FcmCmd(en_functional_state_t enNewState)
{
    DDL_ASSERT(IS_FUNCTIONAL_STATE(enNewState));

    M4_FCM->STR = (Enable == enNewState ? Set : Reset);
}

/**
 *******************************************************************************
 ** \brief  Get fcm counter value.
 **
 ** \param  None
 **
 ** \retval The fcm counter value.
 **
 ** \note   None
 **
 ******************************************************************************/
uint16_t CLK_GetFcmCounter(void)
{
    return M4_FCM->CNTR;
}

/**
 *******************************************************************************
 ** \brief  Get the specified fcm flag status.
 **
 ** \param  [in] enFcmFlag              The specified fcm flag.
 ** \arg    ClkFcmFlagOvf               The fcm counter overflow or not.
 ** \arg    ClkFcmFlagMendf             The end of the measurement or not.
 ** \arg    ClkFcmFlagErrf              Whether the frequency is abnormal or not.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
en_flag_status_t CLK_GetFcmFlag(en_clk_fcm_flag_t enFcmFlag)
{
    en_flag_status_t status;

    DDL_ASSERT(IS_FCM_FLAG(enFcmFlag));

    switch(enFcmFlag)
    {
        case ClkFcmFlagOvf:
            status = ((1 == M4_FCM->SR_f.OVF) ? Set : Reset);
            break;
        case ClkFcmFlagMendf:
            status = ((1 == M4_FCM->SR_f.MENDF) ? Set : Reset);
            break;
        case ClkFcmFlagErrf:
            status = ((1 == M4_FCM->SR_f.ERRF) ? Set : Reset);
            break;
    }

    return status;
}

/**
 *******************************************************************************
 ** \brief  Clear the specified fcm flag status.
 **
 ** \param  [in] enFcmFlag              The specified fcm flag.
 ** \arg    ClkFcmFlagOvf               Clear the fcm counter overflow flag.
 ** \arg    ClkFcmFlagMendf             Clear the end of the measurement flag.
 ** \arg    ClkFcmFlagErrf              Clear the frequency abnormal flag.
 **
 ** \retval None
 **
 ** \note   None
 **
 ******************************************************************************/
void CLK_ClearFcmFlag(en_clk_fcm_flag_t enFcmFlag)
{
    DDL_ASSERT(IS_FCM_FLAG(enFcmFlag));

    switch(enFcmFlag)
    {
        case ClkFcmFlagOvf:
            M4_FCM->CLR_f.OVFCLR = Set;
            break;
        case ClkFcmFlagMendf:
            M4_FCM->CLR_f.MENDFCLR = Set;
            break;
        case ClkFcmFlagErrf:
            M4_FCM->CLR_f.ERRFCLR = Set;
            break;
    }
}

//@} // CmuGroup

#endif /* DDL_CLK_ENABLE */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
