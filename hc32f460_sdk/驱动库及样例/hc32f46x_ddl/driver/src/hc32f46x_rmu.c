/******************************************************************************
 * Copyright (C) 2016, Huada Semiconductor Co.,Ltd All rights reserved.
 *
 * This software is owned and published by:
 * Huada Semiconductor Co.,Ltd. ("HDSC").
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
/** \file hc32f46x_rmu.c
 **
 ** A detailed description is available at
 ** @link RmuGroup RMU description @endlink
 **
 **   - 2018-10-28  1.0  Chengy First version for Device Driver Library of RMU.
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32f46x_rmu.h"
#include "hc32f46x_utility.h"

#if (DDL_RMU_ENABLE == DDL_ON)

/**
 *******************************************************************************
 ** \addtogroup RmuGroup
 ******************************************************************************/
//@{

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define ENABLE_RMU_REG_WRITE()            (M4_SYSREG->PWR_FPRC = 0xa502)
#define DISABLE_RMU_REG_WRITE()           (M4_SYSREG->PWR_FPRC = 0xa500)

#define RMU_FLAG_TIM                      ((uint16_t)0x1000)

#define IS_VALID_POINTER(x)               (NULL != (x))     ///< Parameter validity check for pointer

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
 ** \brief Get the chip reset cause.
 **
 ** \param [in] pstcData          Pointer to return reset cause structure.
 **
 ** \retval Ok                    Get successfully.
 **
 ******************************************************************************/
en_result_t RMU_GetResetCause(stc_rmu_rstcause_t *pstcData)
{
    uint16_t u16RstCause = 0;
    stc_sysreg_rmu_rstf0_field_t *RMU_RSTF0_f = NULL;

    DDL_ASSERT(IS_VALID_POINTER(pstcData));

    if(NULL == pstcData)
    {
        return ErrorInvalidParameter;
    }

    u16RstCause = M4_SYSREG->RMU_RSTF0;
    RMU_RSTF0_f = (stc_sysreg_rmu_rstf0_field_t *)(&u16RstCause);

    pstcData->enMultiRst = ((RMU_RSTF0_f->MULTIRF == 1) ? Set : Reset);
    pstcData->enXtalErr = ((RMU_RSTF0_f->XTALERF == 1) ? Set : Reset);
    pstcData->enClkFreqErr = ((RMU_RSTF0_f->CKFERF == 1) ? Set : Reset);
    pstcData->enRamEcc = ((RMU_RSTF0_f->RAECRF == 1) ? Set : Reset);
    pstcData->enRamParityErr = ((RMU_RSTF0_f->RAPERF == 1) ? Set : Reset);
    pstcData->enMpuErr = ((RMU_RSTF0_f->MPUERF == 1) ? Set : Reset);
    pstcData->enSoftware = ((RMU_RSTF0_f->SWRF == 1) ? Set : Reset);
    pstcData->enPowerDown = ((RMU_RSTF0_f->PDRF == 1) ? Set : Reset);
    pstcData->enSwdt = ((RMU_RSTF0_f->SWDRF == 1) ? Set : Reset);
    pstcData->enWdt = ((RMU_RSTF0_f->WDRF == 1) ? Set : Reset);
    pstcData->enPvd2 = ((RMU_RSTF0_f->PVD2RF == 1) ? Set : Reset);
    pstcData->enPvd1 = ((RMU_RSTF0_f->PVD2RF == 1) ? Set : Reset);
    pstcData->enBrownOut = ((RMU_RSTF0_f->BORF == 1) ? Set : Reset);
    pstcData->enRstPin = ((RMU_RSTF0_f->PINRF == 1) ? Set : Reset);
    pstcData->enPowerOn = ((RMU_RSTF0_f->PORF == 1) ? Set : Reset);

    return Ok;
}

/**
 *******************************************************************************
 ** \brief Clear the reset flag.
 **
 ** \param None
 **
 ** \retval Ok                    Clear successfully.
 **
 ** \note   clear reset flag should be done after read RMU_RSTF0 register.
 ******************************************************************************/
en_result_t RMU_ClrResetFlag(void)
{
    uint16_t u16status = 0;
    uint32_t u32timeout;

    ENABLE_RMU_REG_WRITE();

    do
    {
        u32timeout++;
        M4_SYSREG->RMU_RSTF0_f.CLRF = 1;
        u16status = M4_SYSREG->RMU_RSTF0;
    }while((u32timeout != RMU_FLAG_TIM) && u16status);

     DISABLE_RMU_REG_WRITE();

     if(u32timeout >= RMU_FLAG_TIM)
    {
        return ErrorTimeout;
    }

    return Ok;
}


//@} // RmuGroup

#endif /* DDL_RMU_ENABLE */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

