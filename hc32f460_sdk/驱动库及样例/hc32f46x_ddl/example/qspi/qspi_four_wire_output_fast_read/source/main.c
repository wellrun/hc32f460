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
 ** \brief The example of QSPI four wire output fast read function
 **
 **   - 2018-11-05  1.0  Yangjp First version for Device Driver Library of QSPI.
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
#define LED0_PORT                       PortE
#define LED0_PIN                        Pin06

#define LED0_ON()                       PORT_SetBits(LED0_PORT, LED0_PIN)
#define LED0_OFF()                      PORT_ResetBits(LED0_PORT, LED0_PIN)
#define LED0_TOGGLE()                   PORT_Toggle(LED0_PORT, LED0_PIN)

/* LED1 Port/Pin definition */
#define LED1_PORT                       PortA
#define LED1_PIN                        Pin07

#define LED1_ON()                       PORT_SetBits(LED1_PORT, LED1_PIN)
#define LED1_OFF()                      PORT_ResetBits(LED1_PORT, LED1_PIN)
#define LED1_TOGGLE()                   PORT_Toggle(LED1_PORT, LED1_PIN)

/* KEY0 Port/Pin definition */
#define KEY0_PORT                       PortD
#define KEY0_PIN                        Pin03

/* QSPCK Port/Pin definition */
#define QSPCK_PORT                      PortC
#define QSPCK_PIN                       Pin06

/* QSNSS Port/Pin definition */
#define QSNSS_PORT                      PortC
#define QSNSS_PIN                       Pin07

/* QSIO0 Port/Pin definition */
#define QSIO0_PORT                      PortD
#define QSIO0_PIN                       Pin08

/* QSIO1 Port/Pin definition */
#define QSIO1_PORT                      PortD
#define QSIO1_PIN                       Pin09

/* QSIO2 Port/Pin definition */
#define QSIO2_PORT                      PortD
#define QSIO2_PIN                       Pin10

/* QSIO3 Port/Pin definition */
#define QSIO3_PORT                      PortD
#define QSIO3_PIN                       Pin11

/* QSPI memory bus address definition */
#define QSPI_BUS_ADDRESS                (0x98000000U)

/* FLASH parameters definition */
#define FLASH_PAGE_SIZE                 (0x100u)
#define FLASH_SRCTOR_SIZE               (0x1000u)
#define FALSH_MAX_ADDR                  (0x800000u)
#define FLASH_DUMMY_BYTE_VALUE          (0xffu)
#define FLASH_BUSY_BIT_MASK             (0x01u)

/* FLASH instruction definition */
#define FLASH_INSTR_WRITE_ENABLE        (0x06u)
#define FLASH_INSTR_PAGE_PROGRAM        (0x02u)
#define FLASH_INSTR_ERASE_4KB_SECTOR    (0x20u)
#define FLASH_INSTR_ERASE_CHIP          (0xC7u)
#define FLASH_INSTR_READ_SR1            (0x05u)
#define FLASH_INSTR_READ_SR2            (0x35u)
#define FLASH_INSTR_READ_SR3            (0x15u)

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
static uint8_t u8ExIntFlag = 0;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 *******************************************************************************
 ** \brief ExtInt3 callback function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void ExtInt03_Callback(void)
{
    if (Set == EXINT_IrqFlgGet(ExtiCh03))
    {
        u8ExIntFlag = 1u;
        EXINT_IrqFlgClr(ExtiCh03);
    }
}

/**
 *******************************************************************************
 ** \brief KEY0(SW2) init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void Sw2_Init(void)
{
    stc_port_init_t stcPortInit;
    stc_exint_config_t stcExtiConfig;
    stc_irq_regi_conf_t stcIrqRegiConf;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcExtiConfig);
    MEM_ZERO_STRUCT(stcIrqRegiConf);

    /* Set PD03 as External Int Ch.3 input */
    stcPortInit.enExInt = Enable;
    PORT_Init(KEY0_PORT, KEY0_PIN, &stcPortInit);

    stcExtiConfig.enExitCh = ExtiCh03;
    /* Filter setting */
    stcExtiConfig.enFilterEn = Enable;
    stcExtiConfig.enFltClk = Pclk3Div8;
    /* Both edge */
    stcExtiConfig.enExtiLvl = ExIntFallingEdge;
    EXINT_Init(&stcExtiConfig);

    /* Select External Int Ch.3 */
    stcIrqRegiConf.enIntSrc = INT_PORT_EIRQ3;
    /* Register External Int to Vect.No.007 */
    stcIrqRegiConf.enIRQn = Int007_IRQn;
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
 ** \brief QSPI flash init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void QspiFlash_Init(void)
{
    stc_qspi_init_t stcQspiInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcQspiInit);

    /* Configuration peripheral clock */
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_QSPI, Enable);

    /* Configuration QSPI pin */
    PORT_SetFunc(QSPCK_PORT, QSPCK_PIN, Func_Qspi, Disable);
    PORT_SetFunc(QSNSS_PORT, QSNSS_PIN, Func_Qspi, Disable);
    PORT_SetFunc(QSIO0_PORT, QSIO0_PIN, Func_Qspi, Disable);
    PORT_SetFunc(QSIO1_PORT, QSIO1_PIN, Func_Qspi, Disable);
    PORT_SetFunc(QSIO2_PORT, QSIO2_PIN, Func_Qspi, Disable);
    PORT_SetFunc(QSIO3_PORT, QSIO3_PIN, Func_Qspi, Disable);

    /* Configuration QSPI structure */
    stcQspiInit.enClkDiv = QspiHclkDiv3;
    stcQspiInit.enSpiMode = QspiSpiMode3;
    stcQspiInit.enBusCommMode = QspiBusModeRomAccess;
    stcQspiInit.enPrefetchMode = QspiPrefetchStopComplete;
    stcQspiInit.enPrefetchFuncEn = Disable;
    stcQspiInit.enQssnValidExtendTime = QspiQssnValidExtendSck32;
    stcQspiInit.enQssnIntervalTime = QspiQssnIntervalQsck8;
    stcQspiInit.enQsckDutyCorr = QspiQsckDutyCorrHalfHclk;
    stcQspiInit.enVirtualPeriod = QspiVirtualPeriodQsck8;
    stcQspiInit.enWpPinLevel = QspiWpPinOutputHigh;
    stcQspiInit.enQssnSetupDelayTime = QspiQssnSetupDelay1Dot5Qsck;
    stcQspiInit.enQssnHoldDelayTime = QspiQssnHoldDelay1Dot5Qsck;
    stcQspiInit.enFourByteAddrReadEn = Disable;
    stcQspiInit.enAddrWidth = QspiAddressByteThree;
    stcQspiInit.stcCommProtocol.enReadMode = QspiReadModeFourWiresOutput;
    stcQspiInit.stcCommProtocol.enTransInstrProtocol = QspiProtocolExtendSpi;
    stcQspiInit.stcCommProtocol.enTransAddrProtocol = QspiProtocolExtendSpi;
    stcQspiInit.stcCommProtocol.enReceProtocol = QspiProtocolExtendSpi;
    stcQspiInit.u8RomAccessInstr = QSPI_3BINSTR_FOUR_WIRES_OUTPUT_READ;
    QSPI_Init(&stcQspiInit);
}

/**
 *******************************************************************************
 ** \brief QSPI flash write enable function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void QspiFlash_WriteEnable(void)
{
    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(FLASH_INSTR_WRITE_ENABLE);
    QSPI_ExitDirectCommMode();
}

/**
 *******************************************************************************
 ** \brief QSPI flash wait for write operation end function
 **
 ** \param [in] None
 **
 ** \retval Ok                              Flash internal operation finish
 ** \retval ErrorTimeout                    Flash internal operation timeout
 **
 ******************************************************************************/
en_result_t QspiFlash_WaitForWriteEnd(void)
{
    en_result_t enRet = Ok;
    uint8_t u8Status = 0;
    uint32_t u32Timeout;
    stc_clk_freq_t stcClkFreq;

    CLK_GetClockFreq(&stcClkFreq);
    u32Timeout = stcClkFreq.sysclkFreq / 1000;
    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(FLASH_INSTR_READ_SR1);
    do
    {
        u8Status = QSPI_ReadDirectCommValue();
        u32Timeout--;
    } while ((u32Timeout != 0) &&
             ((u8Status & FLASH_BUSY_BIT_MASK) == FLASH_BUSY_BIT_MASK));

    if (FLASH_BUSY_BIT_MASK == u8Status)
    {
        enRet = ErrorTimeout;
    }
    QSPI_ExitDirectCommMode();

    return enRet;
}

/**
 *******************************************************************************
 ** \brief QSPI flash page write program function
 **
 ** \param [in] u32Addr                     Valid flash address
 **
 ** \param [in] pData                       Pointer to send data buffer
 **
 ** \param [in] len                         Send data length
 **
 ** \retval Error                           Page write program failed
 ** \retval Ok                              Page write program success
 **
 ******************************************************************************/
en_result_t QspiFlash_WritePage(uint32_t u32Addr, uint8_t *pData, uint16_t len)
{
    en_result_t enRet;

    if ((u32Addr > FALSH_MAX_ADDR) || (NULL == pData) || (len > FLASH_PAGE_SIZE))
    {
        return Error;
    }

    QspiFlash_WriteEnable();
    /* Send data to flash */
    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(FLASH_INSTR_PAGE_PROGRAM);
    QSPI_WriteDirectCommValue((u32Addr & 0xFF0000) >> 16);
    QSPI_WriteDirectCommValue((u32Addr & 0xFF00) >> 8);
    QSPI_WriteDirectCommValue(u32Addr & 0xFF);
    while (len--)
    {
        QSPI_WriteDirectCommValue(*pData++);
    }
    QSPI_ExitDirectCommMode();
    /* Wait for flash idle */
    enRet = QspiFlash_WaitForWriteEnd();

    return enRet;
}

/**
 *******************************************************************************
 ** \brief QSPI flash erase 4Kb sector function
 **
 ** \param [in] u32Addr                     Valid flash address
 **
 ** \retval Error                           Sector erase failed
 ** \retval Ok                              Sector erase success
 **
 ******************************************************************************/
en_result_t QspiFlash_Erase4KbSector(uint32_t u32Addr)
{
    en_result_t enRet;

    if (u32Addr >= FALSH_MAX_ADDR)
    {
        return Error;
    }

    QspiFlash_WriteEnable();
    /* Send instruction to flash */
    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(FLASH_INSTR_ERASE_4KB_SECTOR);
    QSPI_WriteDirectCommValue((u32Addr & 0xFF0000) >> 16);
    QSPI_WriteDirectCommValue((u32Addr & 0xFF00) >> 8);
    QSPI_WriteDirectCommValue(u32Addr & 0xFF);
    QSPI_ExitDirectCommMode();
    /* Wait for flash idle */
    enRet = QspiFlash_WaitForWriteEnd();

    return enRet;
}

/**
 *******************************************************************************
 ** \brief QSPI flash erase chip function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void QspiFlash_EraseChip(void)
{
    QspiFlash_WriteEnable();
    /* Send instruction to flash */
    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(FLASH_INSTR_ERASE_CHIP);
    QSPI_ExitDirectCommMode();
    /* Wait for flash idle */
    QspiFlash_WaitForWriteEnd();
}

/**
 *******************************************************************************
 ** \brief QSPI flash read status register function
 **
 ** \param [in] u8Reg                       Need to get status register
 ** \arg FLASH_INSTR_READ_SR1               Status register 1
 ** \arg FLASH_INSTR_READ_SR2               Status register 2
 ** \arg FLASH_INSTR_READ_SR3               Status register 3
 **
 ** \retval uint8_t                         Current register value
 **
 ******************************************************************************/
uint8_t QspiFlash_ReadStatusRegister(uint8_t u8Reg)
{
    uint8_t regSta = 0;

    QSPI_EnterDirectCommMode();
    QSPI_WriteDirectCommValue(u8Reg);
    regSta = QSPI_ReadDirectCommValue();
    QSPI_ExitDirectCommMode();

    return regSta;
}

/**
 *******************************************************************************
 ** \brief System clock init function
 **
 ** \param [in] None
 **
 ** \retval None
 **
 ******************************************************************************/
void SystemClk_Init(void)
{
    en_clk_sys_source_t     enSysClkSrc;
    stc_clk_sysclk_cfg_t    stcSysClkCfg;
    stc_clk_xtal_cfg_t      stcXtalCfg;
    stc_clk_mpll_cfg_t      stcMpllCfg;
    stc_clk_freq_t          stcClkFreq;

    MEM_ZERO_STRUCT(enSysClkSrc);
    MEM_ZERO_STRUCT(stcSysClkCfg);
    MEM_ZERO_STRUCT(stcXtalCfg);
    MEM_ZERO_STRUCT(stcMpllCfg);

    /* Set bus clk div. */
    stcSysClkCfg.enHclkDiv = ClkSysclkDiv1;   // Max 168MHz
    stcSysClkCfg.enExclkDiv = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk0Div = ClkSysclkDiv1;  // Max 168MHz
    stcSysClkCfg.enPclk1Div = ClkSysclkDiv2;  // Max 84MHz
    stcSysClkCfg.enPclk2Div = ClkSysclkDiv4;  // Max 60MHz
    stcSysClkCfg.enPclk3Div = ClkSysclkDiv4;  // Max 42MHz
    stcSysClkCfg.enPclk4Div = ClkSysclkDiv2;  // Max 84MHz
    CLK_SysClkConfig(&stcSysClkCfg);

    /* Switch system clock source to MPLL. */
    /* Use Xtal32 as MPLL source. */
    stcXtalCfg.enMode = ClkXtalModeOsc;
    stcXtalCfg.enDrv = ClkXtalLowDrv;
    stcXtalCfg.enFastStartup = Enable;
    CLK_XtalConfig(&stcXtalCfg);
    CLK_XtalCmd(Enable);

    /* MPLL config. */
    stcMpllCfg.pllmDiv = 1;
    stcMpllCfg.plln = 42;
    stcMpllCfg.PllpDiv = 2;
    stcMpllCfg.PllqDiv = 2;
    stcMpllCfg.PllrDiv = 2;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_5);
    EFM_Lock();

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while (Set != CLK_GetFlagStatus(ClkFlagMPLLRdy));

    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);

    /* Check source and frequence. */
    enSysClkSrc = CLK_GetSysClkSource();
    CLK_GetClockFreq(&stcClkFreq);
}

/**
 *******************************************************************************
 ** \brief  main function for QSPI four wire output fast read function
 **
 ** \param [in] None
 **
 ** \retval int32_t Return value, if needed
 **
 ******************************************************************************/
int32_t main(void)
{
    uint32_t flashAddr = 0;
    uint8_t *pFlashReadAddr, bufferLen = 0;
    uint8_t txBuffer[] = "QSPI read and write flash example: Welcome to use HDSC micro chip";
    stc_port_init_t stcPortInit;
    stc_qspi_comm_protocol_t stcQspiCommProtocol;

    /* configure structure initialization */
    MEM_ZERO_STRUCT(stcPortInit);
    MEM_ZERO_STRUCT(stcQspiCommProtocol);

    /* Configure system clock frequency */
    SystemClk_Init();

    /* LED0 Port/Pin initialization */
    LED0_OFF();
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* LED1 Port/Pin initialization */
    LED1_OFF();
    stcPortInit.enPinMode = Pin_Mode_Out;
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);

    /* Key0 Port/Pin initialization */
    Sw2_Init();
    /* Flash initialization */
    QspiFlash_Init();
    /* Get tx buffer length */
    bufferLen = sizeof(txBuffer);

    while (1)
    {
        if (1u == u8ExIntFlag)
        {
            u8ExIntFlag = 0;
            LED0_OFF();
            LED1_OFF();
            /* Swtich to standard read mode */
            stcQspiCommProtocol.enReadMode = QspiReadModeStandard;
            QSPI_CommProtocolConfig(&stcQspiCommProtocol);
            /* Erase sector */
            QspiFlash_Erase4KbSector(flashAddr);
            /* Write data to flash */
            QspiFlash_WritePage(flashAddr, &txBuffer[0], bufferLen);
            /* Switch to four wire output fast read mode */
            stcQspiCommProtocol.enReadMode = QspiReadModeFourWiresOutput;
            QSPI_CommProtocolConfig(&stcQspiCommProtocol);
            /* Pointer to flash address map */
            pFlashReadAddr = (uint8_t *)((uint32_t)QSPI_BUS_ADDRESS + flashAddr);
            /* Compare txBuffer and flash */
            if (memcmp(txBuffer, pFlashReadAddr, bufferLen) != 0)
            {
                LED0_ON();
            }
            else
            {
                LED1_ON();
            }
            /* Flash address offset */
            flashAddr += FLASH_SRCTOR_SIZE;
            if (flashAddr >= FALSH_MAX_ADDR)
            {
                flashAddr = 0;
            }
        }
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
