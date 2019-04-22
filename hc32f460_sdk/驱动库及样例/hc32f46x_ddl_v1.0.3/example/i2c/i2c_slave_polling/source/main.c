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
 ** \brief I2C slave polling sample
 **
 **   - 2018-11-01  1.0  Wangmin First version for Device Driver Library of I2C
 **     Slave polling example
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
/* Define I2C unit used for the example */
#define I2C_CH                          M4_I2C1
/* Define slave device address for example */
#define SLAVE_ADDRESS                   0x06
/* Define port and pin for SDA and SCL */
#define I2C1_SCL_PORT                   PortC
#define I2C1_SCL_PIN                    Pin04
#define I2C1_SDA_PORT                   PortC
#define I2C1_SDA_PIN                    Pin05

#define TIMEOUT                         ((uint32_t)0x10000)

#define I2C_RET_OK                      0
#define I2C_RET_ERROR                   1

#define GENERATE_START                  0x00
#define GENERATE_RESTART                0x01

#define ADDRESS_W                       0x00
#define ADDRESS_R                       0x01

/* Define Write and read data length for the example */
#define TEST_DATA_LEN                   256
/* Define i2c baudrate */
#define I2C_BAUDRATE                    100000

/* LED0 Port/Pin definition */
#define  LED0_PORT        PortE
#define  LED0_PIN         Pin06

/* LED1 Port/Pin definition */
#define  LED1_PORT        PortA
#define  LED1_PIN         Pin07

/* LED0~1 toggle definition */
#define  LED0_TOGGLE()    PORT_Toggle(LED0_PORT, LED0_PIN)
#define  LED1_TOGGLE()    PORT_Toggle(LED1_PORT, LED1_PIN)

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
 ******************************************************************************
 ** \brief  Slave send data to master
 **
 ** \param  pTxData     The data buffer to be send.
 ** \param  u32Size     The data length to be send.
 **
 ** \return Process result
 **         - I2C_RET_ERROR  Write data failed
 **         - I2C_RET_OK     Write data success
 ******************************************************************************/
static uint8_t Slave_WriteData(uint8_t *pTxData, uint32_t u32Size)
{
    uint32_t u32TimeOut = TIMEOUT;

    while(u32Size--)
    {
        /* Wait tx buffer empty */
        u32TimeOut = TIMEOUT;
        while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_TEMPTYF))
        {
            if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
        }

        /* Send one byte data */
        I2C_SendData(I2C_CH, *pTxData++);

        /* Check ACK */
        u32TimeOut = TIMEOUT;
        while(Set == I2C_GetStatus(I2C_CH, I2C_SR_NACKDETECTF))
        {
            if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
        }
    }

    return I2C_RET_OK;
}

/**
 ******************************************************************************
 ** \brief  Receive the data until recevie stop condition
 **
 ** \param  pu8RxData The receive buffer pointer
 **
 ** \return Process result
 **         - I2C_RET_ERROR  Receive data failed
 **         - I2C_RET_OK     Receive data success
 *****************************************************************************/
static uint8_t Slave_RevData(uint8_t *pu8RxData)
{
    uint8_t i = 0;

    while(1)
    {
        /* Detect the stop signal on the bus */
        if(Set == I2C_GetStatus(I2C_CH, I2C_SR_STOPF))
        {
            I2C_ClearStatus(I2C_CH, I2C_SR_STOPF);
            return I2C_RET_OK;
        }

        /* Wait for the Rx full flag set */
        if(Set == I2C_GetStatus(I2C_CH, I2C_SR_RFULLF))
        {
            /* Read the data from buffer */
            pu8RxData[i++] = I2C_ReadData(I2C_CH);
        }
    }
}


/**
 ******************************************************************************
 ** \brief  Initialize the I2C peripheral for slave
 **
 ** \param  None
 **
 ** \return Process result
 **         - I2C_RET_ERROR  Initialize failed
 **         - I2C_RET_OK     Initialize success
 ******************************************************************************/
uint8_t Slave_Initialize(void)
{
    stc_i2c_init_t stcI2cInit;

    I2C_DeInit(I2C_CH);

    MEM_ZERO_STRUCT(stcI2cInit);
    stcI2cInit.enI2cMode = I2cSlave;
    stcI2cInit.u32Baudrate = I2C_BAUDRATE;
    I2C_Init(I2C_CH, &stcI2cInit);

    I2C_Cmd(I2C_CH, Enable);

    /* Set slave address*/
#ifdef I2C_10BITS_ADDRESS
    I2C_SlaveAdr0Config(I2C_CH, Enable, Adr10bit, SLAVE_ADDRESS);
#else
    I2C_SlaveAdr0Config(I2C_CH, Enable, Adr7bit, SLAVE_ADDRESS);
#endif

    return I2C_RET_OK;
}

/**
 ******************************************************************************
 ** \brief  Initialize the system clock for the sample
 **
 ** \param  None
 **
 ** \return None
 ******************************************************************************/
static void SysClkIni(void)
{
    en_clk_sys_source_t     enSysClkSrc;
    stc_clk_sysclk_cfg_t    stcSysClkCfg;
    stc_clk_xtal_cfg_t      stcXtalCfg;
    stc_clk_mpll_cfg_t      stcMpllCfg;

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
    stcMpllCfg.plln =50;
    stcMpllCfg.PllpDiv = 4;
    stcMpllCfg.PllqDiv = 4;
    stcMpllCfg.PllrDiv = 4;
    CLK_SetPllSource(ClkPllSrcXTAL);
    CLK_MpllConfig(&stcMpllCfg);

    /* flash read wait cycle setting */
    EFM_Unlock();
    EFM_SetLatency(EFM_LATENCY_5);
    EFM_Lock();

    /* Enable MPLL. */
    CLK_MpllCmd(Enable);

    /* Wait MPLL ready. */
    while(Set != CLK_GetFlagStatus(ClkFlagMPLLRdy));

    /* Switch system clock source to MPLL. */
    CLK_SetSysClkSource(CLKSysSrcMPLL);
}



/**
 *******************************************************************************
 ** \brief  Main function of template project
 **
 ** \param  None
 **
 ** \retval int32_t return value, if needed
 **
 ******************************************************************************/
int32_t  main(void)
{
    uint8_t u8RxBuf[TEST_DATA_LEN];
    stc_port_init_t stcPortInit;

    memset(u8RxBuf, 0x00, TEST_DATA_LEN);

    /* Initialize system clock*/
    SysClkIni();

    /*initiallize LED port*/
    MEM_ZERO_STRUCT(stcPortInit);

    stcPortInit.enPinMode = Pin_Mode_Out;
    stcPortInit.enExInt = Enable;
    stcPortInit.enPullUp = Enable;

    /* LED0 Port/Pin initialization */
    PORT_Init(LED0_PORT, LED0_PIN, &stcPortInit);

    /* LED1 Port/Pin initialization */
    PORT_Init(LED1_PORT, LED1_PIN, &stcPortInit);

    /* Initialize I2C port*/
    PORT_SetFunc(I2C1_SCL_PORT, I2C1_SCL_PIN, Func_I2c1_Scl, Disable);
    PORT_SetFunc(I2C1_SDA_PORT, I2C1_SDA_PIN, Func_I2c1_Sda, Disable);

    /* Enable I2C Peripheral*/
    PWC_Fcg1PeriphClockCmd(PWC_FCG1_PERIPH_I2C1, Enable);
    /* Initialize I2C peripheral and enable function*/
    Slave_Initialize();

    while(1)
    {
        /* Wait salve address matched*/
        while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_SLADDR0F));
        I2C_ClearStatus(I2C_CH, I2C_CLR_SLADDR0FCLR);

        if(Reset == I2C_GetStatus(I2C_CH, I2C_SR_TRA))
        {
            /* Slave receive data*/
            Slave_RevData(u8RxBuf);
            continue;
        }
        else
        {
            /* Slave send data*/
            Slave_WriteData(u8RxBuf, TEST_DATA_LEN);
            break;
        }
    }

    /* Communication finished */
    while(1)
    {
        LED1_TOGGLE();
        Ddl_Delay1ms(500);
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
