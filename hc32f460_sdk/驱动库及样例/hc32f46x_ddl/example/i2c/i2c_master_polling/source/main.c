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
 ** \brief I2C master polling sample
 **
 **   - 2018-11-01  1.0  Wangmin First version for Device Driver Library of I2C
 **     master polling example
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
#define DEVICE_ADDRESS                  0x06
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
#define I2C_BAUDRATE                    400000

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
 ** \brief  Send start or restart condition
 **
 ** \param  none
 **
 ** \retval Process result
 **         - I2C_RET_ERROR  Send start or restart failed
 **         - I2C_RET_OK     Send start or restart success
 ******************************************************************************/
static uint8_t Master_StartOrRestart(uint8_t u8Start)
{
    uint32_t u32TimeOut = TIMEOUT;

    /* generate start or restart signal */
    if(GENERATE_START == u8Start)
    {
        /* Wait I2C bus idle */
        while(Set == I2C_GetStatus(I2C_CH, I2C_SR_BUSY))
        {
            if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
        }

        I2C_GenerateStart(I2C_CH , Enable);
    }
    else
    {
        /* Clear start status flag */
        I2C_ClearStatus(I2C_CH, I2C_CLR_STARTFCLR);
        /* Send restart condition */
        I2C_GenerateReStart(I2C_CH , Enable);
    }

    /* Judge if start success*/
    u32TimeOut = TIMEOUT;
    while((Reset == I2C_GetStatus(I2C_CH, I2C_SR_BUSY)) ||
            (Reset == I2C_GetStatus(I2C_CH, I2C_SR_STARTF)))
    {
        if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
    }

    return I2C_RET_OK;
}

/**
 ******************************************************************************
 ** \brief  Send slave address
 **
 ** \param  u16Adr  The slave address
 **
 ** \retval Process result
 **         - I2C_RET_ERROR  Send failed
 **         - I2C_RET_OK     Send success
 ******************************************************************************/
static uint8_t Master_SendAdr(uint8_t u8Adr)
{
    uint32_t u32TimeOut = TIMEOUT;

    /* Wait tx buffer empty */
    while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_TEMPTYF))
    {
        if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
    }

    /* Send I2C address */
    I2C_SendData(I2C_CH, u8Adr);

    if(ADDRESS_W == (u8Adr & 0x01))
    {
        /* If in master transfer process, Need wait transfer end*/
        uint32_t u32TimeOut = TIMEOUT;
        while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_TENDF))
        {
            if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
        }

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
 ** \brief  Send data to slave
 **
 ** \param  pTxData  Pointer to the data buffer
 ** \param  u32Size  Data size
 **
 ** \retval Process result
 **         - I2C_RET_ERROR  Send failed
 **         - I2C_RET_OK     Send success
 ******************************************************************************/
static uint8_t Master_WriteData(uint8_t *pTxData, uint32_t u32Size)
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

        /* Wait transfer end*/
        u32TimeOut = TIMEOUT;
        while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_TENDF))
        {
            if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
        }

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
 ** \brief  Write address and receive data from slave
 **
 ** \param  u8Adr    Device address and R/W bit
 ** \param  pTxData  Pointer to the data buffer
 ** \param  u32Size  Data size
 **
 ** \retval Process result
 **         - I2C_RET_ERROR  Process failed
 **         - I2C_RET_OK     Process success
 ******************************************************************************/
static uint8_t Master_SendAdrRevData(uint8_t u8Adr, uint8_t *pRxData, uint32_t u32Size)
{
    uint32_t u32TimeOut = TIMEOUT;

    /* Wait tx buffer empty */
    while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_TEMPTYF))
    {
        if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
    }

    for(uint32_t i=0; i<u32Size; i++)
    {
        /* if the last byte receive, need config NACK*/
        if(i == (u32Size - 1))
        {
            I2C_NackConfig(I2C_CH, Enable);
        }

        /* if first byte receive, need send adr*/
        if(0 == i)
        {
            I2C_SendData(I2C_CH, u8Adr);
        }

        /* Wait receive full flag*/
        u32TimeOut = TIMEOUT;
        while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_RFULLF))
        {
            if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
        }

        /* read data from register*/
        *pRxData++ = I2C_ReadData(I2C_CH);
    }
    return I2C_RET_OK;
}

/**
 ******************************************************************************
 ** \brief  General stop condition to slave
 **
 ** \param  None
 **
 ** \retval Process result
 **         - I2C_RET_ERROR  Send failed
 **         - I2C_RET_OK     Send success
 ******************************************************************************/
uint8_t Master_Stop(void)
{
    uint32_t u32TimeOut;

    /* Wait I2C bus busy */
    u32TimeOut = TIMEOUT;
    while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_BUSY))
    {
        if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
    }

    I2C_GenerateStop(I2C_CH, Enable);

    /* Wait STOPF */
    u32TimeOut = TIMEOUT;
    while(Reset == I2C_GetStatus(I2C_CH, I2C_SR_STOPF))
    {
        if(0 == (u32TimeOut--)) return I2C_RET_ERROR;
    }
    return I2C_RET_OK;
}

/**
 ******************************************************************************
 ** \brief  Initialize the I2C peripheral for master
 **
 ** \param  None
 **
 ** \retval Process result
 **         - I2C_RET_ERROR  Initialize failed
 **         - I2C_RET_OK     Initialize success
 ******************************************************************************/
uint8_t Master_Initialize(void)
{
    stc_i2c_init_t stcI2cInit;

    I2C_DeInit(I2C_CH);

    MEM_ZERO_STRUCT(stcI2cInit);
    stcI2cInit.enI2cMode = I2cMaster;
    stcI2cInit.u32Baudrate = I2C_BAUDRATE;
    stcI2cInit.u32SclTime = 0;
    I2C_Init(I2C_CH, &stcI2cInit);

    I2C_Cmd(I2C_CH, Enable);

    return I2C_RET_OK;
}

/**
 ******************************************************************************
 ** \brief  Initialize the system clock for the sample
 **
 ** \param  None
 **
 ** \retval None
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
 ******************************************************************************
 ** \brief  Judge the result. LED0 toggle when result is error status.
 **
 ** \param  u8Result
 **
 ** \retval None
 ******************************************************************************/
static void JudgeResult(uint8_t u8Result)
{
    if(I2C_RET_ERROR == u8Result)
    {
        while(1)
        {
            LED0_TOGGLE();
            Ddl_Delay1ms(500);
        }
    }
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
    uint8_t u8TxBuf[TEST_DATA_LEN];
    uint8_t u8RxBuf[TEST_DATA_LEN];
    uint32_t i;
    uint8_t u8Ret = I2C_RET_OK;
    stc_port_init_t stcPortInit;

    for(i=0; i<TEST_DATA_LEN; i++)
    {
        u8TxBuf[i] = i+1;
    }
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
    Master_Initialize();

    Ddl_Delay1ms(5);
    /* I2C master data write*/
    u8Ret = Master_StartOrRestart(GENERATE_START);
    JudgeResult(u8Ret);
    u8Ret = Master_SendAdr((DEVICE_ADDRESS<<1)|ADDRESS_W);
    JudgeResult(u8Ret);
    u8Ret = Master_WriteData(u8TxBuf, TEST_DATA_LEN);
    JudgeResult(u8Ret);
    u8Ret = Master_Stop();
    JudgeResult(u8Ret);

    /* 5mS delay for device*/
    Ddl_Delay1ms(5);

    /* I2C master data read*/
    u8Ret = Master_StartOrRestart(GENERATE_START);
    JudgeResult(u8Ret);
    u8Ret = Master_SendAdrRevData((DEVICE_ADDRESS<<1)|ADDRESS_R, u8RxBuf, TEST_DATA_LEN);
    JudgeResult(u8Ret);
    u8Ret = Master_Stop();
    JudgeResult(u8Ret);

    /* Compare the data */
    for(i=0; i<TEST_DATA_LEN; i++)
    {
        if(u8TxBuf[i] != u8RxBuf[i])
        {
            /* Data write error*/
            while(1)
            {
                LED0_TOGGLE();
                Ddl_Delay1ms(500);
            }
        }
    }

    /* I2C master polling comunication successed */
    while(1)
    {
        LED1_TOGGLE();
        Ddl_Delay1ms(500);
    }
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
