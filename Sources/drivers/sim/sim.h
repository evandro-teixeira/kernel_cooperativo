/******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2013 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
***************************************************************************
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
***************************************************************************//*!
*
* @file sim.h
*
* @author Freescale
*
* @version 0.0.1
*
* @date Jun. 25, 2013
*
* @brief header file for SIM utilities. 
*
*******************************************************************************
*
* provide APIs for accessing SIM
******************************************************************************/

#ifndef SIM_H_
#define SIM_H_

/******************************************************************************
* Includes
******************************************************************************/
//#include "/home/evandro/Documentos/AquariumController/KE06Z/common/common.h"
#include "../../common/common.h"
/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* define SIM device ID types
*
*//*! @addtogroup sim_id_types
* @{
*******************************************************************************/

typedef enum {
    ID_TYPE_FAMID,                  /*!< device Family ID */
    ID_TYPE_SUBFAMID,               /*!< device Subfamily ID */ 
    ID_TYPE_REVID,                  /*!< device Revision ID */
    ID_TYPE_PINID                   /*!< device Pin ID (Pin count) */
} IDType;
/*! @} End of sim_id_types                                                    					*/

/******************************************************************************
* Macros
******************************************************************************/ 

/******************************************************************************
* Types
******************************************************************************/
    
/* SIM configuration structure 
 */ 
 
/******************************************************************************
* define SIM configuration structure
*
*//*! @addtogroup sim_config_type
* @{
*******************************************************************************/

/*!
 * @brief SIM configuration structure.
 *
 */
#if defined(CPU_KE02)
typedef struct{
    struct{
	uint32_t bEnableCLKOUT  : 1;        /*!< 1: enable , 0: disable */
	uint32_t bTXDME         : 1;        /*!< 1: enable TXDME, 0: disable */
	uint32_t bFTMSYNC       : 1;        /*!< 1: enable FTM SYNC, 0: no sync */
	uint32_t bRXDFE         : 1;        /*!< 1: enable RXD filter, 0: no filter */
	uint32_t bRXDCE         : 1;        /*!< 1: enable RXD capture, 0: no capture */
	uint32_t bACIC          : 1;        /*!< 1: ACMP0 to FTM1 channel0 connection, 0: no connection */
	uint32_t bRTCC          : 1;        /*!< 1: RTC overflow connected to FTM1 channel1, 0: no connection */
	uint32_t u8ADHWT        : 2;        /*!< ADC h/w trigger source selection */
	uint32_t bDisableSWD    : 1;        /*!< 1: disable SWD, 0: enable */
	uint32_t bDisableRESET  : 1;        /*!< 1: disable RESET pin, 0: enable */
	uint32_t bDisableNMI    : 1;        /*!< 1: disable NMI pin, 0: enable */
        uint32_t bBusDiv        : 1;        /*!< bus divider BUSDIV value */
    } sBits;
    uint8_t     u8Delay;                /*!< delay value */
    uint8_t     u8BusRef;               /*!< bus reference */
    uint32_t    u32PinSel;              /*!< pin select reg value */
    uint32_t    u32SCGC;                /*!< clock gating value register */   
} SIM_ConfigType, *SIM_ConfigPtr; /*!< sim configuration structure type */
#elif defined(CPU_KE04)
typedef struct{
    struct{
	uint32_t bEnableCLKOUT  : 1;        /*!< 1: enable , 0: disable */
	uint32_t bTXDME         : 1;        /*!< 1: enable TXDME, 0: disable */
	uint32_t bFTMSYNC       : 1;        /*!< 1: enable FTM SYNC, 0: no sync */
	uint32_t bRXDCE         : 1;        /*!< 1: enable RXD capture, 0: no capture */
        uint32_t bRXDFE         : 2;        /*!< 1: enable RXD filter, 0: no filter */
	uint32_t u8ADHWT        : 3;        /*!< ADC h/w trigger source selection */
        uint32_t bFTMIC         : 2;        /*!< FTM0CH0 input capture source selection */
        uint32_t bACTRG         : 1;        /*!< ACMP Trigger FTM2 selection*/
	uint32_t bDisableSWD    : 1;        /*!< 1: disable SWD, 0: enable */
	uint32_t bDisableRESET  : 1;        /*!< 1: disable RESET pin, 0: enable */
	uint32_t bDisableNMI    : 1;        /*!< 1: disable NMI pin, 0: enable */
    } sBits;
    uint8_t     u8Delay;                /*!< delay value */
    uint8_t     u8BusRef;               /*!< bus reference */
    uint32_t    u32PinSel;              /*!< pin select reg value */
    uint32_t    u32SCGC;                /*!< clock gating value register */ 
    uint32_t    u32CLKDIV;              /*!< clock divider CLKDIV value */   
} SIM_ConfigType, *SIM_ConfigPtr; /*!< sim configuration structure type */
#elif defined(CPU_KE06)
typedef struct{
    struct{
	uint32_t bEnableCLKOUT  : 1;        /*!< 1: enable , 0: disable */
	uint32_t bTXDME         : 1;        /*!< 1: enable TXDME, 0: disable */
	uint32_t bFTMSYNC       : 1;        /*!< 1: enable FTM SYNC, 0: no sync */
	uint32_t bRXDCE         : 1;        /*!< 1: enable RXD capture, 0: no capture */
        uint32_t bRXDFE         : 2;        /*!< 1: enable RXD filter, 0: no filter */
	uint32_t u8ADHWT        : 3;        /*!< ADC h/w trigger source selection */
        uint32_t bFTMIC         : 2;        /*!< FTM0CH0 input capture source selection */
        uint32_t bACTRG         : 1;        /*!< ACMP Trigger FTM2 selection*/
	uint32_t bDisableSWD    : 1;        /*!< 1: disable SWD, 0: enable */
	uint32_t bDisableRESET  : 1;        /*!< 1: disable RESET pin, 0: enable */
	uint32_t bDisableNMI    : 1;        /*!< 1: disable NMI pin, 0: enable */
    } sBits;
    uint8_t     u8Delay;                /*!< delay value */
    uint8_t     u8BusRef;               /*!< bus reference */
    uint32_t    u32PinSel;              /*!< pin select reg value */
    uint32_t    u32SCGC;                /*!< clock gating value register */ 
    uint32_t    u32CLKDIV;              /*!< clock divider CLKDIV value */   
} SIM_ConfigType, *SIM_ConfigPtr; /*!< sim configuration structure type */
#endif
/*! @} End of sim_config_type                                                    					*/


  
/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/
/******************************************************************************
* define SIM API list
*
*//*! @addtogroup sim_api_list
* @{
*******************************************************************************/
#if defined(CPU_KE02)
/*****************************************************************************//*!
*
* @brief delay FTM2 triggering ADC for u8Delay bus clock output divide. 
*        
* @param[in]   u8Delay     delay value of Bus clock output divide.
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_DelayFTM2Trig2ADC(uint8_t u8Delay)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_DELAY_MASK)) | SIM_SOPT_DELAY(u8Delay);    
}
/*****************************************************************************//*!
*
* @brief enable clock output.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
* @see  SIM_DisableClockOutput
*****************************************************************************/
__STATIC_INLINE void SIM_EnableClockOutput(void)
{
    SIM->SOPT |= (SIM_SOPT_CLKOE_MASK);    
}

/*****************************************************************************//*!
*
* @brief disable clock output.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
* @see  SIM_EnableClockOutput
*****************************************************************************/
__STATIC_INLINE void SIM_DisableClockOutput(void)
{
    SIM->SOPT &= ~(SIM_SOPT_CLKOE_MASK);    
}
/*****************************************************************************//*!
*
* @brief set bus clock output divide. 
*        
* @param[in]  u8Divide     divide (3-bits)
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SetClockOutputDivide(uint8_t u8Divide)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_BUSREF_MASK)) | SIM_SOPT_BUSREF(u8Divide & 0x07);    
}
/*****************************************************************************//*!
*
* @brief enable UART0 RXD connect with UART0 module  and FTM0 channel 1.. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableUART0RXDConnectFTMOCH1(void)
{
    SIM->SOPT |= (SIM_SOPT_RXDCE_MASK);    
}
/*****************************************************************************//*!
*
* @brief enable UART0 TX modulation. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableUART0Modulation(void)
{
    SIM->SOPT |= (SIM_SOPT_TXDME_MASK);    
}

/*****************************************************************************//*!
*
* @brief disable UART0 TX modulation. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_DisableUART0Modulation(void)
{
    SIM->SOPT &= ~(SIM_SOPT_TXDME_MASK);    
}
/*****************************************************************************//*!
*
* @brief generate a softare sync trigger to FTM2 module (trigger).
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_GenerateSoftwareTrig2FTM2(void)
{
    SIM->SOPT |= (SIM_SOPT_FTMSYNC_MASK);    
}
/*****************************************************************************//*!
*
* @brief remap FTM2CH3 pin from default to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM2CH3Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM2PS3_MASK;    
}

/*****************************************************************************//*!
*
* @brief remap FTM2CH2 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM2CH2Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM2PS2_MASK;    
}
/*****************************************************************************//*!
*
* @brief remap FTM0CH1 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM0CH1Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM0PS1_MASK;    
}
/*****************************************************************************//*!
*
* @brief remap FTM0CH0 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM0CH0Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM0PS0_MASK;    
}
/*****************************************************************************//*!
*
* @brief remap UART0 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapUART0Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_UART0PS_MASK;    
}

/*****************************************************************************//*!
*
* @brief remap SPI0 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapSPI0Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_SPI0PS_MASK;    
}

/*****************************************************************************//*!
*
* @brief remap I2C pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapI2CPin(void)
{
    SIM->PINSEL |= SIM_PINSEL_IICPS_MASK;    
}
/*****************************************************************************//*!
*
* @brief enable UART0 RX filter. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableUART0Filter(void)
{
    SIM->SOPT |= (SIM_SOPT_RXDFE_MASK);    
}
/******************************************************************************!

* @function name: SIM_DisableUART0Filter
*
* @brief disable UART0 RX filter. 
*        
* @param         
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_DisableUART0Filter(void)
{
    SIM->SOPT &= ~(SIM_SOPT_RXDFE_MASK);    
}
/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to RTC overflow. 
*        
* @param  none        
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByRTC(void)
{
    SIM->SOPT &= ~(SIM_SOPT_ADHWT_MASK);    
}
/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to PIT . 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByPIT(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_ADHWT_MASK)) | SIM_SOPT_ADHWT(1);    
}
/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to FTM2 init trigger with 8-bit programmable delay. 
*        
* @param  none      
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByFTM2Init(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_ADHWT_MASK)) | SIM_SOPT_ADHWT(2);    
}

/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to FTM2 match trigger with 8-bit programmable delay. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByFTM2Match(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_ADHWT_MASK)) | SIM_SOPT_ADHWT(3);    
}
/*****************************************************************************//*!
*
* @brief enable RTC capture to FTM1 input channel1. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableRTCCapture(void)
{
    SIM->SOPT |= (SIM_SOPT_RTCC_MASK);    
}
/*****************************************************************************//*!
*
* @brief enable ACMP0 input capture to FTM1 input channel0. 
*        
* @param  none      
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableACMP0InputCapture(void)
{
    SIM->SOPT |= (SIM_SOPT_ACIC_MASK);    
}
/*****************************************************************************//*!
*
* @brief remap RTC pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapRTCPin(void)
{
    SIM->PINSEL |= SIM_PINSEL_RTCPS_MASK;    
}
/*****************************************************************************//*!
*
* @brief set bus divide BUSDIV.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SetBusDivide(uint8_t u8Divide)
{
    SIM->BUSDIV = u8Divide;    
}
/*****************************************************************************//*!
*
* @brief remap FTM2CH1 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM2CH1Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM2PS1_MASK;    
}

/*****************************************************************************//*!
*
* @brief remap FTM2CH0 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM2CH0Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM2PS0_MASK;    
}

/*****************************************************************************//*!
*
* @brief remap FTM1CH1 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM1CH1Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM1PS1_MASK;    
}

/*****************************************************************************//*!
*
* @brief remap FTM1CH0 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM1CH0Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM1PS0_MASK;    
}
#elif defined(CPU_KE04)
/*****************************************************************************//*!
*
* @brief delay FTM2 triggering ADC for u8Delay bus clock output divide. 
*        
* @param[in]   u8Delay     delay value of Bus clock output divide.
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_DelayFTM2Trig2ADC(uint8_t u8Delay)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_DELAY_MASK)) | SIM_SOPT_DELAY(u8Delay);    
}
/*****************************************************************************//*!
*
* @brief enable clock output.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
* @see  SIM_DisableClockOutput
*****************************************************************************/
__STATIC_INLINE void SIM_EnableClockOutput(void)
{
    SIM->SOPT |= (SIM_SOPT_CLKOE_MASK);    
}

/*****************************************************************************//*!
*
* @brief disable clock output.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
* @see  SIM_EnableClockOutput
*****************************************************************************/
__STATIC_INLINE void SIM_DisableClockOutput(void)
{
    SIM->SOPT &= ~(SIM_SOPT_CLKOE_MASK);    
}
/*****************************************************************************//*!
*
* @brief set bus clock output divide. 
*        
* @param[in]  u8Divide     divide (3-bits)
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SetClockOutputDivide(uint8_t u8Divide)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_BUSREF_MASK)) | SIM_SOPT_BUSREF(u8Divide & 0x07);    
}
/*****************************************************************************//*!
*
* @brief enable UART0 RXD connect with UART0 module  and FTM0 channel 1.. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableUART0RXDConnectFTMOCH1(void)
{
    SIM->SOPT |= (SIM_SOPT_RXDCE_MASK);    
}
/*****************************************************************************//*!
*
* @brief enable UART0 TX modulation. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableUART0Modulation(void)
{
    SIM->SOPT |= (SIM_SOPT_TXDME_MASK);    
}

/*****************************************************************************//*!
*
* @brief disable UART0 TX modulation. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_DisableUART0Modulation(void)
{
    SIM->SOPT &= ~(SIM_SOPT_TXDME_MASK);    
}
/*****************************************************************************//*!
*
* @brief generate a softare sync trigger to FTM2 module (trigger).
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_GenerateSoftwareTrig2FTM2(void)
{
    SIM->SOPT |= (SIM_SOPT_FTMSYNC_MASK);    
}
/*****************************************************************************//*!
*
* @brief remap FTM2CH3 pin from default to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM2CH3Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM2PS3_MASK;    
}

/*****************************************************************************//*!
*
* @brief remap FTM2CH2 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM2CH2Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM2PS2_MASK;    
}
/*****************************************************************************//*!
*
* @brief remap FTM0CH1 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM0CH1Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM0PS1_MASK;    
}
/*****************************************************************************//*!
*
* @brief remap FTM0CH0 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM0CH0Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM0PS0_MASK;    
}
/*****************************************************************************//*!
*
* @brief remap UART0 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapUART0Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_UART0PS_MASK;    
}

/*****************************************************************************//*!
*
* @brief remap SPI0 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapSPI0Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_SPI0PS_MASK;    
}

/*****************************************************************************//*!
*
* @brief remap I2C pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapI2CPin(void)
{
    SIM->PINSEL |= SIM_PINSEL_IICPS_MASK;    
}
/*****************************************************************************//*!
*
* @brief UART0 RXD input signal is connected to UART0 module directly. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableUART0Filter(void)
{
    SIM->SOPT &= ~(SIM_SOPT_RXDFE_MASK);   
}
/*****************************************************************************//*!
*
* @brief UART0 RXD input signal is filtered by ACMP0, then injected to UART0. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableUART0FilterByACMP0(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_RXDFE_MASK)) | SIM_SOPT_RXDFE(1);  
}
/*****************************************************************************//*!
*
* @brief UART0 RXD input signal is filtered by ACMP1, then injected to UART0. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableUART0FilterByACMP1(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_RXDFE_MASK)) | SIM_SOPT_RXDFE(2);  
}
/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to RTC overflow. 
*        
* @param  none        
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByRTC(void)
{
    SIM->SOPT &= ~(SIM_SOPT_ADHWT_MASK);    
}
/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to FTM0 init trigger . 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByPIT(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_ADHWT_MASK)) | SIM_SOPT_ADHWT(1);    
}
/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to FTM2 init trigger with 8-bit programmable delay. 
*        
* @param  none      
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByFTM2Init(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_ADHWT_MASK)) | SIM_SOPT_ADHWT(2);    
}

/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to FTM2 match trigger with 8-bit programmable delay. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByFTM2Match(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_ADHWT_MASK)) | SIM_SOPT_ADHWT(3);    
}

/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to PIT channel0 overflow. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByPITCH0Overflow(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_ADHWT_MASK)) | SIM_SOPT_ADHWT(4);    
}
/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to PIT channel1 overflow. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByPITChannel1Overflow(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_ADHWT_MASK)) | SIM_SOPT_ADHWT(5);    
}
/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to ACMP0 out. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByACMP0Out(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_ADHWT_MASK)) | SIM_SOPT_ADHWT(6);    
}
/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to ACMP1 out. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByACMP1Out(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_ADHWT_MASK)) | SIM_SOPT_ADHWT(7);    
}
/*****************************************************************************//*!
*
* @brief Select FTM0CH0 as FTM0CH0 Input Capture Source. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SelFTM0CH0AsFTM0CH0ICS(void)
{
    SIM->SOPT &= ~(SIM_SOPT_FTMIC_MASK);   
}
/*****************************************************************************//*!
*
* @brief Select ACMP0 OUT as FTM0CH0 Input Capture Source. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SelACMP0AsFTM0CH0ICS(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_FTMIC_MASK)) | SIM_SOPT_FTMIC(1);  
}
/*****************************************************************************//*!
*
* @brief Select ACMP1 OUT as FTM0CH0 Input Capture Source. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SelACMP1AsFTM0CH0ICS(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_FTMIC_MASK)) | SIM_SOPT_FTMIC(2);  
}
/*****************************************************************************//*!
*
* @brief Select RTC overflow as FTM0CH0 Input Capture Source. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SelRTCOverflowAsFTM0CH0ICS(void)
{
    SIM->SOPT = (SIM->SOPT & ~(SIM_SOPT_FTMIC_MASK)) | SIM_SOPT_FTMIC(3);  
}
/*****************************************************************************//*!
*
* @brief Select ACMP0 output as the trigger0 input of FTM2. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SelACMP0AsFTM2Trigger0(void)
{
    SIM->SOPT &= ~(SIM_SOPT_ACTRG_MASK);  
}
/*****************************************************************************//*!
*
* @brief Select ACMP1 output as the trigger0 input of FTM2. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SelACMP1AsFTM2Trigger0(void)
{
    SIM->SOPT |= (SIM_SOPT_ACTRG_MASK);  
}
/*****************************************************************************//*!
*
* @brief set clock3 divide CLKDIV.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SetClock3Divide(void)
{
    SIM->CLKDIV |= SIM_CLKDIV_OUTDIV3_MASK;    
}
/*****************************************************************************//*!
*
* @brief set clock2 divide CLKDIV.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SetClock2Divide(void)
{
    SIM->CLKDIV |= SIM_CLKDIV_OUTDIV2_MASK;    
}
/*****************************************************************************//*!
*
* @brief set clock1 divide CLKDIV.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SetClock1Divide(uint8_t u8divide)
{
    SIM->CLKDIV |= SIM_CLKDIV_OUTDIV1(u8divide);    
}
/*****************************************************************************//*!
*
* @brief select TCLK2 for PWT module. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SelectTCLK2ForPWT(void)
{
    SIM->PINSEL |= SIM_PINSEL_PWTCLKPS_MASK;    
}
/*****************************************************************************//*!
*
* @brief select TCLK1 for PWT module. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SelectTCLK1ForPWT(void)
{
    SIM->PINSEL &= ~SIM_PINSEL_PWTCLKPS_MASK;    
}
/*****************************************************************************//*!
*
* @brief select TCLK2 for FTM2 module. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SelectTCLK2ForFTM2(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM2CLKPS_MASK;    
}
/*****************************************************************************//*!
*
* @brief select TCLK1 for FTM2 module. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SelectTCLK1ForFTM2(void)
{
    SIM->PINSEL &= ~SIM_PINSEL_FTM2CLKPS_MASK;    
}
/*****************************************************************************//*!
*
* @brief select TCLK2 for FTM0 module. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SelectTCLK2ForFTM0(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM0CLKPS_MASK;    
}
/*****************************************************************************//*!
*
* @brief select TCLK1 for FTM0 module. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SelectTCLK1ForFTM0(void)
{
    SIM->PINSEL &= ~SIM_PINSEL_FTM0CLKPS_MASK;    
}
#elif defined(CPU_KE06)
/*****************************************************************************//*!
*
* @brief delay FTM2 triggering ADC for u8Delay bus clock output divide. 
*        
* @param[in]   u8Delay     delay value of Bus clock output divide.
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_DelayFTM2Trig2ADC(uint8_t u8Delay)
{
    SIM->SOPT0 = (SIM->SOPT0 & ~(SIM_SOPT0_DELAY_MASK)) | SIM_SOPT0_DELAY(u8Delay);    
}
/*****************************************************************************//*!
*
* @brief enable clock output.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
* @see  SIM_DisableClockOutput
*****************************************************************************/
__STATIC_INLINE void SIM_EnableClockOutput(void)
{
    SIM->SOPT0 |= (SIM_SOPT0_CLKOE_MASK);    
}

/*****************************************************************************//*!
*
* @brief disable clock output.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
* @see  SIM_EnableClockOutput
*****************************************************************************/
__STATIC_INLINE void SIM_DisableClockOutput(void)
{
    SIM->SOPT0 &= ~(SIM_SOPT0_CLKOE_MASK);    
}
/*****************************************************************************//*!
*
* @brief set bus clock output divide. 
*        
* @param[in]  u8Divide     divide (3-bits)
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SetClockOutputDivide(uint8_t u8Divide)
{
    SIM->SOPT0 = (SIM->SOPT0 & ~(SIM_SOPT0_BUSREF_MASK)) | SIM_SOPT0_BUSREF(u8Divide & 0x07);    
}
/*****************************************************************************//*!
*
* @brief enable UART0 RXD connect with UART0 module  and FTM0 channel 1.. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableUART0RXDConnectFTMOCH1(void)
{
    SIM->SOPT0 |= (SIM_SOPT0_RXDCE_MASK);    
}
/*****************************************************************************//*!
*
* @brief enable UART0 TX modulation. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableUART0Modulation(void)
{
    SIM->SOPT0 |= (SIM_SOPT0_TXDME_MASK);    
}

/*****************************************************************************//*!
*
* @brief disable UART0 TX modulation. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_DisableUART0Modulation(void)
{
    SIM->SOPT0 &= ~(SIM_SOPT0_TXDME_MASK);    
}
/*****************************************************************************//*!
*
* @brief generate a softare sync trigger to FTM2 module (trigger).
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_GenerateSoftwareTrig2FTM2(void)
{
    SIM->SOPT0 |= (SIM_SOPT0_FTMSYNC_MASK);    
}
/*****************************************************************************//*!
*
* @brief remap FTM2CH3 pin from default to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM2CH3Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL1_FTM2PS3_MASK;
}

/*****************************************************************************//*!
*
* @brief remap FTM2CH2 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM2CH2Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL1_FTM2PS2_MASK;    
}
/*****************************************************************************//*!
*
* @brief remap FTM0CH1 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM0CH1Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM0PS1_MASK;    
}
/*****************************************************************************//*!
*
* @brief remap FTM0CH0 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM0CH0Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM0PS0_MASK;    
}
/*****************************************************************************//*!
*
* @brief remap UART0 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapUART0Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_UART0PS_MASK;    
}

/*****************************************************************************//*!
*
* @brief remap SPI0 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapSPI0Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_SPI0PS_MASK;    
}

/*****************************************************************************//*!
*
* @brief remap I2C pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapI2CPin(void)
{
    SIM->PINSEL |= SIM_PINSEL_I2C0PS_MASK;    
}
/*****************************************************************************//*!
*
* @brief enable UART0 RX filter. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableUART0Filter(void)
{
    SIM->SOPT0 |= (SIM_SOPT0_RXDFE_MASK);    
}
/******************************************************************************!

* @function name: SIM_DisableUART0Filter
*
* @brief disable UART0 RX filter. 
*        
* @param         
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_DisableUART0Filter(void)
{
    SIM->SOPT0 &= ~(SIM_SOPT0_RXDFE_MASK);    
}
/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to RTC overflow. 
*        
* @param  none        
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByRTC(void)
{
    SIM->SOPT0 &= ~(SIM_SOPT0_ADHWT_MASK);    
}
/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to PIT . 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByPIT(void)
{
    SIM->SOPT0 = (SIM->SOPT0 & ~(SIM_SOPT0_ADHWT_MASK)) | SIM_SOPT0_ADHWT(1);    
}
/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to FTM2 init trigger with 8-bit programmable delay. 
*        
* @param  none      
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByFTM2Init(void)
{
    SIM->SOPT0 = (SIM->SOPT0 & ~(SIM_SOPT0_ADHWT_MASK)) | SIM_SOPT0_ADHWT(2);    
}

/*****************************************************************************//*!
*
* @brief set ADC hardware trigger source to FTM2 match trigger with 8-bit programmable delay. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_TriggerADCByFTM2Match(void)
{
    SIM->SOPT0 = (SIM->SOPT0 & ~(SIM_SOPT0_ADHWT_MASK)) | SIM_SOPT0_ADHWT(3);    
}
/*****************************************************************************//*!
*
* @brief enable RTC capture to FTM1 input channel1. 
*        
* @param  none       
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableRTCCapture(void)
{
    SIM->SOPT0 |= (SIM_SOPT0_RTCC_MASK);    
}
/*****************************************************************************//*!
*
* @brief enable ACMP0 input capture to FTM1 input channel0. 
*        
* @param  none      
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_EnableACMP0InputCapture(void)
{
    SIM->SOPT0 |= (SIM_SOPT0_ACIC_MASK);    
}
/*****************************************************************************//*!
*
* @brief remap RTC pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapRTCPin(void)
{
    SIM->PINSEL |= SIM_PINSEL_RTCPS_MASK;    
}
/*****************************************************************************//*!
*
* @brief set bus divide BUSDIV.
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_SetBusDivide(uint8_t u8Divide)
{
    SIM->CLKDIV = u8Divide;    
}
/*****************************************************************************//*!
*
* @brief remap FTM2CH1 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM2CH1Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL1_FTM2PS1_MASK;    
}

/*****************************************************************************//*!
*
* @brief remap FTM2CH0 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM2CH0Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL1_FTM2PS0_MASK;    
}

/*****************************************************************************//*!
*
* @brief remap FTM1CH1 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM1CH1Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM1PS1_MASK;    
}

/*****************************************************************************//*!
*
* @brief remap FTM1CH0 pin from default  to the other. 
*        
* @param   none
*
* @return none
*
* @ Pass/ Fail criteria: none
*****************************************************************************/
__STATIC_INLINE void SIM_RemapFTM1CH0Pin(void)
{
    SIM->PINSEL |= SIM_PINSEL_FTM1PS0_MASK;    
}
#endif


/*! @} End of sim_api_list                                                   */

void SIM_Init(SIM_ConfigType *pConfig);
void SIM_SetClockGating(uint32_t u32PeripheralMask, uint8_t u8GateOn);
uint32_t SIM_GetStatus(uint32_t u32StatusMask);
uint8_t SIM_ReadID(IDType sID);

#endif /* SIM_H_ */


