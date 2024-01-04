/**
 * I2C2 Generated Driver File
 *
 * @file i2c2.c
 *
 * @ingroup i2c_host
 *
 * @brief This file contains the driver code for I2C2 module.
 *
 * @version I2C2 Driver Version 2.1.0
 */

/*
© [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#include <xc.h>
#include "../../system/config_bits.h"
#include "../i2c2.h"

/* I2C2 event system interfaces */
static void I2C2_ReadStart(void);
static void I2C2_WriteStart(void);
static void I2C2_Close(void);
static void I2C2_DefaultCallback(void);

/* I2C2 interfaces */
static void I2C2_StartSend(void);
static void I2C2_AddrTransmit(uint8_t addr);
static void I2C2_DataTransmit(uint8_t data);
static uint8_t I2C2_DataReceive(void);
static void I2C2_CounterSet(uint16_t counter);
static uint16_t I2C2_CounterGet(void);
static inline void I2C2_BusReset(void);
static inline void I2C2_RestartEnable(void);
static inline void I2C2_RestartDisable(void);
static void I2C2_StopSend(void);
static bool I2C2_IsNack(void);
static bool I2C2_IsBusCol(void);
static bool I2C2_IsBusTimeOut(void);
static bool I2C2_IsData(void);
static bool I2C2_IsAddr(void);
static inline void I2C2_InterruptsEnable(void);
static inline void I2C2_InterruptsDisable(void);
static inline void I2C2_InterruptsClear(void);
static inline void I2C2_ErrorFlagsClear(void);
static inline void I2C2_BufferClear(void);

/**
  Section: Driver Interface
 */
const i2c_host_interface_t I2C2_Host = {
    .Initialize = I2C2_Initialize,
    .Deinitialize = I2C2_Deinitialize,
    .Write = I2C2_Write,
    .Read = I2C2_Read,
    .WriteRead = I2C2_WriteRead,
    .TransferSetup = NULL,
    .ErrorGet = I2C2_ErrorGet,
    .IsBusy = I2C2_IsBusy,
    .CallbackRegister = I2C2_CallbackRegister,
    .Tasks = NULL
};

/**
 Section: Private Variable Definitions
 */
static void (*I2C2_Callback)(void) = NULL;
volatile i2c_host_event_status_t i2c2Status = {0};

/**
 Section: Public Interfaces
 */
void I2C2_Initialize(void)
{
    /* CSTR Enable clocking; S Cleared by hardware after Start; MODE 7-bit address; EN disabled; RSEN disabled;  */
    I2C2CON0 = 0x4;
    /* TXU No underflow; CSD Clock Stretching enabled; RXO No overflow; P Cleared by hardware after sending Stop; ACKDT Acknowledge; ACKCNT Not Acknowledge;  */
    I2C2CON1 = 0x80;
    /* ABD enabled; GCEN disabled; ACNT disabled; SDAHT 30 ns hold time; BFRET 8 I2C Clock pulses;  */
    I2C2CON2 = 0x8;
    /* ACNTMD 8 bits are loaded into I2CxCNTL; FME Standard Mode;  */
    I2C2CON3 = 0x0;
    /* CLK Fosc;  */
    I2C2CLK = 0x1;
    /* WRIF Data byte not detected; CNTIF Byte count is not zero; RSCIF Restart condition not detected; PCIF Stop condition not detected; ACKTIF Acknowledge sequence not detected; ADRIF Address not detected; SCIF Start condition not detected;  */
    I2C2PIR = 0x0;
    /* CNTIE disabled; RSCIE disabled; ACKTIE disabled; SCIE disabled; PCIE disabled; ADRIE disabled; WRIE disabled;  */
    I2C2PIE = 0x0;
    /* BTOIE disabled; BTOIF No bus timeout; NACKIF No NACK/Error detected; BCLIE disabled; BCLIF No bus collision detected; NACKIE disabled;  */
    I2C2ERR = 0x0;
    /* Count register */
    I2C2CNTL = 0x0;
    I2C2CNTH = 0x0;
    /* BAUD 1;  */
    I2C2BAUD = 0x1;
    /* BTOC Reserved;  */
    I2C2BTOC = 0x0;
    I2C2_InterruptsEnable();

    /* Silicon-Errata: Section: 1.3.2 */
    #warning "Refer to erratum DS80000870F: https://www.microchip.com/content/dam/mchp/documents/MCU08/ProductDocuments/Errata/PIC18F27-47-57Q43-Silicon-Errata-and-Datasheet-Clarifications-80000870J.pdf"
    I2C2PIEbits.SCIE = 0;
    I2C2PIEbits.PCIE = 0;
    I2C2CON0bits.EN = 1;
    __delay_us(1);
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    I2C2PIRbits.SCIF = 0;
    I2C2PIRbits.PCIF = 0;
    I2C2PIEbits.PCIE = 1;
    I2C2_CallbackRegister(I2C2_DefaultCallback);
}

void I2C2_Deinitialize(void)
{
    I2C2CON0 = 0x00;
    I2C2CON1 = 0x00;
    I2C2CON2 = 0x00;
    I2C2CON3 = 0x00;
    I2C2CLK = 0x00;
    I2C2PIR = 0x00;
    I2C2PIE = 0x00;
    I2C2ERR = 0x00;
    I2C2CNTL = 0x00;
    I2C2CNTH = 0x00;
    I2C2BAUD = 0x00;
    I2C2_InterruptsDisable();
    I2C2_CallbackRegister(I2C2_DefaultCallback);
}

bool I2C2_Write(uint16_t address, uint8_t *data, size_t dataLength)
{
    bool retStatus = false;
    if (!I2C2_IsBusy())
    {
        i2c2Status.busy = true;
        i2c2Status.address = address;
        i2c2Status.switchToRead = false;
        i2c2Status.writePtr = data;
        i2c2Status.writeLength = dataLength;
        i2c2Status.readPtr = NULL;
        i2c2Status.readLength = 0;
        i2c2Status.errorState = I2C_ERROR_NONE;
        I2C2_WriteStart();
        retStatus = true;
    }
    return retStatus;
}

bool I2C2_Read(uint16_t address, uint8_t *data, size_t dataLength)
{
    bool retStatus = false;
    if (!I2C2_IsBusy())
    {
        i2c2Status.busy = true;
        i2c2Status.address = address;
        i2c2Status.switchToRead = false;
        i2c2Status.readPtr = data;
        i2c2Status.readLength = dataLength;
        i2c2Status.writePtr = NULL;
        i2c2Status.writeLength = 0;
        i2c2Status.errorState = I2C_ERROR_NONE;
        I2C2_ReadStart();
        retStatus = true;
    }
    return retStatus;
}

bool I2C2_WriteRead(uint16_t address, uint8_t *writeData, size_t writeLength, uint8_t *readData, size_t readLength)
{
    bool retStatus = false;
    if (!I2C2_IsBusy())
    {
        i2c2Status.busy = true;
        i2c2Status.address = address;
        i2c2Status.switchToRead = true;
        i2c2Status.writePtr = writeData;
        i2c2Status.writeLength = writeLength;
        i2c2Status.readPtr = readData;
        i2c2Status.readLength = readLength;
        i2c2Status.errorState = I2C_ERROR_NONE;
        I2C2_WriteStart();
        retStatus = true;
    }
    return retStatus;
}

i2c_host_error_t I2C2_ErrorGet(void)
{
    i2c_host_error_t retErrorState = i2c2Status.errorState;
    i2c2Status.errorState = I2C_ERROR_NONE;
    return retErrorState;
}

bool I2C2_IsBusy(void)
{
    return i2c2Status.busy || !I2C2STAT0bits.BFRE;
}

void I2C2_CallbackRegister(void (*callbackHandler)(void))
{
    if (callbackHandler != NULL)
    {
        I2C2_Callback = callbackHandler;
    }
}

void I2C2_ISR()
{
    if (I2C2PIEbits.PCIE && I2C2PIRbits.PCIF)
    {
        I2C2_Close();
    }
    else if (I2C2PIEbits.CNTIE && I2C2PIRbits.CNTIF)
    {
        /*Check if restart is required*/
        if (i2c2Status.switchToRead)
        {
            i2c2Status.switchToRead = false;
            I2C2PIRbits.SCIF = 0;
            I2C2PIRbits.CNTIF = 0;
            I2C2_ReadStart();
        }
        else 
        {
            I2C2_StopSend();
            I2C2_Close();
        }
    }
    else if (I2C2PIEbits.RSCIE && I2C2PIRbits.RSCIF)
    {
        I2C2_RestartDisable();
        I2C2PIRbits.RSCIF = 0;
    }
}

void I2C2_ERROR_ISR()
{
    if (I2C2_IsBusCol())
    {
        i2c2Status.errorState = I2C_ERROR_BUS_COLLISION;
        I2C2ERRbits.BCLIF = 0;
        I2C2_BusReset();
    }
    else if (I2C2_IsAddr() && I2C2_IsNack())
    {
        i2c2Status.errorState = I2C_ERROR_ADDR_NACK;
        I2C2ERRbits.NACKIF = 0;
        I2C2_StopSend();
    }
    else if (I2C2_IsData() && I2C2_IsNack())
    {
        i2c2Status.errorState = I2C_ERROR_DATA_NACK;
        I2C2ERRbits.NACKIF = 0;
        I2C2_StopSend();
    }
    else if (I2C2_IsBusTimeOut())
    {
        i2c2Status.errorState = I2C_ERROR_BUS_COLLISION;
        I2C2ERRbits.BTOIF = 0;
    }
    else
    {
        I2C2ERRbits.NACKIF = 0;
    }

    if (i2c2Status.errorState != I2C_ERROR_NONE)
    {
        I2C2_Callback();
    }
}

void I2C2_RX_ISR()
{
    *i2c2Status.readPtr++ = I2C2_DataReceive();
}

void I2C2_TX_ISR()
{
    I2C2_DataTransmit(*i2c2Status.writePtr++);
}

/**
 Section: Private Interfaces
 */
static void I2C2_ReadStart(void)
{
    if (i2c2Status.readLength)
    {
        I2C2_CounterSet((uint16_t) i2c2Status.readLength);
    }

    I2C2_AddrTransmit((uint8_t) (i2c2Status.address << 1 | 1));
    I2C2_StartSend();
}

static void I2C2_WriteStart(void)
{
    if (i2c2Status.writeLength)
    {
        I2C2_CounterSet((uint16_t) i2c2Status.writeLength);
        if (i2c2Status.switchToRead)
        {
            I2C2_RestartEnable();
        }
    }

    I2C2_AddrTransmit((uint8_t) (i2c2Status.address << 1));
    I2C2_StartSend();
}

static void I2C2_Close(void)
{
    i2c2Status.busy = false;
    i2c2Status.address = 0xFF;
    i2c2Status.writePtr = NULL;
    i2c2Status.readPtr = NULL;
    I2C2_InterruptsClear();
    I2C2_ErrorFlagsClear();
    I2C2_BufferClear();
}

static void I2C2_DefaultCallback(void)
{
    // Default Callback for Error Indication
}

/**
 Section: Register Level Interfaces
 */
static void I2C2_StartSend(void)
{
    I2C2CON0bits.S = 1;
}

static void I2C2_AddrTransmit(uint8_t addr)
{
    I2C2ADB1 = addr;
}

static void I2C2_DataTransmit(uint8_t data)
{
    I2C2TXB = data;
}

static uint8_t I2C2_DataReceive(void)
{
    return I2C2RXB;
}

static void I2C2_CounterSet(uint16_t counter)
{
    I2C2CNTH = ((counter & 0xFF00) >> 8);
    I2C2CNTL = (counter & 0x00FF);
}

static uint16_t I2C2_CounterGet(void)
{
    return (uint16_t) ((I2C2CNTH << 8) | I2C2CNTL);
}

static inline void I2C2_BusReset(void)
{
    I2C2_InterruptsClear();
    I2C2_ErrorFlagsClear();
    I2C2_InterruptsDisable();
    I2C2_BufferClear();
    I2C2CON0bits.EN = 0;
    I2C2_InterruptsEnable();

    /* Silicon-Errata: Section: 1.3.2 */
    #warning "Refer to erratum DS80000870F: https://www.microchip.com/content/dam/mchp/documents/MCU08/ProductDocuments/Errata/PIC18F27-47-57Q43-Silicon-Errata-and-Datasheet-Clarifications-80000870J.pdf"
    I2C2PIEbits.SCIE = 0;
    I2C2PIEbits.PCIE = 0;
    I2C2CON0bits.EN = 1;
    __delay_us(1);
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    I2C2PIRbits.SCIF = 0;
    I2C2PIRbits.PCIF = 0;
    I2C2PIEbits.PCIE = 1;
}

static inline void I2C2_RestartEnable(void)
{
    I2C2CON0bits.RSEN = 1;
}

static inline void I2C2_RestartDisable(void)
{
    I2C2CON0bits.RSEN = 0;
}

static void I2C2_StopSend(void)
{
    I2C2_RestartDisable();
    I2C2CON1bits.P = 1;
}

static bool I2C2_IsNack(void)
{
    return I2C2CON1bits.ACKSTAT;
}

static bool I2C2_IsBusCol(void)
{
    return I2C2ERRbits.BCLIF;
}

static bool I2C2_IsBusTimeOut(void)
{
    return I2C2ERRbits.BTOIF;
}

static bool I2C2_IsData(void)
{
    return I2C2STAT0bits.D;
}

static bool I2C2_IsAddr(void)
{
    return !I2C2STAT0bits.D;
}

static inline void I2C2_InterruptsEnable(void)
{
    PIE1bits.I2C2IE = 1;
    PIE1bits.I2C2EIE = 1;
    PIE1bits.I2C2RXIE = 1;
    PIE1bits.I2C2TXIE = 1;

    I2C2PIEbits.PCIE = 1;
    I2C2PIEbits.RSCIE = 1;
    I2C2PIEbits.CNTIE = 1;
    I2C2ERRbits.NACKIE = 1;
}

static inline void I2C2_InterruptsDisable(void)
{
    PIE1bits.I2C2IE = 0;
    PIE1bits.I2C2EIE = 0;
    PIE1bits.I2C2RXIE = 0;
    PIE1bits.I2C2TXIE = 0;
    I2C2PIE = 0x00;
    I2C2ERR = 0x00;
}

static inline void I2C2_InterruptsClear(void)
{
    I2C2PIR = 0x00;
}

static inline void I2C2_ErrorFlagsClear(void)
{
    I2C2ERRbits.BCLIF = 0;
    I2C2ERRbits.BTOIF = 0;
    I2C2ERRbits.NACKIF = 0;
}

static inline void I2C2_BufferClear(void)
{
    I2C2STAT1 = 0x00;
    I2C2STAT1bits.CLRBF = 1;
}