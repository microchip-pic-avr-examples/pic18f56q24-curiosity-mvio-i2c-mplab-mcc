 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.0
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
#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/i2c_host/i2c1.h"
#include "mcc_generated_files/i2c_host/i2c_host_event_types.h"
     
#define I2C1_CLIENT_ADDR                0b01010110
#define I2C1_REG_ADDR                   0x0102
#define I2C_CLIENT_ADDR                 0x49
#define MCP9800_REG_ADDR_CONFIG         0x01
#define MCP9800_REG_ADDR_TEMPERATURE    0x00
#define CONFIG_DATA_12BIT_RESOLUTION    0x60
#define I2C_RW_BIT                      0x01

void I2C_1ByteAddress(uint8_t address, uint8_t reg, uint8_t data);
void I2C_2_N_2(uint8_t address, uint16_t reg, uint16_t data);
uint16_t I2C1_randomRead2Byte(uint8_t address, uint16_t reg);
uint16_t I2C1_Read_1ByteAdd_2ByteData(uint8_t address, uint8_t reg);

/*
    Main application
*/

int main(void)
{
    uint16_t rawTempValue;
    uint16_t eepromReadValue;

    SYSTEM_Initialize();

    I2C_1ByteAddress(I2C_CLIENT_ADDR, MCP9800_REG_ADDR_CONFIG , CONFIG_DATA_12BIT_RESOLUTION);
    __delay_ms(2);
    rawTempValue = I2C1_Read_1ByteAdd_2ByteData(I2C_CLIENT_ADDR, MCP9800_REG_ADDR_TEMPERATURE);
    __delay_ms(5);
    I2C_2_N_2(I2C1_CLIENT_ADDR, I2C1_REG_ADDR, rawTempValue);
    __delay_ms(5);
    eepromReadValue = I2C1_randomRead2Byte(I2C1_CLIENT_ADDR,I2C1_REG_ADDR);

    while(1)
    {
    }   
}

void I2C_1ByteAddress(uint8_t address, uint8_t reg, uint8_t data){
   
    I2C2ADB1 = (uint8_t) (address<< 1);
    I2C2CNTL = 2; 
    I2C2TXB = reg;
    I2C2CON0bits.S=1; // Sets I2C host Start Mode   
    while(!I2C2STAT1bits.TXBE);// Write address is sent into the TX buffer
    I2C2TXB = data;
    while(!I2C1STAT1bits.TXBE);// Write address is sent into the TX buffer
   }

void I2C_2_N_2(uint8_t address, uint16_t reg, uint16_t data){
    uint8_t regLow = reg & 0xFF;
    uint8_t regHigh = reg >> 8;
    uint8_t dataLow = data & 0xFF;
    uint8_t dataHigh = data >> 8;
    
    I2C1ADB1 = (uint8_t) (address<< 1);
    I2C1TXB = regHigh;
    I2C1CNTL = 1; 
    I2C1CON0bits.S=1; // Sets I2C host Start Mode   
    while(!I2C1STAT1bits.TXBE);// Write address is sent into the TX buffer
    I2C1CNTL = 1; 
    I2C1TXB = regLow;
    while(!I2C1STAT1bits.TXBE);// Write address is sent into the TX buffer
    I2C1CNTL = 1;
    I2C1TXB = dataLow;
    while(!I2C1STAT1bits.TXBE);// Write address is sent into the TX buffer
    I2C1CNTL = 1;
    I2C1TXB = dataHigh;
   }

uint16_t I2C1_randomRead2Byte(uint8_t address, uint16_t reg){
    uint8_t regLow = reg & 0xFF;
    uint8_t regHigh = reg >> 8;
    uint8_t dataLow;
    uint16_t dataHigh;
    uint8_t data;
   
    I2C1ADB1 = (uint8_t) (address<< 1);
    I2C1TXB = regHigh;
    I2C1CNTL = 1; 
    I2C1CON0bits.S=1; // Sets I2C host Start Mode   
    while(!I2C1STAT1bits.TXBE);// Write address is sent into the TX buffer
    I2C1CNTL = 1; 
    I2C1TXB = regLow;
    I2C1CON0bits.RSEN = 1;
    I2C1CON0bits.S=1; // Sets I2C host Start Mode  
    while(!I2C1STAT1bits.TXBE);// Write address is sent into the TX buffer
    
    
    while(!I2C1CON0bits.MDR);//
    address = (uint8_t) (address<< 1);
    I2C1ADB1 = (uint8_t) (address | 1);
    I2C1CNTL = 2; 
    I2C1CON0bits.S=1; // Sets I2C host Start Mode  
    I2C1CON0bits.RSEN = 0;
    while(!I2C1STAT1bits.RXBF);
    dataLow = I2C1RXB; 
    I2C1CON0bits.S=1; // Sets I2C host Start Mode  
    while(!I2C1STAT1bits.RXBF);
    dataHigh = I2C1RXB;

    data = dataHigh << 8;
    data = data | dataLow;
    return data;
   }

uint16_t I2C1_Read_1ByteAdd_2ByteData(uint8_t address, uint8_t reg){
    uint8_t dataLow;
    uint8_t dataHigh;
    uint16_t data;

    I2C2ADB1 = (uint8_t) (address<< 1);
    I2C2TXB = reg;
    I2C2CNTL = 1; 
    I2C2CON0bits.S=1; // Sets I2C host Start Mode   
    while(!I2C2STAT1bits.TXBE);// Write address is sent into the TX buffer
    I2C2CON0bits.RSEN = 1;

    while(!I2C2CON0bits.MDR);//    

    address = (uint8_t) (address<< 1);
    I2C2ADB1 = (uint8_t) (address | 1);
    I2C2CNTL = 2; 
    I2C2CON0bits.S=1; // Sets I2C host Start Mode  
    I2C2CON0bits.RSEN = 0;
    while(!I2C2STAT1bits.RXBF);
    dataLow = I2C2RXB;
    I2C2CON0bits.S=1; // Sets I2C host Start Mode  
    while(!I2C2STAT1bits.RXBF);
    dataHigh = I2C2RXB;        

    data = dataHigh << 8;
    data = data | dataLow;
    return data;
   }