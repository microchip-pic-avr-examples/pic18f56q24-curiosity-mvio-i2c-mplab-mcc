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
© [2023] Microchip Technology Inc. and its subsidiaries.

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
uint8_t buffer[10] = { 0 } ;
uint8_t ii =0x0;
uint8_t i =0x4;

//#define Read=0b10101101;
//#define Write=0b10101100;   


        
           
            


void I2C_RAW_Write (void)
{
        while(ii<0xFF)
    { 
      I2C1ADB1=0b10101100; //address
        
        
        I2C1CNT=3; //byte count
        I2C1CON0bits.S=1;
        
        I2C1TXB=0b00000001;
        while(!I2C1STAT1bits.TXBE);// Write address is sent into the TX buffer
        
        
      
        
            
        I2C1TXB=ii;
        while(!I2C1STAT1bits.TXBE);    
        I2C1CON0bits.S=1;
        I2C1TXB=ii;// Write address is sent into the TX buffer
        while(!I2C1STAT1bits.TXBE);
        //I2C1TXB=0xDD; ////Dummy Data
        ii++;
        
        __delay_ms(50);
        
      }
       
        //        for(ii=0x0; ii<0xFF; ii++) //init,test,update
//{
//        
//        while(!I2C1STAT1bits.TXBE); //waits till transmitter is empty
//        I2C1TXB=ii;// Write address is sent into the TX buffer
//        //I2C1TXB=0xDD; ////Dummy Data
//        __delay_ms(50); 
//        
//        
//}
}
        
        
        



void I2C_DummyWrite(void){
     
    I2C1ADB1=0b10101100; //Address Buffer, This is how we say hi
    I2C1CNT=2; // Byte Count if added gives extra byte
    
    
    I2C1TXB=0b00000001;
    I2C1CON0bits.S=1; // Sets I2C host Start Mode 
    
    while(!I2C1STAT1bits.TXBE);// Write address is sent into the TX buffer
    
    I2C1TXB=i;
    while(!I2C1STAT1bits.TXBE);
   
}
    
    
//    __delay_ms(50);
    
void I2C_Read(void){
    
    
    I2C1ADB1=0b10101101;
    I2C1CNT=1;
//    I2C1CON0bits.RSEN=1;
    I2C1CON0bits.S=1; 
    while(!I2C1STAT1bits.RXBF);//
    buffer[9]=I2C1RXB;
    
   // i++;
    
}

    
    
//    uint8_t * ptr = buffer;
//        
//    
//    while(!I2C1STAT1bits.RXBF)
//      {
//      if( I2C1ERRbits.NACKIF ) goto I2C_EXIT;
//      
//    
//   
//    
//    *ptr++ = I2C1RXB;
//    }
//    I2C_EXIT:
//    I2C1STAT1bits.CLRBF = 1;
    
    

     

int main(void)
{
    SYSTEM_Initialize();
    //I2C1_Initialize();
    
    //registers for I2C control
    
    //for Pins RC3
    
    RC3I2Cbits.SLEW=0b01; //Slew Control Register
    RC3I2Cbits.TH=0b01; //Input Threshold
    
    //for Pins RC4
    
    RC4I2Cbits.SLEW=0b01;   //Slew Control Register
    RC4I2Cbits.TH=0b01;   //Input Threshold
    
    I2C1CON0bits.MODE=0b100;  //Sets the peripheral for I2C control     
    
    I2C1CON2bits.SDAHT=0b10; // Data hold time 3ns
    I2C1CON2bits.BFRET=00; //8 I2CxCLK        
    
    I2C1CON3bits.FME=1; // Fast mode Enabled
    
            
    I2C1CLKbits.CLK=0b00000; //Clock as Fosc/4
    
    I2C1BAUD = 0x27; //sets baud rate at 39
    I2C1CON0bits.EN=1;  // Enables the I2C Modules 
    
    
    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts 
            
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global Interrupts 
    // Use the following macros to: 

    // Enable the Global High Interrupts 
    //INTERRUPT_GlobalInterruptHighEnable(); 

    // Disable the Global High Interrupts 
    //INTERRUPT_GlobalInterruptHighDisable(); 

    // Enable the Global Low Interrupts 
    //INTERRUPT_GlobalInterruptLowEnable(); 

    // Disable the Global Low Interrupts 
    //INTERRUPT_GlobalInterruptLowDisable(); 

//__delay_ms(500);
       
    
    //I2C_RAW_Write();
            I2C_DummyWrite();  
            __delay_ms(50);
            I2C_Read();
        
    while(1)
    { 
      
      
        
       
    }    
}