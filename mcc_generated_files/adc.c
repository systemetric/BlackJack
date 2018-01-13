/**
  ADC Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    adc.c

  @Summary
    This is the generated driver implementation file for the ADC driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  @Description
    This source file provides implementations for driver APIs for ADC.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs  - 1.45
        Device            :  PIC16F1829
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40
*/

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
*/

#include <xc.h>
#include "adc.h"
#include "mcc.h"

/**
  Section: Macro Declarations
*/

#define ACQ_US_DELAY 5
adc_channel_t CurrentChannel = SenseV;
//adc_result_t ARSenseV,ARSenseI,ARGPIO1,ARGPIO3,ARGPIO4;
bool GPIO1Enable,GPIO3Enable,GPIO4Enable;  //Defaults to disabled 


/**
  Section: ADC Module APIs
*/

void ADC_Initialize(void)
{
    // set the ADC to the options selected in the User Interface
    
    // GO_nDONE stop; ADON enabled; CHS AN0; 
    ADCON0 = 0x01;
    
    // ADFM left; ADNREF VSS; ADPREF VDD; ADCS FOSC/32; 
    ADCON1 = 0x20;
    
    // ADRESL 0; 
    ADRESL = 0x00;
    
    // ADRESH 0; 
    ADRESH = 0x00;
    
    // Enabling ADC interrupt.
    PIE1bits.ADIE = 1;
}

void ADC_SelectChannel(adc_channel_t channel)
{
    // select the A/D channel
    ADCON0bits.CHS = channel;    
    // Turn on the ADC module
    ADCON0bits.ADON = 1;  
}

void ADC_StartConversion()
{
    // Start the conversion
    ADCON0bits.GO_nDONE = 1;
}


bool ADC_IsConversionDone()
{
    // Start the conversion
    return ((bool)(!ADCON0bits.GO_nDONE));
}

adc_result_t ADC_GetConversionResult(void)
{
    // Conversion finished, return the result
    return ((adc_result_t)((ADRESH << 8) + ADRESL));
}

adc_result_t ADC_GetConversion(adc_channel_t channel)
{
    // select the A/D channel
    ADCON0bits.CHS = channel;    

    // Turn on the ADC module
    ADCON0bits.ADON = 1;
    // Acquisition time delay
    __delay_us(ACQ_US_DELAY);

    // Start the conversion
    ADCON0bits.GO_nDONE = 1;

    // Wait for the conversion to finish
    while (ADCON0bits.GO_nDONE)
    {
    }

    // Conversion finished, return the result
    return ((adc_result_t)((ADRESH << 8) + ADRESL));
}


void ADC_ISR(void)
{
    uint16_t AnalogReading;

#ifdef PWM_AS_DEBUG
    PWM1_SetHigh();
#endif
    // Clear the ADC interrupt flag
    PIR1bits.ADIF = 0;
      
    
 
    // Update local storage for active channel 
    switch(CurrentChannel)
    {
        case SenseV:
            I2C1_WriteBuffer(I2CSenseV_H,ADRESH);
            I2C1_WriteBuffer(I2CSenseV_L,ADRESL);
            break;
        case SenseI:
            I2C1_WriteBuffer(I2CSenseI_H,ADRESH);
            I2C1_WriteBuffer(I2CSenseI_L,ADRESL);
            break;
        case GPIO1:
            I2C1_WriteBuffer(I2CAGPIO1_H,ADRESH);
            I2C1_WriteBuffer(I2CAGPIO1_L,ADRESH);
            break;
        case GPIO3:
            I2C1_WriteBuffer(I2CAGPIO3_H,ADRESH);
            I2C1_WriteBuffer(I2CAGPIO3_L,ADRESH);
            break;
        case GPIO4:
            I2C1_WriteBuffer(I2CAGPIO4_H,ADRESH);
            I2C1_WriteBuffer(I2CAGPIO4_L,ADRESH);
            break;
    }
    
    // setup the ADC to read from the next active input
    switch(CurrentChannel)
    {
        // NOTE: Fall through breaks are intentional
        case SenseV:
            CurrentChannel = SenseI;
            break;
        case SenseI:
            if (GPIO1Enable) {
                CurrentChannel = GPIO1;   
                break;
            }
        case GPIO1:
            if (GPIO3Enable) {
                CurrentChannel = GPIO3;
                break;
            }
        case GPIO3:
            if (GPIO4Enable) {
                CurrentChannel = GPIO4;
                break;
            }
        case GPIO4:
            CurrentChannel = SenseV;
            ADCValid = 1;
    }
    
    
    
    ADC_SelectChannel(CurrentChannel);                 
#if PWM_AS_DEBUG
    PWM1_SetLow();
#endif
    // Don't actually kick off a sample, this will be done by TMR6 ISR to ensure
    // that the ADC ISR happens at a quiet time, this also means we don't have to 
    // worry about Acquisition time delay caused by switching inputs
}

void ADC_EnableChannel(adc_channel_t Channel) {
    switch(Channel)
    {  
        case GPIO1:
           GPIO1Enable = true;
           break;
        case GPIO3:
           GPIO3Enable = true;
           break;
        case GPIO4:
           GPIO4Enable = true;
           break; 
    }
}

void ADC_DisableChannel(adc_channel_t Channel) {
    switch(Channel)
    {  
        case GPIO1:
           GPIO1Enable = false;
           break;
        case GPIO3:
           GPIO3Enable = false;
           break;
        case GPIO4:
           GPIO4Enable = false;
           break; 
    }
}


/**
 End of File
*/