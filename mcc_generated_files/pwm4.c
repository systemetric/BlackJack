/**
  CCP4 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    ccp4.c

  @Summary
    This is the generated driver implementation file for the CCP4 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  @Description
    This source file provides implementations for driver APIs for CCP4.
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
#include "pwm4.h"
#include "mcc.h"

/**
  Section: Macro Declarations
*/

#define PWM4_INITIALIZE_DUTY_VALUE    767

/**
  Section: PWM Module APIs
*/

void PWM4_Initialize(void)
{
    // Set the PWM to the options selected in the PIC10 / PIC12 / PIC16 / PIC18 MCUs 

    // CCP4M PWM; DC4B 3; 
    CCP4CON = 0x3C;
    
    // CCPR4L 191; 
    CCPR4L = 0xBF;
    
    // CCPR4H 0; 
    CCPR4H = 0x00;
    
    // Selecting Timer 2
    CCPTMRSbits.C4TSEL = 0x0;
}

void PWM4_LoadDutyValue(uint16_t dutyValue)
{
   // Writing to 8 MSBs of pwm duty cycle in CCPRL register
    CCPR4L = ((dutyValue & 0x03FC)>>2);
    
   // Writing to 2 LSBs of pwm duty cycle in CCPCON register
    CCP4CON = ((uint8_t)(CCP4CON & 0xCF) | ((dutyValue & 0x0003)<<4));
}

void PWM4_PowerDown (void) {
    CCP4CON = 0x0;
    PWM4_SetDigitalOutput();
    PWM4_SetLow();
}

/**
 End of File
*/

