/**
  ECCP2 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    eccp2.c

  @Summary
    This is the generated driver implementation file for the ECCP2 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  @Description
    This source file provides APIs for ECCP2.
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
#include "epwm2.h"
#include "mcc.h"

/**
  Section: Macro Declarations
*/

#define PWM2_INITIALIZE_DUTY_VALUE    767

/**
  Section: EPWM Module APIs
*/

void EPWM2_Initialize (void)
{
    // Set the PWM to the options selected in PIC10 / PIC12 / PIC16 / PIC18 MCUs 
    
    // CCP2M P2A: active high; P2B: active high; DC2B 3; P2M single; 
    CCP2CON = 0x3C;
    
    // CCP2ASE operating; PSS2BD low; PSS2AC low; CCP2AS disabled; 
    CCP2AS = 0x00;
    
    // P2RSEN automatic_restart; P2DC 0; 
    PWM2CON = 0x80;
    
    // STR2B P2B_to_port; STR2A P2A_to_CCP2M; STR2SYNC start_at_begin; 
    PSTR2CON = 0x01;
    
    // CCPR2L 191; 
    CCPR2L = 0xBF;
    
    // CCPR2H 0; 
    CCPR2H = 0x00;
    
    
    // Selecting Timer2
    CCPTMRSbits.C2TSEL = 0x0;
}

void EPWM2_LoadDutyValue(uint16_t dutyValue)
{
   // Writing to 8 MSBs of pwm duty cycle in CCPRL register
    CCPR2L = ((dutyValue & 0x03FC)>>2);
    
   // Writing to 2 LSBs of pwm duty cycle in CCPCON register
    CCP2CON = ((uint8_t)(CCP2CON & 0xCF) | ((dutyValue & 0x0003)<<4));
}

void EPWM2_PowerDown (void) {
    CCP2CON = 0x0;
    PWM2_SetDigitalOutput();
    PWM2_SetLow();
}

/**
 End of File
*/
