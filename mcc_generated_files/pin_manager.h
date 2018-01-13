/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB(c) Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - 4.26
        Device            :  PIC16F1829
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.40

    Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

    Microchip licenses to you the right to use, modify, copy and distribute
    Software only when embedded on a Microchip microcontroller or digital signal
    controller that is integrated into your product or third party product
    (pursuant to the sublicense terms in the accompanying license agreement).

    You should refer to the license agreement accompanying this Software for
    additional information regarding your rights and obligations.

    SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
    EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
    MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
    IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
    CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
    OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
    INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
    CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
    SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
    (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

*/


#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set GPIO4 aliases
#define GPIO4_TRIS               TRISAbits.TRISA0
#define GPIO4_LAT                LATAbits.LATA0
#define GPIO4_PORT               PORTAbits.RA0
#define GPIO4_WPU                WPUAbits.WPUA0
#define GPIO4_ANS                ANSELAbits.ANSA0
#define GPIO4_SetHigh()            do { LATAbits.LATA0 = 1; } while(0)
#define GPIO4_SetLow()             do { LATAbits.LATA0 = 0; } while(0)
#define GPIO4_Toggle()             do { LATAbits.LATA0 = ~LATAbits.LATA0; } while(0)
#define GPIO4_GetValue()           PORTAbits.RA0
#define GPIO4_SetDigitalInput()    do { TRISAbits.TRISA0 = 1; } while(0)
#define GPIO4_SetDigitalOutput()   do { TRISAbits.TRISA0 = 0; } while(0)
#define GPIO4_SetPullup()      do { WPUAbits.WPUA0 = 1; } while(0)
#define GPIO4_ResetPullup()    do { WPUAbits.WPUA0 = 0; } while(0)
#define GPIO4_SetAnalogMode()  do { ANSELAbits.ANSA0 = 1; } while(0)
#define GPIO4_SetDigitalMode() do { ANSELAbits.ANSA0 = 0; } while(0)

// get/set GPIO3 aliases
#define GPIO3_TRIS               TRISAbits.TRISA1
#define GPIO3_LAT                LATAbits.LATA1
#define GPIO3_PORT               PORTAbits.RA1
#define GPIO3_WPU                WPUAbits.WPUA1
#define GPIO3_ANS                ANSELAbits.ANSA1
#define GPIO3_SetHigh()            do { LATAbits.LATA1 = 1; } while(0)
#define GPIO3_SetLow()             do { LATAbits.LATA1 = 0; } while(0)
#define GPIO3_Toggle()             do { LATAbits.LATA1 = ~LATAbits.LATA1; } while(0)
#define GPIO3_GetValue()           PORTAbits.RA1
#define GPIO3_SetDigitalInput()    do { TRISAbits.TRISA1 = 1; } while(0)
#define GPIO3_SetDigitalOutput()   do { TRISAbits.TRISA1 = 0; } while(0)
#define GPIO3_SetPullup()      do { WPUAbits.WPUA1 = 1; } while(0)
#define GPIO3_ResetPullup()    do { WPUAbits.WPUA1 = 0; } while(0)
#define GPIO3_SetAnalogMode()  do { ANSELAbits.ANSA1 = 1; } while(0)
#define GPIO3_SetDigitalMode() do { ANSELAbits.ANSA1 = 0; } while(0)

// get/set PWM3 procedures
#define PWM3_SetHigh()    do { LATAbits.LATA2 = 1; } while(0)
#define PWM3_SetLow()   do { LATAbits.LATA2 = 0; } while(0)
#define PWM3_Toggle()   do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define PWM3_GetValue()         PORTAbits.RA2
#define PWM3_SetDigitalInput()   do { TRISAbits.TRISA2 = 1; } while(0)
#define PWM3_SetDigitalOutput()  do { TRISAbits.TRISA2 = 0; } while(0)
#define PWM3_SetPullup()     do { WPUAbits.WPUA2 = 1; } while(0)
#define PWM3_ResetPullup()   do { WPUAbits.WPUA2 = 0; } while(0)
#define PWM3_SetAnalogMode() do { ANSELAbits.ANSA2 = 1; } while(0)
#define PWM3_SetDigitalMode()do { ANSELAbits.ANSA2 = 0; } while(0)

// get/set GPIO1 aliases
#define GPIO1_TRIS               TRISAbits.TRISA4
#define GPIO1_LAT                LATAbits.LATA4
#define GPIO1_PORT               PORTAbits.RA4
#define GPIO1_WPU                WPUAbits.WPUA4
#define GPIO1_ANS                ANSELAbits.ANSA4
#define GPIO1_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define GPIO1_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define GPIO1_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define GPIO1_GetValue()           PORTAbits.RA4
#define GPIO1_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define GPIO1_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define GPIO1_SetPullup()      do { WPUAbits.WPUA4 = 1; } while(0)
#define GPIO1_ResetPullup()    do { WPUAbits.WPUA4 = 0; } while(0)
#define GPIO1_SetAnalogMode()  do { ANSELAbits.ANSA4 = 1; } while(0)
#define GPIO1_SetDigitalMode() do { ANSELAbits.ANSA4 = 0; } while(0)

// get/set GPIO2 aliases
#define GPIO2_TRIS               TRISAbits.TRISA5
#define GPIO2_LAT                LATAbits.LATA5
#define GPIO2_PORT               PORTAbits.RA5
#define GPIO2_WPU                WPUAbits.WPUA5
#define GPIO2_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define GPIO2_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define GPIO2_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define GPIO2_GetValue()           PORTAbits.RA5
#define GPIO2_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define GPIO2_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define GPIO2_SetPullup()      do { WPUAbits.WPUA5 = 1; } while(0)
#define GPIO2_ResetPullup()    do { WPUAbits.WPUA5 = 0; } while(0)

// get/set RB4 procedures
#define SDA_SetHigh()    do { LATBbits.LATB4 = 1; } while(0)
#define SDA_SetLow()   do { LATBbits.LATB4 = 0; } while(0)
#define SDA_Toggle()   do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define SDA_GetValue()         PORTBbits.RB4
#define SDA_SetDigitalInput()   do { TRISBbits.TRISB4 = 1; } while(0)
#define SDA_SetDigitalOutput()  do { TRISBbits.TRISB4 = 0; } while(0)
#define SDA_SetPullup()     do { WPUBbits.WPUB4 = 1; } while(0)
#define SDA_ResetPullup()   do { WPUBbits.WPUB4 = 0; } while(0)
#define SDA_SetAnalogMode() do { ANSELBbits.ANSB4 = 1; } while(0)
#define SDA_SetDigitalMode()do { ANSELBbits.ANSB4 = 0; } while(0)

// get/set PiActive aliases
#define PiActive_TRIS               TRISBbits.TRISB5
#define PiActive_LAT                LATBbits.LATB5
#define PiActive_PORT               PORTBbits.RB5
#define PiActive_WPU                WPUBbits.WPUB5
#define PiActive_ANS                ANSELBbits.ANSB5
#define PiActive_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define PiActive_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define PiActive_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define PiActive_Halted()           PORTBbits.RB5
#define PiActive_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define PiActive_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define PiActive_SetPullup()      do { WPUBbits.WPUB5 = 1; } while(0)
#define PiActive_ResetPullup()    do { WPUBbits.WPUB5 = 0; } while(0)
#define PiActive_SetAnalogMode()  do { ANSELBbits.ANSB5 = 1; } while(0)
#define PiActive_SetDigitalMode() do { ANSELBbits.ANSB5 = 0; } while(0)

// get/set RB6 procedures
#define SCL_SetHigh()    do { LATBbits.LATB6 = 1; } while(0)
#define SCL_SetLow()   do { LATBbits.LATB6 = 0; } while(0)
#define SCL_Toggle()   do { LATBbits.LATB6 = ~LATBbits.LATB6; } while(0)
#define SCL_GetValue()         PORTBbits.RB6
#define SCL_SetDigitalInput()   do { TRISBbits.TRISB6 = 1; } while(0)
#define SCL_SetDigitalOutput()  do { TRISBbits.TRISB6 = 0; } while(0)
#define SCL_SetPullup()     do { WPUBbits.WPUB6 = 1; } while(0)
#define SCL_ResetPullup()   do { WPUBbits.WPUB6 = 0; } while(0)

// get/set Enable12v aliases
#define Enable12v_TRIS               TRISBbits.TRISB7
#define Enable12v_LAT                LATBbits.LATB7
#define Enable12v_PORT               PORTBbits.RB7
#define Enable12v_WPU                WPUBbits.WPUB7
#define Enable12v_SetHigh()            do { LATBbits.LATB7 = 1; } while(0)
#define Enable12v_SetLow()             do { LATBbits.LATB7 = 0; } while(0)
#define Enable12v_Toggle()             do { LATBbits.LATB7 = ~LATBbits.LATB7; } while(0)
#define Enable12v_GetValue()           PORTBbits.RB7
#define Enable12v_SetDigitalInput()    do { TRISBbits.TRISB7 = 1; } while(0)
#define Enable12v_SetDigitalOutput()   do { TRISBbits.TRISB7 = 0; } while(0)
#define Enable12v_SetPullup()      do { WPUBbits.WPUB7 = 1; } while(0)
#define Enable12v_ResetPullup()    do { WPUBbits.WPUB7 = 0; } while(0)

// get/set SenseV aliases
#define SenseV_TRIS               TRISCbits.TRISC0
#define SenseV_LAT                LATCbits.LATC0
#define SenseV_PORT               PORTCbits.RC0
#define SenseV_WPU                WPUCbits.WPUC0
#define SenseV_ANS                ANSELCbits.ANSC0
#define SenseV_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define SenseV_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define SenseV_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define SenseV_GetValue()           PORTCbits.RC0
#define SenseV_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define SenseV_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define SenseV_SetPullup()      do { WPUCbits.WPUC0 = 1; } while(0)
#define SenseV_ResetPullup()    do { WPUCbits.WPUC0 = 0; } while(0)
#define SenseV_SetAnalogMode()  do { ANSELCbits.ANSC0 = 1; } while(0)
#define SenseV_SetDigitalMode() do { ANSELCbits.ANSC0 = 0; } while(0)

// get/set enable5v aliases
#define enable5v_TRIS               TRISCbits.TRISC1
#define enable5v_LAT                LATCbits.LATC1
#define enable5v_PORT               PORTCbits.RC1
#define enable5v_WPU                WPUCbits.WPUC1
#define enable5v_ANS                ANSELCbits.ANSC1
#define enable5v_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define enable5v_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define enable5v_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define enable5v_GetValue()           PORTCbits.RC1
#define enable5v_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define enable5v_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define enable5v_SetPullup()      do { WPUCbits.WPUC1 = 1; } while(0)
#define enable5v_ResetPullup()    do { WPUCbits.WPUC1 = 0; } while(0)
#define enable5v_SetAnalogMode()  do { ANSELCbits.ANSC1 = 1; } while(0)
#define enable5v_SetDigitalMode() do { ANSELCbits.ANSC1 = 0; } while(0)

// get/set PowerGood aliases
#define PowerGood_TRIS               TRISCbits.TRISC2
#define PowerGood_LAT                LATCbits.LATC2
#define PowerGood_PORT               PORTCbits.RC2
#define PowerGood_WPU                WPUCbits.WPUC2
#define PowerGood_ANS                ANSELCbits.ANSC2
#define PowerGood_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define PowerGood_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define PowerGood_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define PowerGood_GetValue()           PORTCbits.RC2
#define PowerGood_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define PowerGood_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define PowerGood_SetPullup()      do { WPUCbits.WPUC2 = 1; } while(0)
#define PowerGood_ResetPullup()    do { WPUCbits.WPUC2 = 0; } while(0)
#define PowerGood_SetAnalogMode()  do { ANSELCbits.ANSC2 = 1; } while(0)
#define PowerGood_SetDigitalMode() do { ANSELCbits.ANSC2 = 0; } while(0)

// get/set PWM2 procedures
#define PWM2_SetHigh()    do { LATCbits.LATC3 = 1; } while(0)
#define PWM2_SetLow()   do { LATCbits.LATC3 = 0; } while(0)
#define PWM2_Toggle()   do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define PWM2_GetValue()         PORTCbits.RC3
#define PWM2_SetDigitalInput()   do { TRISCbits.TRISC3 = 1; } while(0)
#define PWM2_SetDigitalOutput()  do { TRISCbits.TRISC3 = 0; } while(0)
#define PWM2_SetPullup()     do { WPUCbits.WPUC3 = 1; } while(0)
#define PWM2_ResetPullup()   do { WPUCbits.WPUC3 = 0; } while(0)
#define PWM2_SetAnalogMode() do { ANSELCbits.ANSC3 = 1; } while(0)
#define PWM2_SetDigitalMode()do { ANSELCbits.ANSC3 = 0; } while(0)

// get/set Status aliases
#define Status_TRIS               TRISCbits.TRISC4
#define Status_LAT                LATCbits.LATC4
#define Status_PORT               PORTCbits.RC4
#define Status_WPU                WPUCbits.WPUC4
#define Status_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define Status_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define Status_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define Status_GetValue()           PORTCbits.RC4
#define Status_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define Status_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define Status_SetPullup()      do { WPUCbits.WPUC4 = 1; } while(0)
#define Status_ResetPullup()    do { WPUCbits.WPUC4 = 0; } while(0)

// get/set RC5 procedures
#define PWM1_SetHigh()    do { LATCbits.LATC5 = 1; } while(0)
#define PWM1_SetLow()   do { LATCbits.LATC5 = 0; } while(0)
#define PWM1_Toggle()   do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define PWM1_GetValue()         PORTCbits.RC5
#define PWM1_SetDigitalInput()   do { TRISCbits.TRISC5 = 1; } while(0)
#define PWM1_SetDigitalOutput()  do { TRISCbits.TRISC5 = 0; } while(0)
#define PWM1_SetPullup()     do { WPUCbits.WPUC5 = 1; } while(0)
#define PWM1_ResetPullup()   do { WPUCbits.WPUC5 = 0; } while(0)

// get/set RC6 procedures
#define PWM4_SetHigh()    do { LATCbits.LATC6 = 1; } while(0)
#define PWM4_SetLow()   do { LATCbits.LATC6 = 0; } while(0)
#define PWM4_Toggle()   do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define PWM4_GetValue()         PORTCbits.RC6
#define PWM4_SetDigitalInput()   do { TRISCbits.TRISC6 = 1; } while(0)
#define PWM4_SetDigitalOutput()  do { TRISCbits.TRISC6 = 0; } while(0)
#define PWM4_SetPullup()     do { WPUCbits.WPUC6 = 1; } while(0)
#define PWM4_ResetPullup()   do { WPUCbits.WPUC6 = 0; } while(0)
#define PWM4_SetAnalogMode() do { ANSELCbits.ANSC6 = 1; } while(0)
#define PWM4_SetDigitalMode()do { ANSELCbits.ANSC6 = 0; } while(0)

// get/set SenseI aliases
#define SenseI_TRIS               TRISCbits.TRISC7
#define SenseI_LAT                LATCbits.LATC7
#define SenseI_PORT               PORTCbits.RC7
#define SenseI_WPU                WPUCbits.WPUC7
#define SenseI_ANS                ANSELCbits.ANSC7
#define SenseI_SetHigh()            do { LATCbits.LATC7 = 1; } while(0)
#define SenseI_SetLow()             do { LATCbits.LATC7 = 0; } while(0)
#define SenseI_Toggle()             do { LATCbits.LATC7 = ~LATCbits.LATC7; } while(0)
#define SenseI_GetValue()           PORTCbits.RC7
#define SenseI_SetDigitalInput()    do { TRISCbits.TRISC7 = 1; } while(0)
#define SenseI_SetDigitalOutput()   do { TRISCbits.TRISC7 = 0; } while(0)
#define SenseI_SetPullup()      do { WPUCbits.WPUC7 = 1; } while(0)
#define SenseI_ResetPullup()    do { WPUCbits.WPUC7 = 0; } while(0)
#define SenseI_SetAnalogMode()  do { ANSELCbits.ANSC7 = 1; } while(0)
#define SenseI_SetDigitalMode() do { ANSELCbits.ANSC7 = 0; } while(0)

/**
   @Param
    none
   @Returns
    none
   @Description
    GPIO and peripheral I/O initialization
   @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize (void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);



#endif // PIN_MANAGER_H
/**
 End of File
*/