2/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs 

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
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

#include "mcc_generated_files/mcc.h"

typedef enum {
    sSHUTTINGDOWN,
    sRESET,
    sINIT,
    sRUNNING,
} System_State_t;

void Timer6Overflow(void) {
    static uint8_t count = 0;
    //    GPIO2_SetHigh();
    switch (count) {
        case 0:
            TMR2_StopTimer();
            break;
        case 1:
            if (!pwm_lock) EPWM1_LoadDutyValue((uint16_t)I2C1_ReadBuffer(I2CPWM1_L) + ((uint16_t)I2C1_ReadBuffer(I2CPWM1_H) * 0x100) + (uint16_t)00); //reads from buffer scales max of 250 to 500
            break;
        case 2:
            if (!pwm_lock) EPWM2_LoadDutyValue((uint16_t)I2C1_ReadBuffer(I2CPWM2_L) + ((uint16_t)I2C1_ReadBuffer(I2CPWM2_H) * 0x100) + (uint16_t)00);
            break;
        case 3:
            if (!pwm_lock)  PWM3_LoadDutyValue((uint16_t)I2C1_ReadBuffer(I2CPWM3_L) + ((uint16_t)I2C1_ReadBuffer(I2CPWM3_H) * 0x100) + (uint16_t)00);
            break;
        case 4:
            if (!pwm_lock)  PWM4_LoadDutyValue((uint16_t)I2C1_ReadBuffer(I2CPWM4_L) + ((uint16_t)I2C1_ReadBuffer(I2CPWM4_H) * 0x100) + (uint16_t)000);
            break;
        case 9:
            TMR2 = 0xFF;
            TMR2_StartTimer();
    }
    if (count < 9)
        count += 1;
    else
        count = 0;
    ADC_StartConversion(); //update ADC channels
    //    GPIO2_SetLow();
}

/*
                         Main application
 */
void main(void) {
    // initialize the device
    SYSTEM_Initialize();
    //Start in super low power mode
    Enable12v_SetLow();

    EPWM1_PowerDown();
    EPWM2_PowerDown();
    PWM3_PowerDown();
    PWM4_PowerDown();

    Status_SetLow();
    INTERRUPT_GlobalInterruptDisable();
    INTERRUPT_PeripheralInterruptDisable();

    enable5v_SetLow();
    PowerGood_SetLow();
    
    I2C1_Shutdown();
    SDA_SetDigitalOutput();
    SDA_SetLow();
    SCL_SetDigitalOutput();
    SCL_SetLow();

    GPIO1_SetDigitalOutput();
    GPIO1_SetLow();
    GPIO2_SetDigitalOutput();
    GPIO2_SetLow();
    GPIO3_SetDigitalOutput();
    GPIO3_SetLow();
    GPIO4_SetDigitalOutput();
    GPIO4_SetLow();

    //SUPER-DUPER LOW POWER CODE
    TMR6_StopTimer();
    TMR2_StopTimer();
    TMR6_SetInterruptHandler(Timer6Overflow);
    
    
    
    System_State_t System_State = sRESET;
    while (1) {       
        
        switch (System_State) {
            case sRESET://System has just boot dont do anything untill we know that the voltage is ok

                // This is the low power system state, if we are here the 
                // interrupts should be off, so ADC conversions have to happen 
                // manually. The only external stimulas is the vSense, poll this
                // and use sleep for low power consumption

                // GPIO Tri-state
                // Timer Stopped
                // Interrupts off
                // Motor Power off
                // LED off
                // PI power off
                // PWM disabled and pins set Tristate/Analog in using GPIO 
                // I2C tristate

                // Exit Cases:
                // Voltage exceeds 11V
            {
                if (ADC_GetConversion(SenseV) >= V11) {
                    System_State = sINIT;
                } else {
                    SWDTEN = 1;
                    SLEEP();
                    SWDTEN = 0;
                }
                break;
            }
            case sINIT: //Voltage is okay, its showtime
                /* This state starts up the system, it is responsible for bringing all the 
                 * periphals online (except motor power?)
                 */

                // GPIO Set to Zero
                // Timer running
                // Interrupts on
                // Motor Power on?
                // LED on?
                // PI power on
                // PWM enabled
                // I2C enabled
                // I2C memory cleared to default values

                // Exit Cases:
                // When done always exit
            {
                TMR6_StartTimer();
                TMR2_StartTimer();

                ADC_SelectChannel(SenseV);
                ADC_StartConversion();
                // Enable the Global Interrupts
                INTERRUPT_GlobalInterruptEnable();

                // Enable the Peripheral Interrupts
                INTERRUPT_PeripheralInterruptEnable();
                PowerGood_SetHigh();
                
                GPIO1_SetDigitalInput();
                GPIO2_SetDigitalInput();
                GPIO3_SetDigitalInput();
                GPIO4_SetDigitalInput();
                SDA_SetPullup();
                SCL_SetPullup();
                SDA_SetDigitalInput();
                SCL_SetDigitalInput();
                I2C1_Startup();
                enable5v_SetHigh();
                Enable12v_SetHigh();
                
                EPWM1_Initialize();
                EPWM2_Initialize();
                PWM3_Initialize();
                PWM4_Initialize();
                System_State = sRUNNING;
                break;
            }
            case sRUNNING:
                /* This state runs the system, it is responsible for keeping the periphals 
                 * loaded with correct data (where this is not done by the ISRs)
                 */

                // GPIO controlled by I2C
                // Timer running
                // Interrupts on
                // Motor Power on
                // LED strobe?
                // PI power on
                // PWM enabled
                // I2C enabled

                // Exit Cases:
                // Exit when vSense falls to zero
                // Exit when PI quits                
            {
#warning this is where a watchdog would be useful to ensure that IRQs do not get stuck
                Status_SetHigh();
                while (!ADCValid);
                if (I2C1_ReadBuffer(I2CSenseV_H) <= V9_8Bit) {
                    System_State = sSHUTTINGDOWN;
                    break;
                }
                if (PiActive_Halted()) {
                    System_State = sSHUTTINGDOWN;
                    break;
                }

                // Add your application code

                break;
            }
            case sSHUTTINGDOWN:
                /* This state shuts down the system, it is responsible cleanly shutting down 
                 * the PI and for bringing all the periphals into their lowest power state
                 */

                // GPIO Tristate/AnalogIn --DONE
                // Timer stopped          --DONE
                // Interrupts off         --DONE
                // Motor Power off        --DUN  
                // LED off                --DONE
                // PI power down cleanly  --DONE
                // PWM Tristate/AnalogIn  --DONE
                // I2C disabled           --DONE

                // Exit Cases:
                // Exit when PI quits
                // exit if PI fails to quit after a reasonable time
            {
                Enable12v_SetLow();
                
                EPWM1_PowerDown();
                EPWM2_PowerDown();
                PWM3_PowerDown();
                PWM4_PowerDown();
                
                Status_SetLow();
                INTERRUPT_GlobalInterruptDisable();
                INTERRUPT_PeripheralInterruptDisable();

                PowerGood_SetLow(); //Sets power good Low
                int i = 0;
                while (i < 10) {
                    if (PiActive_Halted())
                        break;
                    SWDTEN = 1;
                    SLEEP();
                    i += 1;
                }
                SWDTEN = 0;
                
                enable5v_SetLow();
                
                //Shutdown i2c
                I2C1_Shutdown();
                SDA_SetDigitalOutput();
                SDA_SetLow();
                SCL_SetDigitalOutput();
                SCL_SetLow();
                
                GPIO1_SetDigitalOutput();
                GPIO1_SetLow();
                GPIO2_SetDigitalOutput();
                GPIO2_SetLow();
                GPIO3_SetDigitalOutput();
                GPIO3_SetLow();
                GPIO4_SetDigitalOutput();
                GPIO4_SetLow();
                PowerGood_SetHigh();

                //SUPER LOW POWER CODE
                TMR6_StopTimer();
                TMR2_StopTimer();


#warning We must enter super low power state here - have we done enough?
#warning If the PI initilized the shutdown itself then we are going to 
#warning wake up immediatly, we should pause for a minimum of 1 second to
#warning ensure the PI does receive a power cycle                

                System_State = sRESET;
                break;
            }
        }
    }
}
/**
 End of File
 */