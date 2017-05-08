/**
  TMR2 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    tmr2.c

  @Summary
    This is the generated driver implementation file for the TMR2 driver using MPLAB(c) Code Configurator

  @Description
    This source file provides APIs for TMR2.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - v3.00
        Device            :  PIC18LF46K22
        Driver Version    :  2.00
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.20
*/

/*
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

/**
  Section: Included Files
*/

#include <xc.h>
#include "tmr2.h"
#include "pwm4.h"
#include "pwm5.h"
#include "mcc.h"

/**
  Section: TMR2 APIs
*/
int PWMFlag = 0;
int counter = 0; 

void TMR2_Initialize(void)
{
    // Set TMR2 to the options selected in the User Interface

    // T2CKPS 1:16; T2OUTPS 1:2; TMR2ON off; 
    T2CON = 0x0A;

    // PR2 155; 
    PR2 = 0x9B;

    // TMR2 0; 
    TMR2 = 0x00;

    // Clearing IF flag before enabling the interrupt.
    PIR1bits.TMR2IF = 0;

    // Enabling TMR2 interrupt.
    PIE1bits.TMR2IE = 1;

    // Set Default Interrupt Handler
   // TMR2_SetInterruptHandler(TMR2_DefaultInterruptHandler);

    // Start TMR2
    TMR2_StartTimer();
}

void TMR2_StartTimer(void)
{
    // Start the Timer by writing to TMRxON bit
    T2CONbits.TMR2ON = 1;
}

void TMR2_StopTimer(void)
{
    // Stop the Timer by writing to TMRxON bit
    T2CONbits.TMR2ON = 0;
}

uint8_t TMR2_ReadTimer(void)
{
    uint8_t readVal;

    readVal = TMR2;

    return readVal;
}

void TMR2_WriteTimer(uint8_t timerVal)
{
    // Write to the Timer2 register
    TMR2 = timerVal;
}

void TMR2_LoadPeriodRegister(uint8_t periodVal)
{
   PR2 = periodVal;
}

void TMR2_ISR(void)
{

    // clear the TMR2 interrupt flag
    PIR1bits.TMR2IF = 0;

    // ticker function call;
    // ticker is 1 -> Callback function gets called everytime this ISR executes
    //TMR2_CallBack();
    TMR2_DefaultInterruptHandler();
    
}

void TMR2_CallBack(void)
{
    // Add your custom callback code here
    // this code executes every TMR2_INTERRUPT_TICKER_FACTOR periods of TMR2
    if(TMR2_InterruptHandler)
    {
        TMR2_InterruptHandler();
    }
}

void TMR2_SetInterruptHandler(void* InterruptHandler){
    TMR2_InterruptHandler = InterruptHandler;
}

void TMR2_DefaultInterruptHandler(void){
    // add your TMR2 interrupt custom code
    // or set custom function using TMR2_SetInterruptHandler()
    
   if(on) 
   {//direction = 1 then motors move together
    if(direction1 == 1)
    {
        if(counter>=PWMdelay)
        {
            if(PWMFlag == 1)
            {
             PWM4_LoadDutyValue(529);
             PWM5_LoadDutyValue(529);
             PWMFlag = 0;
            }
            else 
            {
                PWM4_LoadDutyValue(585);
                PWM5_LoadDutyValue(585);//604
                PWMFlag = 1;
            }
            counter = 0;
        }
        else
             counter++;
    }
    else if(direction2 == 1)
    {
        if(counter>=PWMdelay)
        {
            if(PWMFlag == 1)
            {
             PWM4_LoadDutyValue(529);
             PWM5_LoadDutyValue(585);
             PWMFlag = 0;
            }
            else 
            {
                PWM4_LoadDutyValue(585);
                PWM5_LoadDutyValue(529);
                PWMFlag = 1;
            }
            counter = 0;
        }
        else
             counter++;
    }
   }
        
}

/**
  End of File
*/