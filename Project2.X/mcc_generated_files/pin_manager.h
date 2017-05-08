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
        Product Revision  :  MPLAB(c) Code Configurator - v3.00
        Device            :  PIC18LF46K22
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.35
        MPLAB             :  MPLAB X 3.20

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

// get/set IO_RB0 aliases
#define IO_RB0_TRIS               TRISB0
#define IO_RB0_LAT                LATB0
#define IO_RB0_PORT               RB0
#define IO_RB0_WPU                WPUB0
#define IO_RB0_ANS                ANSB0
#define IO_RB0_SetHigh()    do { LATB0 = 1; } while(0)
#define IO_RB0_SetLow()   do { LATB0 = 0; } while(0)
#define IO_RB0_Toggle()   do { LATB0 = ~LATB0; } while(0)
#define IO_RB0_GetValue()         RB0
#define IO_RB0_SetDigitalInput()    do { TRISB0 = 1; } while(0)
#define IO_RB0_SetDigitalOutput()   do { TRISB0 = 0; } while(0)

#define IO_RB0_SetPullup()    do { WPUB0 = 1; } while(0)
#define IO_RB0_ResetPullup()   do { WPUB0 = 0; } while(0)
#define IO_RB0_SetAnalogMode()   do { ANSB0 = 1; } while(0)
#define IO_RB0_SetDigitalMode()   do { ANSB0 = 0; } while(0)


// get/set IO_RB1 aliases
#define IO_RB1_TRIS               TRISB1
#define IO_RB1_LAT                LATB1
#define IO_RB1_PORT               RB1
#define IO_RB1_WPU                WPUB1
#define IO_RB1_ANS                ANSB1
#define IO_RB1_SetHigh()    do { LATB1 = 1; } while(0)
#define IO_RB1_SetLow()   do { LATB1 = 0; } while(0)
#define IO_RB1_Toggle()   do { LATB1 = ~LATB1; } while(0)
#define IO_RB1_GetValue()         RB1
#define IO_RB1_SetDigitalInput()    do { TRISB1 = 1; } while(0)
#define IO_RB1_SetDigitalOutput()   do { TRISB1 = 0; } while(0)

#define IO_RB1_SetPullup()    do { WPUB1 = 1; } while(0)
#define IO_RB1_ResetPullup()   do { WPUB1 = 0; } while(0)
#define IO_RB1_SetAnalogMode()   do { ANSB1 = 1; } while(0)
#define IO_RB1_SetDigitalMode()   do { ANSB1 = 0; } while(0)


// get/set IO_RB2 aliases
#define IO_RB2_TRIS               TRISB2
#define IO_RB2_LAT                LATB2
#define IO_RB2_PORT               RB2
#define IO_RB2_WPU                WPUB2
#define IO_RB2_ANS                ANSB2
#define IO_RB2_SetHigh()    do { LATB2 = 1; } while(0)
#define IO_RB2_SetLow()   do { LATB2 = 0; } while(0)
#define IO_RB2_Toggle()   do { LATB2 = ~LATB2; } while(0)
#define IO_RB2_GetValue()         RB2
#define IO_RB2_SetDigitalInput()    do { TRISB2 = 1; } while(0)
#define IO_RB2_SetDigitalOutput()   do { TRISB2 = 0; } while(0)

#define IO_RB2_SetPullup()    do { WPUB2 = 1; } while(0)
#define IO_RB2_ResetPullup()   do { WPUB2 = 0; } while(0)
#define IO_RB2_SetAnalogMode()   do { ANSB2 = 1; } while(0)
#define IO_RB2_SetDigitalMode()   do { ANSB2 = 0; } while(0)


// get/set IO_RB3 aliases
#define IO_RB3_TRIS               TRISB3
#define IO_RB3_LAT                LATB3
#define IO_RB3_PORT               RB3
#define IO_RB3_WPU                WPUB3
#define IO_RB3_ANS                ANSB3
#define IO_RB3_SetHigh()    do { LATB3 = 1; } while(0)
#define IO_RB3_SetLow()   do { LATB3 = 0; } while(0)
#define IO_RB3_Toggle()   do { LATB3 = ~LATB3; } while(0)
#define IO_RB3_GetValue()         RB3
#define IO_RB3_SetDigitalInput()    do { TRISB3 = 1; } while(0)
#define IO_RB3_SetDigitalOutput()   do { TRISB3 = 0; } while(0)

#define IO_RB3_SetPullup()    do { WPUB3 = 1; } while(0)
#define IO_RB3_ResetPullup()   do { WPUB3 = 0; } while(0)
#define IO_RB3_SetAnalogMode()   do { ANSB3 = 1; } while(0)
#define IO_RB3_SetDigitalMode()   do { ANSB3 = 0; } while(0)


// get/set IO_RB4 aliases
#define IO_RB4_TRIS               TRISB4
#define IO_RB4_LAT                LATB4
#define IO_RB4_PORT               RB4
#define IO_RB4_WPU                WPUB4
#define IO_RB4_ANS                ANSB4
#define IO_RB4_SetHigh()    do { LATB4 = 1; } while(0)
#define IO_RB4_SetLow()   do { LATB4 = 0; } while(0)
#define IO_RB4_Toggle()   do { LATB4 = ~LATB4; } while(0)
#define IO_RB4_GetValue()         RB4
#define IO_RB4_SetDigitalInput()    do { TRISB4 = 1; } while(0)
#define IO_RB4_SetDigitalOutput()   do { TRISB4 = 0; } while(0)

#define IO_RB4_SetPullup()    do { WPUB4 = 1; } while(0)
#define IO_RB4_ResetPullup()   do { WPUB4 = 0; } while(0)
#define IO_RB4_SetAnalogMode()   do { ANSB4 = 1; } while(0)
#define IO_RB4_SetDigitalMode()   do { ANSB4 = 0; } while(0)


// get/set IO_RB5 aliases
#define IO_RB5_TRIS               TRISB5
#define IO_RB5_LAT                LATB5
#define IO_RB5_PORT               RB5
#define IO_RB5_WPU                WPUB5
#define IO_RB5_ANS                ANSB5
#define IO_RB5_SetHigh()    do { LATB5 = 1; } while(0)
#define IO_RB5_SetLow()   do { LATB5 = 0; } while(0)
#define IO_RB5_Toggle()   do { LATB5 = ~LATB5; } while(0)
#define IO_RB5_GetValue()         RB5
#define IO_RB5_SetDigitalInput()    do { TRISB5 = 1; } while(0)
#define IO_RB5_SetDigitalOutput()   do { TRISB5 = 0; } while(0)

#define IO_RB5_SetPullup()    do { WPUB5 = 1; } while(0)
#define IO_RB5_ResetPullup()   do { WPUB5 = 0; } while(0)
#define IO_RB5_SetAnalogMode()   do { ANSB5 = 1; } while(0)
#define IO_RB5_SetDigitalMode()   do { ANSB5 = 0; } while(0)


// get/set IO_RC3 aliases
#define IO_RC3_TRIS               TRISC3
#define IO_RC3_LAT                LATC3
#define IO_RC3_PORT               RC3
#define IO_RC3_ANS                ANSC3
#define IO_RC3_SetHigh()    do { LATC3 = 1; } while(0)
#define IO_RC3_SetLow()   do { LATC3 = 0; } while(0)
#define IO_RC3_Toggle()   do { LATC3 = ~LATC3; } while(0)
#define IO_RC3_GetValue()         RC3
#define IO_RC3_SetDigitalInput()    do { TRISC3 = 1; } while(0)
#define IO_RC3_SetDigitalOutput()   do { TRISC3 = 0; } while(0)

#define IO_RC3_SetAnalogMode()   do { ANSC3 = 1; } while(0)
#define IO_RC3_SetDigitalMode()   do { ANSC3 = 0; } while(0)


// get/set IO_RC4 aliases
#define IO_RC4_TRIS               TRISC4
#define IO_RC4_LAT                LATC4
#define IO_RC4_PORT               RC4
#define IO_RC4_ANS                ANSC4
#define IO_RC4_SetHigh()    do { LATC4 = 1; } while(0)
#define IO_RC4_SetLow()   do { LATC4 = 0; } while(0)
#define IO_RC4_Toggle()   do { LATC4 = ~LATC4; } while(0)
#define IO_RC4_GetValue()         RC4
#define IO_RC4_SetDigitalInput()    do { TRISC4 = 1; } while(0)
#define IO_RC4_SetDigitalOutput()   do { TRISC4 = 0; } while(0)

#define IO_RC4_SetAnalogMode()   do { ANSC4 = 1; } while(0)
#define IO_RC4_SetDigitalMode()   do { ANSC4 = 0; } while(0)


// get/set IO_RC5 aliases
#define IO_RC5_TRIS               TRISC5
#define IO_RC5_LAT                LATC5
#define IO_RC5_PORT               RC5
#define IO_RC5_ANS                ANSC5
#define IO_RC5_SetHigh()    do { LATC5 = 1; } while(0)
#define IO_RC5_SetLow()   do { LATC5 = 0; } while(0)
#define IO_RC5_Toggle()   do { LATC5 = ~LATC5; } while(0)
#define IO_RC5_GetValue()         RC5
#define IO_RC5_SetDigitalInput()    do { TRISC5 = 1; } while(0)
#define IO_RC5_SetDigitalOutput()   do { TRISC5 = 0; } while(0)

#define IO_RC5_SetAnalogMode()   do { ANSC5 = 1; } while(0)
#define IO_RC5_SetDigitalMode()   do { ANSC5 = 0; } while(0)


// get/set IO_RC6 aliases
#define IO_RC6_TRIS               TRISC6
#define IO_RC6_LAT                LATC6
#define IO_RC6_PORT               RC6
#define IO_RC6_ANS                ANSC6
#define IO_RC6_SetHigh()    do { LATC6 = 1; } while(0)
#define IO_RC6_SetLow()   do { LATC6 = 0; } while(0)
#define IO_RC6_Toggle()   do { LATC6 = ~LATC6; } while(0)
#define IO_RC6_GetValue()         RC6
#define IO_RC6_SetDigitalInput()    do { TRISC6 = 1; } while(0)
#define IO_RC6_SetDigitalOutput()   do { TRISC6 = 0; } while(0)

#define IO_RC6_SetAnalogMode()   do { ANSC6 = 1; } while(0)
#define IO_RC6_SetDigitalMode()   do { ANSC6 = 0; } while(0)


// get/set IO_RC7 aliases
#define IO_RC7_TRIS               TRISC7
#define IO_RC7_LAT                LATC7
#define IO_RC7_PORT               RC7
#define IO_RC7_ANS                ANSC7
#define IO_RC7_SetHigh()    do { LATC7 = 1; } while(0)
#define IO_RC7_SetLow()   do { LATC7 = 0; } while(0)
#define IO_RC7_Toggle()   do { LATC7 = ~LATC7; } while(0)
#define IO_RC7_GetValue()         RC7
#define IO_RC7_SetDigitalInput()    do { TRISC7 = 1; } while(0)
#define IO_RC7_SetDigitalOutput()   do { TRISC7 = 0; } while(0)

#define IO_RC7_SetAnalogMode()   do { ANSC7 = 1; } while(0)
#define IO_RC7_SetDigitalMode()   do { ANSC7 = 0; } while(0)


// get/set IO_RC7 aliases
#define IO_RC7_TRIS               TRISC7
#define IO_RC7_LAT                LATC7
#define IO_RC7_PORT               RC7
#define IO_RC7_ANS                ANSC7
#define IO_RC7_SetHigh()    do { LATC7 = 1; } while(0)
#define IO_RC7_SetLow()   do { LATC7 = 0; } while(0)
#define IO_RC7_Toggle()   do { LATC7 = ~LATC7; } while(0)
#define IO_RC7_GetValue()         RC7
#define IO_RC7_SetDigitalInput()    do { TRISC7 = 1; } while(0)
#define IO_RC7_SetDigitalOutput()   do { TRISC7 = 0; } while(0)

#define IO_RC7_SetAnalogMode()   do { ANSC7 = 1; } while(0)
#define IO_RC7_SetDigitalMode()   do { ANSC7 = 0; } while(0)


// get/set IO_RD3 aliases
#define IO_RD3_TRIS               TRISD3
#define IO_RD3_LAT                LATD3
#define IO_RD3_PORT               RD3
#define IO_RD3_ANS                ANSD3
#define IO_RD3_SetHigh()    do { LATD3 = 1; } while(0)
#define IO_RD3_SetLow()   do { LATD3 = 0; } while(0)
#define IO_RD3_Toggle()   do { LATD3 = ~LATD3; } while(0)
#define IO_RD3_GetValue()         RD3
#define IO_RD3_SetDigitalInput()    do { TRISD3 = 1; } while(0)
#define IO_RD3_SetDigitalOutput()   do { TRISD3 = 0; } while(0)

#define IO_RD3_SetAnalogMode()   do { ANSD3 = 1; } while(0)
#define IO_RD3_SetDigitalMode()   do { ANSD3 = 0; } while(0)


// get/set IO_RD4 aliases
#define IO_RD4_TRIS               TRISD4
#define IO_RD4_LAT                LATD4
#define IO_RD4_PORT               RD4
#define IO_RD4_ANS                ANSD4
#define IO_RD4_SetHigh()    do { LATD4 = 1; } while(0)
#define IO_RD4_SetLow()   do { LATD4 = 0; } while(0)
#define IO_RD4_Toggle()   do { LATD4 = ~LATD4; } while(0)
#define IO_RD4_GetValue()         RD4
#define IO_RD4_SetDigitalInput()    do { TRISD4 = 1; } while(0)
#define IO_RD4_SetDigitalOutput()   do { TRISD4 = 0; } while(0)

#define IO_RD4_SetAnalogMode()   do { ANSD4 = 1; } while(0)
#define IO_RD4_SetDigitalMode()   do { ANSD4 = 0; } while(0)


// get/set IO_RD4 aliases
#define IO_RD4_TRIS               TRISD4
#define IO_RD4_LAT                LATD4
#define IO_RD4_PORT               RD4
#define IO_RD4_ANS                ANSD4
#define IO_RD4_SetHigh()    do { LATD4 = 1; } while(0)
#define IO_RD4_SetLow()   do { LATD4 = 0; } while(0)
#define IO_RD4_Toggle()   do { LATD4 = ~LATD4; } while(0)
#define IO_RD4_GetValue()         RD4
#define IO_RD4_SetDigitalInput()    do { TRISD4 = 1; } while(0)
#define IO_RD4_SetDigitalOutput()   do { TRISD4 = 0; } while(0)

#define IO_RD4_SetAnalogMode()   do { ANSD4 = 1; } while(0)
#define IO_RD4_SetDigitalMode()   do { ANSD4 = 0; } while(0)



/**
 * @Param
    none
 * @Returns
    none
 * @Description
    GPIO and peripheral I/O initialization
 * @Example
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