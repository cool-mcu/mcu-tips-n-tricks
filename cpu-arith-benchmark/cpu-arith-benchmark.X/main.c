/*******************************************************************************
 * Copyright (c) 2020 dBm Signal Dynamics Inc.
 * 
 * File:        main.c
 * Project:     cpu-arith-benchmark
 * Compiler:    XC8 v2.20, XC16 v1.60, XC32 v2.41
 * Course:      MCU Tips n' Tricks
 * URL:         https://www.cool-mcu.com/courses/mcu-tips-n-tricks
 * Chapter:     Hardware Tips n Tricks
 * Lesson:      Selecting a Microcontroller
 * 
 * Project uses the simulator stopwatch feature to display
 * cycle-counts for common arithmetic operations on PIC16F1, PIC24F, dsPIC33E, 
 * PIC32MX and PIC32MZ-based MCUs
 * 
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
 * MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 * IN NO EVENT SHALL DBM SIGNAL DYNAMICS OR ITS LICENSORS BE LIABLE OR OBLIGATED
 * UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY,
 * OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
 * INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
 * SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
 * (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
#include <xc.h>
#include <stdint.h>
#include "configBits.h"                     // Configuration bit settings for hardware simulation

// Global Variables
int8_t a, b, c;
int16_t g;
int32_t d, e, f;
int64_t h;
float i, j, k;
double l, m, n;

// Local Function Prototypes
int8_t sum8(int8_t a, int8_t b);            // 8-bit integer addition
int32_t sum32(int32_t a, int32_t b);        // 32-bit integer addition
int16_t mul8(int8_t a, int8_t b);           // 8x8 integer multiplication
int64_t mul32(int32_t a, int32_t b);        // 32x32 integer multiplication
void SetPerformanceMode(void);              // Configure Clocks, Interrupts, Cache, Pre-fetch for desired performance

int main(void) {
  
  SetPerformanceMode();
  
  a = 2;
  b = 3;
  c = sum8(a, b);
  g = mul8(a, b);
  
  d = 4;
  e = 5;
  f = sum32(d, e);
  h = mul32(d, e);
  
  i = 6.0;
  j = -7.125;
  k = i*j;
  
  l = 6.0;
  m = -7.125;
  n = l*m;
  
  while(1);
  
}


int8_t sum8(int8_t a, int8_t b) {
    return (a + b);
}

int32_t sum32(int32_t a, int32_t b) {
  return (a + b);
}

int16_t mul8(int8_t a, int8_t b) {
    return (a * b);
}

int64_t mul32(int32_t a, int32_t b) {
  return (a * b);
}

void SetPerformanceMode(void) {
    
    // At this point, the CPU is operating using the default oscillator and
    // clocking options defined by the hardware configuration bits.
    
    // Add any code below to adjust clocks and any other configuration for
    // desired performance before running your main code loop.
    
#if defined(PIC32MZ2048EFM144_CURIOSITY2) || defined(PIC32MZ1024EFH064_MINI32_BB)
    
    // At this point, the Hardware Clock Initialization is Complete
	// The Internal Fast RC Oscillator (8 MHz) provides the clock source
	// Instruction Clock, PBCLK7 (SYSCLK) set to 8 MHz
    // Peripheral Bus Clocks divisors set to defaults (divide by 2))
    // Flash memory predictive prefetch disabled
    // Cache memory is enabled (per compiler default XC32 startup setting)
    // Interrupt controller is in single vector mode (per compiler default XC32 startup setting)
    // Interrupts are disabled (per compiler default XC32 startup setting)
    
    // Set the desired performance mode:
    // SYSCLK = 8 MHz, All PBCLKx = 8 MHz
    // Cache Disabled (per setting in "pic32_init_cache.S")
    // Pre-Fetch Disabled
    
    unsigned int cp0;
	
    // Unlock Sequence
    SYSKEY = 0xAA996655;
    SYSKEY = 0x556699AA;  

    // PB1DIV
                            // Peripheral Bus 1 cannot be turned off, so there's no need to turn it on
    PB1DIVbits.PBDIV = 0;   // Peripheral Bus 1 Clock Divisor Control (PBCLK1 is SYSCLK divided by 1)

    // PB2DIV
    PB2DIVbits.ON = 1;      // Peripheral Bus 2 Output Clock Enable (Output clock is enabled)
    PB2DIVbits.PBDIV = 0;   // Peripheral Bus 2 Clock Divisor Control (PBCLK2 is SYSCLK divided by 1)

    // PB3DIV
    PB3DIVbits.ON = 1;      // Peripheral Bus 2 Output Clock Enable (Output clock is enabled)
    PB3DIVbits.PBDIV = 0;   // Peripheral Bus 3 Clock Divisor Control (PBCLK3 is SYSCLK divided by 1)

    // PB4DIV
    PB4DIVbits.ON = 1;      // Peripheral Bus 4 Output Clock Enable (Output clock is enabled)
    while (!PB4DIVbits.PBDIVRDY); // Wait until it is ready to write to
    PB4DIVbits.PBDIV = 0;   // Peripheral Bus 4 Clock Divisor Control (PBCLK4 is SYSCLK divided by 1)

    // PB5DIV
    PB5DIVbits.ON = 1;      // Peripheral Bus 5 Output Clock Enable (Output clock is enabled)
    PB5DIVbits.PBDIV = 0;   // Peripheral Bus 5 Clock Divisor Control (PBCLK5 is SYSCLK divided by 1)

    // PB7DIV
    PB7DIVbits.ON = 1;      // Peripheral Bus 7 Output Clock Enable (Output clock is enabled)
    PB7DIVbits.PBDIV = 0;   // Peripheral Bus 7 Clock Divisor Control (PBCLK7 is SYSCLK divided by 1)

    // PB8DIV
    PB8DIVbits.ON = 1;      // Peripheral Bus 8 Output Clock Enable (Output clock is enabled)
    PB8DIVbits.PBDIV = 0;   // Peripheral Bus 8 Clock Divisor Control (PBCLK8 is SYSCLK divided by 1)

    // PRECON - Set up prefetch (60 MHz operation))
    //PRECONbits.PFMSECEN = 0; // Flash SEC Interrupt Enable (Do not generate an interrupt when the PFMSEC bit is set)
    PRECONbits.PREFEN = 0b00; // Predictive Prefetch Enable (Disable predictive prefetch)
    PRECONbits.PFMWS = 0b000; // PFM Access Time Defined in Terms of SYSCLK Wait States (Zero wait states @ 8 MHz)

    // Set up caching
    // See  https://microchipdeveloper.com/32bit:mz-cache-disable
    // Added "pic32_init_cache.S" to project from XC32 tool chain, and set __PIC32_CACHE_MODE to _CACHE_DISABLE

    // Lock Sequence
    SYSKEY = 0x33333333;
    
    
    
#endif
    
}
