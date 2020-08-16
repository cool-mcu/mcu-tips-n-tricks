/*******************************************************************************
 * Copyright (c) 2020 dBm Signal Dynamics Inc.
 * 
 * File:        main.c
 * Project:     cpu-arith-benchmark
 * Compiler:    XC8 v2.20, XC16 v1.50, XC32 v2.41
 * Course:      MCU Tips n' Tricks
 * URL:         https://www.cool-mcu.com/courses/mcu-tips-n-tricks
 * Chapter:     Hardware Tips n Tricks
 * Lesson:      Selecting a Microcontroller
 * 
 * Simple project which uses the simulator stopwatch feature to display
 * cycle-counts for common arithmetic operations on PIC16F1, PIC24F, PIC32MX and
 * PIC32MZ-based MCUs
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

// Global Variables
int8_t a, b, c;
int16_t g;
int32_t d, e, f;
int64_t h;
float i, j, k;

// Local Function Prototypes
int8_t sum8(int8_t a, int8_t b);            // 8-bit integer addition
int32_t sum32(int32_t a, int32_t b);        // 32-bit integer addition
int16_t mul8(int8_t a, int8_t b);           // 8x8 integer multiplication
int64_t mul32(int32_t a, int32_t b);        // 32x32 integer multiplication

int main(void) {
  
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
