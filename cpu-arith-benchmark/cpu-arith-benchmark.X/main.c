/*******************************************************************************
 * Copyright (c) 2020 dBm Signal Dynamics Inc.
 * 
 * File:        main.c
 * Project:     cpu-arith-benchmark
 * Compiler:    XC8 v2.20, XC16 v1.60, XC32 v2.41
 * 
 * Project uses MCU 16-bit timer resource to compute then print out
 * cycle-counts for several arithmetic functions/operations on PIC16F1, PIC24F,
 * and PIC32MZ-based MCUs. Nop() instructions are inserted to facilitate 
 * Hardware Debugger Stopwatch measurement where possible.
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
#include <stdio.h>
#include "configBits.h"     // Hardware Configuration bit settings

#if defined(PIC16LF15356_BB)
#define ELAPSED_TICKS_ADJUSTMENT 9
#define NoOP() NOP()                        // Define macro for NOP assembly instruction
#define MAX_LOOP_COUNT  1
#elif defined(PIC24FJ256GA702_BB)
#include <libpic30.h>                       // Added for printf() --> UART2 redirect support
#define ELAPSED_TICKS_ADJUSTMENT 10         // Timer Tick Adjustment for reading the timer register
#define NoOP() Nop()                        // Define macro for NOP assembly instruction
#define MAX_LOOP_COUNT  1
#elif defined(PIC32MZ1024EFH064_MINI32_BB)
#include <cp0defs.h>                        // CP0 access macros for PIC32M devices
#define ELAPSED_TICKS_ADJUSTMENT 12         // Timer Tick Adjustment for reading the timer register
#define NoOP() Nop()                        // Define macro for NOP assembly instruction
#define MAX_LOOP_COUNT  4
#endif

// Global Variables

const char Project[] = "cpu-arith-benchmark";
const char Version[] = "00.01";
const char CompileDate[] = __DATE__;
const char CompileTime[] = __TIME__;

int8_t a, b, c;
int16_t g;
int32_t d, e, f;
int64_t h;
float i, j, k;
double l, m, n;

typedef union{
    uint16_t Full;      // full 16-bit timer value
    struct{
        uint8_t Low;    // low 8-bit value
        uint8_t High;   // high 8-bit value      
    };
}elapsedTime;           // 16-bit Stopwatch Counter Type


elapsedTime elapsedTicks_sum8, elapsedTicks_sum32, elapsedTicks_mul8, elapsedTicks_mul32, elapsedTicks_fp32, elapsedTicks_fp64;


// Local Function Prototypes
int8_t sum8(int8_t a, int8_t b);            // 8-bit integer addition function
int32_t sum32(int32_t a, int32_t b);        // 32-bit integer addition function
int16_t mul8(int8_t a, int8_t b);           // 8x8 integer multiplication function
int64_t mul32(int32_t a, int32_t b);        // 32x32 integer multiplication function

void Initiallize(void);                     // 1-time System Initialization
void SetPerformanceMode(void);              // Configure Clocks, Interrupts, Cache, Pre-fetch for desired performance
void PinConfig(void);                       // Configure I/O pins
void UartConfig(void);                      // Configure UART for printf() statement
void StopwatchConfig(void);                 // Configure the 16-bit timer resource for the stopwatch function
void StopwatchRestart(void);                // Clear the 16-bit counter and restart.
uint16_t StopwatchRead(void);               // Stop the 16-bit counter and read the elapsed time

int main(void) {
    
    int loop_count;
    
    Initiallize();
  
    // run the loop multiple times for targets that have instruction/data caches
    for(loop_count=0;loop_count<=MAX_LOOP_COUNT;loop_count++){
        
        // Operands for sum8() and mul8()
        a = 2;
        b = 3;
    
        // sum8() benchmark
        StopwatchRestart();
        NoOP();              // NoOP() used with hardware debugger stopwatch function
        c = sum8(a, b);
        NoOP();              // NoOP() used with hardware debugger stopwatch function
        elapsedTicks_sum8.Full = StopwatchRead() - ELAPSED_TICKS_ADJUSTMENT;

        // mul8() benchmark
        StopwatchRestart();
        NoOP();
        g = mul8(a, b);
        NoOP();
        elapsedTicks_mul8.Full = StopwatchRead() - ELAPSED_TICKS_ADJUSTMENT;

        // Operands for sum32() and mul32()
        d = 4;
        e = 5;

        // sum32() benchmark
        StopwatchRestart();
        NoOP();
        f = sum32(d, e);
        NoOP();
        elapsedTicks_sum32.Full = StopwatchRead() - ELAPSED_TICKS_ADJUSTMENT;

        // mul32() benchmark
        StopwatchRestart();
        NoOP();
        h = mul32(d, e);
        NoOP();
        elapsedTicks_mul32.Full = StopwatchRead() - ELAPSED_TICKS_ADJUSTMENT;

        // Operands for single-precision multiply
        i = 6.0;
        j = -7.125;

        // Single precision (32-bit) floating point multiply benchmark
        StopwatchRestart();
        NoOP();
        k = i*j;
        NoOP();
        elapsedTicks_fp32.Full = StopwatchRead() - ELAPSED_TICKS_ADJUSTMENT;

        // Operands for double-precision multiply
        l = 6.8;
        m = -7.125;

        // Double precision (64-bit) floating point multiply benchmark 
        StopwatchRestart();
        NoOP();
        n = l*m;
        NoOP();
        elapsedTicks_fp64.Full = StopwatchRead() - ELAPSED_TICKS_ADJUSTMENT;
    }
    
    // print the results
#if defined(PIC16LF15356_BB)
    printf("Configuration: PIC16LF15356 on a Breadboard @ 8 MIPs\r\n");
#elif defined(PIC24FJ256GA702_BB)
    printf("Configuration: PIC24FJ256GA702 on a Breadboard @ 8 MIPs\r\n");
#elif defined(PIC32MZ1024EFH064_MINI32_BB)
    printf("Configuration: PIC32MZ1024EFH064_MINI32 on a Breadboard @ 8 MIPs\r\n");
#endif
    printf("sum8():  %d cycles\r\n", elapsedTicks_sum8.Full);
    printf("sum32(): %d cycles\r\n", elapsedTicks_sum32.Full);
    printf("mul8():  %d cycles\r\n", elapsedTicks_mul8.Full);
    printf("mul32(): %d cycles\r\n", elapsedTicks_mul32.Full);
    printf("float32mul: %d cycles\r\n", elapsedTicks_fp32.Full);
    printf("float64mul: %d cycles\r\n\r\n", elapsedTicks_fp64.Full);
    
    while(1);
  
}

void Initiallize(void){
    
    // Configure Clocks, Interrupts, Cache, Pre-fetch for desired performance
    SetPerformanceMode();       
    
    // Configure Pins for all I/O in this application
    PinConfig();
    
    // Configure UART resource for printf() statement
    UartConfig();
    
    printf("Cool-MCU.com\r\n");
    printf("Project: %s\r\n", Project);
    printf("Version: %s\r\n", Version);
    printf("Build date: %s\r\n", CompileDate);
    printf("Build time: %s\r\n\r\n", CompileTime);
    
    // Configure/Start timer resource for benchmark time measurement
    StopwatchConfig();          
    
}

void SetPerformanceMode(void) {
    
    // At this point, the CPU is operating using the default oscillator and
    // clocking options defined by the hardware configuration bits.
    
    // Add any code below to adjust clocks and any other configuration for
    // desired performance before running your main code loop.
    
#if defined(PIC32MZ1024EFH064_MINI32_BB)
    
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
    asm volatile("di");     // disable all interrupts
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
    PRECONbits.PREFEN = 0b11; // Predictive Prefetch Enable (Enable predictive prefetch for any address)
    PRECONbits.PFMWS = 0b000; // PFM Access Time Defined in Terms of SYSCLK Wait States (Zero wait states @ 8 MHz)

    // Set up caching
    // See  https://microchipdeveloper.com/32bit:mz-cache-disable
    // Added "pic32_init_cache.S" to project from XC32 tool chain, and set __PIC32_CACHE_MODE to _CACHE_WRITEBACK_WRITEALLOCATE

    // Lock Sequence
    SYSKEY = 0x33333333;
    asm volatile("ei");     // Enable all interrupts
    
#elif defined(PIC24FJ256GA702_BB)
    
    // At this point, the Hardware Clock Initialization is Complete
    // Primary OSC with 4X PLL provides 32 MHz Fosc (16 MIPs)
    
    // Need to set Fosc to 16 MHz (8 MIPs)
    
    CLKDIVbits.CPDIV = 0x01;    // Select DIV2 clock scaling (16 MHz, 8 MIPs)
    
#endif
    
}

void PinConfig(void){
    
#if defined(PIC16LF15356_BB)
    
    // Disable Analog function on PIC16 pins used for this function
    ANSELCbits.ANSC7 = 0;
    ANSELCbits.ANSC6 = 0;
    
    // Configure digital pins
    TRISCbits.TRISC7 = 1;
    TRISCbits.TRISC6 = 0;
    
    // Initiallize PPS Pin Mapping for this Application
    // TX2 --> RC6 
    // RX2 <-- RC7 
    // 1. Unlock PPS registers
    //bit oldGIE = INTCONbits.GIE;
    //INTCONbits.GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0;
    //INTCONbits.GIE = oldGIE;
    // 2. Configure Output Functions
    // Assign TX2 output function to pin RC6
    RC6PPS = 0x11;
    // 3. Configure Input Functions
    // Assign RC7 pin to RX2 input function
    RX2DTPPS = 0x17;
    // 4 Lock the PPS registers
    //oldGIE = INTCONbits.GIE;
    //INTCONbits.GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;
    //INTCONbits.GIE = oldGIE;
    
#elif defined(PIC24FJ256GA702_BB)
    
    // Set up PPS (I/O Pin-Mapping) for all I/O in this application
    // U2RX <-- RP11   (DEBUG PORT PC-TX pin)
    // U2TX --> RP10   (DEBUG PORT PC-RX pin)
    // 1. Unlock PPS registers
    __builtin_write_OSCCONL(OSCCON & 0xBF);
    // 2. Configure Output Functions
    // Assign U2TX output function to pin RP10
    RPOR5bits.RP10R = 5;
    // 3. Configure Input Functions
    // Assign pin RP11 to U2RX input function
    RPINR19bits.U2RXR = 11;
    // 4 Lock the PPS registers
    __builtin_write_OSCCONL(OSCCON | 0x40);
    
#elif defined(PIC32MZ1024EFH064_MINI32_BB)
    
    // Set up PPS (I/O Pin-Mapping) for all I/O in this application
    // U2RX <-- RPD4   (DEBUG PORT PC-TX pin)
    // U2TX --> RPD5   (DEBUG PORT PC-RX pin)
    
    // U2RX pin (RPD4) is not an ADC input, so no need to configure ANSEL register
    
    // PPS unlock sequence
    SYSKEY = 0x0;         
    SYSKEY = 0xAA996655;         
    SYSKEY = 0x556699AA; 
    CFGCONbits.IOLOCK = 0;  // unlock PPS registers for writing
    
    // modify the PPS registers for the application (per table 11-2 in data sheet)
    U2RXRbits.U2RXR = 4;        // Map RPD04 to U2RX
    RPD5Rbits.RPD5R = 2;        // Map U2TX to RPD05 
    
    // PPS re-lock sequence
    CFGCONbits.IOLOCK = 1;         
    SYSKEY = 0x0;
    
#endif
}

void UartConfig(void){
    
#if defined(PIC16LF15356_BB)
    
    // Initialize UART2 for use as the DEBUG PORT for printf() messages
    
    // Turn the UART off
    RC2STAbits.SPEN = 0;
    TX2STAbits.TXEN = 0;

    // Disable U2 Interrupts
    PIR3bits.TX2IF = 0;                                                         // Clear the Transmit Interrupt Flag
    PIE3bits.TX2IE = 0;                                                         // Disable Transmit Interrupts
    PIR3bits.RC2IF = 0;                                                         // Clear the Receive Interrupt Flag
    PIE3bits.RC2IE = 0;                                                         // Disable Receive Interrupts
    
    // Configure TX Channel
    TX2STAbits.SYNC = 0;                                                        // Async mode
    TX2STAbits.BRGH = 1;                                                        // High speed BRG
    
    // Configure RX Channel
    RC2STAbits.CREN = 1;                                                        // Enable continuous RX
    
    // Configure BRG (BRGH = 1, BRG = 1)
    BAUD2CONbits.BRG16 = 1;
    SP2BRGH = 0x00;
    SP2BRGL = 0x44;               // decimal 68 (115200 baud)
    
    // ...And turn the UART on
    RC2STAbits.SPEN = 1;
    TX2STAbits.TXEN = 1;
    
#elif defined(PIC24FJ256GA702_BB)
    
    // Initialize UART2 for use as the DEBUG PORT for printf() messages
    
    __C30_UART = 2;         // MACRO: Divert printf() output to UART 2 - see XC16 manual
    
    U2MODEbits.UARTEN = 0;  // Disable UART  
    U2STAbits.UTXEN = 0;
    U2MODEbits.BRGH = 1;    // Enable high rate baud clock
    U2BRG = 16;             // Baud Rate generator set to 115200 baud
    U2MODEbits.UARTEN = 1;
    U2STAbits.UTXEN = 1;    // Enable UART
    
#elif defined(PIC32MZ1024EFH064_MINI32_BB)
    
    // Initialize UART2 for use as the DEBUG PORT for printf() messages
    
    // Note: No special initialization is required for printf()
    // printf() uses UART 2 for output by default as long as UART 2 is initialized before use
    
    U2MODEbits.UARTEN = 0;  // Disable UART  
    U2STAbits.UTXEN = 0;
    U2MODEbits.BRGH = 1;    // Enable high rate baud clock
    U2BRG = 16;             // Baud Rate generator set to 115200 baud
    U2MODEbits.UARTEN = 1;
    U2STAbits.UTXEN = 1;    // Enable UART
    
#endif
    
}

#if defined(PIC16LF15356_BB)
    
// XC8 Stub required to redirect printf() statements to UART 2

void putch(char c) {
    while(!TX2STAbits.TRMT);   // wait while Tx buffer full
    TX2REG = c;
}

#endif

void StopwatchConfig(void){
    
    // The MCU should ideally have a 16-bit timer resource available that
    // can be clocked at the CPU's frequency, and whose count value can be read
    // atomically.
    
#if defined(PIC16LF15356_BB)
    
    // Use Timer 1 (clocked at Fosc/4 = 8 MIPs)
    
    T1CONbits.ON = 0;       // Disable Timer
    T1CLKbits.CS = 1;       // CLK source = Fosc/4 = Fcyc
    
#elif defined(PIC24FJ256GA702_BB)
    
    // Use Timer 2 (clocked at PBCLK, which is set to CPUCLK via Initialization)
    
    T2CONbits.TON = 0;      // Disable Timer
    T2CONbits.TCS = 0;      // Select internal PBCLK as clock source
    T2CONbits.TGATE = 0;    // Disable gated timer mode
    T2CONbits.TCKPS = 0;    // Select 1:1 CLK prescale
    
#elif defined(PIC32MZ1024EFH064_MINI32_BB)
    
    // Use Timer 2 (clocked at PBCLK, which is set to CPUCLK via Initialization)
    
    T2CONbits.TON = 0;      // Disable Timer
    T2CONbits.TCS = 0;      // Select internal PBCLK as clock source
    T2CONbits.TGATE = 0;    // Disable gated timer mode
    T2CONbits.TCKPS = 0;    // Select 1:1 CLK prescale
    
#endif
    
}

void StopwatchRestart(void){
    
#if defined(PIC16LF15356_BB)
    
    TMR1H = 0;
    TMR1L = 0;
    T1CONbits.ON = 1;
    
#elif defined(PIC24FJ256GA702_BB)
    
    T2CONbits.TON = 0;
    TMR2 = 0;
    T2CONbits.TON = 1;
    
#elif defined(PIC32MZ1024EFH064_MINI32_BB)

    T2CONbits.ON = 0;
    TMR2 = 0;
    T2CONbits.ON = 1;    
    
#endif
}

uint16_t StopwatchRead(void){
    
    elapsedTime tmp;
    
#if defined(PIC16LF15356_BB)
    
    T1CONbits.ON = 0;
    tmp.Low = TMR1L;
    tmp.High = TMR1H;
    return tmp.Full;
    
#elif defined(PIC24FJ256GA702_BB)
    
    tmp.Full = TMR2;
    T2CONbits.TON = 0;
    return tmp.Full;
    
#elif defined(PIC32MZ1024EFH064_MINI32_BB)

    tmp.Full = TMR2;
    T2CONbits.ON = 0;
    return tmp.Full;    
    
#endif
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
