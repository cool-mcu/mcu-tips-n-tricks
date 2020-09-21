/*******************************************************************************
 * Copyright (c) 2020 dBm Signal Dynamics Inc.
 * 
 * File:        main.c
 * Project:     mcu-irq-benchmark
 * Compiler:    XC8 v2.20, XC16 v1.60, XC32 v2.41
 * Hardware:    PIC16F19197 Basic Hookup (Schematic #14-00058A)
 *              PIC24FJ1024GA606 Basic Hookup (Schematic #14-00059A)
 *              PIC32MZ1024EFH064 Basic Hookup (Schematic #14-00060A)
 * 
 * Project used to measure the latency between interrupt assertion and the first
 * instruction executed in the ISR for all 3 MCUs. An Oscilloscope is required.
 * 
 * Project produces a 1 kHz, 50% duty-cycle PWM output signal (and IRQ),
 * The IRQ's ISR routine toggles a digital output pin (USER LED pin).
 * 
 * An Oscilloscope is used to measure IRQ latency (PWM 0-->1 to LED toggle).
 * 
 * The Disassembly listing must be consulted to subtract the #instruction cycles
 * used to toggle the USER LED. 
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

#if defined(PIC16F19197_BH)
#define ELAPSED_TICKS_ADJUSTMENT 9
#define NoOP() NOP()                        // Define macro for NOP assembly instruction
#define MAX_LOOP_COUNT  1
#elif defined(PIC24FJ1024GA606_BH)
#include <libpic30.h>                       // Added for printf() --> UART2 redirect support
#define ELAPSED_TICKS_ADJUSTMENT 10         // Timer Tick Adjustment for reading the timer register
#define NoOP() Nop()                        // Define macro for NOP assembly instruction
#define MAX_LOOP_COUNT  1
#elif defined(PIC32MZ1024EFH064_BH)
#include <cp0defs.h>                        // CP0 access macros for PIC32M devices
#define ELAPSED_TICKS_ADJUSTMENT 12         // Timer Tick Adjustment for reading the timer register
#define NoOP() Nop()                        // Define macro for NOP assembly instruction
#define MAX_LOOP_COUNT  4
#endif

// Global Variables

const char Project[] = "mcu-irq-benchmark";
const char Version[] = "00.01";
const char CompileDate[] = __DATE__;
const char CompileTime[] = __TIME__;


// Local Function Prototypes
void Initiallize(void);                     // 1-time System Initialization
void SetPerformanceMode(void);              // Configure Clocks, Interrupts, Cache, Pre-fetch for desired performance
void PinConfig(void);                       // Configure Programmable I/O pins
void UartConfig(void);                      // Configure UART for printf() statement
void PWMConfig(void);                       // Configure a PWM resource to produce a 1 kHz signal (and irq) on selected output pin

int main(void) {
    
    Initiallize();
    
    while(1);
  
}

void Initiallize(void){
    
    // Configure Clocks, Interrupts, Cache, Pre-fetch for desired performance
    SetPerformanceMode();       
    
    // Configure Programmable I/O Pins for all I/O in this application
    PinConfig();
    
    // Configure UART resource for printf() statement
    UartConfig();
    
    printf("Cool-MCU.com\r\n");
    printf("Project: %s\r\n", Project);
    printf("Version: %s\r\n", Version);
    printf("Build date: %s\r\n", CompileDate);
    printf("Build time: %s\r\n\r\n", CompileTime);

    // Configure a PWM resource to produce a 50% duty cycle, 1 kHz signal (and interrupt)
    PWMConfig();
    
    // Configure LED output

#if defined(PIC16F19197_BH)

    ANSELEbits.ANSE5 = 0;
    TRISEbits.TRISE5 = 0;
    LATEbits.LATE5 = 0;

#elif defined(PIC24FJ1024GA606_BH)
    
    TRISEbits.TRISE2 = 0;
    LATEbits.LATE2 = 0;
    
#endif      
    
    // Enable Interrupts Globally

#if defined(PIC16F19197_BH)
    
    INTCONbits.PEIE = 1;    // Enable all peripheral interrupt sources
    INTCONbits.GIE = 1;     // Enable interrupts globally    

#elif defined(PIC24FJ1024GA606_BH)
    
    INTCON2bits.GIE = 1;    // Enable interrupts globally
    
#endif    
    
    
}

void SetPerformanceMode(void) {
    
    // At this point, the CPU is operating using the default oscillator and
    // clocking options defined by the hardware configuration bits.
    
    // Add any code below to adjust clocks and any other configuration for
    // desired performance before running your main code loop.
    
#if defined(PIC32MZ1024EFH064_BH)
    
    // At this point, the Hardware Clock Initialization is Complete
	// The Primary External XTAL OSC Oscillator Circuit (8 MHz) provides the clock source
	// Instruction Clock, PBCLK7 (SYSCLK) set to 8 MHz
    // Peripheral Bus Clocks divisors set to defaults (divide by 2))
    // Flash memory predictive prefetch is disabled
    // Cache memory is enabled (per compiler default XC32 startup setting)
    // Interrupt controller is in single vector mode (per compiler default XC32 startup setting)
    // Interrupts are disabled (per compiler default XC32 startup setting)
    
    // Set the desired performance mode:
    // SYSCLK = 8 MHz, All PBCLKx = 8 MHz
    // Cache Enabled (per default setting in "pic32_init_cache.S" - _CACHE_WRITEBACK_WRITEALLOCATE)
    // Pre-Fetch Enabled
    
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
    
#elif defined(PIC24FJ1024GA606_BH)
    
    // At this point, the Hardware Clock Initialization is Complete
    // Primary OSC with 4X PLL provides 32 MHz Fosc (16 MIPs)
    
    // Need to set Fosc to 16 MHz (8 MIPs)
    
    CLKDIVbits.CPDIV = 0x01;    // Select DIV2 clock scaling (16 MHz, 8 MIPs)    
    
#endif
    
}

void PinConfig(void){
        
#if defined(PIC16F19197_BH)
    
    // Configure digital pins for UART function
    TRISCbits.TRISC7 = 0;       // TX1 --> RC7
    TRISCbits.TRISC6 = 1;       // RX1 <-- RC6
    
    // RE6 used as CCP1 PWM output signal
    // Make pin digital
    ANSELEbits.ANSE6 = 0;
    // Make pin digital output and initialize level
    TRISEbits.TRISE6 = 0;
    LATEbits.LATE6 = 0;
    
    // Initiallize PPS Pin Mapping for this Application
    // TX1 --> RC7 
    // RX1 <-- RC6
    // CCP1 --> RE6
    // 1. Unlock PPS registers
    //bit oldGIE = INTCONbits.GIE;
    //INTCONbits.GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0;
    //INTCONbits.GIE = oldGIE;
    // 2. Configure Output Functions
    // Assign TX1 output function to pin RC7
    RC7PPS = 0x0D;
    // Assign CCP1 output function to pin RE6
    RE6PPS = 0x09;
    // 3. Configure Input Functions
    // Assign RC6 pin to RX1 input function
    RX1PPS = 0x16;
    // 4 Lock the PPS registers
    //oldGIE = INTCONbits.GIE;
    //INTCONbits.GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;
    //INTCONbits.GIE = oldGIE;
    
#elif defined(PIC24FJ1024GA606_BH)
    
    // RD0 used as OC1 PWM output signal
    // Make pin digital
    // ANSELEbits.ANSE6 = 0;
    // Make pin digital output and initialize level
    TRISDbits.TRISD0 = 0;
    LATDbits.LATD0 = 0;
    
    // Set up PPS (I/O Pin-Mapping) for all I/O in this application
    // U2RX <-- RP10/RF4   (DEBUG PORT PC-TX pin)
    // U2TX --> RP17/RF5   (DEBUG PORT PC-RX pin)
    // OC1 --> RP11/RD0     1 kHz PWM output
    // 1. Unlock PPS registers
    __builtin_write_OSCCONL(OSCCON & 0xBF);
    // 2. Configure Output Functions
    // Assign U2TX output function to pin RP17
    RPOR8bits.RP17R = 5;
    // Assign OC1 output function to pin RP11
    RPOR5bits.RP11R = 13;
    // 3. Configure Input Functions
    // Assign pin RP10 to U2RX input function
    RPINR19bits.U2RXR = 10;
    // 4 Lock the PPS registers
    __builtin_write_OSCCONL(OSCCON | 0x40);    
    
#elif defined(PIC32MZ1024EFH064_BH)
    
    // Set up PPS (I/O Pin-Mapping) for all I/O in this application
    // U2RX <-- RPB15   (DEBUG PORT PC-TX pin)
    // U2TX --> RPB14   (DEBUG PORT PC-RX pin)
    
    // U2RX pin (RPB15) is an ADC input, so need to configure ANSEL register to
    // disable analog pin function
    ANSELBbits.ANSB15 = 0;
    
    // PPS unlock sequence
    SYSKEY = 0x0;         
    SYSKEY = 0xAA996655;         
    SYSKEY = 0x556699AA; 
    CFGCONbits.IOLOCK = 0;  // unlock PPS registers for writing
    
    // modify the PPS registers for the application (per table 11-2 in data sheet)
    U2RXRbits.U2RXR = 3;        // Map RPB15 to U2RX
    RPB14Rbits.RPB14R = 2;      // Map U2TX to RPB14
    
    // PPS re-lock sequence
    CFGCONbits.IOLOCK = 1;         
    SYSKEY = 0x0;    
    
#endif
}

void UartConfig(void){

#if defined(PIC16F19197_BH)
    
    // Initialize UART1 for use as the DEBUG PORT for printf() messages
    
    // Turn the UART off
    RC1STAbits.SPEN = 0;
    TX1STAbits.TXEN = 0;

    // Disable U1 Interrupts
    PIR3bits.TX1IF = 0;                                                         // Clear the Transmit Interrupt Flag
    PIE3bits.TX1IE = 0;                                                         // Disable Transmit Interrupts
    PIR3bits.RC1IF = 0;                                                         // Clear the Receive Interrupt Flag
    PIE3bits.RC1IE = 0;                                                         // Disable Receive Interrupts
    
    // Configure TX Channel
    TX1STAbits.SYNC = 0;                                                        // Async mode
    TX1STAbits.BRGH = 1;                                                        // High speed BRG
    
    // Configure RX Channel
    RC1STAbits.CREN = 1;                                                        // Enable continuous RX
    
    // Configure BRG (BRGH = 1, BRG = 1)
    BAUD1CONbits.BRG16 = 1;
    SP1BRGH = 0x00;
    SP1BRGL = 0x44;               // decimal 68 (115200 baud)
    
    // ...And turn the UART on
    RC1STAbits.SPEN = 1;
    TX1STAbits.TXEN = 1;    
    
#elif defined(PIC24FJ1024GA606_BH)
    
    // Initialize UART2 for use as the DEBUG PORT for printf() messages
    
    __C30_UART = 2;         // MACRO: Divert printf() output to UART 2 - see XC16 manual
    
    U2MODEbits.UARTEN = 0;  // Disable UART  
    U2STAbits.UTXEN = 0;
    U2MODEbits.BRGH = 1;    // Enable high rate baud clock
    U2BRG = 16;             // Baud Rate generator set to 115200 baud
    U2MODEbits.UARTEN = 1;
    U2STAbits.UTXEN = 1;    // Enable UART    
    
#elif defined(PIC32MZ1024EFH064_BH)
    
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

#if defined(PIC16F19197_BH)
    
// XC8 Stub required to redirect printf() statements to UART 1

void putch(char c) {
    while(!TX1STAbits.TRMT);   // wait while Tx buffer full
    TX1REG = c;
}

#endif

void PWMConfig(void){
    
#if defined(PIC16F19197_BH)
    
    T2CONbits.T2ON = 0;         // turn Timer2 off
    T2TMR = 0;                  // reset the count   
    T2CLKCONbits.CS = 1;        // Select Fcyc (8 MHz) as clock source
    T2CONbits.T2CKPS = 5;       // 1:32 prescale (250 kHz)
    T2PR = 250;                 // set the PWM period value for 1000 uS (1mS)
    
    CCP1CONbits.EN = 0;         // turn CCP1 off
    CCP1CONbits.MODE = 15;      // set CCP1 mode to PWM
    CCPR1H = 0x01;
    CCPR1L = 0xF4;              // set up 50% duty cycle on output (count = 500)
    CCP1CONbits.EN = 1;         // turn CCP1 on
    
    PIR4bits.TMR2IF = 0;
    PIE4bits.TMR2IE = 1;        // enable Timer2 interrupt on start of PWM cycle
    T2CONbits.T2ON = 1;         // turn Timer2 on

#elif defined(PIC24FJ1024GA606_BH)
 
    T2CONbits.TON = 0;          // turn Timer2 off
    TMR2 = 0x00;                // reset the count
    T2CONbits.TCS = 0;          // Select Fcyc (8 MHz) as clock source
    T2CONbits.TCKPS = 1;        // 1:8 prescale (1 MHz)
    PR2 = 1000;                 // set the PWM period value for 1000 uS (1mS)
    
    OC1CON1bits.OCM = 0;        // turn OC1 off
    OC1R = 500;                 // set up 50% duty cycle on output (count = 500)
    OC1CON1bits.OCTSEL = 0;     // select Timer 2 as the OC time base
    OC1CON1bits.OCM = 0b110;    // set OC1 mode to PWM (Edge PWM)
    
    IFS0bits.T2IF = 0;          // clear Timer 2 interrupt flag
    //IEC0bits.T2IE = 1;          // enable Timer 2 interrupts
    IFS0bits.OC1IF = 0;
    IEC0bits.OC1IE = 1;
    T2CONbits.TON = 1;          // start timer (starts PWMs)
    
#endif    
    
}

// 1 kHz ISR (generated on rising edge of PWM signal) to toggle USER LED

#if defined(PIC16F19197_BH)

void __interrupt() interruptHandler(void)
{
    LATEbits.LATE5 ^= 1;
    PIR4bits.TMR2IF = 0;
}

#elif defined(PIC24FJ1024GA606_BH)

void __attribute__((interrupt, no_auto_psv)) _OC1Interrupt(void)
{
    LATEbits.LATE2 ^= 1;
    IFS0bits.OC1IF = 0;
}

#endif




