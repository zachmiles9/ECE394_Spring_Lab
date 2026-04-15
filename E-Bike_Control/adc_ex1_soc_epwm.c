//#############################################################################
//
// FILE:   adc_ex1_soc_epwm.c
//
// TITLE:  ADC ePWM Triggering
//
//! \addtogroup bitfield_example_list
//! <h1>ADC ePWM Triggering</h1>
//!
//! This example sets up ePWM1 to periodically trigger a conversion on ADCA.
//!
//! \b External \b Connections \n
//!  - A1 should be connected to a signal to convert
//!
//! \b Watch \b Variables \n
//! - \b adcAResults - A sequence of analog-to-digital conversion samples from
//!   pin A1. The time between samples is determined based on the period
//!   of the ePWM timer.
//!
//
//#############################################################################
//
//
// 
// C2000Ware v6.00.00.00
//
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "f28x_project.h"
#include <stdio.h>

//
// Defines
//
#define RESULTS_BUFFER_SIZE     256
#define num_bits 12
#define V_cc 3.3

//
// Globals
//
uint16_t adcAResults[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t index;                              // Index into result buffer
volatile uint16_t bufferFull;                // Flag to indicate buffer is full
float adc_out; // digital output of ADC (x_{A/D})
float V_ADC;


//
// Function Prototypes
//
void initADC(void);
void initEPWM(void);
void initADCSOC(void);
__interrupt void adcA1ISR(void);

//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    InitSysCtrl();

    //
    // Initialize GPIO
    //
    InitGpio();

    //
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    InitPieVectTable();

    //
    // Map ISR functions
    //
    EALLOW;
    PieVectTable.ADCA1_INT = &adcA1ISR;     // Function for ADCA interrupt 1
    EDIS;

    //
    // Configure the ADC and power it up
    //
    initADC();

    //
    // Configure the ePWM
    //
    initEPWM();

    //
    // Setup the ADC for ePWM triggered conversions on channel 1
    //
    initADCSOC();

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    IER |= M_INT1;  // Enable group 1 interrupts

    EINT;           // Enable Global interrupt INTM
    ERTM;           // Enable Global realtime interrupt DBGM

    //
    // Initialize results buffer
    //
    for(index = 0; index < RESULTS_BUFFER_SIZE; index++)
    {
        adcAResults[index] = 0;
    }

    index = 0;
    bufferFull = 0;

    //
    // Enable PIE interrupt
    //
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

    //
    // Sync ePWM
    //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    //
    // Take conversions indefinitely in loop
    //
    while(1)
    {
        
    }
}

//
// initADC - Function to configure and power up ADCA.
//
void initADC(void)
{
    //
    // Setup VREF as internal
    //
    SetVREF(ADC_ADCA, ADC_INTERNAL, ADC_VREF3P3);

    EALLOW;

    //
    // Set ADCCLK divider to /1
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 0b0000; // Option to divide original clock for the ADC clock. Divide by 1 for now

    //
    // Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    // Power up the ADC and then delay for 1 ms
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    EDIS;

    DELAY_US(1000);
}

//
// initEPWM - Function to configure ePWM1 to generate the SOC.
//
void initEPWM(void)
{
    EALLOW;

    // PWM 1: fs = 200 kHz, up-down count, D = 0.5
    // EPwm1Regs.TBPRD = 375; // Set Nr to 375, this is max value it counts to
    // EPwm1Regs.CMPA.bit.CMPA = 188; // Set compare A value to 188 counts (rounded up from 187.5)
    // EPwm1Regs.TBCTL.bit.CTRMODE = 0b10;    // 10 is up-down count mode; 01 is down count mode; 00 is up count mode 

    // // For the following registers: 
    // // - 00 means 'do nothing' to the PWM signal
    // // - 01 means force PWM signal to 'low'
    // // - 10 means force PWM signal to 'high'
    // // - 11 means 'toggle' the PWM signal
    // EPwm1Regs.AQCTLA.bit.CAD = 0b10; // set PWM to 'high' when: counter is going 'down' AND equal to the compare value
    // EPwm1Regs.AQCTLA.bit.CAU = 0b01; // set PWM to 'low' when: counter is going 'up' AND equal to the compare value
    // EPwm1Regs.AQCTLA.bit.ZRO = 0; // set PWM to 'do nothing' when counter is at zero (minimum)
    // EPwm1Regs.AQCTLA.bit.PRD = 0; // set PWM to 'do nothing' when counter is at Nr (peak)
    // EPwm1Regs.AQCTLB.bit.CAD = 0b01; // set PWM to 'low' when: counter is going 'down' AND equal to the compare value
    // EPwm1Regs.AQCTLB.bit.CAU = 0b10; // set PWM to 'high' when: counter is going 'up' AND equal to the compare value
    // EPwm1Regs.AQCTLB.bit.ZRO = 0; // set PWM to 'do nothing' when: counter is at zero (minimum)
    // EPwm1Regs.AQCTLB.bit.PRD = 0; // set PWM to 'do nothing' when: counter is at Nr (peak)

    // PWM 1 (with fs = 10kHz instead of 200kHz): fs = 10 kHz, up-down count, D = 0.5
    EPwm1Regs.TBPRD = 7500; // Set Nr to 7500, this is max value it counts to
    EPwm1Regs.CMPA.bit.CMPA = 3750; // Set compare A value to 3750 counts
    EPwm1Regs.TBCTL.bit.CTRMODE = 0b10;    // 10 is up-down count mode; 01 is down count mode; 00 is up count mode 

    // For the following registers: 
    // - 00 means 'do nothing' to the PWM signal
    // - 01 means force PWM signal to 'low'
    // - 10 means force PWM signal to 'high'
    // - 11 means 'toggle' the PWM signal
    EPwm1Regs.AQCTLA.bit.CAD = 0b10; // set PWM to 'high' when: counter is going 'down' AND equal to the compare value
    EPwm1Regs.AQCTLA.bit.CAU = 0b01; // set PWM to 'low' when: counter is going 'up' AND equal to the compare value
    EPwm1Regs.AQCTLA.bit.ZRO = 0; // set PWM to 'do nothing' when counter is at zero (minimum)
    EPwm1Regs.AQCTLA.bit.PRD = 0; // set PWM to 'do nothing' when counter is at Nr (peak)
    EPwm1Regs.AQCTLB.bit.CAD = 0b01; // set PWM to 'low' when: counter is going 'down' AND equal to the compare value
    EPwm1Regs.AQCTLB.bit.CAU = 0b10; // set PWM to 'high' when: counter is going 'up' AND equal to the compare value
    EPwm1Regs.AQCTLB.bit.ZRO = 0; // set PWM to 'do nothing' when: counter is at zero (minimum)
    EPwm1Regs.AQCTLB.bit.PRD = 0; // set PWM to 'do nothing' when: counter is at Nr (peak)

   
    // PWM 2: fs = 75 kHz, down count, D = 0.7
    // EPwm1Regs.TBPRD = 2000; // Set Nr to 2000, this is max value it counts to
    // EPwm1Regs.CMPA.bit.CMPA = 1400; // Set compare A value to 1400 counts
    // EPwm1Regs.TBCTL.bit.CTRMODE = 0b01; // 10 is up-down count mode; 01 is down count mode; 00 is up count mode 

    // // For the following registers: 
    // // - 00 means 'do nothing' to the PWM signal
    // // - 01 means force PWM signal to 'low'
    // // - 10 means force PWM signal to 'high'
    // // - 11 means 'toggle' the PWM signal
    // EPwm1Regs.AQCTLA.bit.CAD = 0b10; // set PWM to 'high' when: counter is going 'down' AND equal to the compare value
    // EPwm1Regs.AQCTLA.bit.CAU = 0; // set PWM to 'do nothing' when: counter is going 'up' AND equal to the compare value
    // EPwm1Regs.AQCTLA.bit.ZRO = 0; // set PWM to 'do nothing' when: counter is at zero (minimum)
    // EPwm1Regs.AQCTLA.bit.PRD = 0b01; // set PWM to 'low' when: counter is at Nr (peak)
    // EPwm1Regs.AQCTLB.bit.CAD = 0b01; // set PWM to 'low' when: counter is going 'down' AND equal to the compare value
    // EPwm1Regs.AQCTLB.bit.CAU = 0; // set PWM to 'do nothing' when: counter is going 'up' AND equal to the compare value
    // EPwm1Regs.AQCTLB.bit.ZRO = 0; // set PWM to 'do nothing' when: counter is at zero (minimum)
    // EPwm1Regs.AQCTLB.bit.PRD = 0b10; // set PWM to 'high' when: counter is at Nr (peak)
    

    // PWM 3: fs = 10 kHz, up count, D = 0.3
    // EPwm1Regs.TBPRD = 15000; // Set Nr to 15000, this is max value it counts to
    // EPwm1Regs.CMPA.bit.CMPA = 4500; // Set compare A value to 4500 counts
    // EPwm1Regs.TBCTL.bit.CTRMODE = 0b00; // 10 is up-down count mode; 01 is down count mode; 00 is up count mode 

    // // For the following registers: 
    // // - 00 means 'do nothing' to the PWM signal
    // // - 01 means force PWM signal to 'low'
    // // - 10 means force PWM signal to 'high'
    // // - 11 means 'toggle' the PWM signal
    // EPwm1Regs.AQCTLA.bit.CAD = 0; // set PWM to 'do nothing' when: counter is going 'down' AND equal to the compare value
    // EPwm1Regs.AQCTLA.bit.CAU = 0b01; // set PWM to 'low' when: counter is going 'up' AND equal to the compare value
    // EPwm1Regs.AQCTLA.bit.ZRO = 0b10; // set PWM to 'high' when: counter is at zero (minimum)
    // EPwm1Regs.AQCTLA.bit.PRD = 0; // set PWM to 'do nothing' when: counter is at Nr (peak)
    // EPwm1Regs.AQCTLB.bit.CAD = 0; // set PWM to 'do nothing' when: counter is going 'down' AND equal to the compare value
    // EPwm1Regs.AQCTLB.bit.CAU = 0b10; // set PWM to 'high' when: counter is going 'up' AND equal to the compare value
    // EPwm1Regs.AQCTLB.bit.ZRO = 0b01; // set PWM to 'low' when: counter is at zero (minimum)
    // EPwm1Regs.AQCTLB.bit.PRD = 0; // set PWM to 'do nothing' when: counter is at Nr (peak)
    
    // Lab 1 Week 2
    EPwm1Regs.ETSEL.bit.SOCAEN = 0b1; // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 0b011; // Select SOC on up-down count - enable tb counter when counter equal to 0 () or Nr (TBPRD)
    EPwm1Regs.ETPS.bit.SOCAPRD = 0b01; // Generate pulse on 1st event (using 2nd/3rd/etc would be to sample ADC slower)
    EPwm1Regs.ETPS.bit.INTPRD = 1; // Not using for now... works similarly to SOCAPRD (maybe can work faster!)

    EPwm1Regs.ETSOCPS.bit.SOCAPRD2 = 10; // Additional selection on top of SOCAPRD (which had only 2 bits)
    // 2 means ADC samples once every switch cycle
    // 10 means ADC samples once every 5 switch cycles
    // This is because SOCASEL is set to select SOC on up-down count (see comment further above)
    EPwm1Regs.ETPS.bit.SOCPSSEL = 1; // select SOCAPRD2 instead of SOCAPRD (which would be 0)
    // end Lab 1 Week 2

    // Optional stuff to edit CLK
    EPwm1Regs.TBCTL.bit.CLKDIV = 0b000; // how fast the counter increments (can divide the clock to slow it down)
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0b000; // high speed clock (not used)

    // Set the PWM signals to output pins
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0b01; // amux is the lower order bits, we want to set the lowest bit to 1 to use gpio0 for pwm1
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0b00; // gmux is the higher order bits, we want to clear all of these to only use the gpio0
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0b01; // amux is the lower order bits, we want to set the lowest bit to 1 to use gpio1 for pwm1
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0b00; // gmux is the higher order bits, we want to clear all of these to only use the gpio1

    // this is how to configure dead time correctly 
    EPwm1Regs.DBCTL.bit.IN_MODE =  0; // we want to access only PWM_A, using PWM_B is unnecessary
    EPwm1Regs.DBCTL.bit.POLSEL = 0b10; // we want to invert only one of the signals so that it mimics the original PWM_B
    EPwm1Regs.DBCTL.bit.OUT_MODE = 0b11; // we want to access the new signals which are both delayed via FED & RED; we don't want to access the original PWM_A or PWM_B

    EPwm1Regs.DBFED.bit.DBFED = 15; // Falling edge delay is set as 15 clock periods (which evaluates to 100ns of dead time)
    EPwm1Regs.DBRED.bit.DBRED = 15; // Rising edge delay is set as 15 clock periods (which evaluates to 100ns of dead time)

    EDIS;
}

//
// initADCSOC - Function to configure ADCA's SOC0 to be triggered by ePWM1.
//
void initADCSOC(void)
{
    //
    // Select the channels to convert and the end of conversion flag
    //
    EALLOW;

    // AdcaRegs.ADCCTL2.bit.PRESCALE = 0b0000; // Option to divide original clock for the ADC clock. Divide by 1 for now
    // AdcaRegs.ADCSOCxCTL.bit.CHSEL = ; //
    // AdcaRegs.ADCSOCxCTL.bit.ACQPS = ; //
    // AdcaRegs.ADCSOCxCTL.bit.TRIGSEL = ; //

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 1;     // SOC0 will convert pin A1
                                           // 0:A0  1:A1  2:A2  3:A3
                                           // 4:A4   5:A5   6:A6   7:A7
                                           // 8:A8   9:A9   A:A10  B:A11
                                           // C:A12  D:A13  E:A14  F:A15
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 14;    // Sample window is 15 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;   // Trigger on ePWM1 SOCA

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Make sure INT1 flag is cleared

    EDIS;
}

//
// adcA1ISR - ADC A Interrupt 1 ISR
//
__interrupt void adcA1ISR(void)
{
    //
    // Add the latest result to the buffer
    // ADCRESULT0 is the result register of SOC0
    adcAResults[index++] = AdcaResultRegs.ADCRESULT0;
    adc_out = AdcaResultRegs.ADCRESULT0;
    // V_ADC = (adc_out * V_cc)/(pow(2, num_bits) - 1);
    V_ADC = (adc_out * V_cc)/((1 << num_bits) - 1);

    //
    // Set the bufferFull flag if the buffer is full
    //
    if(RESULTS_BUFFER_SIZE <= index)
    {
        index = 0;
        bufferFull = 1;
    }

    //
    // Clear the interrupt flag
    //
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    //
    // Check if overflow has occurred
    //
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; //clear INT1 overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    }

    //
    // Acknowledge the interrupt
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// End of File
//
