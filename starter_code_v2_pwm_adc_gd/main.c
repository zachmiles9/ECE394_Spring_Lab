//#############################################################################
//
// ECE 394 Control of Power Electronics
// Starter Code v2: PWM & ADC & GD Disable
//
// Coming soon: PI, DQ, and Hall Sensing
// 
// ============================================================================
// ADD THESE EXPRESSIONS TO YOUR WATCH WINDOW
// ============================================================================
//
//   boost_pwm    : Control - Toggles the Boost converter PWM, LED4, and Gate Driver Disable Pin (0 = PWM Off, 1 = PWM On)
//   inv_pwm      : Control - Toggles the 3-Phase Inverter PWM (0 = PWM Off, 1 = PWM On)
//   pwm7_duty    : Input   - Boost duty cycle [0.0 to 1.0]
//   pwm8_duty    : Input   - Inverter Phase A duty cycle [0.0 to 1.0]
//   pwm1_duty    : Input   - Inverter Phase B duty cycle [0.0 to 1.0]
//   pwm3_duty    : Input   - Inverter Phase C duty cycle [0.0 to 1.0]
//   Vin, Vout    : Monitor - ADC readings for DC input/output
//   Ia, Ib, Ic   : Monitor - ADC readings for motor 3-phase currents
//   IL           : Monitor - ADC reading for boost inductor current
//   throttle     : Monitor - Live normalized ADC reading of the throttle input [0.0 to 1.0]
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
//#############################################################################

//
// Included Files
//
#include "f28x_project.h"
#include "hall_estimator.h"
#include "dq_controller.h"

//
// Defines
//
#define SYSTEM_CLK_FREQ 150e6           // Hz
#define ADC_FS 3.3f                     // V
#define ADC_RES (1/(4095.0f))
#define ADC_CONV_COEFF (ADC_FS*ADC_RES)
// Boost PWM
#define PWM7_FREQ 375e3                 // Hz
#define PWM7_INIT_DUTY 0.50             // range: [0, 1]
// Inverter Phase A PWM
#define PWM8_FREQ 15e3                  // Hz
#define PWM8_INIT_DUTY 0.50             // range: [0, 1]
// Inverter Phase B PWM
#define PWM1_FREQ 15e3                  // Hz
#define PWM1_INIT_DUTY 0.50             // range: [0, 1]
// Inverter Phase C PWM
#define PWM3_FREQ 15e3                  // Hz
#define PWM3_INIT_DUTY 0.50             // range: [0, 1]
// Dead Time
#define DEAD_TIME_RED 10                // rising edge delay (clock cycles)
#define DEAD_TIME_FED 10                // falling edge delay (clock cycles)
// Control Interrupt
#define CONTROL_DIVIDER 5               // number of boost switching cycles in a control cycle
#define Tctrl ((float)CONTROL_DIVIDER/PWM7_FREQ)
// Scaling for ADC values
#define VIN_DIV_RHIGH 8.06e3f           // ohms
#define VIN_DIV_RLOW 1e3f               // ohms
#define VOUT_DIV_RHIGH 18e3f            // ohms
#define VOUT_DIV_RLOW 1e3f              // ohms
#define ISENSE_G_INV (1/(0.055f))       // A/V
#define ISENSE_OFFSET 1.65f             // V

//
// Globals
//

volatile uint16_t boost_pwm = 0; // 0 = Boost Off, 1 = Boost Switching Active
volatile uint16_t inv_pwm = 0; // 0 = Inverter Off, 1 = Inverter Switching Active

uint16_t HallA;
uint16_t HallB;
uint16_t HallC;

float pwm8_duty = PWM8_INIT_DUTY;
float pwm1_duty = PWM1_INIT_DUTY;
float pwm3_duty = PWM3_INIT_DUTY;
float pwm7_duty = PWM7_INIT_DUTY;

float Vin;
float Vout;
float IL;
float Ia;
float Ib;
float Ic;
float throttle;

float dacVoltage = 0.0f;
uint16_t dacBits;

//
// Function Prototypes
//
void initADC(void);
void initEPWM(void);
void initADCSOC(void);
void routeGPIOs(void);
void initDAC(void);
__interrupt void adcA1ISR(void);
__interrupt void adcB1ISR(void);

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
    PieVectTable.ADCB1_INT = &adcB1ISR;     // Function for ADCB interrupt 1
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
    // Setup the ADC for ePWM triggered conversions
    //
    initADCSOC();

    //
    // Configure GPIO pins to appropriate signals
    //
    routeGPIOs();

    initDAC();

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    IER |= M_INT1;  // Enable group 1 interrupts

    EINT;           // Enable Global interrupt INTM
    ERTM;           // Enable Global realtime interrupt DBGM

    //
    // Enable PIE interrupt
    //
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;

    //
    // Sync ePWM
    //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    while(1)
    {
        
    }
}

//
// initADC - Function to configure and power up ADCA and ADCB.
//
void initADC(void)
{
    //
    // Setup VREF as internal
    //
    SetVREF(ADC_ADCA, ADC_INTERNAL, ADC_VREF3P3);
    SetVREF(ADC_ADCB, ADC_INTERNAL, ADC_VREF3P3);

    EALLOW;

    //
    // Set ADCCLK divider to /4
    //
    AdcaRegs.ADCCTL2.bit.PRESCALE = 0;
    AdcbRegs.ADCCTL2.bit.PRESCALE = 0;

    //
    // Set pulse positions to late
    //
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    //
    // Power up the ADC and then delay for 1 ms
    //
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    EDIS;

    DELAY_US(1000);
}

//
// initEPWM - Function to configure PWMs
//
void initEPWM(void)
{
    EALLOW;
    // ==========================================================
    // Boost PWM Configuration
    // ==========================================================
    // Time-Base Setup
    EPwm7Regs.TBCTL.bit.CLKDIV = TB_CLOCK_DIV1;      
    EPwm7Regs.TBCTL.bit.HSPCLKDIV = TB_HSDIV1;
    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    // Action Qualifiers
    EPwm7Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm7Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm7Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    // Frequency and Duty
    EPwm7Regs.TBPRD = SYSTEM_CLK_FREQ/(2*PWM7_FREQ);
    EPwm7Regs.CMPA.bit.CMPA = PWM7_INIT_DUTY*(SYSTEM_CLK_FREQ/(2*PWM7_FREQ));
    // Sync to PWM8
    EPwm7Regs.TBCTL.bit.PHSEN = TB_ENABLE;      // Enable phase loading
    EPwm7Regs.TBPHS.bit.TBPHS = 0;              // Phase offset = 0 for perfect alignment
    EPwm7Regs.TBCTL.bit.PHSDIR = TB_UP;         // Count up after sync
    EPwm7Regs.EPWMSYNCINSEL.bit.SEL = 8;        // Sync to PWM8
    EPwm7Regs.EPWMSYNCOUTEN.all = 0x0;          // Clear defaults

    // ==========================================================
    // Inverter Phase A PWM Configuration
    // ==========================================================
    // Time-Base Setup
    EPwm8Regs.TBCTL.bit.CLKDIV = TB_CLOCK_DIV1;      
    EPwm8Regs.TBCTL.bit.HSPCLKDIV = TB_HSDIV1;
    EPwm8Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    // Action Qualifiers
    EPwm8Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm8Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm8Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm8Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    // Frequency and Duty
    EPwm8Regs.TBPRD = SYSTEM_CLK_FREQ/(2*PWM8_FREQ);
    EPwm8Regs.CMPA.bit.CMPA = PWM8_INIT_DUTY*(SYSTEM_CLK_FREQ/(2*PWM8_FREQ));
    // Dead Time
    EPwm8Regs.DBCTL.bit.IN_MODE = DBA_ALL;      
    EPwm8Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  
    EPwm8Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;   
    EPwm8Regs.DBRED.bit.DBRED = DEAD_TIME_RED;
    EPwm8Regs.DBFED.bit.DBFED = DEAD_TIME_FED;
    // Choose self as source of syncing signal
    EPwm8Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm8Regs.EPWMSYNCOUTEN.all = 0x0;       // Clear any default routing
    EPwm8Regs.EPWMSYNCOUTEN.bit.ZEROEN = 1;  // Send Sync pulse when counter = 0

    // Need TBPRD from phase A to introduce phase shift for phase B & C
    // "EPwm8Regs.TBPRD"

    // ==========================================================
    // Inverter Phase B PWM Configuration
    // ==========================================================
    // Time-Base Setup
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_CLOCK_DIV1;      
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_HSDIV1;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    // Action Qualifiers
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm1Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    // Frequency and Duty
    EPwm1Regs.TBPRD = SYSTEM_CLK_FREQ/(2*PWM1_FREQ);
    EPwm1Regs.CMPA.bit.CMPA = PWM1_INIT_DUTY*(SYSTEM_CLK_FREQ/(2*PWM1_FREQ));
    // Dead Time
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;      
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;   
    EPwm1Regs.DBRED.bit.DBRED = DEAD_TIME_RED;
    EPwm1Regs.DBFED.bit.DBFED = DEAD_TIME_FED;
    EPwm1Regs.DBCTL.bit.OUTSWAP = 0b11; // IMPORTANT: flips PWMA and PWMB
    // Sync to PWM8
    EPwm1Regs.TBCTL.bit.PHSEN = TB_ENABLE;      // Enable phase loading
    EPwm1Regs.TBPHS.bit.TBPHS = 2.0*EPwm8Regs.TBPRD*1.0/3.0; // Phase offset = 120 deg for B-phase offset
    EPwm1Regs.TBCTL.bit.PHSDIR = TB_DOWN;         // Count up after sync
    EPwm1Regs.EPWMSYNCINSEL.bit.SEL = 8;        // Sync to PWM8
    EPwm1Regs.EPWMSYNCOUTEN.all = 0x0;          // Clear defaults

    // implement TBPHS

    // ==========================================================
    // Inverter Phase C PWM Configuration
    // ==========================================================
    // Time-Base Setup
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_CLOCK_DIV1;      
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_HSDIV1;
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    // Action Qualifiers
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_NO_ACTION;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm3Regs.AQCTLA.bit.PRD = AQ_NO_ACTION;
    // Frequency and Duty
    EPwm3Regs.TBPRD = SYSTEM_CLK_FREQ/(2*PWM3_FREQ);
    EPwm3Regs.CMPA.bit.CMPA = PWM3_INIT_DUTY*(SYSTEM_CLK_FREQ/(2*PWM3_FREQ));
    // Dead Time
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;      
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;   
    EPwm3Regs.DBRED.bit.DBRED = DEAD_TIME_RED;
    EPwm3Regs.DBFED.bit.DBFED = DEAD_TIME_FED;
    // Sync to PWM8
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;      // Enable phase loading
    EPwm3Regs.TBPHS.bit.TBPHS = 2.0*EPwm8Regs.TBPRD*1.0/3.0; // Phase offset = 240 deg for C-phase offset
    EPwm3Regs.TBCTL.bit.PHSDIR = TB_UP;         // Count up after sync
    EPwm3Regs.EPWMSYNCINSEL.bit.SEL = 8;        // Sync to PWM8
    EPwm3Regs.EPWMSYNCOUTEN.all = 0x0;          // Clear defaults

    // ==========================================================
    // Disable Switching
    // ==========================================================
    
    EPwm7Regs.AQCSFRC.all = 0x05; // Forces outputs low
    EPwm8Regs.AQCSFRC.all = 0x05; // Forces outputs low
    EPwm1Regs.AQCSFRC.all = 0x05;
    EPwm3Regs.AQCSFRC.all = 0x05;
    GpioDataRegs.GPASET.bit.GPIO20 = 1; // LED4 OFF

    EDIS;
}

//
// initADCSOC - Function to configure ADCA's SOCs to be triggered by ePWM7.
//
void initADCSOC(void)
{
    EALLOW;
    // ==========================================================
    // Boost ADC Setup
    // ==========================================================

    // Boost SOC Trigger
    EPwm7Regs.ETSEL.bit.SOCAEN = 1;		        // enable SOC pulse generation
    EPwm7Regs.ETSEL.bit.SOCASEL = 2;	        // SOC when TBPRD is zero (1) or peak (2)
    EPwm7Regs.ETPS.bit.SOCPSSEL = 1;            // Use SOCAPRD2 for 
    EPwm7Regs.ETSOCPS.bit.SOCAPRD2 = CONTROL_DIVIDER;
    EPwm7Regs.ETCNTINITCTL.bit.SOCAINITEN = 1;  // Allow sync signal to reset event counter

    // SOC0: Vout
	AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0;			// convert ADCINB0
	AdcbRegs.ADCSOC0CTL.bit.ACQPS = 9;			// number of clock cycles - 1
	AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0x11;		// trigger on ePWM7 SOCA
    AnalogSubsysRegs.AGPIOCTRLH.bit.GPIO253 = 1; 
    GpioCtrlRegs.GPHAMSEL.bit.GPIO253 = 1;

    // SOC1: Vin
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 12;			// convert ADCINA12
	AdcaRegs.ADCSOC1CTL.bit.ACQPS = 9;			// number of clock cycles - 1
	AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0x11;		// trigger on ePWM7 SOCA

    // SOC2: IL
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 5;			// convert ADCINB5
	AdcbRegs.ADCSOC2CTL.bit.ACQPS = 9;			// number of clock cycles - 1
	AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 0x11;		// trigger on ePWM7 SOCA

    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 2; 		// End of SOC2 will set INT1 flag
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   		// Enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; 		// Make sure INT1 flag is cleared

    // ==========================================================
    // Inverter ADC Setup
    // ==========================================================

    // Inverter SOC Trigger
    EPwm8Regs.ETSEL.bit.SOCAEN = 1;		// enable SOC pulse generation
    EPwm8Regs.ETSEL.bit.SOCASEL = 1;	// SOC when TBPRD is zero (1) or peak (2)
    EPwm8Regs.ETPS.bit.SOCAPRD = 1;     // Trigger on first event

    // SOC3: Ia
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 3;			// convert ADCINB3
	AdcbRegs.ADCSOC3CTL.bit.ACQPS = 9;			// number of clock cycles - 1
	AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 0x13;		// trigger on ePWM8 SOCA
    AnalogSubsysRegs.AGPIOCTRLH.bit.GPIO242 = 1; 
    GpioCtrlRegs.GPHAMSEL.bit.GPIO242 = 1;

    // SOC4: Ic
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 2;			// convert ADCINA2
	AdcaRegs.ADCSOC4CTL.bit.ACQPS = 9;			// number of clock cycles - 1
	AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 0x13;		// trigger on ePWM8 SOCA
    AnalogSubsysRegs.AGPIOCTRLH.bit.GPIO224 = 1; 
    GpioCtrlRegs.GPHAMSEL.bit.GPIO224 = 1;

    // SOC5: Vthrottle
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 10;			// convert ADCINA10
	AdcaRegs.ADCSOC5CTL.bit.ACQPS = 9;			// number of clock cycles - 1
	AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 0x13;		// trigger on ePWM8 SOCA
    AnalogSubsysRegs.AGPIOCTRLH.bit.GPIO230 = 1; 
    GpioCtrlRegs.GPHAMSEL.bit.GPIO230 = 1;

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 5; 		// End of SOC5 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   		// Enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; 		// Make sure INT1 flag is cleared

    /* NOTE
    If any of the ADC channels are an "AGPIO" pin, must add the following code:
        AnalogSubsysRegs.AGPIOCTRLH.bit.GPIO227 = 1;
        GpioCtrlRegs.GPHAMSEL.bit.GPIO227 = 1;
    
    This includes the following ADC channels: 
        A2 (AGPIO224), A6 (AGPIO228), A9 (AGPIO227), A10 (AGPIO230), 
        A15 (AGPIO233), B0 (AGPIO253), B2 (AGPIO226), B3 (AGPIO242), 
        B4 (AGPIO236), C14 (AGPIO247) 
    */
    EDIS;
}

//
// routeGPIOs - Function to configure GPIO pins to appropriately send and receive signals
//
void routeGPIOs(void)
{
    EALLOW;
    // Reference Table 10-8 for GPIO Muxing

    // Boost Interrupt GPIO
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0 % 4;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO22 = 0 / 4;
    GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1;

    // Inverter Interrupt GPIO
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0 % 4;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO23 = 0 / 4;
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 1;

    // Status LED for Boost
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0 % 4;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = 0 / 4;
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;
    GpioDataRegs.GPASET.bit.GPIO20 = 1; // Start off

    // Disable Pin for Gate Driver
    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0 % 4;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO60 = 0 / 4;
    GpioCtrlRegs.GPBDIR.bit.GPIO60 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 1;    
    GpioDataRegs.GPBSET.bit.GPIO60 = 1; // Start off

    // ==========================================================
    // Hall Sensor GPIO Routing
    // ==========================================================
    // Hall Sensor A GPIO
    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0 % 4;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO51 = 0 / 4;
    GpioCtrlRegs.GPBDIR.bit.GPIO51 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO51 = 1;

    // Hall Sensor B GPIO
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0 % 4;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO34 = 0 / 4;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;

    // Hall Sensor C GPIO
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 0 % 4;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO56 = 0 / 4;
    GpioCtrlRegs.GPBDIR.bit.GPIO56 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO56 = 1;

    // ==========================================================
    // Boost GPIO Routing
    // ==========================================================
    // ePWM7A -> GPIO 12 -> Input X-BAR 6 -> Output X-BAR 4 -> GPIO27
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1 % 4;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = 1 / 4;
    InputXbarRegs.INPUT6SELECT = 12;
    OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX11 = 0x01;
    OutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX11 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 1 % 4;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO27 = 1 / 4;

    // ==========================================================
    // Inverter Phase A GPIO Routing
    // ==========================================================
    // ePWM8A -> GPIO 14 -> Input X-BAR 1 -> Output X-BAR 1 -> GPIO2
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1 % 4;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 1 / 4;
    InputXbarRegs.INPUT1SELECT = 14;
    OutputXbarRegs.OUTPUT1MUX0TO15CFG.bit.MUX1 = 0b01;
    OutputXbarRegs.OUTPUT1MUXENABLE.bit.MUX1 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 5 % 4;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 5 / 4;

    // ePWM8B -> GPIO 15 -> Input X-BAR 2 -> Output X-BAR 7 -> GPIO11
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1 % 4;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = 1 / 4;
    InputXbarRegs.INPUT2SELECT = 15;
    OutputXbarRegs.OUTPUT7MUX0TO15CFG.bit.MUX3 = 0b01;
    OutputXbarRegs.OUTPUT7MUXENABLE.bit.MUX3 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 3 % 4;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO11 = 3 / 4;

    // ==========================================================
    // Inverter Phase B GPIO Routing
    // ==========================================================
    // ePWM1"B" -> GPIO 1
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1 % 4;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 1 / 4;

    // ePWM1"A" -> CLB Global Signals -> CLB Tile 1 -> CLB Output X-BAR 4 -> GPIO 10
    Clb1LogicCtrlRegs.CLB_GLBL_MUX_SEL_1.bit.GLBL_MUX_SEL_IN_0 = 0;     // Table 30-2
    Clb1LogicCtrlRegs.CLB_LCL_MUX_SEL_1.bit.LCL_MUX_SEL_IN_0 = 0;       // Table 30-3
    Clb1LogicCtrlRegs.CLB_IN_MUX_SEL_0.bit.SEL_GP_IN_0 = 0;             // Figure 30-7
    Clb1LogicCtrlRegs.CLB_INPUT_FILTER.bit.SYNC0 = 1;                   // Figure 30-7
    Clb1LogicCtrlRegs.CLB_INPUT_FILTER.bit.FIN0 = 0b00;                 // Figure 30-7
    Clb1LogicCtrlRegs.CLB_INPUT_FILTER.bit.PIPE0 = 0;                   // Figure 30-7
    Clb1LogicCfgRegs.CLB_OUTPUT_LUT_0.bit.IN0 = 24;                     // Table 30-5
    Clb1LogicCfgRegs.CLB_OUTPUT_LUT_0.bit.IN1 = 0;                      // Table 30-5
    Clb1LogicCfgRegs.CLB_OUTPUT_LUT_0.bit.IN2 = 0;                      // Table 30-5
    Clb1LogicCfgRegs.CLB_OUTPUT_LUT_0.bit.FN = 0xAA;                    // Only pass IN0
    Clb1LogicCtrlRegs.CLB_LOAD_EN.bit.LOAD_EN = 1;
    Clb1LogicCtrlRegs.CLB_LOAD_EN.bit.GLOBAL_EN = 1;
    Clb1LogicCtrlRegs.CLB_OUT_EN = 1;
    ClbOutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX0 = 0b00;               // Table 11-6
    ClbOutputXbarRegs.OUTPUT4MUXENABLE.bit.MUX0 = 1;                    // Table 11-6
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 15 % 4;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO10 = 15 / 4;

    // ==========================================================
    // Inverter Phase C GPIO Routing
    // ==========================================================
    // ePWM3A -> GPIO0
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 15 % 4;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 15 / 4;

    // ePWM3B -> GPIO 5 -> Input X-BAR 5 -> Output X-BAR 2 -> GPIO3
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1 % 4;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO5 = 1 / 4;
    InputXbarRegs.INPUT5SELECT = 5;
    OutputXbarRegs.OUTPUT2MUX0TO15CFG.bit.MUX9 = 0b01;
    OutputXbarRegs.OUTPUT2MUXENABLE.bit.MUX9 = 1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 2 % 4;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 2 / 4;
    EDIS;
}

void initDAC(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR16.bit.DAC_A = 1;
    DacaRegs.DACCTL.bit.DACREFSEL = 1; // Reference Voltage of 1.65
    DacaRegs.DACCTL.bit.MODE = 1; // 2x Gain on output
    
    // Enable DAC output
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1;

    DELAY_US(10);
    
    // Load DAC value immediately on write to DACVALS
    DacaRegs.DACCTL.bit.LOADMODE = 0;
    
    // Initialize with 0
    DacaRegs.DACVALS.bit.DACVALS = 0;

    EDIS;
}

//
// adcB1ISR - Boost Converter Control Interrupt
//
__interrupt void adcB1ISR(void)
{
    // Toggle GPIO for measuring length of interrupt
    GpioDataRegs.GPASET.bit.GPIO22 = 1;

    // Check if overflow has occurred
    if(1 == AdcbRegs.ADCINTOVF.bit.ADCINT1)
    {
        AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; // clear overflow flag
        AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // clear INT1 flag
    }
    
    // ADC Read In
    Vout = (AdcbResultRegs.ADCRESULT0 * ADC_CONV_COEFF) * ((VOUT_DIV_RHIGH+VOUT_DIV_RLOW)/VOUT_DIV_RLOW);
    Vin = (AdcaResultRegs.ADCRESULT1 * ADC_CONV_COEFF) * ((VIN_DIV_RHIGH+VIN_DIV_RLOW)/VIN_DIV_RLOW);
    IL = ((AdcbResultRegs.ADCRESULT2 * ADC_CONV_COEFF) - ISENSE_OFFSET) * ISENSE_G_INV;

    // Update duty cycle
    EPwm7Regs.CMPA.bit.CMPA = pwm7_duty*EPwm7Regs.TBPRD;    

    // Enable/disable switching
    if (boost_pwm == 1)
    {
        EPwm7Regs.AQCSFRC.all = 0x00; // Clear any software trips/forces
        GpioDataRegs.GPACLEAR.bit.GPIO20 = 1; // LED4 ON
        GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1; // Gate Driver Disable LO
    }
    else
    {
        EPwm7Regs.AQCSFRC.all = 0x05; // Forces outputs low
        GpioDataRegs.GPASET.bit.GPIO20 = 1; // LED4 OFF
        GpioDataRegs.GPBSET.bit.GPIO60 = 1; // Gate Driver Disable HI
    }

    // Acknowledge the interrupt and clear the flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    // Toggle GPIO for measuring length of interrupt
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
}

//
// adcA1ISR - Inverter/Motor Control Interrupt
//
__interrupt void adcA1ISR(void)
{
    // Toggle GPIO for measuring length of interrupt
    GpioDataRegs.GPASET.bit.GPIO23 = 1;

    // Check if overflow has occurred
    if(1 == AdcaRegs.ADCINTOVF.bit.ADCINT1) 
    {
        AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1; // clear overflow flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // clear INT1 flag
    }
    
    // ADC Read In
    Ia = ((AdcbResultRegs.ADCRESULT3 * ADC_CONV_COEFF) - ISENSE_OFFSET) * ISENSE_G_INV;
    Ic = ((AdcaResultRegs.ADCRESULT4 * ADC_CONV_COEFF) - ISENSE_OFFSET) * ISENSE_G_INV;
    Ib = -Ia - Ic;
    throttle = AdcaResultRegs.ADCRESULT5 * ADC_RES;

    // Hall Sensor Read In
    HallA = GpioDataRegs.GPBDAT.bit.GPIO51;
    HallB = GpioDataRegs.GPBDAT.bit.GPIO34;
    HallC = GpioDataRegs.GPBDAT.bit.GPIO56;

    // Update duty cycle
    EPwm8Regs.CMPA.bit.CMPA = pwm8_duty*EPwm8Regs.TBPRD;
    EPwm1Regs.CMPA.bit.CMPA = pwm1_duty*EPwm1Regs.TBPRD;
    EPwm3Regs.CMPA.bit.CMPA = pwm3_duty*EPwm3Regs.TBPRD;  

    // For outputting an internal signal as a voltage on DAC A output pin
    if(dacVoltage > 3.3f) dacVoltage = 3.3f;
    if(dacVoltage < 0.0f) dacVoltage = 0.0f;
    dacBits = (uint16_t)(dacVoltage * 1240.9f); // 4095/3.3
    DacaRegs.DACVALS.bit.DACVALS = dacBits;

    // Enable/disable switching
    if (inv_pwm == 1)
    {
        EPwm8Regs.AQCSFRC.all = 0x00; // Clear any software trips/forces
        EPwm1Regs.AQCSFRC.all = 0x00;
        EPwm3Regs.AQCSFRC.all = 0x00;
    }
    else
    {
        EPwm8Regs.AQCSFRC.all = 0x05; // Forces outputs low
        EPwm1Regs.AQCSFRC.all = 0x05;
        EPwm3Regs.AQCSFRC.all = 0x05;
    }

    // Acknowledge the interrupt and clear the flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    // Toggle GPIO for measuring length of interrupt
    GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;
}

//
// End of File
//
