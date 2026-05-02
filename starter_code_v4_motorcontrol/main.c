//#############################################################################
//
// ECE 394 Control of Power Electronics
// Starter Code v4: Motor Current Control
//
// Notes: Make sure to edit the #define for HALL_OFFSET to match your 
// calibration angle for your bike. See the line 837 of code for an explanation
// of how to calibrate this angle.
//
// Coming soon: Boost Voltage Control
// 
// ============================================================================
// WATCH WINDOW EXPRESSIONS
// ============================================================================
//
//   boost_mode             : Control - 0 = PWM Off, 1 = Constant Duty, 2 = Current Control
//   inv_mode               : Control - 0 = PWM Off, 1 = Constant Duty, 2 = Sine PWM, 3 = Motor Control
//
//   d_boost_fixed          : Input   - Boost duty cycle [0.0 to 1.0]
//   d_a_fixed              : Input   - Inverter Phase A duty cycle [0.0 to 1.0]
//   d_b_fixed              : Input   - Inverter Phase B duty cycle [0.0 to 1.0]
//   d_c_fixed              : Input   - Inverter Phase C duty cycle [0.0 to 1.0]
//   Vin, Vout              : Monitor - ADC readings for DC input/output
//   Ia, Ib, Ic             : Monitor - ADC readings for motor 3-phase currents
//   IL                     : Monitor - ADC reading for boost inductor current
//   HallA, HallB, HallC    : Monitor - Motor Hall Sensor States [0 or 1]
//   omega_out              : Monitor - Calculated electrical frequency (rad/s) of motor based on Hall
//   motor_recon            : Monitor - Estimated V_AB of motor based on Hall estimation, use DACA_OUT pin to measure
//   IL_ref                 : Input   - Boost inductor current control reference
//   d_boost                : Monitor - Control output of current controller
//   omega_fixed            : Input   - Frequency (rad/s) of 3-phase Sine PWM
//   v_ac                   : Input   - Magnitude of sinusoidal component of Sine PWM voltage
//   id_ref                 : Input   - Motor d-axis current reference
//   iq_ref                 : Input   - Motor q-axis current reference
//   Id                     : Monitor - Motor d-axis current
//   Iq                     : Monitor - Motor q-axis current
//   
//###################################################################
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
// (INCLUDING NEGLIGENCE OR OTHER_MOTORISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//#############################################################################

//
// Included Files
//
#include "f28x_project.h"
#include "hall_estimator.h"
#include "pi_controller.h"
#include "dq_controller.h"

//
// Defines
//

// ===== Constants =====
#define PI 3.14159265359f

// ===== C2000 Parameters =====
#define SYSTEM_CLK_FREQ 150e6f          // Hz
#define ADC_FS 3.3f                     // V
#define ADC_RES (1/(4095.0f))
#define ADC_CONV_COEFF (ADC_FS*ADC_RES)
#define DAC_CONV_COEFF (1/ADC_CONV_COEFF)

// ===== PWM Settings =====
// Boost PWM
#define PWM7_FREQ 375e3f                // Hz
#define PWM7_INIT_DUTY 0.50f            // range: [0, 1]
// Inverter Phase A PWM
#define PWM8_FREQ 15e3f                 // Hz
#define PWM8_INIT_DUTY 0.50f            // range: [0, 1]
// Inverter Phase B PWM
#define PWM1_FREQ 15e3f                 // Hz
#define PWM1_INIT_DUTY 0.50f            // range: [0, 1]
// Inverter Phase C PWM
#define PWM3_FREQ 15e3f                 // Hz
#define PWM3_INIT_DUTY 0.50f            // range: [0, 1]
// Dead Time
#define DEAD_TIME_RED 300               // rising edge delay (clock cycles)
#define DEAD_TIME_FED 300               // falling edge delay (clock cycles)

// ===== Boost Control Parameters =====
#define BOOST_CTRL_DIV 5               // number of boost switching cycles in a control cycle
#define TSAMP_BOOST ((float)BOOST_CTRL_DIV/PWM7_FREQ)
#define BW_IL (2.0f*PI*3000.0f)
#define L 6.8e-6f
#define R_L 3.3e-3f
#define KP_IL (BW_IL*L)
#define KI_IL (BW_IL*R_L)

// ===== DQ Control Parameters  =====
#define R_RL 0.1f
#define L_RL 0.5e-3f
#define BW_RL (2.0f*PI*1.5e3f)
#define KP_RL (BW_RL*L_RL)
#define KI_RL (BW_RL*R_RL)

// ===== Motor Control Parameters =====
#define TSAMP_INV (1.0f/PWM8_FREQ)
#define VDC 48.0f
#define PP 23.0f
#define HALL_OFFSET ((PI / 180.0f) * (63.0f)) // CHANGE THIS FOR YOUR SPECIFIC E-BIKE (done 42826)
#define L_MOTOR 405e-6f
#define R_MOTOR 0.2f
#define LAMBDA 0.036f
#define BW_MOTOR (2.0f*PI*1.5e3f)
#define KP_MOTOR (BW_MOTOR * L_MOTOR)
#define KI_MOTOR (BW_MOTOR * R_MOTOR)

// ===== ADC Scaling Factors =====
#define VIN_DIV_RHIGH 8.2e3f            // ohms
#define VIN_DIV_RLOW 1e3f               // ohms
#define VOUT_DIV_RHIGH 18e3f            // ohms
#define VOUT_DIV_RLOW 1e3f              // ohms
#define ISENSE_G_INV (1/(0.055f))       // A/V
#define ISENSE_OFFSET 1.65f             // V

#define OFFSET (2.0f*PI/3.0f)

//
// Globals
//

volatile uint16_t boost_mode = 0;
volatile uint16_t inv_mode = 0;

uint16_t HallA;
uint16_t HallB;
uint16_t HallC;

float d_a_fixed = PWM8_INIT_DUTY;
float d_b_fixed = PWM1_INIT_DUTY;
float d_c_fixed = PWM3_INIT_DUTY;
float d_boost_fixed = PWM7_INIT_DUTY;

float Vin;
float Vout;
float IL;
float Ia;
float Ib;
float Ic;
float throttle;

uint16_t dacBits = 0;

static PIController pi_IL;
float IL_ref = 1.5f;
float pi_IL_out = 0.0f;
float error_IL = 0.0f;

float theta_fixed = 0.0f;
float omega_fixed = (2.0f*PI*60.0f);
float v_ac = 0.0f;

float m_a;
float m_b;
float m_c;
float d_a;
float d_b;
float d_c;

float d_boost;

static HallEstimator hall_ctrl;
float sector_out;
float omega_out;
float angle_out;
float motor_recon = 0.0f;

static DQController dq_motor;
float id_ref = 0.0f;
float iq_ref = 0.0f;
float Id;
float Iq;
float Vd;
float Vq;

float omega_m = 0;
float theta_m = 0;

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
    // Configure GPIO pins to aPPropriate signals
    //
    routeGPIOs();

    initDAC();

    hall_init(&hall_ctrl, TSAMP_INV, PP, HALL_OFFSET);

    pi_init(&pi_IL, KP_IL, KI_IL, 0.0, 0.0); // initialize pi for boost current control
    
    dq_init(&dq_motor, KP_MOTOR, KI_MOTOR, TSAMP_INV, VDC, L_MOTOR);
    
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
    EPwm1Regs.TBPHS.bit.TBPHS = 0;              // Phase offset = 0 for perfect alignment
    EPwm1Regs.TBCTL.bit.PHSDIR = TB_UP;         // Count up after sync
    EPwm1Regs.EPWMSYNCINSEL.bit.SEL = 8;        // Sync to PWM8
    EPwm1Regs.EPWMSYNCOUTEN.all = 0x0;          // Clear defaults

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
    EPwm3Regs.TBPHS.bit.TBPHS = 0;              // Phase offset = 0 for perfect alignment
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
    EPwm7Regs.ETSOCPS.bit.SOCAPRD2 = BOOST_CTRL_DIV;
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
	AdcbRegs.ADCSOC3CTL.bit.ACQPS = 14;			// number of clock cycles - 1
	AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 0x13;		// trigger on ePWM8 SOCA
    AnalogSubsysRegs.AGPIOCTRLH.bit.GPIO242 = 1; 
    GpioCtrlRegs.GPHAMSEL.bit.GPIO242 = 1;

    // SOC4: Ic
    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 2;			// convert ADCINA2
	AdcaRegs.ADCSOC4CTL.bit.ACQPS = 14;			// number of clock cycles - 1
	AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 0x13;		// trigger on ePWM8 SOCA
    AnalogSubsysRegs.AGPIOCTRLH.bit.GPIO224 = 1; 
    GpioCtrlRegs.GPHAMSEL.bit.GPIO224 = 1;

    // SOC5: Vthrottle
    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 10;			// convert ADCINA10
	AdcaRegs.ADCSOC5CTL.bit.ACQPS = 14;			// number of clock cycles - 1
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

    // Sampling Interrupt GPIO
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0 % 4;
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4 = 0 / 4;
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;

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
    GpioCtrlRegs.GPBPUD.bit.GPIO60 = 0;    
    GpioDataRegs.GPBSET.bit.GPIO60 = 1; // Start HI

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
    OutputXbarRegs.OUTPUT4MUX0TO15CFG.bit.MUX11 = 0b01;
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
    if (boost_mode == 1) {
        
        EPwm7Regs.CMPA.bit.CMPA = d_boost_fixed*EPwm7Regs.TBPRD;

    }
    else if (boost_mode == 2) {

        error_IL = IL_ref - IL;
        pi_update(&pi_IL, error_IL, TSAMP_BOOST, true, &pi_IL_out);
        d_boost = (Vin - pi_IL_out)/(Vout + 0.001f); // duty for high side sw
        
        // clamp duty
        if (d_boost < 0.1f) d_boost = 0.1f;
        if (d_boost > 0.9f) d_boost = 0.9f;

        EPwm7Regs.CMPA.bit.CMPA = d_boost*EPwm7Regs.TBPRD;
    }

    // Enable/disable switching
    if (boost_mode > 0)
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
    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;

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

    // NESTED INTERRUPT HANDLING ------------------ START
    // Acknowledge the PIE group early so the PIE hardware can route new interrupts
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    // Save the current interrupt enable registers
    uint16_t tempPIEIER = PieCtrlRegs.PIEIER1.all;
    uint16_t tempIER = IER;
    
    // Re-enable Group 1 interrupts at the CPU core level!
    //    (The hardware disabled this when it entered the ISR)
    IER |= M_INT1;

    // Modify PIEIER1 to ONLY allow ADCB1 (INT1.2) to interrupt.
    // 0x0002 corresponds to INTx2. This prevents ADCA1 from recursively interrupting itself!
    PieCtrlRegs.PIEIER1.all = 0x0002; 

    // Re-enable global interrupts to allow the preemption
    EINT;
    // NESTED INTERRUPT HANDLING ------------------ END
    
    // ADC Read In
    Ia = ((AdcbResultRegs.ADCRESULT3 * ADC_CONV_COEFF) - ISENSE_OFFSET) * ISENSE_G_INV;
    Ic = ((AdcaResultRegs.ADCRESULT4 * ADC_CONV_COEFF) - ISENSE_OFFSET) * ISENSE_G_INV + 0.4f;
    Ib = -Ia - Ic;
    throttle = AdcaResultRegs.ADCRESULT5 * ADC_RES;

    // Hall Sensor Read In
    HallA = GpioDataRegs.GPBDAT.bit.GPIO51;
    // HallB = GpioDataRegs.GPBDAT.bit.GPIO34; (original config)
    HallC = GpioDataRegs.GPBDAT.bit.GPIO34; // swap GPIO for Hall phase B->C (new config)
    // HallC = GpioDataRegs.GPBDAT.bit.GPIO56; (original config)
    HallB = GpioDataRegs.GPBDAT.bit.GPIO56; // swap GPIO for Hall phase C->B (new config)
    
    hall_update(&hall_ctrl,
        HallA, HallB, HallC, 
        &sector_out, &omega_out, &angle_out);

    // HALL SENSOR ANGLE TESTING:
    // The hall_update function above estimates the angle of the motor based on
    // readings from the Hall sensors. We can test if this angle is accurate using 
    // the DAC (Digital-Analog Converter) of the C2000. We will try to reconstruct 
    // the voltage waveform we expect to see from the motor back EMFs and compare
    // it to the actual voltage waveforms. motor_recon will show up on the DACA_OUT
    // pin of the C2000, or Pin 70. Measure this voltage using a regular scope 
    // probe, and measure the voltage between phase A/U and B/V of the motor. If 
    // the waveforms do not look aligned, adjust the #define HALL_OFFSET value.
    // Do not turn on any 24 V or 48 V supplies, and leave boost_mode and inv_mode
    // at 0.

    motor_recon = -1.5f * sinf(angle_out + (PI / 6.0f)) + 1.5f; // -sin because q-axis, + PI / 6.0f because line-to-line
    dacBits = (uint16_t)(motor_recon * DAC_CONV_COEFF);
    DacaRegs.DACVALS.bit.DACVALS = dacBits;

    if (inv_mode == 1) {

        d_a = d_a_fixed;
        d_b = d_b_fixed;
        d_c = d_c_fixed;

    }
    else if (inv_mode == 2) {

        theta_fixed = theta_fixed + omega_fixed*TSAMP_INV;
        if (theta_fixed >= (2.0f * PI)) {
            theta_fixed -= (2.0f * PI);
        }

        m_a = v_ac * (2.0f / VDC) * cosf(theta_fixed);
        m_b = v_ac * (2.0f / VDC) * cosf(theta_fixed - (OFFSET));
        m_c = v_ac * (2.0f / VDC) * cosf(theta_fixed + (OFFSET));

        d_a = 0.5f * (m_a + 1.0f);
        d_b = 0.5f * (m_b + 1.0f); 
        d_c = 0.5f * (m_c + 1.0f);

        Id = (2.0f / 3.0f) * ( Ia * cosf(theta_fixed) + 
                           Ib * cosf(theta_fixed - OFFSET) + 
                           Ic * cosf(theta_fixed + OFFSET) );

        Iq = (2.0f / 3.0f) * ( -Ia * sinf(theta_fixed) 
                           -Ib * sinf(theta_fixed - OFFSET) 
                           -Ic * sinf(theta_fixed + OFFSET) );

    }
    else if (inv_mode == 3) {
        
        dq_update(&dq_motor, 
            Ia, Ib, Ic, 
            id_ref, iq_ref, omega_out, angle_out,
            &m_a, &m_b, &m_c, 
            &Id, &Iq, &Vd, &Vq);
          
        d_a = 0.5f * (m_a + 1.0f);
        d_b = 0.5f * (m_b + 1.0f); 
        d_c = 0.5f * (m_c + 1.0f);      

    }

    // Update Compare Value
    EPwm8Regs.CMPA.bit.CMPA = d_a*EPwm8Regs.TBPRD;
    EPwm1Regs.CMPA.bit.CMPA = d_b*EPwm1Regs.TBPRD;
    EPwm3Regs.CMPA.bit.CMPA = d_c*EPwm3Regs.TBPRD;  
    
    // Enable/disable switching
    if (inv_mode > 0 )
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

    // NESTED INTERRUPT HANDLING ------------------ START
    // Disable global interrupts before restoring context
    DINT;

    // Restore the original interrupt registers
    PieCtrlRegs.PIEIER1.all = tempPIEIER;
    IER = tempIER;

    // NESTED INTERRUPT HANDLING ------------------ END

    // Acknowledge the interrupt and clear the flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Comment out for nested interrupt handling

    // Toggle GPIO for measuring length of interrupt
    GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;
    
}

//
// End of File
//
