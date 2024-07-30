//#############################################################################
//UART_Segbot Steer and Celebrate
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "f28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398

//NOTES DEFINED IN song.h
#define songsize    14
#define sampling_rate 10000 // 10 kHz
#define NOTE    625 // Cheer
#define n_samples   1000 // with a sampling frequency of 10kHz, this runs Goertzl 10x per second with 1000 ADC samples

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);
__interrupt void ADCA_ISR(void);
__interrupt void ADCB_ISR(void);

void setupSpib(void);
void serialRXA(serial_t *s, char data);
void serialRXC(serial_t *s, char data);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
void setEPWM2A(float);
void setEPWM2B(float);
float goertzel_mag(int numSamples,int TARGET_FREQUENCY,int SAMPLING_RATE, float* data);

typedef union phonedata_s{
    uint16_t rawdata[6];
    float fltdata[3];

}phonedata_t;
phonedata_t phoneaccel; //use new data type defined in union

//Song
float song[songsize] = {F5NOTE,F5NOTE,F5NOTE,F5NOTE,
                        E5NOTE,E5NOTE,F5NOTE,F5NOTE,
                        E5NOTE,E5NOTE,D5NOTE,D5NOTE,
                        G5NOTE,G5NOTE
                        };
float dance[songsize] = {-6,6,-6,6,
                         -6,6,-6,6,
                         -6,6,-6,6,
                         -6,6
                        };

int16_t pwm12prd = 50000000/sampling_rate;
int16_t note_detected = 0;
int16_t thresh = 150; //tune this value
float adcb_arrayPing[n_samples]= {0};
float adcb_arrayPong[n_samples]= {0};
int16_t adcb0result = 0;
int32_t adcbcount = 0;
int16_t PingPong = 0;
int16_t RunPong = 0;
int16_t RunPing = 0;
float goer_result = 0;
int16_t steeroff = 0;

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint16_t UARTPrint2 = 0;
uint16_t LEDdisplaynum = 0;

int16_t Junk = 0;

int16_t accelxraw = 0;
int16_t accelyraw = 0;
int16_t accelzraw = 0;
int16_t gyroxraw = 0;
int16_t gyroyraw = 0;
int16_t gyrozraw = 0;

float accelx = 0;
float accely = 0;
float accelz = 0;
float gyrox = 0;
float gyroy = 0;
float gyroz = 0;
float LeftWheel = 0;
float RightWheel = 0;
float LeftWheel_1 = 0;
float RightWheel_1 = 0;
float LeftVel = 0;
float RightVel = 0;
float LeftVel_1 = 0;
float RightVel_1 = 0;
float ubal = 0;
float K1 = -60;
//float K2 = -4.5;
float K2 = -7.0;
float K3 = -1.1;
float WheelDif = 0;
float velWheelDif = 0;
float WheelDif_1 = 0;
float velWheelDif_1 = 0;
float turnref = 0;
float turnref_1 = 0;
float uLeft = 5.0;
float uRight = 5.0;
float eDif = 0;
float iDif = 0;
float eDif_1 = 0;
float iDif_1 = 0;
float turn = 0; //turn setpoint
float Kp = 3.0;
float Ki = 20.0;
//float Kd = .08;
float Kd = .01;
float FwdBackOffset = 0;
float turnrate = 0;
float turnrate_1 = 0;


//lab4 variables
float adca0_volts = 0;
float adca1_volts = 0;
int16_t adca0result = 0;
int16_t adca1result = 0;

// Needed global Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset  = 0;
float gyroy_offset  = 0;
float gyroz_offset  = 0;
//float accelzBalancePoint = -.76; // tune this
float accelzBalancePoint = -.73;
int16 IMU_data[9];
uint16_t temp=0;
int16_t doneCal = 0;
float tilt_value    = 0;
float tilt_array[4] = {0, 0, 0, 0};
float gyro_value    = 0;
float gyro_array[4] = {0, 0, 0, 0};
float LeftWheelArray[4] = {0,0,0,0};
float RightWheelArray[4] = {0,0,0,0};
// Kalman Filter vars
float T = 0.001;        //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000;//50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;

//BLE DATA
float ble_x = 0;
float ble_y = 0;
float ble_z = 0;

void main(void)
 {
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LuanchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    //Ground GPIO1 Connected to BT CTS Pin
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    setupSpib();//run SPI setup code

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.SPIB_RX_INT = &SPIB_isr;
    PieVectTable.ADCA1_INT = &ADCA_ISR;
    PieVectTable.ADCB1_INT = &ADCB_ISR;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 243500); // 123 bpm
    ConfigCpuTimer(&CpuTimer1, 200, 4000);
    ConfigCpuTimer(&CpuTimer2, 200, 200000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
    init_serial(&SerialC,9600,serialRXC); //set to 9600 for Bluetooth Chip, initializations and muxing done in Serial.c
//    init_serial(&SerialD,115200,serialRXD);

    //Use as timer for ADCA
    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1  50Mhz Clock
    EPwm5Regs.TBPRD = 50000;  // Set Period to 1ms sample. Rampling rate: 1000 Hz
    //(1/50MHz)*TBPRD = .001 s
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

    //PWM9A - Buzzer (Change TBPRD to play song, tempo in Timer0)
    EPwm9Regs.TBCTL.bit.CTRMODE = 0; //count up mode
    EPwm9Regs.TBCTL.bit.FREE_SOFT = 2; //free run
    EPwm9Regs.TBCTL.bit.PHSEN = 0; //disable phase loading
    EPwm9Regs.TBCTL.bit.CLKDIV = 1; //clock divide = 2
    EPwm9Regs.TBCTR = 0; //start the timers at 0
    EPwm9Regs.TBPRD = OFFNOTE;
//    EPwm9Regs.CMPA.bit.CMPA = 0; //start duty cycle at 0%
    // duty cycle = CMPA/TBPRD
    EPwm9Regs.AQCTLA.bit.CAU = 0; //do nothing TBCTR = CMPA (not using compare register)
    EPwm9Regs.AQCTLA.bit.ZRO = 3; //toggle when TBCTR resets
    EPwm9Regs.TBPHS.bit.TBPHS = 0; //set phase to 0
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5); //Buzzer

    //use as timer for ADCB
    EPwm12Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm12Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm12Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm12Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm12Regs.TBCTR = 0x0; // Clear counter
    EPwm12Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm12Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm12Regs.TBCTL.bit.CLKDIV = 0; // divide by 1  50Mhz Clock
    EPwm12Regs.TBPRD = pwm12prd;  // Set Period to 1 ms sample. fs = 1000 Hz  Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm12Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm12Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;

    EALLOW;
    //write configurations for all ADCs  ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    //        AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    //        AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    //        AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    //        AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //        AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //        AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //        AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //        AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);


    //Select the channels to convert and end of conversion flag
    //Many statements commented out,  To be used when using ADCA or ADCB
    //ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert Channel you choose Does not have to be A0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0D;// EPWM5 ADCSOCA will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert Channel you choose Does not have to be A1
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0D;// EPWM5 ADCSOCA or another trigger you choose will trigger SOC1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to last SOC that is converted and it will set INT1 flag ADCA1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4;  //SOC0 will convert Channel B4
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 14; //sample window is acqps + 1 SYSCLK cycles = 75ns
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0x1B; // EPWM12 ADCSOCA will trigger SOC0
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;

    //2A and 2B - Motors
    EPwm2Regs.TBCTL.bit.CTRMODE = 0; //count up mode
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; //free run
    EPwm2Regs.TBCTL.bit.PHSEN = 0; //disable phase loading
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; //clock divide = 1
    EPwm2Regs.TBCTR = 0; //start the timers at 0
    EPwm2Regs.TBPRD = 2500; //set period to 50 microseconds (20 kHz)
    // (1/50MHz)*TBPRD = 50 us
    EPwm2Regs.CMPA.bit.CMPA = 0; //start duty cycle at 0%
    EPwm2Regs.CMPB.bit.CMPB = 0; //start duty cycle at 0%
    // duty cycle = CMPA/TBPRD
    EPwm2Regs.AQCTLA.bit.CAU = 1; //clear signal when TBCTR = CMPA
    EPwm2Regs.AQCTLB.bit.CBU = 1; //clear signal when TBCTR = CMPB
    EPwm2Regs.AQCTLA.bit.ZRO = 2; //set to high at 0
    EPwm2Regs.AQCTLB.bit.ZRO = 2; //set to high at 0
    EPwm2Regs.TBPHS.bit.TBPHS = 0; //set phase to 0
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); //ePWM 2A
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); //ePWM 2B

    init_eQEPs(); //call eQEP setup function

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6; //SPI

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    //Enable SPI
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    //Enable ADCA1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    //Enable ADCB1
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
//            serial_printf(&SerialC,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA); //prints to BT app
//            serial_printf(&SerialC,"FwdBackOffset: %.2f TurnRate: %.2f\r\n",FwdBackOffset, turnrate); //prints to BT app
            serial_printf(&SerialA,"X: %.2f Y: %.2f Z: %.2f\r\n",ble_x, ble_y, ble_z);//print acceleration data from phone to tera term
            UARTPrint = 0;
        }
       if(UARTPrint2 == 1){
           serial_printf(&SerialC,"LeftVel: %.2f RightVel: %.2f\r\n",LeftVel, RightVel); //prints to BT app
           UARTPrint2 = 0;
       }

       //after n samples, pass data through Goertzel fxn
       // use Ping Pong buffer

       if(RunPing == 1){
           goer_result = goertzel_mag(n_samples,NOTE,sampling_rate, adcb_arrayPing);
           RunPing = 0;
           if(goer_result > thresh){
               note_detected = 1;
           }
       }
       if(RunPong == 1){
           goer_result = goertzel_mag(n_samples,NOTE,sampling_rate, adcb_arrayPong);
           RunPong = 0;
           if(goer_result > thresh){
               note_detected = 1;
           }
       }

    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......
    //solve for filtered velocity in rads/s
    LeftVel = .6*LeftVel_1 + 100*(LeftWheel-LeftWheel_1);
    RightVel = .6*RightVel_1 + 100*(RightWheel-RightWheel_1);

    //Turning
    WheelDif = LeftWheel - RightWheel;
    velWheelDif = .33*velWheelDif_1 + .67*(WheelDif-WheelDif_1)/.004;
    eDif = turnref - WheelDif;
    //calculate iDif without integral windup
    if(fabs(turn)>3){
        iDif = iDif_1;
    }else{
        iDif = iDif_1 + .004*(eDif+eDif_1)/2;
    }

    //Calculate Balance and Turn COntrols
    ubal = -K1*tilt_value -K2*gyro_value - K3*(LeftVel+RightVel)/2.0;
    turn = Kp*eDif + Ki*iDif - Kd*velWheelDif;
    //saturate turn so balance control dominates (controllefford between +-10)
    if(turn>4){
        turn = 4;
    }else if(turn<-4){
        turn = -4;
    }

    turnref = turnref_1 + (turnrate + turnrate_1)*.004/2.0;

    //Calculate controleffort for each motor
    uRight = ubal/2.0 - turn + FwdBackOffset;
    uLeft = ubal/2.0 + turn + FwdBackOffset;

    //Send ControlEffort to PWM Motor Command
    setEPWM2A(uRight);
    setEPWM2B(-uLeft);

    //Save Past States
    LeftVel_1 = LeftVel;
    RightVel_1 = RightVel;
    LeftWheel_1 = LeftWheel;
    RightWheel_1 = RightWheel;
    eDif_1 = eDif;
    iDif_1 = iDif;
    turnrate_1 = turnrate;
    turnref_1 = turnref;

    numSWIcalls++;
    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    if(note_detected == 1){
        steeroff = 1; //when the note is detected, stop accepting accelerometer data as steering commands

        // Blink a number of LEDS if the Goertzel value exceeds the threshold
        GpioDataRegs.GPBTOGGLE.bit.GPIO52 = 1;
        GpioDataRegs.GPCTOGGLE.bit.GPIO94 = 1;
        GpioDataRegs.GPDTOGGLE.bit.GPIO97 = 1;

        if(numTimer0calls <= songsize){
            //play the song
            EPwm9Regs.TBPRD = song[numTimer0calls];
            //do the dance
            FwdBackOffset = 0;
            turnrate = dance[numTimer0calls];
            numTimer0calls++; // only increment after note is detected
        }else if (numTimer0calls > songsize){
            note_detected = 0;
            numTimer0calls = 0;
            turnrate = 0;
            steeroff = 0; //can now control again with phone
        }
    }
    CpuTimer0.InterruptCount++;
    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{

    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{

    UARTPrint2 = 1; //every 200 ms

    CpuTimer2.InterruptCount++;

}

// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;
}

char sendback[10];
char past[14];
void serialRXC(serial_t *s, char data) {
//    sendback[0] = data;
//    serial_send(&SerialA, sendback, 1); //every character that comes into BT will be echoed to Tera Term

    //save past states
    past[0] = past[1];
    past[1] = past[2];
    past[2] = past[3];
    past[3] = past[4];
    past[4] = past[5];
    past[5] = past[6];
    past[6] = past[7];
    past[7] = past[8];
    past[8] = past[9];
    past[9] = past[10];
    past[10] = past[11];
    past[11] = past[12];
    past[12] = past[13];
    past[13] = data;

    if(steeroff == 0){
        //use phone accelerometer data to steer
        if((past[0] == '!')&&(past[1] == 'A')){
            //incoming accelerometer data (arbitrary start point 0)

            //float is a 32-bit value (4x8-bit)
            //mantissa*2^exp where mantissa takes ~upper 21 bits and ~lower 10 is exponent
            //meaning it is not a standard ascii
            //create a union
            //there will be six 16-bit ints that make up 3 floats
            //for a union, takes up same place in memory
            //float comes over in 4 8 bit chunks

            //X
            phoneaccel.rawdata[0] = (past[3]<<8)|past[2];
            phoneaccel.rawdata[1] = (past[5]<<8)|past[4];
            //Y
            phoneaccel.rawdata[2] = (past[7]<<8)|past[6];
            phoneaccel.rawdata[3] = (past[9]<<8)|past[8];
            //Z
            phoneaccel.rawdata[4] = (past[11]<<8)|past[10];
            phoneaccel.rawdata[5] = (past[13]<<8)|past[12];

            ble_x = phoneaccel.fltdata[0];
            ble_y = phoneaccel.fltdata[1];
            ble_z = phoneaccel.fltdata[2];

            //Interpret phone accel data as steering instructions
            if(ble_y >= 0.3){
                //turn left
    //            turnrate = -2.0;
                turnrate = -4.0;
            }else if(ble_y <= -0.3){
                //turn right
    //            turnrate = 2.0;
                turnrate = 4.0;
            }else if(fabs(ble_y) < 0.3){
                //don't turn
                turnrate = 0;
            }

            if(ble_z <= -0.3){
                //go forward
                FwdBackOffset = -1.0;
            }else if(ble_z >= 0.3){
                //go backward
                FwdBackOffset = 1.0;
            }else if(fabs(ble_z) < 0.3){
                //don't go forward or backward
                FwdBackOffset = 0;
            }

            UARTPrint = 1;
        }
    }

}

__interrupt void ADCA_ISR(void){
        adca0result = AdcaResultRegs.ADCRESULT0; //read ADC values
        adca1result = AdcaResultRegs.ADCRESULT1;

        // Here covert ADCIND0, ADCIND1 to volts
        adca0_volts = adca0result/1365.0;
        adca1_volts = adca1result/1365.0;

        //SPI Read Gyros and Accelerometers

        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
        SpibRegs.SPIFFRX.bit.RXFFIL = 8; // Issue the SPIB_RX_INT when two values are in the RX FIFO

        //write to TXBUF
        SpibRegs.SPITXBUF = 0x8000|0x3A00; //start address (one register before accel)
        SpibRegs.SPITXBUF = 0x0000; //write 0x00 to read ACCEL_XOUT
        SpibRegs.SPITXBUF = 0x0000; //write 0x00 to read ACCEL_YOUT
        SpibRegs.SPITXBUF = 0x0000; //write 0x00 to read ACCEL_ZOUT
        SpibRegs.SPITXBUF = 0x0000; //write 0x00 to read TEMP_OUT
        SpibRegs.SPITXBUF = 0x0000; //write 0x00 to read GYRO_XOUT
        SpibRegs.SPITXBUF = 0x0000; //write 0x00 to read GYRO_YOUT
        SpibRegs.SPITXBUF = 0x0000; //write 0x00 to read GYRO_ZOUT

        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //clear interrupt flag
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void SPIB_isr(void){

    //read from RX FIFO
    Junk = SpibRegs.SPIRXBUF; //garbage value read
    accelxraw = SpibRegs.SPIRXBUF;
    accelyraw = SpibRegs.SPIRXBUF;
    accelzraw = SpibRegs.SPIRXBUF;
    Junk = SpibRegs.SPIRXBUF;
    gyroxraw = SpibRegs.SPIRXBUF;
    gyroyraw = SpibRegs.SPIRXBUF;
    gyrozraw = SpibRegs.SPIRXBUF;

    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High

    accelx = (float)(accelxraw*4.0/32767.0); // m/s2, this value is saturated due to offset
    accely = (float)(accelyraw*4.0/32767.0); // m/s2
    accelz = (float)(accelzraw*4.0/32767.0); // m/s2
    gyrox = (float)(gyroxraw*250.0/32767.0); //deg per second
    gyroy = (float)(gyroyraw*250.0/32767.0); //deg per second
    gyroz = (float)(gyrozraw*250.0/32767.0); //deg per second

    //Code to be copied into SPIB_ISR interrupt function after the IMU measurements have been collected.
      if(calibration_state == 0){
            calibration_count++;
            if (calibration_count == 2000) {
                calibration_state = 1;
                calibration_count = 0;
            }
        } else if(calibration_state == 1){
            accelx_offset+=accelx;
            accely_offset+=accely;
            accelz_offset+=accelz;
            gyrox_offset+=gyrox;
            gyroy_offset+=gyroy;
            gyroz_offset+=gyroz;
            calibration_count++;
            if (calibration_count == 2000) {
                calibration_state = 2;
                accelx_offset/=2000.0;
                accely_offset/=2000.0;
                accelz_offset/=2000.0;
                gyrox_offset/=2000.0;
                gyroy_offset/=2000.0;
                gyroz_offset/=2000.0;
                calibration_count = 0;
                doneCal = 1;
            }

        } else if(calibration_state == 2){

            accelx -=(accelx_offset);
            accely -=(accely_offset);
            accelz -=(accelz_offset-accelzBalancePoint);
            gyrox -= gyrox_offset;
            gyroy -= gyroy_offset;
            gyroz -= gyroz_offset;

            /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
            float tiltrate = (gyrox*PI)/180.0; // rad/s
            float pred_tilt, z, y, S;

            // Prediction Step
            pred_tilt = kalman_tilt + T*tiltrate;
            pred_P = kalman_P + Q;

            // Update Step
            z = -accelz;  // Note the negative here due to the polarity of AccelZ
            y = z - pred_tilt;
            S = pred_P + R;
            kalman_K = pred_P/S;
            kalman_tilt = pred_tilt + kalman_K*y;
            kalman_P = (1 - kalman_K)*pred_P;

            SpibNumCalls++;
            // Kalman Filter used
            tilt_array[SpibNumCalls] = kalman_tilt;
            gyro_array[SpibNumCalls] = tiltrate;
            LeftWheelArray[SpibNumCalls] = -readEncLeft();
            RightWheelArray[SpibNumCalls] = readEncRight();

            if (SpibNumCalls >= 3) {  // should never be greater than 3
                tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2] + tilt_array[3])/4.0;
                gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2] + gyro_array[3])/4.0;
                LeftWheel=(LeftWheelArray[0]+LeftWheelArray[1]+LeftWheelArray[2]+LeftWheelArray[3])/4.0;
                RightWheel=(RightWheelArray[0]+RightWheelArray[1]+RightWheelArray[2]+RightWheelArray[3])/4.0;
                SpibNumCalls = -1;

                PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
            }
        }

        timecount++;

        if((timecount%200) == 0)
        {
            if(doneCal == 0) {
                GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;  // Blink Blue LED while calibrating
            }
            GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;  // Always Block Red LED

//            UARTPrint = 1;  // Tell While loop to print
        }

        SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
        SpibRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
        PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

__interrupt void ADCB_ISR (void)
{
    adcb0result = AdcbResultRegs.ADCRESULT0; //read value between 0 and 4095 from ADC
    //MAX9814 sensitivity = -44 dBV meaning the signal corresponds to 6.3096mV/Pa of sound pressure
    //ADC values 0 to 4095 correspond to voltages 0-3V, mic produces 0.5 to 2.5 V (values ~682 to ~3412)

    //Ping
    if(PingPong == 0){
        adcb_arrayPing[adcbcount] = adcb0result; // add ADC reading to array
        if(adcbcount == (n_samples-1)){
            adcbcount = -1; //incremented to 0 at end of interrupt
            RunPing = 1; //check in while loop
            PingPong = 1; //switch to Pong buffer
        }
    }
    //Pong
    if(PingPong == 1){
        adcb_arrayPong[adcbcount] = adcb0result;
        if(adcbcount == (n_samples-1)){
            adcbcount = -1; //incremented to 0 at end of interrupt
            RunPong = 1;
            PingPong = 0; //switch to Pong buffer
        }
    }

    adcbcount++; //increment count
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; //clear PIE peripheral so processor waits until next interrupt flag
}

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U

    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue;  // I don't think this is needed and never true

    // 5 North South magnet poles in the encoder disk so 5 square waves per one revolution of the
    // DC motor's back shaft.  Then Quadrature Decoder mode multiplies this by 4 so 20 counts per one rev
    // of the DC motor's back shaft.  Then the gear motor's gear ratio is 30:1.
    return (raw*(TWOPI/600.0)); //wheel rotation in radians
}

float readEncRight(void) {

    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U  -1 32bit signed int

    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue;  // I don't think this is needed and never true

    // 5 North South magnet poles in the encoder disk so 5 square waves per one revolution of the
    // DC motor's back shaft.  Then Quadrature Decoder mode multiplies this by 4 so 20 counts per one rev
    // of the DC motor's back shaft.  Then the gear motor's gear ratio is 30:1.
    return (raw*(TWOPI/600.0));
}

void init_eQEPs(void) {

    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;    // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;    // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2;   // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2;   // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0;    // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0;   // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0;      // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0;      // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0;        // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF;   // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;  // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1;    // Enable EQep


    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;    // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1;    // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2;   // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2;   // Qual every 6 samples
    EDIS;
...

This file has been truncated, please download it to see its full contents.