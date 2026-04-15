// pic18f46q71-x2timer.c
// X2-timer-ng build-3 with the PIC18F46Q71
//
// PJ, 2024-07-11 Start with the command interpreter.
//     2024-07-13 Virtual registers and DACs plus ADC.
//     2024-07-14 Simple trigger implemented.
//     2024-07-15 Fixed delays implemented.
//     2024-07-16 Add third delay to simple trigger and implement TOF trigger.
//     2024-07-17 Refactor code for setting of latches.
//     2026-04-13 Add RA7 driving REMOTE_LEDa
//
#define VERSION_STR "v0.11 PIC18F46Q71 X2-timer-ng build-3 2026-04-13"
//
// PIC18F46Q71 Configuration Bit Settings (generated in Memory View)
// CONFIG1
#pragma config FEXTOSC = OFF
#pragma config RSTOSC = HFINTOSC_64MHZ

// CONFIG2
#pragma config CLKOUTEN = OFF
#pragma config PR1WAY = OFF
#pragma config BBEN = OFF
#pragma config CSWEN = OFF
#pragma config FCMEN = OFF
#pragma config FCMENP = OFF
#pragma config FCMENS = OFF

// CONFIG3
#pragma config MCLRE = EXTMCLR
#pragma config PWRTS = PWRT_64
#pragma config MVECEN = OFF
#pragma config IVT1WAY = OFF
#pragma config LPBOREN = OFF
#pragma config BOREN = SBORDIS

// CONFIG4
#pragma config BORV = VBOR_1P9
#pragma config ZCD = OFF
#pragma config PPS1WAY = OFF
#pragma config STVREN = ON
#pragma config LVP = ON
#pragma config DEBUG = OFF
#pragma config XINST = OFF

// CONFIG5
#pragma config WDTCPS = WDTCPS_31
#pragma config WDTE = ON

// CONFIG6
#pragma config WDTCWS = WDTCWS_7
#pragma config WDTCCS = SC

// CONFIG7
// BBSIZE = No Setting

// CONFIG8
#pragma config SAFSZ = SAFSZ_NONE

// CONFIG9
#pragma config WRTB = OFF
#pragma config WRTC = OFF
#pragma config WRTD = OFF
#pragma config WRTSAF = OFF
#pragma config WRTAPP = OFF

// CONFIG10
#pragma config CPD = OFF

// CONFIG11
#pragma config CP = OFF

#include <xc.h>
#include "global_defs.h"
#include <stdint.h>
#include <stdlib.h>

#include "uart.h"
#include "eeprom.h"
#include <stdio.h>
#include <string.h>

#define LED_POWER LATEbits.LATE0
#define LED_ARMa LATEbits.LATE1
#define LED_ARMb LATEbits.LATE2
#define LED_REMOTE LATAbits.LATA7

#define OUT0a LATCbits.LATC2
#define OUT0b LATCbits.LATC3
#define OUT1a LATDbits.LATD0
#define OUT1b LATDbits.LATD1
#define OUT2a LATDbits.LATD2
#define OUT2b LATDbits.LATD3
#define OUT3a LATCbits.LATC4
#define OUT3b LATCbits.LATC5
#define OUT4a LATDbits.LATD4
#define OUT4b LATDbits.LATD5
#define OUT5a LATDbits.LATD6
#define OUT5b LATDbits.LATD7
#define OUT6a LATBbits.LATB2
#define OUT6b LATBbits.LATB3
#define OUT7a LATBbits.LATB4
#define OUT7b LATBbits.LATB5

// Parameters controlling the device are stored in virtual registers.
#define NUMREG 9
int16_t vregister[NUMREG]; // working copy in SRAM
#define REG_MODE 0
#define REG_LEVELa 1
#define REG_LEVELb 2
#define REG_VREF 3
#define REG_DELAY0 4
#define REG_DELAY1 5
#define REG_DELAY2 6
#define REG_NUMER 7
#define REG_DENOM 8
const char* hints[NUMREG] = { 
  "mode",
  "level-a",
  "level-b",
  "Vref",
  "delay-0",
  "delay-1",
  "delay-2",
  "numer",
  "denom"
}; 

void set_registers_to_original_values()
{
    vregister[REG_MODE] = 0;    // mode
    // 0=simple trigger from INa, 1=time-of-flight(TOF) trigger
    vregister[REG_LEVELa] = 5;  // trigger level INa as a 8-bit count, 0-255
    vregister[REG_LEVELb] = 5;  // trigger level INb as a 8-bit count, 0-255
    vregister[REG_VREF] = 3;    // Vref selection for DAC
    // 0=off, 1=1024mV, 2=2048mV, 3=4096mV
    vregister[REG_DELAY0] = 0;  // delay 0 as a 16-bit count
    vregister[REG_DELAY1] = 0;  // delay 1
    vregister[REG_DELAY2] = 0;  // delay 2
    vregister[REG_NUMER] = 17;  // scaling numerator (1-255); keep small
    vregister[REG_DENOM] = 4;   // scaling denominator (1-255)
    // Scaling of 17/4 = 4.25 matches the X2 geometry of yesteryear.
}

// EEPROM is used to hold the parameters when the power is off.
// Note little-endian layout.
__EEPROM_DATA(0,0, 5,0, 5,0, 3,0);
__EEPROM_DATA(0,0, 0,0, 0,0, 17,0);
__EEPROM_DATA(4,0, 0,0, 0,0, 0,0);

char save_registers_to_EEPROM()
{
    for (uint16_t i=0; i < NUMREG; ++i) {
        DATAEE_WriteByte(2*i, (char)(vregister[i] & 0x00FF));
        DATAEE_WriteByte((2*i)+1, (char)((vregister[i] >> 8) & 0x00FF));
    }
    return 0;
}

char restore_registers_from_EEPROM()
{
    for (uint16_t i=0; i < NUMREG; ++i) {
        vregister[i] = (DATAEE_ReadByte((2*i)+1) << 8) | DATAEE_ReadByte(2*i);
    }
    return 0;
}

void init_pins()
{
    // Outputs for LEDs and debugging
    LED_POWER = 0; TRISEbits.TRISE0 = 0; ANSELEbits.ANSELE0 = 0;
    LED_ARMa = 0; TRISEbits.TRISE1 = 0; ANSELEbits.ANSELE1 = 0;
    LED_ARMb = 0; TRISEbits.TRISE2 = 0; ANSELEbits.ANSELE2 = 0;
    LED_REMOTE = 0; TRISAbits.TRISA7 = 0; ANSELAbits.ANSELA7 = 0;
    // We have comparator input pins C1IN0-/RA0 and C2IN3-/RB1.
    TRISAbits.TRISA0 = 1;
    ANSELAbits.ANSELA0 = 1;
    TRISBbits.TRISB1 = 1;
    ANSELBbits.ANSELB1 = 1;
    // Digital output signals.
    // Set their default level low until a peripheral drives them.
    OUT0a = 0; TRISCbits.TRISC2 = 0; ANSELCbits.ANSELC2 = 0;
    OUT0b = 0; TRISCbits.TRISC3 = 0; ANSELCbits.ANSELC3 = 0;
    OUT1a = 0; TRISDbits.TRISD0 = 0; ANSELDbits.ANSELD0 = 0;
    OUT1b = 0; TRISDbits.TRISD1 = 0; ANSELDbits.ANSELD1 = 0;
    OUT2a = 0; TRISDbits.TRISD2 = 0; ANSELDbits.ANSELD2 = 0;
    OUT2b = 0; TRISDbits.TRISD3 = 0; ANSELDbits.ANSELD3 = 0;
    OUT3a = 0; TRISCbits.TRISC4 = 0; ANSELCbits.ANSELC4 = 0;
    OUT3b = 0; TRISCbits.TRISC5 = 0; ANSELCbits.ANSELC5 = 0;
    OUT4a = 0; TRISDbits.TRISD4 = 0; ANSELDbits.ANSELD4 = 0;
    OUT4b = 0; TRISDbits.TRISD5 = 0; ANSELDbits.ANSELD5 = 0;
    OUT5a = 0; TRISDbits.TRISD6 = 0; ANSELDbits.ANSELD6 = 0;
    OUT5b = 0; TRISDbits.TRISD7 = 0; ANSELDbits.ANSELD7 = 0;
    OUT6a = 0; TRISBbits.TRISB2 = 0; ANSELBbits.ANSELB2 = 0;
    OUT6b = 0; TRISBbits.TRISB3 = 0; ANSELBbits.ANSELB3 = 0;
    OUT7a = 0; TRISBbits.TRISB4 = 0; ANSELBbits.ANSELB4 = 0;
    OUT7b = 0; TRISBbits.TRISB5 = 0; ANSELBbits.ANSELB5 = 0;
}

void update_FVRs()
{
    // We want to both the ADC and the DAC references set the same.
    uint8_t vref = vregister[REG_VREF] & 0x03;
    FVRCONbits.ADFVR = vref;
    FVRCONbits.CDAFVR = vref;
    FVRCONbits.EN = 1;
    while (!FVRCONbits.RDY) { /* should be less than 25 microseconds */ }
    return;
}

void FVR_close()
{
    FVRCONbits.EN = 0;
    FVRCONbits.ADFVR = 0;
    FVRCONbits.CDAFVR = 0;
    return;
}

void update_DACs()
{
    DAC2CONbits.PSS = 0b10; // FVR Buffer 2
    DAC2CONbits.NSS = 0; // VSS
    DAC2CONbits.EN = 1;
    DAC2DATL = (uint8_t)vregister[REG_LEVELa];
    //
    DAC3CONbits.PSS = 0b10; // FVR Buffer 2
    DAC3CONbits.NSS = 0; // VSS
    DAC3CONbits.EN = 1;
    DAC3DATL = (uint8_t)vregister[REG_LEVELb];
    //
    return;
}

void ADC_init()
{
    ADCON0bits.IC = 0; // single-ended mode
    ADCON0bits.CS = 1; // use dedicated RC oscillator
    ADCON0bits.FM = 0b01; // right-justified result
    ADCON2bits.ADMD = 0b000; // basic (legacy) behaviour
    PIR1bits.ADIF = 0;
    ADREFbits.NREF = 0; // negative reference is Vss
    ADREFbits.PREF = 0b11; // positive reference is FVR
    ADACQ = 0x10; // 16TAD acquisition period
    ADPCH = 0x00; // select RA0/C1IN0-
    ADCON0bits.ON = 1; // power on the device
    return;
}

uint16_t ADC_read(uint8_t i)
{
    // Returns the value from the ADC.
    ADPCH = i;
    ADCON0bits.GO = 1;
    NOP();
    while (ADCON0bits.GO) { /* wait, should be brief */ }
    PIR1bits.ADIF = 0;
    return ADRES;
}

void ADC_close()
{
    ADCON0bits.ON = 0;
    return;
}

void setup_CLCn_as_latch(uint8_t n, uint8_t source_S)
{
    // Follow the set-up description in Section 24.6 of data sheet.
    // We program a source for S and none for R.
    //
    // Note that the CLC number is one larger than the value
    // used in the select register.
    CLCSELECT = n-1;
    CLCnCONbits.EN = 0; // Disable while setting up.
    // Data select from outside world
    CLCnSEL0 = source_S; // data1 gets source_S as input (table 24.2)
    CLCnSEL1 = 0; // data2 gets CLCIN0PPS as input, but gets ignored in logic select
    CLCnSEL2 = 0; // data3 as for data2
    CLCnSEL3 = 0; // data4 as for data2
    // Logic select into gates
    CLCnGLS0 = 0b10; // data1 goes through true to gate 1 (S-R set)
    CLCnGLS1 = 0; // gate 2 gets logic 0 (S-R set)
    CLCnGLS2 = 0; // gate 3 gets logic 0 (S-R reset)
    CLCnGLS3 = 0; // gate 4 gets logic 0 (S-R reset)
    // Gate output polarities
    CLCnPOLbits.G1POL = 0;
    CLCnPOLbits.G2POL = 0;
    CLCnPOLbits.G3POL = 0;
    CLCnPOLbits.G4POL = 0;
    // Logic function is S-R latch
    CLCnCONbits.MODE = 0b011;
    // Do not invert output.
    CLCnPOLbits.POL = 0;
    // Finally, enable latch.
    CLCnCONbits.EN = 1;
} // end setup_CLCn_as_latch()

uint8_t trigger_simple()
{
    // Set up comparator 1 to monitor the analog input INa
    // and trigger on that voltage exceeding the specified level.
    // Use CLCs to latch the comparator output and use timers
    // to allow a couple of the output signals to be delayed.
    //
    // Returns:
    // 0 if successfully set up and the event passes,
    // 1 if the comparator is already high at set-up time.
    // 2 the delay timer TU16A started prematurely
    // 3 the delay timer TU16B started prematurely
    // 4 the delay time TMR1/CCP1 is high too soon
    //
    update_FVRs();
    update_DACs();
    // Connect INa through comparator 1.
    // Our external signal goes into the inverting input of the comparator,
    // so we need to invert the polarity to get trigger on positive slope
    // of the external signal.
    CM1NCH = 0b000; // C1IN0- pin
    CM1PCH = 0b100; // DAC2_Output
    CM1CON0bits.POL = 1;
    CM1CON0bits.HYS = 0; // no hysteresis
    CM1CON0bits.SYNC = 0; // async output
    CM1CON0bits.EN = 1;
    // The signal out of the comparator should transition 0 to 1
    // as the external trigger voltage crosses the specified level.
    __delay_ms(1);
    if (CMOUTbits.MC1OUT) {
        // Fail early because the comparator is already triggered.
        return 1;
    }
    // The use of two CLCs gets us access to all ports for the output pins.
    // Table 23-2 in data sheet states that:
    //   CLC1 can reach ports A,C
    //   CLC3 can reach ports B,D
    //
    setup_CLCn_as_latch(1, 0x20); // CLC1 latches CMP1_OUT (table 24-2)
    setup_CLCn_as_latch(3, 0x20); // CLC3 latches CMP1_OUT also
    //
    // Some out the outputs may be delayed so set up timers.
    uint16_t delay0 = (uint16_t)vregister[REG_DELAY0];
    uint16_t delay1 = (uint16_t)vregister[REG_DELAY1];
    uint16_t delay2 = (uint16_t)vregister[REG_DELAY2];
    //
    TUCHAINbits.CH16AB = 0; // independent counters
    if (delay0) {
        // OUT0 is delayed; use universal timer A started by CLC1_OUT.
        TU16ACON0bits.ON = 0;
        TU16ACLK = 0b00010; // FOSC
        TU16APS = 7; // With FOSC=64MHz, we want 125ns ticks
        TU16AHLTbits.CSYNC = 1;
        TU16ACON1bits.OSEN = 0; // not one shot
        TU16ACON0bits.OM = 0; // pulse mode output
        TU16AHLTbits.START = 0b10; // rising ERS edge
        TU16AHLTbits.RESET = 0; // none
        TU16AHLTbits.STOP = 0b11; // at PR match
        TU16AERS = 0b01110; // CLC1_OUT
        TU16APR = delay0 - 1;
        TU16ACON1bits.CLR = 1; // clear count
        // The timer output will be a pulse at PR match.
        // Use CLC2 as an SR latch on this output.
        setup_CLCn_as_latch(2, 0x36);
        TU16ACON0bits.ON = 1;
        __delay_ms(1);
        if (TU16ACON1bits.RUN) {
            // Fail early because the counter has started prematurely.
            return 2;
        }
    }
    if (delay1) {
        // OUT1 is delayed; use universal timer B started by CLC1_OUT.
        TU16BCON0bits.ON = 0;
        TU16BCLK = 0b00010; // FOSC
        TU16BPS = 7; // With FOSC=64MHz, we want 125ns ticks
        TU16BHLTbits.CSYNC = 1;
        TU16BCON1bits.OSEN = 0; // not one shot
        TU16BCON0bits.OM = 0; // pulse mode output
        TU16BHLTbits.START = 0b10; // rising ERS edge
        TU16BHLTbits.RESET = 0; // none
        TU16BHLTbits.STOP = 0b11; // at PR match
        TU16BERS = 0b01110; // CLC1_OUT
        TU16BPR = delay1 - 1;
        TU16BCON1bits.CLR = 1; // clear count
        // The timer output will be a pulse at PR match.
        // Use CLC4 as an SR latch on this output.
        setup_CLCn_as_latch(4, 0x37);
        TU16BCON0bits.ON = 1;
        __delay_ms(1);
        if (TU16BCON1bits.RUN) {
            // Fail early because the counter has started prematurely.
            return 3;
        }
    }
    if (delay2) {
        // OUT2 is delayed; use Timer1 gated by CLC1_OUT.
        T1CONbits.ON = 0;
        T1CLKbits.CS = 0b00001; // FOSC/4
        T1CONbits.CKPS = 0b01; // prescale 1:2 to get 125ns ticks
        T1CONbits.RD16 = 1;
        T1GATEbits.GSS = 0b10010; // CLC1_OUT
        T1GCONbits.GPOL = 1; // timer gate is active high
        T1GCONbits.GE = 1; // count controlled with gate input
        PIR3bits.TMR1IF = 0;
        PIR3bits.TMR1GIF = 0;
        TMR1 = 0;
        //
        CCP1CONbits.MODE = 0b1000; // want to set output on compare
        PIR3bits.CCP1IF = 0; // clear after changing mode
        CCPR1 = delay2;
        CCP1CONbits.EN = 0; // clear the output
        NOP(); NOP();
        CCP1CONbits.EN = 1;
        T1CONbits.ON = 1; // enable count
        __delay_ms(1);
        if (CCP1CONbits.OUT) {
            // Fail early because the TMR1/CCP1 is already showing high.
            return 4;
        }
    }
    //
    // Connect the output of CLCs to the relevant output pins.
    GIE = 0; // We run without interrupt.
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    // Table 23-2 in data sheet.
    // CLC1 can reach ports A,C
    // CLC3 can reach ports B,D
    if (delay0 == 0) {
        RC2PPS = 0x01; // OUT0a from CLC1
        RC3PPS = 0x01; // OUT0b
    } else {
        RC2PPS = 0x02; // OUT0a from TU16A latched by CLC2
        RC3PPS = 0x02; // OUT0b
    }
    if (delay1 == 0) {
        RD0PPS = 0x03; // OUT1a from CLC3
        RD1PPS = 0x03; // OUT1b
    } else {
        RD0PPS = 0x04; // OUT1a from TU16B latched by CLC4
        RD1PPS = 0x04; // OUT1b
    }
    if (delay2 == 0) {
        RD2PPS = 0x03; // OUT2a
        RD3PPS = 0x03; // OUT2b
    } else {
        RD2PPS = 0x0D; // OUT2a from CCP1 (and TMR1)
        RD3PPS = 0x0D; // OUT2b
    }
    RC4PPS = 0x01; // OUT3a
    RC5PPS = 0x01; // OUT3b
    RD4PPS = 0x03; // OUT4a
    RD5PPS = 0x03; // OUT4b
    RD6PPS = 0x03; // OUT5a
    RD7PPS = 0x03; // OUT5b
    RB2PPS = 0x03; // OUT6a
    RB3PPS = 0x03; // OUT6b
    RB4PPS = 0x03; // OUT7a
    RB5PPS = 0x03; // OUT7b
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    //
    LED_ARMa = 1; // Indicate that we are armed and waiting.
    LED_ARMb = 1; // Second LED indicator.
    LED_REMOTE = 1;
    //
    // Wait until the event and then clean up.
    while (!CLCDATAbits.CLC1OUT) { CLRWDT(); }
    // Have seen Event1.
    LED_ARMa = 0; // No longer armed and waiting.
    LED_ARMb = 0;
    LED_REMOTE = 0;
    // The delayed outputs may happen later, so wait for those, too.
    while (!PORTCbits.RC2) { CLRWDT(); }  // OUT0
    while (!PORTDbits.RD0) { CLRWDT(); }  // OUT1
    while (!PORTDbits.RD2) { CLRWDT(); }  // OUT2
    //
    // After the event, keep the outputs high for a short while
    // and then clean up.
    __delay_ms(100);
    // Redirect the output pins to their latches (which are all low).
    GIE = 0; // We run without interrupt.
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    RC2PPS = 0x00; RC3PPS = 0x00; // OUT0
    RD0PPS = 0x00; RD1PPS = 0x00; // OUT1
    RD2PPS = 0x00; RD3PPS = 0x00; // OUT2
    RC4PPS = 0x00; RC5PPS = 0x00; // OUT3
    RD4PPS = 0x00; RD5PPS = 0x00; // OUT4
    RD6PPS = 0x00; RD7PPS = 0x00; // OUT5
    RB2PPS = 0x00; RB3PPS = 0x00; // OUT6
    RB4PPS = 0x00; RB5PPS = 0x00; // OUT7b
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    //
    // Cleanup and disable peripherals.
    for (uint8_t i=0; i < 4; i++) {
        CLCSELECT = i;
        CLCnCONbits.EN = 0;
    }
    TU16ACON0bits.ON = 0;
    TU16BCON0bits.ON = 0;
    T1CONbits.ON = 0;
    CCP1CONbits.EN = 0;
    CM1CON0bits.EN = 0;
    //
    return 0; // Success is presumed at this point.
} // end trigger_simple()

uint8_t trigger_TOF()
{
    // Set up comparator 1 to monitor the analog input INa
    // and comparator 2 to monitor the analog input INb.
    // Event1 is CM1 going high.
    // Event2 is CM2 going high.
    // Measure the time of flight between Event1 and Event2 using Timer1+CCP1.
    // Compute the estimated time of arrival at test section (Event3)
    // and use Timer3+CCP2 to generate Event3.
    // Event3 drives the immediate outputs and starts the fixed delay timers.
    //
    // Returns:
    // 0 if successfully set up and the event passes,
    // 1 if CM1 is already high at set-up time.
    // 2 if CM2 is already high at set-up time.
    // 3 if CCP1 capture already happened at set-up time.
    // 4 if CCP2 compare already happened at set-up time.
    // 5 the delay timer TU16A started prematurely
    // 6 the delay timer TU16B started prematurely
    //
    update_FVRs();
    update_DACs();
    // Connect INa through comparator 1 to generate Event1.
    // Our external signal goes into the inverting input of the comparator,
    // so we need to invert the polarity to get trigger on positive slope
    // of the external signal.
    CM1NCH = 0b000; // C1IN0- pin
    CM1PCH = 0b100; // DAC2_Output
    CM1CON0bits.POL = 1;
    CM1CON0bits.HYS = 0; // no hysteresis
    CM1CON0bits.SYNC = 0; // async output
    CM1CON0bits.EN = 1;
    // The signal out of the comparator should transition 0 to 1
    // as the external trigger voltage crosses the specified level.
    __delay_ms(1);
    if (CMOUTbits.MC1OUT) {
        // Fail early because the comparator is already triggered.
        return 1;
    }
    // For Event1, use CLC3 for the output pins on PortB.
    // Table 23-2 in data sheet states that:
    //   CLC1,2 can reach ports A,C
    //   CLC3,4 can reach ports B,D
    //   CLC5,6 can reach ports A,C
    //   CLC7,8 can reach ports B,D
    setup_CLCn_as_latch(3, 0x20);
    //
    // Connect INb through comparator 2 to generate Event2.
    // Our external signal goes into the inverting input of the comparator,
    // so we need to invert the polarity to get trigger on positive slope
    // of the external signal.
    CM2NCH = 0b011; // C1IN3- pin
    CM2PCH = 0b101; // DAC3_Output
    CM2CON0bits.POL = 1;
    CM2CON0bits.HYS = 0; // no hysteresis
    CM2CON0bits.SYNC = 0; // async output
    CM2CON0bits.EN = 1;
    // The signal out of the comparator should transition 0 to 1
    // as the external trigger voltage crosses the specified level.
    __delay_ms(1);
    if (CMOUTbits.MC2OUT) {
        // Fail early because the comparator is already triggered.
        return 2;
    }
    setup_CLCn_as_latch(4, 0x21); // CLC4 to latch CMP2_OUT for Event2.
    //
    // Set up TMR1+CCP1 to capture the TOF period, following Event1
    // up to Event2.
    T1CONbits.ON = 0;
    T1CLKbits.CS = 0b00001; // FOSC/4
    T1CONbits.CKPS = 0b01; // prescale 1:2 to get 125ns ticks
    T1CONbits.RD16 = 1;
    T1GATEbits.GSS = 0b10100; // CLC3_OUT
    T1GCONbits.GPOL = 1; // timer gate is active high
    T1GCONbits.GE = 1; // count controlled with gate input
    PIR3bits.TMR1IF = 0;
    PIR3bits.TMR1GIF = 0;
    TMR1 = 0;
    // By default CCPn looks at TMR1.
    CCP1CONbits.MODE = 0b0101; // want to capture the TMR1 value at Event2
    CCP1CAPbits.CTS = 0b0111; // CLC4_OUT
    PIR3bits.CCP1IF = 0; // clear after changing mode
    CCP1CONbits.EN = 0; // clear the output
    NOP(); NOP();
    CCP1CONbits.EN = 1; // enable capture
    T1CONbits.ON = 1; // enable count
    __delay_ms(1);
    if (PIR3bits.CCP1IF) {
        // Fail early because capture has already happened.
        return 3;
    }
    //
    // Event3 will be generated after a delay following Event2
    // This delay is computed from the TOF period between Events 1 and 2.
    //
    // Set up CCP2 to look at Timer1 (default) and generate Event3
    // on match with the Period register.
    // Note that this compare value will need to be set after Event2 
    // actually occurs.
    //
    CCP2CONbits.MODE = 0b1000; // want to set output on compare
    PIR8bits.CCP2IF = 0; // clear after changing mode
    // Note that we do not know yet the actual count for the computed delay.
    CCP2CONbits.EN = 0; // clear the output
    __delay_ms(1);
    if (PIR8bits.CCP2IF) {
        // Fail early because compare has already happened.
        return 4;
    }
    //
    // Latch the output of CCP2 with CLC5 and CLC7
    // so that we can feed IO pins on all of the ports.
    setup_CLCn_as_latch(5, 0x18);
    setup_CLCn_as_latch(7, 0x18);
    //
    // Some out the outputs may be delayed so set up timers.
    uint16_t delay0 = (uint16_t)vregister[4];
    uint16_t delay1 = (uint16_t)vregister[5];
    //
    TUCHAINbits.CH16AB = 0; // independent counters
    if (delay0) {
        // OUT0 is delayed; use universal timer A started by CLC5_OUT.
        TU16ACON0bits.ON = 0;
        TU16ACLK = 0b00010; // FOSC
        TU16APS = 7; // With FOSC=64MHz, we want 125ns ticks
        TU16AHLTbits.CSYNC = 1;
        TU16ACON1bits.OSEN = 0; // not one shot
        TU16ACON0bits.OM = 0; // pulse mode output
        TU16AHLTbits.START = 0b10; // rising ERS edge
        TU16AHLTbits.RESET = 0; // none
        TU16AHLTbits.STOP = 0b11; // at PR match
        TU16AERS = 0b10010; // CLC5_OUT
        TU16APR = delay0 - 1;
        TU16ACON1bits.CLR = 1; // clear count
        // The timer output will be a pulse at PR match.
        // Use CLC1 as an SR latch on TU16A output.
        setup_CLCn_as_latch(1, 0x36);
        TU16ACON0bits.ON = 1;
        __delay_ms(1);
        if (TU16ACON1bits.RUN) {
            // Fail early because the counter has started prematurely.
            return 5;
        }
    }
    if (delay1) {
        // OUT1 is delayed; use universal timer B started by CLC5_OUT.
        TU16BCON0bits.ON = 0;
        TU16BCLK = 0b00010; // FOSC
        TU16BPS = 7; // With FOSC=64MHz, we want 125ns ticks
        TU16BHLTbits.CSYNC = 1;
        TU16BCON1bits.OSEN = 0; // not one shot
        TU16BCON0bits.OM = 0; // pulse mode output
        TU16BHLTbits.START = 0b10; // rising ERS edge
        TU16BHLTbits.RESET = 0; // none
        TU16BHLTbits.STOP = 0b11; // at PR match
        TU16BERS = 0b10010; // CLC5_OUT
        TU16BPR = delay1 - 1;
        TU16BCON1bits.CLR = 1; // clear count
        // The timer output will be a pulse at PR match.
        // Use CLC8 as an SR latch on this output.
        setup_CLCn_as_latch(8, 0x37);
        TU16BCON0bits.ON = 1;
        __delay_ms(1);
        if (TU16BCON1bits.RUN) {
            // Fail early because the counter has started prematurely.
            return 6;
        }
    }
    //
    // Connect the output of CLCs to the relevant output pins.
    GIE = 0; // We run without interrupt.
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    // Table 23-2 in data sheet.
    // CLC5 can reach ports A,C
    // CLC7 can reach ports B,D
    if (delay0 == 0) {
        RC2PPS = 0x05; // OUT0a from CLC5 for Event3
        RC3PPS = 0x05; // OUT0b
    } else {
        RC2PPS = 0x01; // OUT0a from TU16A latched by CLC1
        RC3PPS = 0x01; // OUT0b
    }
    if (delay1 == 0) {
        RD0PPS = 0x07; // OUT1a from CLC7 for Event3
        RD1PPS = 0x07; // OUT1b
    } else {
        RD0PPS = 0x08; // OUT1a from TU16B latched by CLC8
        RD1PPS = 0x08; // OUT1b
    }
    // Event3
    RD2PPS = 0x07; // OUT2a
    RD3PPS = 0x07; // OUT2b
    RC4PPS = 0x05; // OUT3a
    RC5PPS = 0x05; // OUT3b
    RD4PPS = 0x07; // OUT4a
    RD5PPS = 0x07; // OUT4b
    RD6PPS = 0x07; // OUT5a
    RD7PPS = 0x07; // OUT5b
    // Event2
    RB2PPS = 0x04; // OUT6a
    RB3PPS = 0x04; // OUT6b
    // Event1
    RB4PPS = 0x03; // OUT7a
    RB5PPS = 0x03; // OUT7b
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    //
    LED_ARMa = 1; // Indicate that we are armed and waiting.
    LED_ARMb = 1; // Second LED indicator.
    LED_REMOTE = 1;
    //
    // Event3 will be generated after a delay computed from 
    // the TOF between Events 1 and 2.
    uint16_t delay_extra = (uint16_t)vregister[REG_DELAY2];
    uint8_t numer = (uint8_t)vregister[REG_NUMER];
    uint8_t denom = (uint8_t)vregister[REG_DENOM];
    if (denom == 0) denom = 1; // Cannot tolerate a zero value here.
    //
    // Wait for Event1.
    while (!CLCDATAbits.CLC3OUT) { CLRWDT(); }
    LED_ARMa = 0; // No longer armed and waiting.
    LED_REMOTE = 0;
    //
    // Wait for Event2.
    while (!CLCDATAbits.CLC4OUT) { CLRWDT(); }
    NOP(); NOP();
    uint16_t tof = CCPR1;
    LED_ARMb = 0;
    //
    // Compute the tick count to get to Event3.
    // For X2 AT4-AT7 sensors, set the delay to be 4.25 times the TOF period.
    // uint16_t pr_value = (tof << 2) + (tof >> 2) + delay_extra;
    // numer=17, denom=4
    // We have tried to keep this calculation cheap and fast, however, 
    // it is the user's responsibility that the intermediate result
    // (tof*numer) does not overflow.  Keep to small values for numer.
    uint16_t pr_value = (tof*numer)/denom + delay_extra;
    CCPR2 = pr_value;
    NOP(); NOP();
    CCP2CONbits.EN = 1;
    //
    // Wait until Event3.
    while (!CLCDATAbits.CLC5OUT) { CLRWDT(); }
    // The delayed outputs may happen later, so wait for those, too.
    while (!PORTCbits.RC2) { CLRWDT(); }  // OUT0
    while (!PORTDbits.RD0) { CLRWDT(); }  // OUT1
    //
    // After the event, keep the outputs high for a short while
    // and then clean up.
    __delay_ms(100);
    //
    // Redirect the output pins to their latches (which are all low).
    GIE = 0; // We run without interrupt.
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    RC2PPS = 0x00; RC3PPS = 0x00; // OUT0
    RD0PPS = 0x00; RD1PPS = 0x00; // OUT1
    RD2PPS = 0x00; RD3PPS = 0x00; // OUT2
    RC4PPS = 0x00; RC5PPS = 0x00; // OUT3
    RD4PPS = 0x00; RD5PPS = 0x00; // OUT4
    RD6PPS = 0x00; RD7PPS = 0x00; // OUT5
    RB2PPS = 0x00; RB3PPS = 0x00; // OUT6
    RB4PPS = 0x00; RB5PPS = 0x00; // OUT7b
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    //
    // Cleanup and disable peripherals.
    for (uint8_t i=0; i < 8; i++) {
        CLCSELECT = i;
        CLCnCONbits.EN = 0;
    }
    TU16ACON0bits.ON = 0;
    TU16BCON0bits.ON = 0;
    T1CONbits.ON = 0;
    T3CONbits.ON = 0;
    CCP1CONbits.EN = 0;
    CCP2CONbits.EN = 0;
    CM1CON0bits.EN = 0;
    CM2CON0bits.EN = 0;
    //
    // Some debug (but, maybe, we'll keep it)
    printf("tof=%u pr=%d ", tof, pr_value);
    return 0;
}

void arm_and_wait_for_event(void)
{
    int nchar;
    uint8_t flag;
    switch (vregister[0]) {
        case 0:
            putstr("Armed simple trigger, using INa only: ");
            flag = trigger_simple();
            if (flag == 1) {
                putstr("C1OUT already high. fail\n");
            } else if (flag == 2) {
                putstr("delay0 timer TU16A started too soon. fail\n");
            } else if (flag == 3) {
                putstr("delay1 timer TU16B started too soon. fail\n");
            } else if (flag == 4) {
                putstr("delay2 timer TMR1/CCP1 output set too soon. fail\n");
            } else if (flag == 0) {
                putstr("triggered. ok\n");
            } else {
                putstr("unknown flag value. fail\n");
            }
            break;
        case 1:
            putstr("Armed time-of-flight trigger, using INa followed by INb: ");
            flag = trigger_TOF();
            if (flag == 1) {
                putstr("C1OUT already high. fail\n");
            } else if (flag == 2) {
                putstr("C2OUT already high. fail\n");
            } else if (flag == 3) {
                putstr("CCP1 capture already happened at set-up. fail\n");
            } else if (flag == 4) {
                putstr("CCP2 compare already happened at set-up. fail\n");
            } else if (flag == 5) {
                putstr("delay0 timer TU16A started too soon. fail\n");
            } else if (flag == 6) {
                putstr("delay1 timer TU16B started too soon. fail\n");
            } else if (flag == 0) {
                putstr("triggered. ok\n");
            } else {
                putstr("unknown flag value. fail\n");
            }
            break;
        default:
            putstr("Unknown mode. fail\n");
    }
}

// For incoming serial communication
#define NBUFA 80
char bufA[NBUFA];
// For outgoing serial communication
#define NBUFB 128
char bufB[NBUFB];

void interpret_command(char* cmdStr)
// We intend that valid commands are answered quickly
// so that the supervisory PC can infer the absence of a node
// by the lack of a prompt response.
// A command that does not do what is expected should return a message
// that includes the word "error".
{
    char* token_ptr;
    const char* sep_tok = ", ";
    int nchar;
    uint8_t i, j;
    int16_t v;
    // nchar = printf("DEBUG: cmdStr=%s", cmdStr);
    switch (cmdStr[0]) {
        case 'v':
            nchar = snprintf(bufB, NBUFB, "%s\n", VERSION_STR);
            putstr(bufB);
            break;
        case 'n':
            nchar = snprintf(bufB, NBUFB, "%u ok\n", NUMREG);
            putstr(bufB);
            break;
        case 'p':
            nchar = snprintf(bufB, NBUFB, "Register values:\n");
            putstr(bufB);
            for (i=0; i < NUMREG; ++i) {
                nchar = snprintf(bufB, NBUFB, "reg[%d]=%d (%s)\n",
                        i, vregister[i], hints[i]);
                putstr(bufB);
            }
            putstr("ok\n");
            break;
        case 'r':
            // Report a register value.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text, assume register number.
                i = (uint8_t) atoi(token_ptr);
                if (i < NUMREG) {
                    v = vregister[i];
                    nchar = snprintf(bufB, NBUFB, "%d (%s) ok\n", v, hints[i]);
                    putstr(bufB);
                } else {
                    putstr("fail\n");
                }
            } else {
                putstr("fail\n");
            }
            break;
        case 's':
            // Set a register value.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text; assume register number.
                // printf("text:\"%s\"", token_ptr);
                i = (uint8_t) atoi(token_ptr);
                if (i < NUMREG) {
                    token_ptr = strtok(NULL, sep_tok);
                    if (token_ptr) {
                        // Assume text is value for register.
                        v = (int16_t) atoi(token_ptr);
                        vregister[i] = v;
                        nchar = snprintf(bufB, NBUFB, "reg[%u] %d (%s) ok\n", i, v, hints[i]);
                        puts(bufB);
                        if (i == 3) { update_FVRs(); }
                        if (i == 1 || i == 2) { update_DACs(); }
                    } else {
                        putstr("fail\n");
                    }
                } else {
                    putstr("fail\n");
                }
            } else {
                putstr("fail\n");
            }
            break;
        case 'R':
            if (restore_registers_from_EEPROM()) {
                putstr("fail\n");
            } else {
                putstr("ok\n");
            }
            break;
        case 'S':
            if (save_registers_to_EEPROM()) {
                putstr("fail\n");
            } else {
                putstr("ok\n");
            }
            break;
        case 'F':
            set_registers_to_original_values();
            putstr("ok\n");
            break;
        case 'a':
            arm_and_wait_for_event();
            break;
        case 'c':
            // Report an ADC value.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some nonblank text, assume channel number.
                i = (uint8_t) atoi(token_ptr);
                if (i == 0 || i == 9 || i == 57 || i == 58) {
                    v = (int16_t)ADC_read(i);
                    nchar = snprintf(bufB, NBUFB, "%d ok\n", v);
                    putstr(bufB);
                } else {
                    putstr("fail\n");
                }
            } else {
                putstr("fail\n");
            }
            break;
        case 'h':
        case '?':
            putstr("\nPIC18F46Q71-I/P X2-trigger+timer commands and registers\n");
            putstr("\n");
            putstr("Commands:\n");
            putstr(" h or ? print this help message\n");
            putstr(" v      report version of firmware\n");
            putstr(" n      report number of registers\n");
            putstr(" p      report register values\n");
            putstr(" r <i>  report value of register i\n");
            putstr(" s <i> <j>  set register i to value j\n");
            putstr(" R      restore register values from EEPROM\n");
            putstr(" S      save register values to EEPROM\n");
            putstr(" F      set register values to original values\n");
            putstr(" a      arm device and wait for event\n");
            // Get ADC Positive Input Channel Selections from Table 41-7 in the data sheet
            putstr(" c <i>  convert analogue channel i (12-bit result, 0-4095)\n");
            putstr("        i=57 DAC2_output (INa)\n");
            putstr("        i=58 DAC3_output (INb)\n");
            putstr("        i=0  RA0/C1IN0- (INa)\n");
            putstr("        i=9  RB1/C2IN3- (INb)\n");
            putstr("\n");
            putstr("Registers:\n");
            putstr(" 0  mode: 0= simple trigger from INa signal\n");
            putstr("          1= time-of-flight(TOF) trigger\n");
            putstr(" 1  trigger level for INa as an 8-bit count, 0-255\n");
            putstr(" 2  trigger level for INb as an 8-bit count, 0-255\n");
            putstr(" 3  Vref selection for DACs 0=off, 1=1024mV, 2=2048mV, 3=4096mV\n");
            putstr(" 4  delay 0 as 16-bit count (8 ticks per us)\n");
            putstr(" 5  delay 1 as 16-bit count (8 ticks per us)\n");
            putstr(" 6  delay 2 as 16-bit count (8 ticks per us)\n");
            putstr(" 7  numerator for scaling (1-255) keep small\n");
            putstr(" 8  denominator for scaling (1-255)\n");
            putstr("ok\n");
            break;
        default:
            nchar = snprintf(bufB, NBUFB, "Error, unknown command: '%c'\n", cmdStr[0]);
            putstr(bufB);
    }
} // end interpret_command()

int main(void)
{
    int m;
    int n;
    init_pins();
    uart1_init(115200);
    restore_registers_from_EEPROM();
    __delay_ms(10);
    update_FVRs();
    update_DACs();
    ADC_init();
    __delay_ms(10);
    // Flash LED twice at start-up to indicate that the MCU is ready.
    for (int8_t i=0; i < 2; ++i) {
        LED_POWER = 1;
        __delay_ms(250);
        LED_POWER = 0;
        __delay_ms(250);
    }
    // Wait until we are reasonably sure that the MCU has restarted
    // and then flush the incoming serial buffer.
    __delay_ms(100);
    uart1_flush_rx();
    // We will operate the MCU as a slave, waiting for commands
    // and only responding then.
    LED_POWER = 1;  // Indicate that we are running. 
    while (1) {
        // Characters are not echoed as they are typed.
        // Backspace deleting is allowed.
        // CR signals end of incoming command string.
        m = getstr(bufA, NBUFA);
        if (m > 0) {
            interpret_command(bufA);
        }
    }
    ADC_close();
    FVR_close();
    uart1_flush_rx();
    uart1_close();
    LED_POWER = 0;
    return 0; // Expect that the MCU will reset.
} // end main
