/*
 * File:   main.c
 * Author: geoff
 *
 * Created on January 26, 2025, 12:07 AM
 */

#pragma config FEXTOSC = OFF    // External Oscillator Mode Selection bits (EC (external clock) 16 MHz and above)
#pragma config RSTOSC = HFINTOSC_32MHZ// Power-up Default Value for COSC bits
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O function on RA4)
#pragma config VDDAR = HI       // VDD Range Analog Calibration Selection bit (Internal analog systems are calibrated for operation between VDD = 2.3V - 5.5V)

// CONFIG2
#pragma config MCLRE = EXTMCLR  // Master Clear Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RA3 pin function is MCLR)
#pragma config PWRTS = PWRT_OFF // Power-up Timer Selection bits (PWRT is disabled)
#pragma config WDTE = SWDTEN    // WDT enabled in software by SEN
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection bit (Brown-out Reset Voltage (VBOR) set to 1.9V)
#pragma config PPS1WAY = ON     // PPSLOCKED One-Way Set Enable bit (The PPSLOCKED bit can be set once after an unlocking sequence is executed; once PPSLOCKED is set, all future changes to PPS registers are prevented)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3

// CONFIG4
#pragma config BBSIZE = BB512   // Boot Block Size Selection bits (512 words boot block size)
#pragma config BBEN = OFF       // Boot Block Enable bit (Boot Block is disabled)
#pragma config SAFEN = OFF      // SAF Enable bit (SAF is disabled)
#pragma config WRTAPP = OFF     // Application Block Write Protection bit (Application Block is not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block is not write-protected)
#pragma config WRTC = OFF       // Configuration Registers Write Protection bit (Configuration Registers are not write-protected)
#pragma config WRTSAF = OFF     // Storage Area Flash (SAF) Write Protection bit (SAF is not write-protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR. MCLRE Configuration bit is ignored.)

// CONFIG5
#pragma config CP = OFF         // User Program Flash Memory Code Protection bit (User Program Flash Memory code protection is disabled)

#include <xc.h>
#include <stdio.h>
#include <string.h>

//#define TIMER1_ENABLE 0b00110001
//#define TIMER1_DISABLE 0b00000000

#define WIDTH_TIMEOUT -1

char message[64];
volatile int index, length;
volatile int width;

void xmitMessage(const char *pmessage) {
    strcpy(message, pmessage);
    length = strlen(pmessage);
    index = 0;
    PIE1 = 0b00001000;
}

void xmitByte(uint8_t value) {
    message[0] = value;
    length = 1;
    index = 0;
    PIE1 = 0b00001000;
}

void __interrupt() handleInterrupt() {
    if (PIR1 & 0b00001000) {
        // Tx buffer is ready for another character
        if (index < length) {
            TX1REG = message[index];
            if (++index == length) {
                PIE1 = 0b00000000;
            }
        }
    }

    if (PIR0 & 0b00010000) {
        if (PORTC & 0b00001000) {
            // Rising edge of echo detected, clear TM1.
            TMR1CON = 0b00000000;
            TMR1 = 0;
            // Enable timer at 1us per tick.
            TMR1CON = 0b00000001;
        } else {
            // Falling edge of echo detected, stop the timer and record the pulse width.
            TMR1CON = 0b00000000;
            width = TMR1;
        }
        IOCCF = 0x00;
    }

    if (PIR0 & 0b00100000) {
        // Timer 0 has overflowed => edge detection timeout.
        T1CON = 0b00000000;
        width = WIDTH_TIMEOUT;
        PIR0 = 0;
    }
}

void pulse(int us) {
    TMR1CON = 0b00000000;
    TMR1 = 0;

    RC4PPS = 0x01;
    CCPR1 = us;
    CCP1CON = 0b00000000;
    CCP1CON = 0b10001001;

    LATC |= 0b00010000;
    TMR1CON = 0b00000001;
}

#define NRF_CS 0b00000010

#define NRF_COMMAND_W_REGISTER 0b00100000
#define NRF_COMMAND_W_TX_PAYLOAD 0b10100000
#define NRF_COMMAND_FLUSH_TX 0b11100001
#define NRF_COMMAND_W_TX_PAYLOAD_NO_ACK 0b10110000

#define NRF_REGISTER_CONFIG 0x00
#define NRF_REGISTER_STATUS 0x07
#define NRF_REGISTER_RF_SETUP 0x06
#define NRF_REGISTER_FEATURE 0x1D

#define NRF_TX_DS 0b00100000
#define NRF_MAX_RT 0b00010000

void activateCS() {
    LATCbits.LATC1 = 0;
}

void deactivateCS() {
    LATCbits.LATC1 = 1;
}

uint8_t sendByte(uint8_t value) {
    int n, m;
    uint8_t received;

    received = 0;

    // Send byte by shifting one bit at a time onto MOSI while raising CLK for each bit.
    for (n = 0; n < 8; n++) {
        if (value & 0x80) {
            LATCbits.LATC2 = 1;
        } else {
            LATCbits.LATC2 = 0;
        }

        // Delay to allow the data to settle before signalling to device to sample it on leading clock edge.
        _delay(10);

        LATAbits.LATA2 = 1;

        // While CLK is high, shift the next bit into the received data.
        received <<= 1;
        if (PORTC & 0x01) {
            received |= 0x01;
        }

        LATAbits.LATA2 = 0;

        value <<= 1;
    }

    return received;
}

void writeCommand(uint8_t command) {
    activateCS();
    sendByte(command);
    deactivateCS();
}

void writeRegister(uint8_t target, uint8_t value) {
    activateCS();
    sendByte(target | NRF_COMMAND_W_REGISTER);
    sendByte(value);
    deactivateCS();
}

uint8_t getStatus() {
    // Send a NOP just to get the status back.
    activateCS();
    uint8_t status = sendByte(0xFF);
    deactivateCS();

    return status;
}

void flushTx() {
    activateCS();

    sendByte(NRF_COMMAND_FLUSH_TX);

    deactivateCS();
}

void enableTx() {
    writeCommand(NRF_COMMAND_FLUSH_TX);

    // Allow NO_ACK packets so we can fire and forget to multiple receivers.
    writeRegister(NRF_REGISTER_FEATURE, 0b00000001);
    
    // Set 250kps air data rate.
    writeRegister(NRF_REGISTER_RF_SETUP, 0b00101110);

    // Clear any previous transmission error so we're not blocked with writing.
    writeRegister(NRF_REGISTER_STATUS, 0b01110000);

    // Write nRF config to power up in Tx mode.
    writeRegister(NRF_REGISTER_CONFIG, 0b00001010);
}

void disableTx() {
    // Write nRF config to power down.
    writeRegister(NRF_REGISTER_CONFIG, 0b00001000);
}

void transmit(void *pdata, int length) {
    int n;
    uint8_t packet[32];

    memset(packet, 0, 32);
    memcpy(packet, pdata, length);

    activateCS();

    sendByte(NRF_COMMAND_W_TX_PAYLOAD_NO_ACK);

    for (n = 0; n < 32; n++) {
        sendByte(packet[n]);
    }

    deactivateCS();
}

void transmitString(char *pstring) {
    transmit(pstring, strlen(pstring) + 1);
}

void sleepDelay() {
    // Enable watchdog timer as this will wake from sleep.
    WDTCON = 0b00011011;

    // Sleep, WDT will wake us and continue execution.
    SLEEP();

    // Disable WDT on waking.
    WDTCON = 0b00000000;
}

/*
 * Distance sensor:
 *   RC5 = Vcc
 *   RC4 = Trigger
 *   RC3 = Echo
 * 
 * nRF radio:
 *   RA2 = CLK
 *   RC0 = MISO
 *   RC1 = CS
 *   RC2 = MOSI
 */

void main(void) {
    char s[64];
    int n, distance;

    // Timer 1 using 1MHz.
    T1CLK = 0b00000101;

    INTCON = 0b11000000;

    TRISA = 0x00;
    PORTA = 0x00;

    // Baud rate 19200 on RA5 for 32MHz system clock.
    TX1STA = 0b00100100;
    RC1STA = 0b10000000;
    SP1BRG = 103;
    RA5PPS = 0x05;

    // Enable echo pin and MISO inputs.
    ANSELC = 0x00;
    TRISC = 0b00001001;
    PORTC = 0x00;

    // Enable +ve and -ve edge detection on echo pin.
    IOCCP = 0b00001000;
    IOCCN = 0b00001000;

    // Set TTL input level for MISO as it's coming from a 3V device and we're at 4.5V.
    INLVLC = 0b11111110;

    while (1) {
        // Power up distance sensor and give it time to settle.
        LATC |= 0b00100000;
        _delay(1000000L);

        // Enable pin change interrupt so we can measure the echo width, and timer 0 for echo timeout.
        PIE0 = 0b00110000;
        width = 0;
        pulse(10);

        // Start Timer 0 for roughly 1s timeout.
        TMR0 = 0;
        T0CON1 = 0b10110011;
        T0CON0 = 0b10010000;

        while (width == 0);

        // Stop Timer 0
        T0CON0 = 0b00000000;

        // Disable pin change interrupt so it doesn't affect sleep.
        // TODO Is this needed?
        PIE0 = 0b000000000;

        // Power down sensor.
        LATC &= ~0b00100000;

        if (width == WIDTH_TIMEOUT) {
            goto sleep;
        }

        distance = (width * 171.5f) / 1000;

        // Power up radio and give it time to settle.
        enableTx();
        _delay(100L);

        sprintf(s, "sensors/distance-1=%d", distance);
        transmitString(s);

        // Add a delay or else MISO isn't at full output voltage. MISO is already borderline because it's coming from a 3V device.
        _delay(10000);

        // Wait for status to show packet processed.
        while ((getStatus() & 0b00110000) == 0);

        disableTx();

sleep:
        sleepDelay();
    }
}
