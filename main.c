
#include <xc.h>
#include <stdbool.h>

// Configuration bits
#pragma config FOSC = INTRC_NOCLKOUT // Internal oscillator
#pragma config WDTE = OFF       // Watchdog Timer disabled
#pragma config PWRTE = ON       // Power-up Timer enabled
#pragma config MCLRE = ON       // MCLR pin enabled
#pragma config CP = OFF         // Code protection disabled
#pragma config CPD = OFF        // Data memory protection disabled
#pragma config BOREN = ON       // Brown-out Reset enabled
#pragma config IESO = OFF       // Internal/External Switchover disabled
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor disabled
#pragma config LVP = OFF        // Low-Voltage Programming disabled

#define _XTAL_FREQ 4000000      // 4 MHz internal oscillator

// Function prototypes
void initSystem(void);
unsigned int readADC(uint8_t channel);
void init_timer(void);
void __interrupt() timer_isr(void);

#define voltagechannel 3
#define dryrunpotchannel 4

#define MAINS_LED        PORTAbits.RA0   // Pin 2
#define MOTOR_ON_LED     PORTAbits.RA1   // Pin 3
#define DRY_RUN_LED      PORTAbits.RA2   // Pin 4
#define VOLTAGE_ALERT_LED PORTAbits.RA4  // Pin 6

#define PUSHBUTTON       PORTCbits.RC1   // Pin 12

// Relays
#define RELAY_STARTER    PORTCbits.RC2   // Pin 13
#define RELAY_MOTOR      PORTCbits.RC3   // Pin 14 (combines phase and neutral)

// Buzzer
#define BUZZER           PORTCbits.RC4   // Pin 15

#define LOW_SENSOR_PIN   PORTBbits.RB3   // Pin 24
#define HIGH_SENSOR_PIN  PORTBbits.RB4   // Pin 25
#define FLOW_SENSOR_PIN  PORTBbits.RB5   // Pin 26
volatile unsigned long seconds_counter = 0;
bool to = 0;
bool smc = 0;
bool settingsmode = 0;
unsigned int potraw;
unsigned int voltageraw;
void main(void) {
    
    
    
    // Initialize the system
    initSystem();
    
    for(uint8_t i = 0; i < 10; i++) {
        MAINS_LED = 1;
        MOTOR_ON_LED = 1;
        DRY_RUN_LED = 1;
        VOLTAGE_ALERT_LED = 1;
        if( i == 0){
            if(PUSHBUTTON == 0){
                smc = 1;
            }
        }else if (i ==3){
            if(smc == 1){
                if(PUSHBUTTON == 0){
                    settingsmode = 1;
                    i =10;
                }
            }
        }
        __delay_ms(1000);
        MAINS_LED = 0;
        MOTOR_ON_LED = 0;
        DRY_RUN_LED = 0;
        VOLTAGE_ALERT_LED = 0;
        __delay_ms(1000);
    }
    init_timer();
    
    while (settingsmode){
        MAINS_LED = 0;
        __delay_ms(1000);
        MAINS_LED = 1;
        __delay_ms(1000);
    }
    while(1) {
        // Read ADC value from RA0 (AN0)
        potraw = readADC(dryrunpotchannel);
        voltageraw = readADC(voltagechannel);
        if(seconds_counter % 2 == 0) {
            // Even second - turn ON
            PORTAbits.RA1 = 1;
        } else {
            // Odd second - turn OFF
            PORTAbits.RA1 = 0;
        }
    }
}

void initSystem(void) {
    // Configure oscillator
    OSCCONbits.IRCF = 0b110;    // 4 MHz internal oscillator  
    // Configure I/O ports
    ANSEL = 0b00011000;  // AN0 (RA0) as analog input
    ANSELH = 0x00;       // All PORTB pins digital
    // Set RA0 as input, RC0 as output
    TRISA = 0b11101000;
    TRISB = 0b00111000;
    TRISC = 0b00000011;
    
    ADCON0 = 0b00001101;
    ADCON1 = 0b10000000;
    
    __delay_ms(10);
    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
}

unsigned int readADC(uint8_t channel) {
    // Start conversion
    ADCON0 = (ADCON0 & 0b11000011) | ((uint8_t)(channel << 2));
    __delay_us(10);
    ADCON0bits.GO = 1;
    
    // Wait for conversion to complete
    while(ADCON0bits.GO);
    
    // Return result (10-bit)
    return ((unsigned int)ADRESH << 8) | ADRESL;
}

void __interrupt() timer_isr(void)
{
    if(PIR1bits.TMR1IF)
    {
        if(to == 0){
            seconds_counter++;
            to = 1;
        }else{
            to = 0;
        }
        PIR1bits.TMR1IF = 0;  // Clear interrupt flag
        TMR1H = 0x12;  // High byte of 49911
        TMR1L = 0x38;  // Low byte of 49911
        
    }
}

void init_timer(void)
{
    T1CON = 0b00110000;
    
    // Initial preload values
    TMR1H = 0x12;  // High byte of 49911
    TMR1L = 0x38;  // Low byte of 49911
    
    PIE1bits.TMR1IE = 1;  // Enable Timer1 interrupt
    INTCONbits.PEIE = 1;  // Enable peripheral interrupts
    INTCONbits.GIE = 1;   // Enable global interrupts
    
    T1CONbits.TMR1ON = 1; // Start Timer1
}