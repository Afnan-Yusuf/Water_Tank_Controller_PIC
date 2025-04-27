
#include <stdbool.h>
#include <stdint.h>
#include <xc.h>

// Configuration bits
#pragma config FOSC = INTRC_NOCLKOUT // Internal oscillator
#pragma config WDTE = OFF            // Watchdog Timer disabled
#pragma config PWRTE = ON            // Power-up Timer enabled
#pragma config MCLRE = ON            // MCLR pin enabled
#pragma config CP = OFF              // Code protection disabled
#pragma config CPD = OFF             // Data memory protection disabled
#pragma config BOREN = ON            // Brown-out Reset enabled
#pragma config IESO = OFF            // Internal/External Switchover disabled
#pragma config FCMEN = OFF           // Fail-Safe Clock Monitor disabled
#pragma config LVP = OFF             // Low-Voltage Programming disabled

#define _XTAL_FREQ 8000000 // 4 MHz internal oscillator

#define ADDR_SIGNATURE 0x00    // 1 byte signature (0xAA)
#define ADDR_TIMOUT_INDEX 0x01 // 1 byte
#define ADDR_LOWVOLTAGE 0x02   // 2 bytes (0x02-0x03)
#define ADDR_HIGHVOLTAGE 0x04  // 2 bytes (0x04-0x05)
#define ADDR_LOWRUNNINGV 0x06  // 2 bytes (0x06-0x07)
#define ADDR_HIGHRUNNINGV 0x08 // 2 bytes (0x08-0x09)
#define EEPROM_SIGNATURE 0xAA

#define MAINS_LED PORTAbits.RA0         // Pin 2
#define MOTOR_ON_LED PORTAbits.RA1      // Pin 3
#define DRY_RUN_LED PORTAbits.RA2       // Pin 4
#define VOLTAGE_ALERT_LED PORTAbits.RA4 // Pin 6
#define RELAY_STARTER PORTCbits.RC2     // Pin 13
#define RELAY_MOTOR PORTCbits.RC3       // Pin 14 (combines phase and neutral)
#define BUZZER PORTCbits.RC4            // Pin 15

#define voltagechannel 3
#define dryrunpotchannel 4
#define PUSHBUTTON PORTCbits.RC1      // Pin 12
#define LOW_SENSOR_PIN PORTBbits.RB5  // Pin 24
#define HIGH_SENSOR_PIN PORTBbits.RB4 // Pin 25
#define FLOW_SENSOR_PIN PORTBbits.RB3 // Pin 26

// LCD pin definitions
#define LCD_RS PORTCbits.RC5
#define LCD_EN PORTCbits.RC6
#define LCD_D7 PORTCbits.RC7
#define LCD_D4 PORTBbits.RB0
#define LCD_D5 PORTBbits.RB1
#define LCD_D6 PORTBbits.RB2

volatile unsigned long seconds_counter = 0;
volatile unsigned long millis = 0;
unsigned long last_millis = 0;

unsigned long lastdryruncheck = 0;
unsigned long motorstarttime = 0;
unsigned long lastvoltageerror = 0;
unsigned int maxvoltageerrortime = 10;

unsigned int minvoltagelimit = 200;
unsigned int maxvoltagelimit = 230;
unsigned int minimumrunningvoltage = 170;
unsigned int maximumrinningvoltage = 285;

uint8_t maxruntimeindex = 3;
uint16_t maxruntime[5] = {30, 45, 60, 120, 0xFFFF};

bool to = 0;
bool smc = 0;
bool settingsmode = 0;

#define voltagesamples 16
unsigned int potraw;
unsigned int voltageraw;
unsigned int dryruntime = 0;
uint8_t voltage = 0;
uint16_t voltagesum = 0;

volatile uint8_t reading_count = 0;
volatile uint8_t low_sensor_high_count = 0;
volatile uint8_t high_sensor_high_count = 0;
volatile uint8_t flow_sensor_high_count = 0;
volatile bool sensors_reading_complete = false;
volatile bool sensors_reading_in_progress = false;

bool low_sensor_active, high_sensor_active, flow_sensor_active;
bool mannualon = false;
bool voltageerror = false;
bool dryrunerror = false;
bool timeouterror = false;
bool motorrunning = false;
bool tankempty = false;
bool pretankempty = false;
bool flowactive = false;
bool preflowactive = false;
unsigned int sensorbuffer = 10000;
unsigned int lastflowcheck = 0;
unsigned long lastsensorcheck = 0;

unsigned long buzzer_start_time = 0;
unsigned int buzzer_duration = 0;
bool buzzer_active = 0;

unsigned long lt = 0;
unsigned long lastdispupdt = 0;

bool check_button_press(void);
void trigger_buzzer(unsigned int duration_seconds);
void EEPROM_Write(unsigned char address, unsigned char data);
unsigned char EEPROM_Read(unsigned char address);
void EEPROM_Write16(unsigned char address, unsigned int data);
unsigned int EEPROM_Read16(unsigned char address);
bool loadSettings(unsigned char *value8bit, unsigned int *value16bit1,
                  unsigned int *value16bit2, unsigned int *value16bit3,
                  unsigned int *value16bit4);
void saveSettings(unsigned char value8bit, unsigned int value16bit1,
                  unsigned int value16bit2, unsigned int value16bit3,
                  unsigned int value16bit4);
void initSystem(void);
unsigned int readADC(uint8_t channel);
void init_timer(void);
void __interrupt() timer_isr(void);
bool getSensorResults(bool *low_active, bool *high_active, bool *flow_active);
void startSensorReading(void);
void setupTimer0(void);
void buzzer_update();
void getsensorreadings(void);

void initLCD(void);
void lcd_cmd(unsigned char cmd);
void lcd_data(unsigned char data);
void lcd_string(const char *str);
void lcd_set_cursor(unsigned char row, unsigned char col);
void lcd_display_int(int num);
void lcd_display_bool_binary(bool value);
void dispinfo(uint8_t refreshtime);

void main(void) {

  // Initialize the system
  initSystem();

  // EEPROM wasn't properly initialized, set defaults
  // maxruntimeindex = 2; // Default threshold
  minvoltagelimit = 200;
  maxvoltagelimit = 240;
  minimumrunningvoltage = 190;
  maximumrinningvoltage = 250;
  maxruntimeindex = EEPROM_Read(ADDR_TIMOUT_INDEX);

  // Save defaults to EEPROM
  // saveSettings(maxruntimeindex, minvoltagelimit, maxvoltagelimit,
  //              minimumrunningvoltage, maximumrinningvoltage);

  potraw = readADC(dryrunpotchannel);
  // dryruntime = 30000;
  dryruntime = (((uint32_t)potraw * 360) / 1023) + 120;
  ///*
  for (uint8_t i = 0; i < 10; i++) {
    MAINS_LED = 1;
    MOTOR_ON_LED = 1;
    DRY_RUN_LED = 1;
    VOLTAGE_ALERT_LED = 1;
    if (i == 0) {
      if (PUSHBUTTON == 0) {
        smc = 1;
      }
    } else if (i == 3) {
      if (smc == 1) {
        if (PUSHBUTTON == 0) {
          settingsmode = 1;
          i = 10;
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
  //*/
  trigger_buzzer(3000);
  buzzer_update();

  init_timer();
  initLCD();

  while (settingsmode) {
    buzzer_update();

    getsensorreadings();
    potraw = readADC(dryrunpotchannel);
    dryruntime = (((uint32_t)potraw * 360) / 1023) + 120;

    if (PUSHBUTTON == 0) {
      bool is_long_press = check_button_press();

      if (is_long_press) {
        tankempty = true;
        pretankempty = true;
        // trigger_buzzer(1); // Confirmation beep
      } else {
        maxruntimeindex++;
        if (maxruntimeindex >= 5) {
          maxruntimeindex = 0;
        }
        EEPROM_Write(ADDR_TIMOUT_INDEX, maxruntimeindex);
      }
    }
    switch (maxruntimeindex) {
    case 0:
      MAINS_LED = 0;
      MOTOR_ON_LED = 0;
      DRY_RUN_LED = 0;
      VOLTAGE_ALERT_LED = 1;
      break;
    case 1:
      MAINS_LED = 0;
      MOTOR_ON_LED = 0;
      DRY_RUN_LED = 1;
      VOLTAGE_ALERT_LED = 0;
      break;
    case 2:
      MAINS_LED = 0;
      MOTOR_ON_LED = 1;
      DRY_RUN_LED = 0;
      VOLTAGE_ALERT_LED = 0;
      break;
    case 3:
      MAINS_LED = 1;
      MOTOR_ON_LED = 0;
      DRY_RUN_LED = 0;
      VOLTAGE_ALERT_LED = 0;
      break;
    case 4:
      MAINS_LED = 0;
      MOTOR_ON_LED = 1;
      DRY_RUN_LED = 1;
      VOLTAGE_ALERT_LED = 0;
      break;
    }

    dispinfo(100);
  }
  while (1) {

    if (PUSHBUTTON == 0) {
      bool is_long_press = check_button_press();

      if (is_long_press) {
        tankempty = true;
        pretankempty = true;
        // trigger_buzzer(1); // Confirmation beep
      } else {
        DRY_RUN_LED = ~DRY_RUN_LED; // Toggle LED
      }
    }

    buzzer_update();

    getsensorreadings();

    if (tankempty) {
      if (!timeouterror && !voltageerror && !dryrunerror && !motorrunning) {
        // Start the motor
        RELAY_MOTOR = 1;
        motorstarttime = seconds_counter;
        motorrunning = true;
        trigger_buzzer(1000);
      }
      if (motorrunning) {
        if (seconds_counter - motorstarttime >= maxruntime[4]) {
          timeouterror = true;
        }
        if (!flowactive) {
          if (seconds_counter % 2 == 0) {
            // Even second - turn ON
            DRY_RUN_LED = 1;
          } else {
            // Odd second - turn OFF
            DRY_RUN_LED = 0;
          }
          if (lastdryruncheck == 0) {
            lastdryruncheck = seconds_counter;
          } else if (seconds_counter - lastdryruncheck >= dryruntime) {
            dryrunerror = true;
            DRY_RUN_LED = 1;
          }
        } else {
          lastdryruncheck = 0;
          DRY_RUN_LED = 0;
        }

         if (voltage > maximumrinningvoltage || voltage < minimumrunningvoltage) {
          voltageerror = true;
         }
      }
    } else {
      if (motorrunning) {
        motorrunning = false;
        mannualon = false;
        RELAY_MOTOR = 0;
      }
    }
    if (timeouterror) {
      motorrunning = false;
    }
    if (dryrunerror) {
      motorrunning = false;
    }
    
    if (!motorrunning) {
      if (voltage > maxvoltagelimit || voltage < minvoltagelimit) {
        voltageerror = true;
      }
      RELAY_MOTOR = 0;
      MOTOR_ON_LED = 0;
    } else {
      MOTOR_ON_LED = 1;
      RELAY_MOTOR = 1;
      if (millis - lt >= 20000) {
        lt = millis;

        trigger_buzzer(200);
      }
    }



    if (voltageerror) {
      if (lastvoltageerror == 0) {
        lastvoltageerror = seconds_counter;
      } else if (seconds_counter - lastvoltageerror >= maxvoltageerrortime) {
        if (voltage > maxvoltagelimit || voltage < minvoltagelimit) {
        voltageerror = true;
      }else{
        voltageerror = false;

      }
        lastvoltageerror = 0;
      }

      motorrunning = false;
    } else if (lastvoltageerror != 0 &&
               (seconds_counter - lastvoltageerror >= maxvoltageerrortime)) {
      lastvoltageerror = 0;
    }

    dispinfo(100);
  }
}

void initSystem(void) {
  // Configure oscillator
  OSCCONbits.IRCF = 0b111; // 8 MHz
  ANSEL = 0b00011000;
  ANSELH = 0x00;
  TRISA = 0b11101000;
  TRISB = 0b00111000;
  TRISC = 0b00000011;

  ADCON0 = 0b00001101;
  ADCON1 = 0b10000000;

  __delay_ms(10);
  PORTA = 0x00;
  PORTB = 0x00;
  PORTC = 0x00;

  LCD_RS = 0;
  LCD_EN = 0;
  LCD_D4 = 0;
  LCD_D5 = 0;
  LCD_D6 = 0;
  LCD_D7 = 0;

  BUZZER = 0;
}

unsigned int readADC(uint8_t channel) {
  // Start conversion
  ADCON0 = (ADCON0 & 0b11000011) | ((uint8_t)(channel << 2));
  __delay_us(10);
  ADCON0bits.GO = 1;

  // Wait for conversion to complete
  while (ADCON0bits.GO)
    ;

  // Return result (10-bit)
  return ((unsigned int)ADRESH << 8) | ADRESL;
}

void setupTimer0(void) {
  OPTION_REGbits.T0CS = 0; // Use internal instruction cycle clock (FOSC/4)
  OPTION_REGbits.PSA = 0;  // Prescaler is assigned to Timer0
  OPTION_REGbits.PS = 0b011;

  // 4MHz / 4 = 1MHz instruction clock
  // 1MHz / 8 = 125kHz timer increment
  // 2ms ? 125kHz = 250 counts needed
  // 256 - 250 = 6 (preload value for 2ms delay)

  TMR0 = 6;              // Preload Timer0
  INTCONbits.TMR0IE = 1; // Enable Timer0 interrupt
}

void startSensorReading(void) {
  if (!sensors_reading_in_progress) {
    reading_count = 0;
    low_sensor_high_count = 0;
    high_sensor_high_count = 0;
    flow_sensor_high_count = 0;
    sensors_reading_complete = false;
    sensors_reading_in_progress = true;
  }
}

bool getSensorResults(bool *low_active, bool *high_active, bool *flow_active) {
  if (sensors_reading_complete) {
    *low_active = (low_sensor_high_count < 45);
    *high_active = (high_sensor_high_count < 45);
    *flow_active = (flow_sensor_high_count < 45);
    sensors_reading_complete = false;
    return true;
  }
  return false;
}

void __interrupt() isr(void) {
  // Timer1 interrupt handler (your existing code)
  if (PIR1bits.TMR1IF) {
    millis++;
    PIR1bits.TMR1IF = 0; // Clear interrupt flag
    TMR1H = 255;         // Reload Timer1
    TMR1L = 14;
  }

  if (INTCONbits.TMR0IF) {
    // Reload Timer0 for next 2ms interrupt
    TMR0 = 6;
    INTCONbits.TMR0IF = 0;

    // Only process if sensor reading is in progress
    if (sensors_reading_in_progress) {
      // Read sensors
      if (LOW_SENSOR_PIN)
        low_sensor_high_count++;
      if (HIGH_SENSOR_PIN)
        high_sensor_high_count++;
      if (FLOW_SENSOR_PIN)
        flow_sensor_high_count++;

      reading_count++;

      // Check if we've completed 50 readings
      if (reading_count >= 50) {
        sensors_reading_in_progress = false;
        sensors_reading_complete = true;
        INTCONbits.TMR0IE =
            0; // Disable Timer0 interrupt until next reading cycle
      }
    }
  }
}

void init_timer(void) {
  T1CON = 0b00110000;

  TMR1H = 255; // High byte of 49911
  TMR1L = 14;  // Low byte of 49911

  PIE1bits.TMR1IE = 1; // Enable Timer1 interrupt
  INTCONbits.PEIE = 1; // Enable peripheral interrupts
  INTCONbits.GIE = 1;  // Enable global interrupts

  T1CONbits.TMR1ON = 1; // Start Timer1
}

void EEPROM_Write(unsigned char address, unsigned char data) {
  while (EECON1bits.WR)
    ;

  EEADR = address;
  EEDATA = data;

  EECON1bits.EEPGD = 0; // Select EEPROM data memory
  EECON1bits.WREN = 1;  // Enable writes to EEPROM

  INTCONbits.GIE = 0; // Disable interrupts
  EECON2 = 0x55;      // Magic sequence
  EECON2 = 0xAA;
  EECON1bits.WR = 1; // Start write

  while (EECON1bits.WR)
    ;

  EECON1bits.WREN = 0; // Disable writes
  INTCONbits.GIE = 1;  // Enable interrupts if they were enabled before
}

unsigned char EEPROM_Read(unsigned char address) {
  while (EECON1bits.WR)
    ;
  EEADR = address;
  EECON1bits.EEPGD = 0; // Select EEPROM data memory
  EECON1bits.RD = 1;    // Initiate read
  return EEDATA;
}

void EEPROM_Write16(unsigned char address, unsigned int data) {
  EEPROM_Write(address, data & 0xFF);
  EEPROM_Write(address + 1, data >> 8);
}

unsigned int EEPROM_Read16(unsigned char address) {
  unsigned int result;
  result = EEPROM_Read(address);
  result |= ((unsigned int)EEPROM_Read(address + 1)) << 8;
  return result;
}

bool loadSettings(unsigned char *value8bit, unsigned int *value16bit1,
                  unsigned int *value16bit2, unsigned int *value16bit3,
                  unsigned int *value16bit4) {
  if (EEPROM_Read(ADDR_SIGNATURE) != EEPROM_SIGNATURE) {
    return false; // No valid data found
  }
  *value8bit = EEPROM_Read(ADDR_TIMOUT_INDEX);
  *value16bit1 = EEPROM_Read16(ADDR_LOWVOLTAGE);
  *value16bit2 = EEPROM_Read16(ADDR_HIGHVOLTAGE);
  *value16bit3 = EEPROM_Read16(ADDR_LOWRUNNINGV);
  *value16bit4 = EEPROM_Read16(ADDR_HIGHRUNNINGV);
  return true; // Valid data loaded
}

void saveSettings(unsigned char value8bit, unsigned int value16bit1,
                  unsigned int value16bit2, unsigned int value16bit3,
                  unsigned int value16bit4) {
  // Write all values first
  EEPROM_Write(ADDR_TIMOUT_INDEX, value8bit);
  EEPROM_Write16(ADDR_LOWVOLTAGE, value16bit1);
  EEPROM_Write16(ADDR_HIGHVOLTAGE, value16bit2);
  EEPROM_Write16(ADDR_LOWRUNNINGV, value16bit3);
  EEPROM_Write16(ADDR_HIGHRUNNINGV, value16bit4);
  EEPROM_Write(ADDR_SIGNATURE, EEPROM_SIGNATURE);
}

void trigger_buzzer(unsigned int duration_seconds) {
  if (!buzzer_active) {
    BUZZER = 1;
    buzzer_start_time = millis;
    buzzer_duration = duration_seconds;
    buzzer_active = 1;
  }
}

void buzzer_update() {
  if (buzzer_active && (millis - buzzer_start_time >= buzzer_duration)) {
    BUZZER = 0;
    buzzer_active = 0;
    buzzer_duration = 0;
  }
}

bool check_button_press(void) {
  bool long_press = false;

  // Wait for button press
  if (PUSHBUTTON == 0) {
    // Button is pressed, debounce
    __delay_ms(50);

    // Check if still pressed after debounce
    if (PUSHBUTTON == 0) {
      // Count how long the button is held
      unsigned int hold_count = 0;
      unsigned int long_press_threshold = 20; // 3 seconds (300 * 10ms)

      // Wait for button release or timeout
      while (PUSHBUTTON == 0) {
        __delay_ms(10);
        hold_count++;

        // Check for long press (3 seconds)
        if (hold_count >= long_press_threshold) {
          long_press = true;

          // Signal long press detected (e.g., with LED)

          // Wait for button release
          while (PUSHBUTTON == 0) {
            __delay_ms(10);
          }

          // Debounce release
          __delay_ms(50);
          break;
        }
      }

      // If not a long press, it was a short press
      if (!long_press) {
        // Debounce release
        __delay_ms(50);
        return false; // Short press
      }

      return true; // Long press
    }
  }

  return false; // No press
}

void initLCD(void) {
  // Wait for LCD to power up
  __delay_ms(15);

  // Init sequence for 4-bit mode
  // First command (0x3) - sent 3 times
  LCD_RS = 0; // Command mode
  LCD_D7 = 0;
  LCD_D6 = 0;
  LCD_D5 = 1;
  LCD_D4 = 1;
  LCD_EN = 1;
  __delay_us(1);
  LCD_EN = 0;
  __delay_ms(5); // Wait > 4.1ms

  // Second command (0x3) - sent again
  LCD_EN = 1;
  __delay_us(1);
  LCD_EN = 0;
  __delay_ms(1); // Wait > 100us

  // Third command (0x3) - sent again
  LCD_EN = 1;
  __delay_us(1);
  LCD_EN = 0;
  __delay_ms(1); // Wait > 100us

  // Set to 4-bit mode (0x2)
  LCD_D7 = 0;
  LCD_D6 = 0;
  LCD_D5 = 1;
  LCD_D4 = 0;
  LCD_EN = 1;
  __delay_us(1);
  LCD_EN = 0;
  __delay_ms(1); // Wait > 100us

  // Now LCD is in 4-bit mode, we can use lcd_cmd function
  // Function set: 4-bit, 2 lines, 5x8 font
  lcd_cmd(0x28);

  // Display off
  lcd_cmd(0x08);

  // Clear display
  lcd_cmd(0x01);
  __delay_ms(2); // Clear command needs longer delay

  // Entry mode set: Increment cursor, no display shift
  lcd_cmd(0x06);

  // Display on, cursor off, blinking off
  lcd_cmd(0x0C);
}

void lcd_cmd(unsigned char cmd) {
  LCD_RS = 0; // Command mode

  // Send higher nibble
  LCD_D7 = (cmd & 0x80) ? 1 : 0;
  LCD_D6 = (cmd & 0x40) ? 1 : 0;
  LCD_D5 = (cmd & 0x20) ? 1 : 0;
  LCD_D4 = (cmd & 0x10) ? 1 : 0;

  LCD_EN = 1;
  __delay_us(1);
  LCD_EN = 0;
  __delay_us(100);

  // Send lower nibble
  LCD_D7 = (cmd & 0x08) ? 1 : 0;
  LCD_D6 = (cmd & 0x04) ? 1 : 0;
  LCD_D5 = (cmd & 0x02) ? 1 : 0;
  LCD_D4 = (cmd & 0x01) ? 1 : 0;

  LCD_EN = 1;
  __delay_us(1);
  LCD_EN = 0;
  __delay_us(100);
}

void lcd_set_cursor(unsigned char row, unsigned char col) {
  unsigned char address;

  // Calculate DDRAM address based on row and column
  if (row == 0) {
    address = 0x80 + col; // First row starts at 0x80
  } else {
    address = 0xC0 + col; // Second row starts at 0xC0
  }

  lcd_cmd(address);
}

void lcd_data(unsigned char data) {
  LCD_RS = 1; // Data mode

  // Send higher nibble
  LCD_D7 = (data & 0x80) ? 1 : 0;
  LCD_D6 = (data & 0x40) ? 1 : 0;
  LCD_D5 = (data & 0x20) ? 1 : 0;
  LCD_D4 = (data & 0x10) ? 1 : 0;

  LCD_EN = 1;
  __delay_us(1);
  LCD_EN = 0;
  __delay_us(100);

  // Send lower nibble
  LCD_D7 = (data & 0x08) ? 1 : 0;
  LCD_D6 = (data & 0x04) ? 1 : 0;
  LCD_D5 = (data & 0x02) ? 1 : 0;
  LCD_D4 = (data & 0x01) ? 1 : 0;

  LCD_EN = 1;
  __delay_us(1);
  LCD_EN = 0;
  __delay_us(100);
}

void lcd_display_int(int num) {
  // Handle negative numbers if needed
  if (num < 0) {
    lcd_data('-');
    num = -num;
  }

  // Limit to 999 for display simplicity
  if (num > 999)
    num = 999;

  // Display hundreds digit
  lcd_data('0' + (num / 100));

  // Display tens digit
  lcd_data('0' + ((num / 10) % 10));

  // Display ones digit
  lcd_data('0' + (num % 10));
}

void getsensorreadings(void) {
  for (uint8_t i = 0; i < voltagesamples; i++) {
    voltagesum += readADC(voltagechannel);
  }
  voltageraw = voltagesum >> 4; // Average the readings
  voltagesum = 0;
  voltage = (((uint32_t)voltageraw * 235) / 1023) + 85;

  

  if (seconds_counter % 1 == 0 && !sensors_reading_in_progress &&
      !sensors_reading_complete) {
    setupTimer0(); // Set up and enable Timer0 for readings
    startSensorReading();
  }

  if (getSensorResults(&low_sensor_active, &high_sensor_active,
                       &flow_sensor_active)) {

    if (high_sensor_active) {
      // VOLTAGE_ALERT_LED = 1;
      pretankempty = false;
    }
    if (!low_sensor_active && !high_sensor_active) {
      // VOLTAGE_ALERT_LED = 0;
      pretankempty = true;
    }
  }

  if (pretankempty != tankempty) {
    if (lastsensorcheck == 0) {
      lastsensorcheck = millis;
    } else if (millis - lastsensorcheck >= sensorbuffer) {
      lastsensorcheck = 0;
      tankempty = pretankempty;
    }
  } else if (lastsensorcheck != 0 &&
             (millis - lastsensorcheck >= sensorbuffer)) {
    lastsensorcheck = 0;
  }

  if (flow_sensor_active != flowactive) {
    if (lastflowcheck == 0) {
      lastflowcheck = millis;
    } else if (millis - lastflowcheck >= sensorbuffer) {
      lastflowcheck = 0;
      flowactive = flow_sensor_active;
    }
  } else if (lastflowcheck != 0 && (millis - lastflowcheck >= sensorbuffer)) {
    lastflowcheck = 0;
  }
  if ((millis - last_millis) >= 1000) {
    seconds_counter++;
    last_millis = millis;
  }
}

void lcd_display_bool_binary(bool value) {
  if (value) {
    lcd_data('1');
  } else {
    lcd_data('0');
  }
}

void dispinfo(uint8_t refreshtime) {
  if (millis - lastdispupdt >= refreshtime) {
    lastdispupdt = millis;
    // lcd_cmd(0x01);
    lcd_set_cursor(0, 0);
    lcd_display_int(voltage);
    lcd_set_cursor(0, 4);
    lcd_display_int(seconds_counter / 60);
    lcd_set_cursor(0, 8);
    lcd_display_int(dryruntime);
    lcd_set_cursor(0, 12);
    lcd_display_int(maxruntimeindex);
    lcd_set_cursor(1, 0);
    lcd_display_int(EEPROM_Read(ADDR_HIGHVOLTAGE));

    lcd_set_cursor(1, 4);
    lcd_display_bool_binary(low_sensor_active);
    lcd_set_cursor(1, 6);
    lcd_display_bool_binary(high_sensor_active);
    lcd_set_cursor(1, 8);
    lcd_display_bool_binary(flow_sensor_active);
    lcd_set_cursor(1, 10);
    lcd_display_bool_binary(tankempty);
    lcd_display_bool_binary(motorrunning);
    lcd_display_bool_binary(dryrunerror);
    lcd_display_bool_binary(voltageerror);


  }
}