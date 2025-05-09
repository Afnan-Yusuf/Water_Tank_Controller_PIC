# 1 "main.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 285 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "/opt/microchip/xc8/v3.00/pic/include/language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "main.c" 2

# 1 "/opt/microchip/xc8/v3.00/pic/include/c99/stdbool.h" 1 3
# 3 "main.c" 2
# 1 "/opt/microchip/xc8/v3.00/pic/include/c99/stdint.h" 1 3



# 1 "/opt/microchip/xc8/v3.00/pic/include/c99/musl_xc8.h" 1 3
# 5 "/opt/microchip/xc8/v3.00/pic/include/c99/stdint.h" 2 3
# 26 "/opt/microchip/xc8/v3.00/pic/include/c99/stdint.h" 3
# 1 "/opt/microchip/xc8/v3.00/pic/include/c99/bits/alltypes.h" 1 3
# 133 "/opt/microchip/xc8/v3.00/pic/include/c99/bits/alltypes.h" 3
typedef unsigned short uintptr_t;
# 148 "/opt/microchip/xc8/v3.00/pic/include/c99/bits/alltypes.h" 3
typedef short intptr_t;
# 164 "/opt/microchip/xc8/v3.00/pic/include/c99/bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;




typedef __int24 int24_t;




typedef long int32_t;
# 192 "/opt/microchip/xc8/v3.00/pic/include/c99/bits/alltypes.h" 3
typedef int32_t intmax_t;







typedef unsigned char uint8_t;




typedef unsigned short uint16_t;




typedef __uint24 uint24_t;




typedef unsigned long uint32_t;
# 233 "/opt/microchip/xc8/v3.00/pic/include/c99/bits/alltypes.h" 3
typedef uint32_t uintmax_t;
# 27 "/opt/microchip/xc8/v3.00/pic/include/c99/stdint.h" 2 3

typedef int8_t int_fast8_t;




typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;
typedef int24_t int_fast24_t;

typedef int32_t int_least32_t;




typedef uint8_t uint_fast8_t;




typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;
typedef uint24_t uint_fast24_t;

typedef uint32_t uint_least32_t;
# 148 "/opt/microchip/xc8/v3.00/pic/include/c99/stdint.h" 3
# 1 "/opt/microchip/xc8/v3.00/pic/include/c99/bits/stdint.h" 1 3
typedef int16_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint16_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 149 "/opt/microchip/xc8/v3.00/pic/include/c99/stdint.h" 2 3
# 4 "main.c" 2
# 1 "/opt/microchip/xc8/v3.00/pic/include/xc.h" 1 3
# 18 "/opt/microchip/xc8/v3.00/pic/include/xc.h" 3
extern const char __xc8_OPTIM_SPEED;

extern double __fpnormalize(double);


# 1 "/opt/microchip/xc8/v3.00/pic/include/c99/xc8debug.h" 1 3



# 1 "/opt/microchip/xc8/v3.00/pic/include/c99/stdlib.h" 1 3
# 10 "/opt/microchip/xc8/v3.00/pic/include/c99/stdlib.h" 3
# 1 "/opt/microchip/xc8/v3.00/pic/include/c99/features.h" 1 3
# 11 "/opt/microchip/xc8/v3.00/pic/include/c99/stdlib.h" 2 3
# 21 "/opt/microchip/xc8/v3.00/pic/include/c99/stdlib.h" 3
# 1 "/opt/microchip/xc8/v3.00/pic/include/c99/bits/alltypes.h" 1 3
# 24 "/opt/microchip/xc8/v3.00/pic/include/c99/bits/alltypes.h" 3
typedef long int wchar_t;
# 128 "/opt/microchip/xc8/v3.00/pic/include/c99/bits/alltypes.h" 3
typedef unsigned size_t;
# 22 "/opt/microchip/xc8/v3.00/pic/include/c99/stdlib.h" 2 3

int atoi (const char *);
long atol (const char *);



double atof (const char *);


float strtof (const char *restrict, char **restrict);
double strtod (const char *restrict, char **restrict);
long double strtold (const char *restrict, char **restrict);



long strtol (const char *restrict, char **restrict, int);
unsigned long strtoul (const char *restrict, char **restrict, int);





unsigned long __strtoxl(const char * s, char ** endptr, int base, char is_signed);
# 55 "/opt/microchip/xc8/v3.00/pic/include/c99/stdlib.h" 3
int rand (void);
void srand (unsigned);

void *malloc (size_t);
void *calloc (size_t, size_t);
void *realloc (void *, size_t);
void free (void *);

          void abort (void);
int atexit (void (*) (void));
          void exit (int);
          void _Exit (int);

void *bsearch (const void *, const void *, size_t, size_t, int (*)(const void *, const void *));







__attribute__((nonreentrant)) void qsort (void *, size_t, size_t, int (*)(const void *, const void *));

int abs (int);
long labs (long);




typedef struct { int quot, rem; } div_t;
typedef struct { long quot, rem; } ldiv_t;




div_t div (int, int);
ldiv_t ldiv (long, long);




typedef struct { unsigned int quot, rem; } udiv_t;
typedef struct { unsigned long quot, rem; } uldiv_t;
udiv_t udiv (unsigned int, unsigned int);
uldiv_t uldiv (unsigned long, unsigned long);
# 5 "/opt/microchip/xc8/v3.00/pic/include/c99/xc8debug.h" 2 3







#pragma intrinsic(__builtin_software_breakpoint)
extern void __builtin_software_breakpoint(void);
# 24 "/opt/microchip/xc8/v3.00/pic/include/xc.h" 2 3
# 1 "/opt/microchip/xc8/v3.00/pic/include/builtins.h" 1 3






#pragma intrinsic(__nop)
extern void __nop(void);
# 19 "/opt/microchip/xc8/v3.00/pic/include/builtins.h" 3
#pragma intrinsic(_delay)
extern __attribute__((nonreentrant)) void _delay(uint32_t);
#pragma intrinsic(_delaywdt)
extern __attribute__((nonreentrant)) void _delaywdt(uint32_t);
# 25 "/opt/microchip/xc8/v3.00/pic/include/xc.h" 2 3



# 1 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/pic.h" 1 3




# 1 "/opt/microchip/xc8/v3.00/pic/include/htc.h" 1 3






# 1 "/opt/microchip/xc8/v3.00/pic/include/xc.h" 1 3
# 8 "/opt/microchip/xc8/v3.00/pic/include/htc.h" 2 3
# 6 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/pic.h" 2 3







# 1 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/pic_chip_select.h" 1 3
# 693 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/pic_chip_select.h" 3
# 1 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 1 3
# 44 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
# 1 "/opt/microchip/xc8/v3.00/pic/include/__at.h" 1 3
# 45 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 2 3







extern volatile unsigned char INDF __attribute__((address(0x000)));

__asm("INDF equ 00h");




extern volatile unsigned char TMR0 __attribute__((address(0x001)));

__asm("TMR0 equ 01h");




extern volatile unsigned char PCL __attribute__((address(0x002)));

__asm("PCL equ 02h");




extern volatile unsigned char STATUS __attribute__((address(0x003)));

__asm("STATUS equ 03h");


typedef union {
    struct {
        unsigned C :1;
        unsigned DC :1;
        unsigned Z :1;
        unsigned nPD :1;
        unsigned nTO :1;
        unsigned RP :2;
        unsigned IRP :1;
    };
    struct {
        unsigned :5;
        unsigned RP0 :1;
        unsigned RP1 :1;
    };
    struct {
        unsigned CARRY :1;
        unsigned :1;
        unsigned ZERO :1;
    };
} STATUSbits_t;
extern volatile STATUSbits_t STATUSbits __attribute__((address(0x003)));
# 159 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char FSR __attribute__((address(0x004)));

__asm("FSR equ 04h");




extern volatile unsigned char PORTA __attribute__((address(0x005)));

__asm("PORTA equ 05h");


typedef union {
    struct {
        unsigned RA0 :1;
        unsigned RA1 :1;
        unsigned RA2 :1;
        unsigned RA3 :1;
        unsigned RA4 :1;
        unsigned RA5 :1;
        unsigned RA6 :1;
        unsigned RA7 :1;
    };
} PORTAbits_t;
extern volatile PORTAbits_t PORTAbits __attribute__((address(0x005)));
# 228 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char PORTB __attribute__((address(0x006)));

__asm("PORTB equ 06h");


typedef union {
    struct {
        unsigned RB0 :1;
        unsigned RB1 :1;
        unsigned RB2 :1;
        unsigned RB3 :1;
        unsigned RB4 :1;
        unsigned RB5 :1;
        unsigned RB6 :1;
        unsigned RB7 :1;
    };
} PORTBbits_t;
extern volatile PORTBbits_t PORTBbits __attribute__((address(0x006)));
# 290 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char PORTC __attribute__((address(0x007)));

__asm("PORTC equ 07h");


typedef union {
    struct {
        unsigned RC0 :1;
        unsigned RC1 :1;
        unsigned RC2 :1;
        unsigned RC3 :1;
        unsigned RC4 :1;
        unsigned RC5 :1;
        unsigned RC6 :1;
        unsigned RC7 :1;
    };
} PORTCbits_t;
extern volatile PORTCbits_t PORTCbits __attribute__((address(0x007)));
# 352 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char PORTE __attribute__((address(0x009)));

__asm("PORTE equ 09h");


typedef union {
    struct {
        unsigned :3;
        unsigned RE3 :1;
    };
} PORTEbits_t;
extern volatile PORTEbits_t PORTEbits __attribute__((address(0x009)));
# 373 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char PCLATH __attribute__((address(0x00A)));

__asm("PCLATH equ 0Ah");




extern volatile unsigned char INTCON __attribute__((address(0x00B)));

__asm("INTCON equ 0Bh");


typedef union {
    struct {
        unsigned RBIF :1;
        unsigned INTF :1;
        unsigned T0IF :1;
        unsigned RBIE :1;
        unsigned INTE :1;
        unsigned T0IE :1;
        unsigned PEIE :1;
        unsigned GIE :1;
    };
    struct {
        unsigned :2;
        unsigned TMR0IF :1;
        unsigned :2;
        unsigned TMR0IE :1;
    };
} INTCONbits_t;
extern volatile INTCONbits_t INTCONbits __attribute__((address(0x00B)));
# 458 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char PIR1 __attribute__((address(0x00C)));

__asm("PIR1 equ 0Ch");


typedef union {
    struct {
        unsigned TMR1IF :1;
        unsigned TMR2IF :1;
        unsigned CCP1IF :1;
        unsigned SSPIF :1;
        unsigned TXIF :1;
        unsigned RCIF :1;
        unsigned ADIF :1;
    };
} PIR1bits_t;
extern volatile PIR1bits_t PIR1bits __attribute__((address(0x00C)));
# 514 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char PIR2 __attribute__((address(0x00D)));

__asm("PIR2 equ 0Dh");


typedef union {
    struct {
        unsigned CCP2IF :1;
        unsigned :1;
        unsigned ULPWUIF :1;
        unsigned BCLIF :1;
        unsigned EEIF :1;
        unsigned C1IF :1;
        unsigned C2IF :1;
        unsigned OSFIF :1;
    };
} PIR2bits_t;
extern volatile PIR2bits_t PIR2bits __attribute__((address(0x00D)));
# 571 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned short TMR1 __attribute__((address(0x00E)));

__asm("TMR1 equ 0Eh");




extern volatile unsigned char TMR1L __attribute__((address(0x00E)));

__asm("TMR1L equ 0Eh");




extern volatile unsigned char TMR1H __attribute__((address(0x00F)));

__asm("TMR1H equ 0Fh");




extern volatile unsigned char T1CON __attribute__((address(0x010)));

__asm("T1CON equ 010h");


typedef union {
    struct {
        unsigned TMR1ON :1;
        unsigned TMR1CS :1;
        unsigned nT1SYNC :1;
        unsigned T1OSCEN :1;
        unsigned T1CKPS :2;
        unsigned TMR1GE :1;
        unsigned T1GINV :1;
    };
    struct {
        unsigned :2;
        unsigned T1INSYNC :1;
        unsigned :1;
        unsigned T1CKPS0 :1;
        unsigned T1CKPS1 :1;
        unsigned :1;
        unsigned T1GIV :1;
    };
    struct {
        unsigned :2;
        unsigned T1SYNC :1;
    };
} T1CONbits_t;
extern volatile T1CONbits_t T1CONbits __attribute__((address(0x010)));
# 686 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char TMR2 __attribute__((address(0x011)));

__asm("TMR2 equ 011h");




extern volatile unsigned char T2CON __attribute__((address(0x012)));

__asm("T2CON equ 012h");


typedef union {
    struct {
        unsigned T2CKPS :2;
        unsigned TMR2ON :1;
        unsigned TOUTPS :4;
    };
    struct {
        unsigned T2CKPS0 :1;
        unsigned T2CKPS1 :1;
        unsigned :1;
        unsigned TOUTPS0 :1;
        unsigned TOUTPS1 :1;
        unsigned TOUTPS2 :1;
        unsigned TOUTPS3 :1;
    };
} T2CONbits_t;
extern volatile T2CONbits_t T2CONbits __attribute__((address(0x012)));
# 764 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char SSPBUF __attribute__((address(0x013)));

__asm("SSPBUF equ 013h");




extern volatile unsigned char SSPCON __attribute__((address(0x014)));

__asm("SSPCON equ 014h");


typedef union {
    struct {
        unsigned SSPM :4;
        unsigned CKP :1;
        unsigned SSPEN :1;
        unsigned SSPOV :1;
        unsigned WCOL :1;
    };
    struct {
        unsigned SSPM0 :1;
        unsigned SSPM1 :1;
        unsigned SSPM2 :1;
        unsigned SSPM3 :1;
    };
} SSPCONbits_t;
extern volatile SSPCONbits_t SSPCONbits __attribute__((address(0x014)));
# 841 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned short CCPR1 __attribute__((address(0x015)));

__asm("CCPR1 equ 015h");




extern volatile unsigned char CCPR1L __attribute__((address(0x015)));

__asm("CCPR1L equ 015h");




extern volatile unsigned char CCPR1H __attribute__((address(0x016)));

__asm("CCPR1H equ 016h");




extern volatile unsigned char CCP1CON __attribute__((address(0x017)));

__asm("CCP1CON equ 017h");


typedef union {
    struct {
        unsigned CCP1M :4;
        unsigned DC1B :2;
        unsigned P1M :2;
    };
    struct {
        unsigned CCP1M0 :1;
        unsigned CCP1M1 :1;
        unsigned CCP1M2 :1;
        unsigned CCP1M3 :1;
        unsigned DC1B0 :1;
        unsigned DC1B1 :1;
        unsigned P1M0 :1;
        unsigned P1M1 :1;
    };
    struct {
        unsigned :4;
        unsigned CCP1Y :1;
        unsigned CCP1X :1;
    };
} CCP1CONbits_t;
extern volatile CCP1CONbits_t CCP1CONbits __attribute__((address(0x017)));
# 959 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char RCSTA __attribute__((address(0x018)));

__asm("RCSTA equ 018h");


typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
    struct {
        unsigned RCD8 :1;
        unsigned :5;
        unsigned RC9 :1;
    };
    struct {
        unsigned :6;
        unsigned nRC8 :1;
    };
    struct {
        unsigned :6;
        unsigned RC8_9 :1;
    };
} RCSTAbits_t;
extern volatile RCSTAbits_t RCSTAbits __attribute__((address(0x018)));
# 1054 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char TXREG __attribute__((address(0x019)));

__asm("TXREG equ 019h");




extern volatile unsigned char RCREG __attribute__((address(0x01A)));

__asm("RCREG equ 01Ah");




extern volatile unsigned short CCPR2 __attribute__((address(0x01B)));

__asm("CCPR2 equ 01Bh");




extern volatile unsigned char CCPR2L __attribute__((address(0x01B)));

__asm("CCPR2L equ 01Bh");




extern volatile unsigned char CCPR2H __attribute__((address(0x01C)));

__asm("CCPR2H equ 01Ch");




extern volatile unsigned char CCP2CON __attribute__((address(0x01D)));

__asm("CCP2CON equ 01Dh");


typedef union {
    struct {
        unsigned CCP2M :4;
        unsigned DC2B0 :1;
        unsigned DC2B1 :1;
    };
    struct {
        unsigned CCP2M0 :1;
        unsigned CCP2M1 :1;
        unsigned CCP2M2 :1;
        unsigned CCP2M3 :1;
        unsigned CCP2Y :1;
        unsigned CCP2X :1;
    };
} CCP2CONbits_t;
extern volatile CCP2CONbits_t CCP2CONbits __attribute__((address(0x01D)));
# 1159 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char ADRESH __attribute__((address(0x01E)));

__asm("ADRESH equ 01Eh");




extern volatile unsigned char ADCON0 __attribute__((address(0x01F)));

__asm("ADCON0 equ 01Fh");


typedef union {
    struct {
        unsigned ADON :1;
        unsigned GO_nDONE :1;
        unsigned CHS :4;
        unsigned ADCS :2;
    };
    struct {
        unsigned :1;
        unsigned GO :1;
        unsigned CHS0 :1;
        unsigned CHS1 :1;
        unsigned CHS2 :1;
        unsigned CHS3 :1;
        unsigned ADCS0 :1;
        unsigned ADCS1 :1;
    };
    struct {
        unsigned :1;
        unsigned nDONE :1;
    };
    struct {
        unsigned :1;
        unsigned GO_DONE :1;
    };
} ADCON0bits_t;
extern volatile ADCON0bits_t ADCON0bits __attribute__((address(0x01F)));
# 1267 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char OPTION_REG __attribute__((address(0x081)));

__asm("OPTION_REG equ 081h");


typedef union {
    struct {
        unsigned PS :3;
        unsigned PSA :1;
        unsigned T0SE :1;
        unsigned T0CS :1;
        unsigned INTEDG :1;
        unsigned nRBPU :1;
    };
    struct {
        unsigned PS0 :1;
        unsigned PS1 :1;
        unsigned PS2 :1;
    };
} OPTION_REGbits_t;
extern volatile OPTION_REGbits_t OPTION_REGbits __attribute__((address(0x081)));
# 1337 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char TRISA __attribute__((address(0x085)));

__asm("TRISA equ 085h");


typedef union {
    struct {
        unsigned TRISA0 :1;
        unsigned TRISA1 :1;
        unsigned TRISA2 :1;
        unsigned TRISA3 :1;
        unsigned TRISA4 :1;
        unsigned TRISA5 :1;
        unsigned TRISA6 :1;
        unsigned TRISA7 :1;
    };
} TRISAbits_t;
extern volatile TRISAbits_t TRISAbits __attribute__((address(0x085)));
# 1399 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char TRISB __attribute__((address(0x086)));

__asm("TRISB equ 086h");


typedef union {
    struct {
        unsigned TRISB0 :1;
        unsigned TRISB1 :1;
        unsigned TRISB2 :1;
        unsigned TRISB3 :1;
        unsigned TRISB4 :1;
        unsigned TRISB5 :1;
        unsigned TRISB6 :1;
        unsigned TRISB7 :1;
    };
} TRISBbits_t;
extern volatile TRISBbits_t TRISBbits __attribute__((address(0x086)));
# 1461 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char TRISC __attribute__((address(0x087)));

__asm("TRISC equ 087h");


typedef union {
    struct {
        unsigned TRISC0 :1;
        unsigned TRISC1 :1;
        unsigned TRISC2 :1;
        unsigned TRISC3 :1;
        unsigned TRISC4 :1;
        unsigned TRISC5 :1;
        unsigned TRISC6 :1;
        unsigned TRISC7 :1;
    };
} TRISCbits_t;
extern volatile TRISCbits_t TRISCbits __attribute__((address(0x087)));
# 1523 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char TRISE __attribute__((address(0x089)));

__asm("TRISE equ 089h");


typedef union {
    struct {
        unsigned :3;
        unsigned TRISE3 :1;
    };
} TRISEbits_t;
extern volatile TRISEbits_t TRISEbits __attribute__((address(0x089)));
# 1544 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char PIE1 __attribute__((address(0x08C)));

__asm("PIE1 equ 08Ch");


typedef union {
    struct {
        unsigned TMR1IE :1;
        unsigned TMR2IE :1;
        unsigned CCP1IE :1;
        unsigned SSPIE :1;
        unsigned TXIE :1;
        unsigned RCIE :1;
        unsigned ADIE :1;
    };
} PIE1bits_t;
extern volatile PIE1bits_t PIE1bits __attribute__((address(0x08C)));
# 1600 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char PIE2 __attribute__((address(0x08D)));

__asm("PIE2 equ 08Dh");


typedef union {
    struct {
        unsigned CCP2IE :1;
        unsigned :1;
        unsigned ULPWUIE :1;
        unsigned BCLIE :1;
        unsigned EEIE :1;
        unsigned C1IE :1;
        unsigned C2IE :1;
        unsigned OSFIE :1;
    };
} PIE2bits_t;
extern volatile PIE2bits_t PIE2bits __attribute__((address(0x08D)));
# 1657 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char PCON __attribute__((address(0x08E)));

__asm("PCON equ 08Eh");


typedef union {
    struct {
        unsigned nBOR :1;
        unsigned nPOR :1;
        unsigned :2;
        unsigned SBOREN :1;
        unsigned ULPWUE :1;
    };
    struct {
        unsigned nBO :1;
    };
} PCONbits_t;
extern volatile PCONbits_t PCONbits __attribute__((address(0x08E)));
# 1704 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char OSCCON __attribute__((address(0x08F)));

__asm("OSCCON equ 08Fh");


typedef union {
    struct {
        unsigned SCS :1;
        unsigned LTS :1;
        unsigned HTS :1;
        unsigned OSTS :1;
        unsigned IRCF :3;
    };
    struct {
        unsigned :4;
        unsigned IRCF0 :1;
        unsigned IRCF1 :1;
        unsigned IRCF2 :1;
    };
} OSCCONbits_t;
extern volatile OSCCONbits_t OSCCONbits __attribute__((address(0x08F)));
# 1769 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char OSCTUNE __attribute__((address(0x090)));

__asm("OSCTUNE equ 090h");


typedef union {
    struct {
        unsigned TUN :5;
    };
    struct {
        unsigned TUN0 :1;
        unsigned TUN1 :1;
        unsigned TUN2 :1;
        unsigned TUN3 :1;
        unsigned TUN4 :1;
    };
} OSCTUNEbits_t;
extern volatile OSCTUNEbits_t OSCTUNEbits __attribute__((address(0x090)));
# 1821 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char SSPCON2 __attribute__((address(0x091)));

__asm("SSPCON2 equ 091h");


typedef union {
    struct {
        unsigned SEN :1;
        unsigned RSEN :1;
        unsigned PEN :1;
        unsigned RCEN :1;
        unsigned ACKEN :1;
        unsigned ACKDT :1;
        unsigned ACKSTAT :1;
        unsigned GCEN :1;
    };
} SSPCON2bits_t;
extern volatile SSPCON2bits_t SSPCON2bits __attribute__((address(0x091)));
# 1883 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char PR2 __attribute__((address(0x092)));

__asm("PR2 equ 092h");




extern volatile unsigned char SSPADD __attribute__((address(0x093)));

__asm("SSPADD equ 093h");




extern volatile unsigned char SSPMSK __attribute__((address(0x093)));

__asm("SSPMSK equ 093h");


extern volatile unsigned char MSK __attribute__((address(0x093)));

__asm("MSK equ 093h");


typedef union {
    struct {
        unsigned MSK0 :1;
        unsigned MSK1 :1;
        unsigned MSK2 :1;
        unsigned MSK3 :1;
        unsigned MSK4 :1;
        unsigned MSK5 :1;
        unsigned MSK6 :1;
        unsigned MSK7 :1;
    };
} SSPMSKbits_t;
extern volatile SSPMSKbits_t SSPMSKbits __attribute__((address(0x093)));
# 1962 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
typedef union {
    struct {
        unsigned MSK0 :1;
        unsigned MSK1 :1;
        unsigned MSK2 :1;
        unsigned MSK3 :1;
        unsigned MSK4 :1;
        unsigned MSK5 :1;
        unsigned MSK6 :1;
        unsigned MSK7 :1;
    };
} MSKbits_t;
extern volatile MSKbits_t MSKbits __attribute__((address(0x093)));
# 2019 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char SSPSTAT __attribute__((address(0x094)));

__asm("SSPSTAT equ 094h");


typedef union {
    struct {
        unsigned BF :1;
        unsigned UA :1;
        unsigned R_nW :1;
        unsigned S :1;
        unsigned P :1;
        unsigned D_nA :1;
        unsigned CKE :1;
        unsigned SMP :1;
    };
    struct {
        unsigned :2;
        unsigned R :1;
        unsigned :2;
        unsigned D :1;
    };
    struct {
        unsigned :2;
        unsigned I2C_READ :1;
        unsigned I2C_START :1;
        unsigned I2C_STOP :1;
        unsigned I2C_DATA :1;
    };
    struct {
        unsigned :2;
        unsigned nW :1;
        unsigned :2;
        unsigned nA :1;
    };
    struct {
        unsigned :2;
        unsigned nWRITE :1;
        unsigned :2;
        unsigned nADDRESS :1;
    };
    struct {
        unsigned :2;
        unsigned R_W :1;
        unsigned :2;
        unsigned D_A :1;
    };
    struct {
        unsigned :2;
        unsigned READ_WRITE :1;
        unsigned :2;
        unsigned DATA_ADDRESS :1;
    };
} SSPSTATbits_t;
extern volatile SSPSTATbits_t SSPSTATbits __attribute__((address(0x094)));
# 2188 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char WPUB __attribute__((address(0x095)));

__asm("WPUB equ 095h");


typedef union {
    struct {
        unsigned WPUB :8;
    };
    struct {
        unsigned WPUB0 :1;
        unsigned WPUB1 :1;
        unsigned WPUB2 :1;
        unsigned WPUB3 :1;
        unsigned WPUB4 :1;
        unsigned WPUB5 :1;
        unsigned WPUB6 :1;
        unsigned WPUB7 :1;
    };
} WPUBbits_t;
extern volatile WPUBbits_t WPUBbits __attribute__((address(0x095)));
# 2258 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char IOCB __attribute__((address(0x096)));

__asm("IOCB equ 096h");


typedef union {
    struct {
        unsigned IOCB :8;
    };
    struct {
        unsigned IOCB0 :1;
        unsigned IOCB1 :1;
        unsigned IOCB2 :1;
        unsigned IOCB3 :1;
        unsigned IOCB4 :1;
        unsigned IOCB5 :1;
        unsigned IOCB6 :1;
        unsigned IOCB7 :1;
    };
} IOCBbits_t;
extern volatile IOCBbits_t IOCBbits __attribute__((address(0x096)));
# 2328 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char VRCON __attribute__((address(0x097)));

__asm("VRCON equ 097h");


typedef union {
    struct {
        unsigned VR :4;
        unsigned VRSS :1;
        unsigned VRR :1;
        unsigned VROE :1;
        unsigned VREN :1;
    };
    struct {
        unsigned VR0 :1;
        unsigned VR1 :1;
        unsigned VR2 :1;
        unsigned VR3 :1;
    };
} VRCONbits_t;
extern volatile VRCONbits_t VRCONbits __attribute__((address(0x097)));
# 2398 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char TXSTA __attribute__((address(0x098)));

__asm("TXSTA equ 098h");


typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned SENDB :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
    struct {
        unsigned TXD8 :1;
        unsigned :5;
        unsigned nTX8 :1;
    };
    struct {
        unsigned :6;
        unsigned TX8_9 :1;
    };
} TXSTAbits_t;
extern volatile TXSTAbits_t TXSTAbits __attribute__((address(0x098)));
# 2484 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char SPBRG __attribute__((address(0x099)));

__asm("SPBRG equ 099h");


typedef union {
    struct {
        unsigned BRG0 :1;
        unsigned BRG1 :1;
        unsigned BRG2 :1;
        unsigned BRG3 :1;
        unsigned BRG4 :1;
        unsigned BRG5 :1;
        unsigned BRG6 :1;
        unsigned BRG7 :1;
    };
} SPBRGbits_t;
extern volatile SPBRGbits_t SPBRGbits __attribute__((address(0x099)));
# 2546 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char SPBRGH __attribute__((address(0x09A)));

__asm("SPBRGH equ 09Ah");


typedef union {
    struct {
        unsigned SPBRGH :8;
    };
    struct {
        unsigned BRG8 :1;
        unsigned BRG9 :1;
        unsigned BRG10 :1;
        unsigned BRG11 :1;
        unsigned BRG12 :1;
        unsigned BRG13 :1;
        unsigned BRG14 :1;
        unsigned BRG15 :1;
    };
} SPBRGHbits_t;
extern volatile SPBRGHbits_t SPBRGHbits __attribute__((address(0x09A)));
# 2616 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char PWM1CON __attribute__((address(0x09B)));

__asm("PWM1CON equ 09Bh");


typedef union {
    struct {
        unsigned PDC :7;
        unsigned PRSEN :1;
    };
    struct {
        unsigned PDC0 :1;
        unsigned PDC1 :1;
        unsigned PDC2 :1;
        unsigned PDC3 :1;
        unsigned PDC4 :1;
        unsigned PDC5 :1;
        unsigned PDC6 :1;
    };
} PWM1CONbits_t;
extern volatile PWM1CONbits_t PWM1CONbits __attribute__((address(0x09B)));
# 2686 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char ECCPAS __attribute__((address(0x09C)));

__asm("ECCPAS equ 09Ch");


typedef union {
    struct {
        unsigned PSSBD :2;
        unsigned PSSAC :2;
        unsigned ECCPAS :3;
        unsigned ECCPASE :1;
    };
    struct {
        unsigned PSSBD0 :1;
        unsigned PSSBD1 :1;
        unsigned PSSAC0 :1;
        unsigned PSSAC1 :1;
        unsigned ECCPAS0 :1;
        unsigned ECCPAS1 :1;
        unsigned ECCPAS2 :1;
    };
} ECCPASbits_t;
extern volatile ECCPASbits_t ECCPASbits __attribute__((address(0x09C)));
# 2768 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char PSTRCON __attribute__((address(0x09D)));

__asm("PSTRCON equ 09Dh");


typedef union {
    struct {
        unsigned STRA :1;
        unsigned STRB :1;
        unsigned STRC :1;
        unsigned STRD :1;
        unsigned STRSYNC :1;
    };
} PSTRCONbits_t;
extern volatile PSTRCONbits_t PSTRCONbits __attribute__((address(0x09D)));
# 2812 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char ADRESL __attribute__((address(0x09E)));

__asm("ADRESL equ 09Eh");




extern volatile unsigned char ADCON1 __attribute__((address(0x09F)));

__asm("ADCON1 equ 09Fh");


typedef union {
    struct {
        unsigned :4;
        unsigned VCFG0 :1;
        unsigned VCFG1 :1;
        unsigned :1;
        unsigned ADFM :1;
    };
} ADCON1bits_t;
extern volatile ADCON1bits_t ADCON1bits __attribute__((address(0x09F)));
# 2853 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char WDTCON __attribute__((address(0x105)));

__asm("WDTCON equ 0105h");


typedef union {
    struct {
        unsigned SWDTEN :1;
        unsigned WDTPS :4;
    };
    struct {
        unsigned :1;
        unsigned WDTPS0 :1;
        unsigned WDTPS1 :1;
        unsigned WDTPS2 :1;
        unsigned WDTPS3 :1;
    };
} WDTCONbits_t;
extern volatile WDTCONbits_t WDTCONbits __attribute__((address(0x105)));
# 2906 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char CM1CON0 __attribute__((address(0x107)));

__asm("CM1CON0 equ 0107h");


typedef union {
    struct {
        unsigned C1CH :2;
        unsigned C1R :1;
        unsigned :1;
        unsigned C1POL :1;
        unsigned C1OE :1;
        unsigned C1OUT :1;
        unsigned C1ON :1;
    };
    struct {
        unsigned C1CH0 :1;
        unsigned C1CH1 :1;
    };
} CM1CON0bits_t;
extern volatile CM1CON0bits_t CM1CON0bits __attribute__((address(0x107)));
# 2971 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char CM2CON0 __attribute__((address(0x108)));

__asm("CM2CON0 equ 0108h");


typedef union {
    struct {
        unsigned C2CH :2;
        unsigned C2R :1;
        unsigned :1;
        unsigned C2POL :1;
        unsigned C2OE :1;
        unsigned C2OUT :1;
        unsigned C2ON :1;
    };
    struct {
        unsigned C2CH0 :1;
        unsigned C2CH1 :1;
    };
} CM2CON0bits_t;
extern volatile CM2CON0bits_t CM2CON0bits __attribute__((address(0x108)));
# 3036 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char CM2CON1 __attribute__((address(0x109)));

__asm("CM2CON1 equ 0109h");


typedef union {
    struct {
        unsigned C2SYNC :1;
        unsigned T1GSS :1;
        unsigned :2;
        unsigned C2RSEL :1;
        unsigned C1RSEL :1;
        unsigned MC2OUT :1;
        unsigned MC1OUT :1;
    };
} CM2CON1bits_t;
extern volatile CM2CON1bits_t CM2CON1bits __attribute__((address(0x109)));
# 3087 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char EEDATA __attribute__((address(0x10C)));

__asm("EEDATA equ 010Ch");


extern volatile unsigned char EEDAT __attribute__((address(0x10C)));

__asm("EEDAT equ 010Ch");




extern volatile unsigned char EEADR __attribute__((address(0x10D)));

__asm("EEADR equ 010Dh");




extern volatile unsigned char EEDATH __attribute__((address(0x10E)));

__asm("EEDATH equ 010Eh");




extern volatile unsigned char EEADRH __attribute__((address(0x10F)));

__asm("EEADRH equ 010Fh");




extern volatile unsigned char SRCON __attribute__((address(0x185)));

__asm("SRCON equ 0185h");


typedef union {
    struct {
        unsigned FVREN :1;
        unsigned :1;
        unsigned PULSR :1;
        unsigned PULSS :1;
        unsigned C2REN :1;
        unsigned C1SEN :1;
        unsigned SR0 :1;
        unsigned SR1 :1;
    };
} SRCONbits_t;
extern volatile SRCONbits_t SRCONbits __attribute__((address(0x185)));
# 3177 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char BAUDCTL __attribute__((address(0x187)));

__asm("BAUDCTL equ 0187h");


typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned SCKP :1;
        unsigned :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
} BAUDCTLbits_t;
extern volatile BAUDCTLbits_t BAUDCTLbits __attribute__((address(0x187)));
# 3229 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char ANSEL __attribute__((address(0x188)));

__asm("ANSEL equ 0188h");


typedef union {
    struct {
        unsigned ANS0 :1;
        unsigned ANS1 :1;
        unsigned ANS2 :1;
        unsigned ANS3 :1;
        unsigned ANS4 :1;
    };
} ANSELbits_t;
extern volatile ANSELbits_t ANSELbits __attribute__((address(0x188)));
# 3273 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char ANSELH __attribute__((address(0x189)));

__asm("ANSELH equ 0189h");


typedef union {
    struct {
        unsigned ANS8 :1;
        unsigned ANS9 :1;
        unsigned ANS10 :1;
        unsigned ANS11 :1;
        unsigned ANS12 :1;
        unsigned ANS13 :1;
    };
} ANSELHbits_t;
extern volatile ANSELHbits_t ANSELHbits __attribute__((address(0x189)));
# 3323 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char EECON1 __attribute__((address(0x18C)));

__asm("EECON1 equ 018Ch");


typedef union {
    struct {
        unsigned RD :1;
        unsigned WR :1;
        unsigned WREN :1;
        unsigned WRERR :1;
        unsigned :3;
        unsigned EEPGD :1;
    };
} EECON1bits_t;
extern volatile EECON1bits_t EECON1bits __attribute__((address(0x18C)));
# 3368 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile unsigned char EECON2 __attribute__((address(0x18D)));

__asm("EECON2 equ 018Dh");
# 3385 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/proc/pic16f883.h" 3
extern volatile __bit ABDEN __attribute__((address(0xC38)));


extern volatile __bit ABDOVF __attribute__((address(0xC3F)));


extern volatile __bit ACKDT __attribute__((address(0x48D)));


extern volatile __bit ACKEN __attribute__((address(0x48C)));


extern volatile __bit ACKSTAT __attribute__((address(0x48E)));


extern volatile __bit ADCS0 __attribute__((address(0xFE)));


extern volatile __bit ADCS1 __attribute__((address(0xFF)));


extern volatile __bit ADDEN __attribute__((address(0xC3)));


extern volatile __bit ADFM __attribute__((address(0x4FF)));


extern volatile __bit ADIE __attribute__((address(0x466)));


extern volatile __bit ADIF __attribute__((address(0x66)));


extern volatile __bit ADON __attribute__((address(0xF8)));


extern volatile __bit ANS0 __attribute__((address(0xC40)));


extern volatile __bit ANS1 __attribute__((address(0xC41)));


extern volatile __bit ANS10 __attribute__((address(0xC4A)));


extern volatile __bit ANS11 __attribute__((address(0xC4B)));


extern volatile __bit ANS12 __attribute__((address(0xC4C)));


extern volatile __bit ANS13 __attribute__((address(0xC4D)));


extern volatile __bit ANS2 __attribute__((address(0xC42)));


extern volatile __bit ANS3 __attribute__((address(0xC43)));


extern volatile __bit ANS4 __attribute__((address(0xC44)));


extern volatile __bit ANS8 __attribute__((address(0xC48)));


extern volatile __bit ANS9 __attribute__((address(0xC49)));


extern volatile __bit BCLIE __attribute__((address(0x46B)));


extern volatile __bit BCLIF __attribute__((address(0x6B)));


extern volatile __bit BF __attribute__((address(0x4A0)));


extern volatile __bit BRG0 __attribute__((address(0x4C8)));


extern volatile __bit BRG1 __attribute__((address(0x4C9)));


extern volatile __bit BRG10 __attribute__((address(0x4D2)));


extern volatile __bit BRG11 __attribute__((address(0x4D3)));


extern volatile __bit BRG12 __attribute__((address(0x4D4)));


extern volatile __bit BRG13 __attribute__((address(0x4D5)));


extern volatile __bit BRG14 __attribute__((address(0x4D6)));


extern volatile __bit BRG15 __attribute__((address(0x4D7)));


extern volatile __bit BRG16 __attribute__((address(0xC3B)));


extern volatile __bit BRG2 __attribute__((address(0x4CA)));


extern volatile __bit BRG3 __attribute__((address(0x4CB)));


extern volatile __bit BRG4 __attribute__((address(0x4CC)));


extern volatile __bit BRG5 __attribute__((address(0x4CD)));


extern volatile __bit BRG6 __attribute__((address(0x4CE)));


extern volatile __bit BRG7 __attribute__((address(0x4CF)));


extern volatile __bit BRG8 __attribute__((address(0x4D0)));


extern volatile __bit BRG9 __attribute__((address(0x4D1)));


extern volatile __bit BRGH __attribute__((address(0x4C2)));


extern volatile __bit C1CH0 __attribute__((address(0x838)));


extern volatile __bit C1CH1 __attribute__((address(0x839)));


extern volatile __bit C1IE __attribute__((address(0x46D)));


extern volatile __bit C1IF __attribute__((address(0x6D)));


extern volatile __bit C1OE __attribute__((address(0x83D)));


extern volatile __bit C1ON __attribute__((address(0x83F)));


extern volatile __bit C1OUT __attribute__((address(0x83E)));


extern volatile __bit C1POL __attribute__((address(0x83C)));


extern volatile __bit C1R __attribute__((address(0x83A)));


extern volatile __bit C1RSEL __attribute__((address(0x84D)));


extern volatile __bit C1SEN __attribute__((address(0xC2D)));


extern volatile __bit C2CH0 __attribute__((address(0x840)));


extern volatile __bit C2CH1 __attribute__((address(0x841)));


extern volatile __bit C2IE __attribute__((address(0x46E)));


extern volatile __bit C2IF __attribute__((address(0x6E)));


extern volatile __bit C2OE __attribute__((address(0x845)));


extern volatile __bit C2ON __attribute__((address(0x847)));


extern volatile __bit C2OUT __attribute__((address(0x846)));


extern volatile __bit C2POL __attribute__((address(0x844)));


extern volatile __bit C2R __attribute__((address(0x842)));


extern volatile __bit C2REN __attribute__((address(0xC2C)));


extern volatile __bit C2RSEL __attribute__((address(0x84C)));


extern volatile __bit C2SYNC __attribute__((address(0x848)));


extern volatile __bit CARRY __attribute__((address(0x18)));


extern volatile __bit CCP1IE __attribute__((address(0x462)));


extern volatile __bit CCP1IF __attribute__((address(0x62)));


extern volatile __bit CCP1M0 __attribute__((address(0xB8)));


extern volatile __bit CCP1M1 __attribute__((address(0xB9)));


extern volatile __bit CCP1M2 __attribute__((address(0xBA)));


extern volatile __bit CCP1M3 __attribute__((address(0xBB)));


extern volatile __bit CCP1X __attribute__((address(0xBD)));


extern volatile __bit CCP1Y __attribute__((address(0xBC)));


extern volatile __bit CCP2IE __attribute__((address(0x468)));


extern volatile __bit CCP2IF __attribute__((address(0x68)));


extern volatile __bit CCP2M0 __attribute__((address(0xE8)));


extern volatile __bit CCP2M1 __attribute__((address(0xE9)));


extern volatile __bit CCP2M2 __attribute__((address(0xEA)));


extern volatile __bit CCP2M3 __attribute__((address(0xEB)));


extern volatile __bit CCP2X __attribute__((address(0xED)));


extern volatile __bit CCP2Y __attribute__((address(0xEC)));


extern volatile __bit CHS0 __attribute__((address(0xFA)));


extern volatile __bit CHS1 __attribute__((address(0xFB)));


extern volatile __bit CHS2 __attribute__((address(0xFC)));


extern volatile __bit CHS3 __attribute__((address(0xFD)));


extern volatile __bit CKE __attribute__((address(0x4A6)));


extern volatile __bit CKP __attribute__((address(0xA4)));


extern volatile __bit CREN __attribute__((address(0xC4)));


extern volatile __bit CSRC __attribute__((address(0x4C7)));


extern volatile __bit DATA_ADDRESS __attribute__((address(0x4A5)));


extern volatile __bit DC __attribute__((address(0x19)));


extern volatile __bit DC1B0 __attribute__((address(0xBC)));


extern volatile __bit DC1B1 __attribute__((address(0xBD)));


extern volatile __bit DC2B0 __attribute__((address(0xEC)));


extern volatile __bit DC2B1 __attribute__((address(0xED)));


extern volatile __bit D_A __attribute__((address(0x4A5)));


extern volatile __bit D_nA __attribute__((address(0x4A5)));


extern volatile __bit ECCPAS0 __attribute__((address(0x4E4)));


extern volatile __bit ECCPAS1 __attribute__((address(0x4E5)));


extern volatile __bit ECCPAS2 __attribute__((address(0x4E6)));


extern volatile __bit ECCPASE __attribute__((address(0x4E7)));


extern volatile __bit EEIE __attribute__((address(0x46C)));


extern volatile __bit EEIF __attribute__((address(0x6C)));


extern volatile __bit EEPGD __attribute__((address(0xC67)));


extern volatile __bit FERR __attribute__((address(0xC2)));


extern volatile __bit FVREN __attribute__((address(0xC28)));


extern volatile __bit GCEN __attribute__((address(0x48F)));


extern volatile __bit GIE __attribute__((address(0x5F)));


extern volatile __bit GO __attribute__((address(0xF9)));


extern volatile __bit GO_DONE __attribute__((address(0xF9)));


extern volatile __bit GO_nDONE __attribute__((address(0xF9)));


extern volatile __bit HTS __attribute__((address(0x47A)));


extern volatile __bit I2C_DATA __attribute__((address(0x4A5)));


extern volatile __bit I2C_READ __attribute__((address(0x4A2)));


extern volatile __bit I2C_START __attribute__((address(0x4A3)));


extern volatile __bit I2C_STOP __attribute__((address(0x4A4)));


extern volatile __bit INTE __attribute__((address(0x5C)));


extern volatile __bit INTEDG __attribute__((address(0x40E)));


extern volatile __bit INTF __attribute__((address(0x59)));


extern volatile __bit IOCB0 __attribute__((address(0x4B0)));


extern volatile __bit IOCB1 __attribute__((address(0x4B1)));


extern volatile __bit IOCB2 __attribute__((address(0x4B2)));


extern volatile __bit IOCB3 __attribute__((address(0x4B3)));


extern volatile __bit IOCB4 __attribute__((address(0x4B4)));


extern volatile __bit IOCB5 __attribute__((address(0x4B5)));


extern volatile __bit IOCB6 __attribute__((address(0x4B6)));


extern volatile __bit IOCB7 __attribute__((address(0x4B7)));


extern volatile __bit IRCF0 __attribute__((address(0x47C)));


extern volatile __bit IRCF1 __attribute__((address(0x47D)));


extern volatile __bit IRCF2 __attribute__((address(0x47E)));


extern volatile __bit IRP __attribute__((address(0x1F)));


extern volatile __bit LTS __attribute__((address(0x479)));


extern volatile __bit MC1OUT __attribute__((address(0x84F)));


extern volatile __bit MC2OUT __attribute__((address(0x84E)));


extern volatile __bit MSK0 __attribute__((address(0x498)));


extern volatile __bit MSK1 __attribute__((address(0x499)));


extern volatile __bit MSK2 __attribute__((address(0x49A)));


extern volatile __bit MSK3 __attribute__((address(0x49B)));


extern volatile __bit MSK4 __attribute__((address(0x49C)));


extern volatile __bit MSK5 __attribute__((address(0x49D)));


extern volatile __bit MSK6 __attribute__((address(0x49E)));


extern volatile __bit MSK7 __attribute__((address(0x49F)));


extern volatile __bit OERR __attribute__((address(0xC1)));


extern volatile __bit OSFIE __attribute__((address(0x46F)));


extern volatile __bit OSFIF __attribute__((address(0x6F)));


extern volatile __bit OSTS __attribute__((address(0x47B)));


extern volatile __bit P1M0 __attribute__((address(0xBE)));


extern volatile __bit P1M1 __attribute__((address(0xBF)));


extern volatile __bit PDC0 __attribute__((address(0x4D8)));


extern volatile __bit PDC1 __attribute__((address(0x4D9)));


extern volatile __bit PDC2 __attribute__((address(0x4DA)));


extern volatile __bit PDC3 __attribute__((address(0x4DB)));


extern volatile __bit PDC4 __attribute__((address(0x4DC)));


extern volatile __bit PDC5 __attribute__((address(0x4DD)));


extern volatile __bit PDC6 __attribute__((address(0x4DE)));


extern volatile __bit PEIE __attribute__((address(0x5E)));


extern volatile __bit PEN __attribute__((address(0x48A)));


extern volatile __bit PRSEN __attribute__((address(0x4DF)));


extern volatile __bit PS0 __attribute__((address(0x408)));


extern volatile __bit PS1 __attribute__((address(0x409)));


extern volatile __bit PS2 __attribute__((address(0x40A)));


extern volatile __bit PSA __attribute__((address(0x40B)));


extern volatile __bit PSSAC0 __attribute__((address(0x4E2)));


extern volatile __bit PSSAC1 __attribute__((address(0x4E3)));


extern volatile __bit PSSBD0 __attribute__((address(0x4E0)));


extern volatile __bit PSSBD1 __attribute__((address(0x4E1)));


extern volatile __bit PULSR __attribute__((address(0xC2A)));


extern volatile __bit PULSS __attribute__((address(0xC2B)));


extern volatile __bit RA0 __attribute__((address(0x28)));


extern volatile __bit RA1 __attribute__((address(0x29)));


extern volatile __bit RA2 __attribute__((address(0x2A)));


extern volatile __bit RA3 __attribute__((address(0x2B)));


extern volatile __bit RA4 __attribute__((address(0x2C)));


extern volatile __bit RA5 __attribute__((address(0x2D)));


extern volatile __bit RA6 __attribute__((address(0x2E)));


extern volatile __bit RA7 __attribute__((address(0x2F)));


extern volatile __bit RB0 __attribute__((address(0x30)));


extern volatile __bit RB1 __attribute__((address(0x31)));


extern volatile __bit RB2 __attribute__((address(0x32)));


extern volatile __bit RB3 __attribute__((address(0x33)));


extern volatile __bit RB4 __attribute__((address(0x34)));


extern volatile __bit RB5 __attribute__((address(0x35)));


extern volatile __bit RB6 __attribute__((address(0x36)));


extern volatile __bit RB7 __attribute__((address(0x37)));


extern volatile __bit RBIE __attribute__((address(0x5B)));


extern volatile __bit RBIF __attribute__((address(0x58)));


extern volatile __bit RC0 __attribute__((address(0x38)));


extern volatile __bit RC1 __attribute__((address(0x39)));


extern volatile __bit RC2 __attribute__((address(0x3A)));


extern volatile __bit RC3 __attribute__((address(0x3B)));


extern volatile __bit RC4 __attribute__((address(0x3C)));


extern volatile __bit RC5 __attribute__((address(0x3D)));


extern volatile __bit RC6 __attribute__((address(0x3E)));


extern volatile __bit RC7 __attribute__((address(0x3F)));


extern volatile __bit RC8_9 __attribute__((address(0xC6)));


extern volatile __bit RC9 __attribute__((address(0xC6)));


extern volatile __bit RCD8 __attribute__((address(0xC0)));


extern volatile __bit RCEN __attribute__((address(0x48B)));


extern volatile __bit RCIDL __attribute__((address(0xC3E)));


extern volatile __bit RCIE __attribute__((address(0x465)));


extern volatile __bit RCIF __attribute__((address(0x65)));


extern volatile __bit RD __attribute__((address(0xC60)));


extern volatile __bit RE3 __attribute__((address(0x4B)));


extern volatile __bit READ_WRITE __attribute__((address(0x4A2)));


extern volatile __bit RP0 __attribute__((address(0x1D)));


extern volatile __bit RP1 __attribute__((address(0x1E)));


extern volatile __bit RSEN __attribute__((address(0x489)));


extern volatile __bit RX9 __attribute__((address(0xC6)));


extern volatile __bit RX9D __attribute__((address(0xC0)));


extern volatile __bit R_W __attribute__((address(0x4A2)));


extern volatile __bit R_nW __attribute__((address(0x4A2)));


extern volatile __bit SBOREN __attribute__((address(0x474)));


extern volatile __bit SCKP __attribute__((address(0xC3C)));


extern volatile __bit SCS __attribute__((address(0x478)));


extern volatile __bit SEN __attribute__((address(0x488)));


extern volatile __bit SENDB __attribute__((address(0x4C3)));


extern volatile __bit SMP __attribute__((address(0x4A7)));


extern volatile __bit SPEN __attribute__((address(0xC7)));


extern volatile __bit SR0 __attribute__((address(0xC2E)));


extern volatile __bit SR1 __attribute__((address(0xC2F)));


extern volatile __bit SREN __attribute__((address(0xC5)));


extern volatile __bit SSPEN __attribute__((address(0xA5)));


extern volatile __bit SSPIE __attribute__((address(0x463)));


extern volatile __bit SSPIF __attribute__((address(0x63)));


extern volatile __bit SSPM0 __attribute__((address(0xA0)));


extern volatile __bit SSPM1 __attribute__((address(0xA1)));


extern volatile __bit SSPM2 __attribute__((address(0xA2)));


extern volatile __bit SSPM3 __attribute__((address(0xA3)));


extern volatile __bit SSPOV __attribute__((address(0xA6)));


extern volatile __bit STRA __attribute__((address(0x4E8)));


extern volatile __bit STRB __attribute__((address(0x4E9)));


extern volatile __bit STRC __attribute__((address(0x4EA)));


extern volatile __bit STRD __attribute__((address(0x4EB)));


extern volatile __bit STRSYNC __attribute__((address(0x4EC)));


extern volatile __bit SWDTEN __attribute__((address(0x828)));


extern volatile __bit SYNC __attribute__((address(0x4C4)));


extern volatile __bit T0CS __attribute__((address(0x40D)));


extern volatile __bit T0IE __attribute__((address(0x5D)));


extern volatile __bit T0IF __attribute__((address(0x5A)));


extern volatile __bit T0SE __attribute__((address(0x40C)));


extern volatile __bit T1CKPS0 __attribute__((address(0x84)));


extern volatile __bit T1CKPS1 __attribute__((address(0x85)));


extern volatile __bit T1GINV __attribute__((address(0x87)));


extern volatile __bit T1GIV __attribute__((address(0x87)));


extern volatile __bit T1GSS __attribute__((address(0x849)));


extern volatile __bit T1INSYNC __attribute__((address(0x82)));


extern volatile __bit T1OSCEN __attribute__((address(0x83)));


extern volatile __bit T1SYNC __attribute__((address(0x82)));


extern volatile __bit T2CKPS0 __attribute__((address(0x90)));


extern volatile __bit T2CKPS1 __attribute__((address(0x91)));


extern volatile __bit TMR0IE __attribute__((address(0x5D)));


extern volatile __bit TMR0IF __attribute__((address(0x5A)));


extern volatile __bit TMR1CS __attribute__((address(0x81)));


extern volatile __bit TMR1GE __attribute__((address(0x86)));


extern volatile __bit TMR1IE __attribute__((address(0x460)));


extern volatile __bit TMR1IF __attribute__((address(0x60)));


extern volatile __bit TMR1ON __attribute__((address(0x80)));


extern volatile __bit TMR2IE __attribute__((address(0x461)));


extern volatile __bit TMR2IF __attribute__((address(0x61)));


extern volatile __bit TMR2ON __attribute__((address(0x92)));


extern volatile __bit TOUTPS0 __attribute__((address(0x93)));


extern volatile __bit TOUTPS1 __attribute__((address(0x94)));


extern volatile __bit TOUTPS2 __attribute__((address(0x95)));


extern volatile __bit TOUTPS3 __attribute__((address(0x96)));


extern volatile __bit TRISA0 __attribute__((address(0x428)));


extern volatile __bit TRISA1 __attribute__((address(0x429)));


extern volatile __bit TRISA2 __attribute__((address(0x42A)));


extern volatile __bit TRISA3 __attribute__((address(0x42B)));


extern volatile __bit TRISA4 __attribute__((address(0x42C)));


extern volatile __bit TRISA5 __attribute__((address(0x42D)));


extern volatile __bit TRISA6 __attribute__((address(0x42E)));


extern volatile __bit TRISA7 __attribute__((address(0x42F)));


extern volatile __bit TRISB0 __attribute__((address(0x430)));


extern volatile __bit TRISB1 __attribute__((address(0x431)));


extern volatile __bit TRISB2 __attribute__((address(0x432)));


extern volatile __bit TRISB3 __attribute__((address(0x433)));


extern volatile __bit TRISB4 __attribute__((address(0x434)));


extern volatile __bit TRISB5 __attribute__((address(0x435)));


extern volatile __bit TRISB6 __attribute__((address(0x436)));


extern volatile __bit TRISB7 __attribute__((address(0x437)));


extern volatile __bit TRISC0 __attribute__((address(0x438)));


extern volatile __bit TRISC1 __attribute__((address(0x439)));


extern volatile __bit TRISC2 __attribute__((address(0x43A)));


extern volatile __bit TRISC3 __attribute__((address(0x43B)));


extern volatile __bit TRISC4 __attribute__((address(0x43C)));


extern volatile __bit TRISC5 __attribute__((address(0x43D)));


extern volatile __bit TRISC6 __attribute__((address(0x43E)));


extern volatile __bit TRISC7 __attribute__((address(0x43F)));


extern volatile __bit TRISE3 __attribute__((address(0x44B)));


extern volatile __bit TRMT __attribute__((address(0x4C1)));


extern volatile __bit TUN0 __attribute__((address(0x480)));


extern volatile __bit TUN1 __attribute__((address(0x481)));


extern volatile __bit TUN2 __attribute__((address(0x482)));


extern volatile __bit TUN3 __attribute__((address(0x483)));


extern volatile __bit TUN4 __attribute__((address(0x484)));


extern volatile __bit TX8_9 __attribute__((address(0x4C6)));


extern volatile __bit TX9 __attribute__((address(0x4C6)));


extern volatile __bit TX9D __attribute__((address(0x4C0)));


extern volatile __bit TXD8 __attribute__((address(0x4C0)));


extern volatile __bit TXEN __attribute__((address(0x4C5)));


extern volatile __bit TXIE __attribute__((address(0x464)));


extern volatile __bit TXIF __attribute__((address(0x64)));


extern volatile __bit UA __attribute__((address(0x4A1)));


extern volatile __bit ULPWUE __attribute__((address(0x475)));


extern volatile __bit ULPWUIE __attribute__((address(0x46A)));


extern volatile __bit ULPWUIF __attribute__((address(0x6A)));


extern volatile __bit VCFG0 __attribute__((address(0x4FC)));


extern volatile __bit VCFG1 __attribute__((address(0x4FD)));


extern volatile __bit VR0 __attribute__((address(0x4B8)));


extern volatile __bit VR1 __attribute__((address(0x4B9)));


extern volatile __bit VR2 __attribute__((address(0x4BA)));


extern volatile __bit VR3 __attribute__((address(0x4BB)));


extern volatile __bit VREN __attribute__((address(0x4BF)));


extern volatile __bit VROE __attribute__((address(0x4BE)));


extern volatile __bit VRR __attribute__((address(0x4BD)));


extern volatile __bit VRSS __attribute__((address(0x4BC)));


extern volatile __bit WCOL __attribute__((address(0xA7)));


extern volatile __bit WDTPS0 __attribute__((address(0x829)));


extern volatile __bit WDTPS1 __attribute__((address(0x82A)));


extern volatile __bit WDTPS2 __attribute__((address(0x82B)));


extern volatile __bit WDTPS3 __attribute__((address(0x82C)));


extern volatile __bit WPUB0 __attribute__((address(0x4A8)));


extern volatile __bit WPUB1 __attribute__((address(0x4A9)));


extern volatile __bit WPUB2 __attribute__((address(0x4AA)));


extern volatile __bit WPUB3 __attribute__((address(0x4AB)));


extern volatile __bit WPUB4 __attribute__((address(0x4AC)));


extern volatile __bit WPUB5 __attribute__((address(0x4AD)));


extern volatile __bit WPUB6 __attribute__((address(0x4AE)));


extern volatile __bit WPUB7 __attribute__((address(0x4AF)));


extern volatile __bit WR __attribute__((address(0xC61)));


extern volatile __bit WREN __attribute__((address(0xC62)));


extern volatile __bit WRERR __attribute__((address(0xC63)));


extern volatile __bit WUE __attribute__((address(0xC39)));


extern volatile __bit ZERO __attribute__((address(0x1A)));


extern volatile __bit nA __attribute__((address(0x4A5)));


extern volatile __bit nADDRESS __attribute__((address(0x4A5)));


extern volatile __bit nBO __attribute__((address(0x470)));


extern volatile __bit nBOR __attribute__((address(0x470)));


extern volatile __bit nDONE __attribute__((address(0xF9)));


extern volatile __bit nPD __attribute__((address(0x1B)));


extern volatile __bit nPOR __attribute__((address(0x471)));


extern volatile __bit nRBPU __attribute__((address(0x40F)));


extern volatile __bit nRC8 __attribute__((address(0xC6)));


extern volatile __bit nT1SYNC __attribute__((address(0x82)));


extern volatile __bit nTO __attribute__((address(0x1C)));


extern volatile __bit nTX8 __attribute__((address(0x4C6)));


extern volatile __bit nW __attribute__((address(0x4A2)));


extern volatile __bit nWRITE __attribute__((address(0x4A2)));
# 694 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/pic_chip_select.h" 2 3
# 14 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/pic.h" 2 3
# 76 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/pic.h" 3
__attribute__((__unsupported__("The " "FLASH_READ" " macro function is no longer supported. Please use the MPLAB X MCC."))) unsigned char __flash_read(unsigned short addr);

__attribute__((__unsupported__("The " "FLASH_WRITE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_write(unsigned short addr, unsigned short data);

__attribute__((__unsupported__("The " "FLASH_ERASE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_erase(unsigned short addr);


# 1 "/opt/microchip/xc8/v3.00/pic/include/eeprom_routines.h" 1 3
# 114 "/opt/microchip/xc8/v3.00/pic/include/eeprom_routines.h" 3
extern void eeprom_write(unsigned char addr, unsigned char value);
extern unsigned char eeprom_read(unsigned char addr);
# 84 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/pic.h" 2 3
# 118 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC16Fxxx_DFP/1.6.156/xc8/pic/include/pic.h" 3
extern __bank0 unsigned char __resetbits;
extern __bank0 __bit __powerdown;
extern __bank0 __bit __timeout;
# 29 "/opt/microchip/xc8/v3.00/pic/include/xc.h" 2 3
# 5 "main.c" 2


#pragma config FOSC = INTRC_NOCLKOUT
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config MCLRE = ON
#pragma config CP = OFF
#pragma config CPD = OFF
#pragma config BOREN = ON
#pragma config IESO = OFF
#pragma config FCMEN = OFF
#pragma config LVP = OFF
# 51 "main.c"
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

_Bool to = 0;
_Bool smc = 0;
_Bool settingsmode = 0;


unsigned int potraw;
unsigned int voltageraw;
unsigned int dryruntime = 0;
uint8_t voltage = 0;
uint16_t voltagesum = 0;

volatile uint8_t reading_count = 0;
volatile uint8_t low_sensor_high_count = 0;
volatile uint8_t high_sensor_high_count = 0;
volatile uint8_t flow_sensor_high_count = 0;
volatile _Bool sensors_reading_complete = 0;
volatile _Bool sensors_reading_in_progress = 0;

_Bool low_sensor_active, high_sensor_active, flow_sensor_active;
_Bool mannualon = 0;
_Bool voltageerror = 0;
_Bool dryrunerror = 0;
_Bool timeouterror = 0;
_Bool motorrunning = 0;
_Bool tankempty = 0;
_Bool pretankempty = 0;
_Bool flowactive = 0;
_Bool preflowactive = 0;
unsigned int sensorbuffer = 10000;
unsigned int lastflowcheck = 0;
unsigned long lastsensorcheck = 0;

unsigned long buzzer_start_time = 0;
unsigned int buzzer_duration = 0;
_Bool buzzer_active = 0;

unsigned long lt = 0;
unsigned long lastdispupdt = 0;

_Bool check_button_press(void);
void trigger_buzzer(unsigned int duration_seconds);
void EEPROM_Write(unsigned char address, unsigned char data);
unsigned char EEPROM_Read(unsigned char address);
void EEPROM_Write16(unsigned char address, unsigned int data);
unsigned int EEPROM_Read16(unsigned char address);
_Bool loadSettings(unsigned char *value8bit, unsigned int *value16bit1,
                  unsigned int *value16bit2, unsigned int *value16bit3,
                  unsigned int *value16bit4);
void saveSettings(unsigned char value8bit, unsigned int value16bit1,
                  unsigned int value16bit2, unsigned int value16bit3,
                  unsigned int value16bit4);
void initSystem(void);
unsigned int readADC(uint8_t channel);
void init_timer(void);
void __attribute__((picinterrupt(("")))) timer_isr(void);
_Bool getSensorResults(_Bool *low_active, _Bool *high_active, _Bool *flow_active);
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
void lcd_display_bool_binary(_Bool value);
void dispinfo(uint8_t refreshtime);

void main(void) {


  initSystem();



  minvoltagelimit = 200;
  maxvoltagelimit = 240;
  minimumrunningvoltage = 190;
  maximumrinningvoltage = 250;
  maxruntimeindex = EEPROM_Read(0x01);





  potraw = readADC(4);

  dryruntime = (((uint32_t)potraw * 360) / 1023) + 120;

  for (uint8_t i = 0; i < 10; i++) {
    PORTAbits.RA0 = 1;
    PORTAbits.RA1 = 1;
    PORTAbits.RA2 = 1;
    PORTAbits.RA4 = 1;
    if (i == 0) {
      if (PORTCbits.RC1 == 0) {
        smc = 1;
      }
    } else if (i == 3) {
      if (smc == 1) {
        if (PORTCbits.RC1 == 0) {
          settingsmode = 1;
          i = 10;
        }
      }
    }
    _delay((unsigned long)((1000)*(8000000/4000.0)));
    PORTAbits.RA0 = 0;
    PORTAbits.RA1 = 0;
    PORTAbits.RA2 = 0;
    PORTAbits.RA4 = 0;
    _delay((unsigned long)((1000)*(8000000/4000.0)));
  }

  trigger_buzzer(3000);
  buzzer_update();

  init_timer();
  initLCD();

  while (settingsmode) {
    buzzer_update();

    getsensorreadings();
    potraw = readADC(4);
    dryruntime = (((uint32_t)potraw * 360) / 1023) + 120;

    if (PORTCbits.RC1 == 0) {
      _Bool is_long_press = check_button_press();

      if (is_long_press) {
        tankempty = 1;
        pretankempty = 1;

      } else {
        maxruntimeindex++;
        if (maxruntimeindex >= 5) {
          maxruntimeindex = 0;
        }
        EEPROM_Write(0x01, maxruntimeindex);
      }
    }
    switch (maxruntimeindex) {
    case 0:
      PORTAbits.RA0 = 0;
      PORTAbits.RA1 = 0;
      PORTAbits.RA2 = 0;
      PORTAbits.RA4 = 1;
      break;
    case 1:
      PORTAbits.RA0 = 0;
      PORTAbits.RA1 = 0;
      PORTAbits.RA2 = 1;
      PORTAbits.RA4 = 0;
      break;
    case 2:
      PORTAbits.RA0 = 0;
      PORTAbits.RA1 = 1;
      PORTAbits.RA2 = 0;
      PORTAbits.RA4 = 0;
      break;
    case 3:
      PORTAbits.RA0 = 1;
      PORTAbits.RA1 = 0;
      PORTAbits.RA2 = 0;
      PORTAbits.RA4 = 0;
      break;
    case 4:
      PORTAbits.RA0 = 0;
      PORTAbits.RA1 = 1;
      PORTAbits.RA2 = 1;
      PORTAbits.RA4 = 0;
      break;
    }

    dispinfo(100);
  }
  while (1) {

    if (PORTCbits.RC1 == 0) {
      _Bool is_long_press = check_button_press();

      if (is_long_press) {
        tankempty = 1;
        pretankempty = 1;

      } else {
        PORTAbits.RA2 = ~PORTAbits.RA2;
      }
    }

    buzzer_update();

    getsensorreadings();

    if (tankempty) {
      if (!timeouterror && !voltageerror && !dryrunerror && !motorrunning) {

        PORTCbits.RC3 = 1;
        motorstarttime = seconds_counter;
        motorrunning = 1;
        trigger_buzzer(1000);
      }
      if (motorrunning) {
        if (seconds_counter - motorstarttime >= maxruntime[4]) {
          timeouterror = 1;
        }
        if (!flowactive) {
          if (seconds_counter % 2 == 0) {

            PORTAbits.RA2 = 1;
          } else {

            PORTAbits.RA2 = 0;
          }
          if (lastdryruncheck == 0) {
            lastdryruncheck = seconds_counter;
          } else if (seconds_counter - lastdryruncheck >= dryruntime) {
            dryrunerror = 1;
            PORTAbits.RA2 = 1;
          }
        } else {
          lastdryruncheck = 0;
          PORTAbits.RA2 = 0;
        }

         if (voltage > maximumrinningvoltage || voltage < minimumrunningvoltage) {
          voltageerror = 1;
         }
      }
    } else {
      if (motorrunning) {
        motorrunning = 0;
        mannualon = 0;
        PORTCbits.RC3 = 0;
      }
    }
    if (timeouterror) {
      motorrunning = 0;
    }
    if (dryrunerror) {
      motorrunning = 0;
    }

    if (!motorrunning) {
      if (voltage > maxvoltagelimit || voltage < minvoltagelimit) {
        voltageerror = 1;
      }
      PORTCbits.RC3 = 0;
      PORTAbits.RA1 = 0;
    } else {
      PORTAbits.RA1 = 1;
      PORTCbits.RC3 = 1;
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
        voltageerror = 1;
      }else{
        voltageerror = 0;

      }
        lastvoltageerror = 0;
      }

      motorrunning = 0;
    } else if (lastvoltageerror != 0 &&
               (seconds_counter - lastvoltageerror >= maxvoltageerrortime)) {
      lastvoltageerror = 0;
    }

    dispinfo(100);
  }
}

void initSystem(void) {

  OSCCONbits.IRCF = 0b111;
  ANSEL = 0b00011000;
  ANSELH = 0x00;
  TRISA = 0b11101000;
  TRISB = 0b00111000;
  TRISC = 0b00000011;

  ADCON0 = 0b00001101;
  ADCON1 = 0b10000000;

  _delay((unsigned long)((10)*(8000000/4000.0)));
  PORTA = 0x00;
  PORTB = 0x00;
  PORTC = 0x00;

  PORTCbits.RC5 = 0;
  PORTCbits.RC6 = 0;
  PORTBbits.RB0 = 0;
  PORTBbits.RB1 = 0;
  PORTBbits.RB2 = 0;
  PORTCbits.RC7 = 0;

  PORTCbits.RC4 = 0;
}

unsigned int readADC(uint8_t channel) {

  ADCON0 = (ADCON0 & 0b11000011) | ((uint8_t)(channel << 2));
  _delay((unsigned long)((10)*(8000000/4000000.0)));
  ADCON0bits.GO = 1;


  while (ADCON0bits.GO)
    ;


  return ((unsigned int)ADRESH << 8) | ADRESL;
}

void setupTimer0(void) {
  OPTION_REGbits.T0CS = 0;
  OPTION_REGbits.PSA = 0;
  OPTION_REGbits.PS = 0b011;






  TMR0 = 6;
  INTCONbits.TMR0IE = 1;
}

void startSensorReading(void) {
  if (!sensors_reading_in_progress) {
    reading_count = 0;
    low_sensor_high_count = 0;
    high_sensor_high_count = 0;
    flow_sensor_high_count = 0;
    sensors_reading_complete = 0;
    sensors_reading_in_progress = 1;
  }
}

_Bool getSensorResults(_Bool *low_active, _Bool *high_active, _Bool *flow_active) {
  if (sensors_reading_complete) {
    *low_active = (low_sensor_high_count < 45);
    *high_active = (high_sensor_high_count < 45);
    *flow_active = (flow_sensor_high_count < 45);
    sensors_reading_complete = 0;
    return 1;
  }
  return 0;
}

void __attribute__((picinterrupt(("")))) isr(void) {

  if (PIR1bits.TMR1IF) {
    millis++;
    PIR1bits.TMR1IF = 0;
    TMR1H = 255;
    TMR1L = 14;
  }

  if (INTCONbits.TMR0IF) {

    TMR0 = 6;
    INTCONbits.TMR0IF = 0;


    if (sensors_reading_in_progress) {

      if (PORTBbits.RB5)
        low_sensor_high_count++;
      if (PORTBbits.RB4)
        high_sensor_high_count++;
      if (PORTBbits.RB3)
        flow_sensor_high_count++;

      reading_count++;


      if (reading_count >= 50) {
        sensors_reading_in_progress = 0;
        sensors_reading_complete = 1;
        INTCONbits.TMR0IE =
            0;
      }
    }
  }
}

void init_timer(void) {
  T1CON = 0b00110000;

  TMR1H = 255;
  TMR1L = 14;

  PIE1bits.TMR1IE = 1;
  INTCONbits.PEIE = 1;
  INTCONbits.GIE = 1;

  T1CONbits.TMR1ON = 1;
}

void EEPROM_Write(unsigned char address, unsigned char data) {
  while (EECON1bits.WR)
    ;

  EEADR = address;
  EEDATA = data;

  EECON1bits.EEPGD = 0;
  EECON1bits.WREN = 1;

  INTCONbits.GIE = 0;
  EECON2 = 0x55;
  EECON2 = 0xAA;
  EECON1bits.WR = 1;

  while (EECON1bits.WR)
    ;

  EECON1bits.WREN = 0;
  INTCONbits.GIE = 1;
}

unsigned char EEPROM_Read(unsigned char address) {
  while (EECON1bits.WR)
    ;
  EEADR = address;
  EECON1bits.EEPGD = 0;
  EECON1bits.RD = 1;
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

_Bool loadSettings(unsigned char *value8bit, unsigned int *value16bit1,
                  unsigned int *value16bit2, unsigned int *value16bit3,
                  unsigned int *value16bit4) {
  if (EEPROM_Read(0x00) != 0xAA) {
    return 0;
  }
  *value8bit = EEPROM_Read(0x01);
  *value16bit1 = EEPROM_Read16(0x02);
  *value16bit2 = EEPROM_Read16(0x04);
  *value16bit3 = EEPROM_Read16(0x06);
  *value16bit4 = EEPROM_Read16(0x08);
  return 1;
}

void saveSettings(unsigned char value8bit, unsigned int value16bit1,
                  unsigned int value16bit2, unsigned int value16bit3,
                  unsigned int value16bit4) {

  EEPROM_Write(0x01, value8bit);
  EEPROM_Write16(0x02, value16bit1);
  EEPROM_Write16(0x04, value16bit2);
  EEPROM_Write16(0x06, value16bit3);
  EEPROM_Write16(0x08, value16bit4);
  EEPROM_Write(0x00, 0xAA);
}

void trigger_buzzer(unsigned int duration_seconds) {
  if (!buzzer_active) {
    PORTCbits.RC4 = 1;
    buzzer_start_time = millis;
    buzzer_duration = duration_seconds;
    buzzer_active = 1;
  }
}

void buzzer_update() {
  if (buzzer_active && (millis - buzzer_start_time >= buzzer_duration)) {
    PORTCbits.RC4 = 0;
    buzzer_active = 0;
    buzzer_duration = 0;
  }
}

_Bool check_button_press(void) {
  _Bool long_press = 0;


  if (PORTCbits.RC1 == 0) {

    _delay((unsigned long)((50)*(8000000/4000.0)));


    if (PORTCbits.RC1 == 0) {

      unsigned int hold_count = 0;
      unsigned int long_press_threshold = 20;


      while (PORTCbits.RC1 == 0) {
        _delay((unsigned long)((10)*(8000000/4000.0)));
        hold_count++;


        if (hold_count >= long_press_threshold) {
          long_press = 1;




          while (PORTCbits.RC1 == 0) {
            _delay((unsigned long)((10)*(8000000/4000.0)));
          }


          _delay((unsigned long)((50)*(8000000/4000.0)));
          break;
        }
      }


      if (!long_press) {

        _delay((unsigned long)((50)*(8000000/4000.0)));
        return 0;
      }

      return 1;
    }
  }

  return 0;
}

void initLCD(void) {

  _delay((unsigned long)((15)*(8000000/4000.0)));



  PORTCbits.RC5 = 0;
  PORTCbits.RC7 = 0;
  PORTBbits.RB2 = 0;
  PORTBbits.RB1 = 1;
  PORTBbits.RB0 = 1;
  PORTCbits.RC6 = 1;
  _delay((unsigned long)((1)*(8000000/4000000.0)));
  PORTCbits.RC6 = 0;
  _delay((unsigned long)((5)*(8000000/4000.0)));


  PORTCbits.RC6 = 1;
  _delay((unsigned long)((1)*(8000000/4000000.0)));
  PORTCbits.RC6 = 0;
  _delay((unsigned long)((1)*(8000000/4000.0)));


  PORTCbits.RC6 = 1;
  _delay((unsigned long)((1)*(8000000/4000000.0)));
  PORTCbits.RC6 = 0;
  _delay((unsigned long)((1)*(8000000/4000.0)));


  PORTCbits.RC7 = 0;
  PORTBbits.RB2 = 0;
  PORTBbits.RB1 = 1;
  PORTBbits.RB0 = 0;
  PORTCbits.RC6 = 1;
  _delay((unsigned long)((1)*(8000000/4000000.0)));
  PORTCbits.RC6 = 0;
  _delay((unsigned long)((1)*(8000000/4000.0)));



  lcd_cmd(0x28);


  lcd_cmd(0x08);


  lcd_cmd(0x01);
  _delay((unsigned long)((2)*(8000000/4000.0)));


  lcd_cmd(0x06);


  lcd_cmd(0x0C);
}

void lcd_cmd(unsigned char cmd) {
  PORTCbits.RC5 = 0;


  PORTCbits.RC7 = (cmd & 0x80) ? 1 : 0;
  PORTBbits.RB2 = (cmd & 0x40) ? 1 : 0;
  PORTBbits.RB1 = (cmd & 0x20) ? 1 : 0;
  PORTBbits.RB0 = (cmd & 0x10) ? 1 : 0;

  PORTCbits.RC6 = 1;
  _delay((unsigned long)((1)*(8000000/4000000.0)));
  PORTCbits.RC6 = 0;
  _delay((unsigned long)((100)*(8000000/4000000.0)));


  PORTCbits.RC7 = (cmd & 0x08) ? 1 : 0;
  PORTBbits.RB2 = (cmd & 0x04) ? 1 : 0;
  PORTBbits.RB1 = (cmd & 0x02) ? 1 : 0;
  PORTBbits.RB0 = (cmd & 0x01) ? 1 : 0;

  PORTCbits.RC6 = 1;
  _delay((unsigned long)((1)*(8000000/4000000.0)));
  PORTCbits.RC6 = 0;
  _delay((unsigned long)((100)*(8000000/4000000.0)));
}

void lcd_set_cursor(unsigned char row, unsigned char col) {
  unsigned char address;


  if (row == 0) {
    address = 0x80 + col;
  } else {
    address = 0xC0 + col;
  }

  lcd_cmd(address);
}

void lcd_data(unsigned char data) {
  PORTCbits.RC5 = 1;


  PORTCbits.RC7 = (data & 0x80) ? 1 : 0;
  PORTBbits.RB2 = (data & 0x40) ? 1 : 0;
  PORTBbits.RB1 = (data & 0x20) ? 1 : 0;
  PORTBbits.RB0 = (data & 0x10) ? 1 : 0;

  PORTCbits.RC6 = 1;
  _delay((unsigned long)((1)*(8000000/4000000.0)));
  PORTCbits.RC6 = 0;
  _delay((unsigned long)((100)*(8000000/4000000.0)));


  PORTCbits.RC7 = (data & 0x08) ? 1 : 0;
  PORTBbits.RB2 = (data & 0x04) ? 1 : 0;
  PORTBbits.RB1 = (data & 0x02) ? 1 : 0;
  PORTBbits.RB0 = (data & 0x01) ? 1 : 0;

  PORTCbits.RC6 = 1;
  _delay((unsigned long)((1)*(8000000/4000000.0)));
  PORTCbits.RC6 = 0;
  _delay((unsigned long)((100)*(8000000/4000000.0)));
}

void lcd_display_int(int num) {

  if (num < 0) {
    lcd_data('-');
    num = -num;
  }


  if (num > 999)
    num = 999;


  lcd_data('0' + (num / 100));


  lcd_data('0' + ((num / 10) % 10));


  lcd_data('0' + (num % 10));
}

void getsensorreadings(void) {
  for (uint8_t i = 0; i < 16; i++) {
    voltagesum += readADC(3);
  }
  voltageraw = voltagesum >> 4;
  voltagesum = 0;
  voltage = (((uint32_t)voltageraw * 235) / 1023) + 85;



  if (seconds_counter % 1 == 0 && !sensors_reading_in_progress &&
      !sensors_reading_complete) {
    setupTimer0();
    startSensorReading();
  }

  if (getSensorResults(&low_sensor_active, &high_sensor_active,
                       &flow_sensor_active)) {

    if (high_sensor_active) {

      pretankempty = 0;
    }
    if (!low_sensor_active && !high_sensor_active) {

      pretankempty = 1;
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

void lcd_display_bool_binary(_Bool value) {
  if (value) {
    lcd_data('1');
  } else {
    lcd_data('0');
  }
}

void dispinfo(uint8_t refreshtime) {
  if (millis - lastdispupdt >= refreshtime) {
    lastdispupdt = millis;

    lcd_set_cursor(0, 0);
    lcd_display_int(voltage);
    lcd_set_cursor(0, 4);
    lcd_display_int(seconds_counter / 60);
    lcd_set_cursor(0, 8);
    lcd_display_int(dryruntime);
    lcd_set_cursor(0, 12);
    lcd_display_int(maxruntimeindex);
    lcd_set_cursor(1, 0);
    lcd_display_int(EEPROM_Read(0x04));

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
