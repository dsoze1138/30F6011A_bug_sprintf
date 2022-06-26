/*
 * File:   main.c
 * Author: Dan1138
 *
 * Created on June 25, 2022, 7:36 PM
 */

/*
 * Define the system oscillator this code will setup
 */
#define FSRC  (7372800ul)   /* nominal fast RC frequency */
#define PLL_MODE (4ul)      /* Use 4x PLL mode set in configuration words */
#define FSYS (FSRC*PLL_MODE)
#define FCY  (FSYS/4ul)     /* dsPIC30 uses a 4 clock instruction cycle */

// dsPIC30F6011A Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FOSFPR = FRC_PLL4        // Oscillator (FRC w/PLL 4x)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = NONE            // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FBS
#pragma config BWRP = WR_PROTECT_BOOT_OFF// Boot Segment Program Memory Write Protect (Boot Segment Program Memory may be written)
#pragma config BSS = NO_BOOT_CODE       // Boot Segment Program Flash Memory Code Protection (No Boot Segment)
#pragma config EBS = NO_BOOT_EEPROM     // Boot Segment Data EEPROM Protection (No Boot EEPROM)
#pragma config RBS = NO_BOOT_RAM        // Boot Segment Data RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WR_PROT_SEC_OFF   // Secure Segment Program Write Protect (Disabled)
#pragma config SSS = NO_SEC_CODE        // Secure Segment Program Flash Memory Code Protection (No Secure Segment)
#pragma config ESS = NO_SEC_EEPROM      // Secure Segment Data EEPROM Protection (No Segment Data EEPROM)
#pragma config RSS = NO_SEC_RAM         // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = GSS_OFF            // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <libpic30.h>
#include <stdio.h>

/* UART1 I/O PINS */ /* must list GPIO pin */
#define U1_TXD      _LATF3
#define U1_TXD_DIR  _TRISF3
#define U1_RXD      _RF2
#define U1_RXD_DIR  _TRISF2
         
#define U1_BAUD 9600UL
#define U1_BRGH_VALUE 1

#define _U1_UARTEN _UARTEN
#define _U1_UTXEN  _UTXEN
#define _U1_UTXBF  _UTXBF
#define _U1_TRMT   _TRMT

#define _U1_OERR   _OERR
#define _U1_URXDA  _URXDA
/*
** U1BRG register value and baudrate error calculation
*/
#define U1_BRGH_SCALE 16L

#define U1_BRGREG ( (FCY + (U1_BRGH_SCALE * U1_BAUD)/1 )/(U1_BRGH_SCALE * U1_BAUD)-1L)

#if U1_BRGREG > 65535
#error Cannot set up UART1 for the FCY and BAUDRATE. Correct values in init.h and uart.h files.
#endif

/*
** Check if baud error greater than 2.5 percent
*/
#if 0
    #define REAL_BAUDRATE ( FCY / ( U1_BRGH_SCALE * ( U1_BRGREG + 1L) ) )
    #if (REAL_BAUDRATE > (U1_BAUD + (U1_BAUD * 25L) / 1000L)) || (REAL_BAUDRATE < (U1_BAUD - (U1_BAUD * 25L) / 1000L))
    #error UART baudrate error greater than 2.5 percent for the FCY and U1_BAUD. Correct values in uart.c file.
    #endif
#endif
#undef REAL_BAUDRATE
/*
** Function: U1_Init
**
** Precondition: None.
**
** Overview: Setup UART2 module.
**
** Input: None.
**
** Output: None.
**
*/
void U1_Init( void ) {
    /* Disable interrupts */
    _U1TXIE = 0;
    _U1RXIE = 0;
    _U1RXIP = 0b100;
    _U1TXIP = 0b100;
    /* Turn off UART */
    U1MODE = 0;
    U1STA = 0;
    /* Setup default GPIO states for UART pins */
#ifdef U1_TXD
    U1_TXD = 1;
#endif

#ifdef U1_TXD_DIR
    U1_TXD_DIR = 0;
#endif

#ifdef U1_RXD_DIR
    U1_RXD_DIR = 1;
#endif
    /* Initialize the UART */
    U1BRG = U1_BRGREG;
    _U1_UARTEN = 1;
    _U1_UTXEN  = 1;
    _U1RXIF = 0;        /* reset RX flag */
}

/*
** Function: U1_PutChar
**
** Precondition: U1_Init must be called before.
**
** Overview: Wait for free UART transmission buffer and send a byte.
**
** Input: Byte to be sent.
**
** Output: None.
**
*/
void U1_PutChar( char Ch ) {
    // wait for empty buffer  
    while(_U1_TRMT == 0);
      U1TXREG = Ch;
}

/*
** Function: U1_HasData
**
** Precondition: UART2Init must be called before.
**
** Overview: Check if there's a new byte in UART reception buffer.
**
** Input: None.
**
** Output: Zero if there's no new data received.
**
*/
char U1_HasData( void ) {
    char Temp;

    if (_U1_OERR != 0)
    {
        Temp = U1RXREG; /* clear overrun error */
        Temp = U1RXREG;
        Temp = U1RXREG;
        Temp = U1RXREG;
        Temp = U1RXREG;
        _U1_OERR = 0;
    }
    if(_U1RXIF == 1)
        return 1;
    return 0;
}

/*
** Function: U1_GetChar
**
** Precondition: U1_Init must be called before.
**
** Overview: Wait for a byte.
**
** Input: None.
**
** Output: Byte received.
**
**
*/
char U1_GetChar( void ) {
    char Temp;

    if (_U1_OERR != 0)
    {
        Temp = U1RXREG; /* clear overrun error */
        Temp = U1RXREG;
        Temp = U1RXREG;
        Temp = U1RXREG;
        Temp = U1RXREG;
        _U1_OERR = 0;
    }
    while(_U1RXIF == 0);
    Temp = U1RXREG;
    if (_U1_URXDA == 0)
    {
        _U1RXIF = 0;
    }
    return Temp;
}

/*
 * Hook to send printf output to LCD
 */
int __attribute__((__section__(".libc.write"))) write(int handle, void *buffer, unsigned int len) 
{
    unsigned int i;
    char *pStr = (char *)(buffer);
    
    for (i = len; i; --i)
    {
        U1_PutChar(*pStr++);
    }
    return(len);
}
/*
 * Main application
 */
int main(void) {
    double fOutput;
    /* Initialize application */
    U1_Init();
    printf("\r\ndsPIC30F6011A start build on %s at %s with XC16 version %u\r\n", __DATE__, __TIME__, __XC16_VERSION__);
    fOutput = -99.996589;
    printf("%.2f\r\n",fOutput);
    
    /* Application process loop */
    for(;;) {
        __delay_ms(500);
        Nop();
    }
}
