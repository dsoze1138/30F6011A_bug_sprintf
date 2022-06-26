# dsPIC30F6011A bug sprintf

Configure a dsPIC30F6011A to run using the FRC as the primary clock 
with the 4x PLL for system oscillator frequency 29.4912MHZ.

Initialize UART1 at 9600 baud.

Use printf to display a startup message.

Use printf to display the decimal value -99.996589 to 2 decimal places.

XC16 v1.70 shows the correctly rounded value of -100.00

XC16 v2.00 shows an incorrectly rounded value of -:0.00

See Microchip forum post: https://www.microchip.com/forums/FindPost/1202509