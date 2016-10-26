/* 
 * File:   main.c
 * Author: Arsapol
 *
 * Created on October 25, 2016, 1:21 PM
 */
#include <24FJ48GA002.h>
#device adc=16
#FUSES NOWDT //No Watch Dog Timer
#FUSES HS //High speed Osc (> 4mhz for PCM/PCH) (>10mhz for PCD)
//#FUSES NOPUT //No Power Up Timer
//#FUSES NOBROWNOUT //No brownout reset
//#FUSES NOLVP //No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O
//#FUSES NOCPD //No EE protection
#FUSES WRT //Program Memory Write Protected
#FUSES NODEBUG //No Debug mode for ICD
#FUSES NOPROTECT //Code not protected from reading

#use delay (clock = 32000000)

#use rs232(baud=57600, xmit=PIN_B14, rcv=PIN_B15)
// DEFINE LED
//#define led1(x) output_bit(pin_d0,x)
//#define led2(x) output_bit(pin_d1,x)
char cmd;
#int_rda

void rs232_isr(void) {
    cmd = getc();
}

void main(void) {
    enable_interrupts(INT_RDA);
    enable_interrupts(GLOBAL);

    cmd = 0;

    printf("** Control LED **\n\r");
    printf("A: LED1 is on.\n\r");
    printf("a: LED1 is off.\n\r");
    printf("B: LED2 is on.\n\r");
    printf("b: LED2 is off.\n\r");


    while (TRUE) {

        if (cmd != 0) {

            switch (cmd) {
                case 'A': printf("Now A");
                    break;
                case 'a': printf("Now a");
                    break;
                case 'B': printf("Now B");
                    break;
                case 'b': printf("Now b");
                    break;
            }
            cmd = 0;
        }

    }

}