
#define F_CPU 16000000UL

#include <avr/io.h>
#include "ALL.h"
#include <avr/interrupt.h>
#include <avr/iom32a.h>
#include <util/delay.h>

//SLAVE
/*
 The slave microcontroller recieved data from the master and this data starts to be compared by the defined variables to know what action will be taken 
 * the data is defined as char so that it only 1 byte storage 
 */

ISR(SPI_STC_vect) {
    char data = SPDR;
    if (data == '*') {
        LCD_clear_4bits();

    } else {
        LCD_write_4bits(data);
    }
    if (data == 'A') {
        LED0_ON();
        LCD_clear_4bits();
        char I[] = "LED0_ON";
        LCD_write_str_4bits(I);
    } else if (data == 'a') {
        LED0_OFF();
        LCD_clear_4bits();
        char i[] = "LED0_OFF";
        LCD_write_str_4bits(i);
        _delay_ms(700);
        LCD_clear_4bits();
    } 
     else if (data == 'F') {
        LED1_ON();
        LCD_clear_4bits();
        char K[] = "LED1_ON";
        LCD_write_str_4bits(K);

    } else if (data == 'f') {
        LED1_OFF();
        LCD_clear_4bits();
        char k[] = "LED1_OFF";
        LCD_write_str_4bits(k);
        _delay_ms(700);
        LCD_clear_4bits();
    } else if (data == 'U') {
        LED2_ON();
        LCD_clear_4bits();
        char J[] = "LED2_ON";
        LCD_write_str_4bits(J);

    } else if (data == 'u') {
        LED2_OFF();
        LCD_clear_4bits();
        char j[] = "LED2_OFF";
        LCD_write_str_4bits(j);
        _delay_ms(700);
        LCD_clear_4bits();
    }


}

int main(void) {
    LCD_init(_4BITS);
    LCD_clear_4bits();
    LEDs_init();

    SPI_init(Sleev, f_128);
    SPI_enable();


    sei();

    while (1) {

    }
    return 0;
}


/* MASTER 
 
 /*
 The Master recieve the data from the UART the send it to the salve , as the data will be send SPI will be the same data for the data recived UART 
 *   

char data;

int main(void) {

  
    UART_init(9600);
    SPI_init(Master, f_128);
    SPI_enable();


    while (1) {

        data = UART_recieve();

        SPI_send(data);
        _delay_ms(200);


    }
}
*/