
#ifndef ALL_H
#define	ALL_H

#include <xc.h> // include processor files - each processor file is guarded.  

#define _4BITS    4
#define _8BITS    8
#define normal   0
#define PWM    1
#define CTC    2
#define FPWM    3 
#define  No_clock_source 0
#define  No_prescaling 1
#define  clk8 2
#define clk64  3
#define  clk256  4
#define  clk1024  5
#define  falling_edge  6
#define  rising_edge  7 
#define ENABLE 1
#define DISABLE 0

#define OC0_Disconnected 0 //for all modes 

#define CTC_OC0_Toggke 1
#define CTC_Clear 2
#define CTC_Set 3 

#define PWM_REVERSED 1
#define PWM_CLEAR 2
#define PWM_SET 3

#define Vref_Internal 3
#define Vref_AVCC     1
#define Vref_AREF     0
#define PS_2   1
#define PS_4   2
#define PS_8   3
#define PS_16   4
#define PS_32    5
#define PS_64     6
#define PS_128      7 

#define Master 1
#define Sleev 0 
#define  f_4  0
#define  f_16  1
#define  f_64    2
#define  f_128   3
#define  f_2   4
#define  f_8   5
#define  f_32  6
//#define   f_64   7

#define Read_M      0
#define Read_L   1

#define  MOSI  5
#define  MISO 6
#define  SS  4
#define  SCK    7 


void LCD_cmd(char);

void LCD_init(int mode);  // initiate driver
void LCD_clear();
void LCD_write(char); // location???

void LCD_write_str(char*);
void LCD_write_num(unsigned int);

void LCD_goto_line1();
void LCD_goto_line2();

void LCD_goto_xy(int , int);


void LCD_cmd_4bits(char);
void LCD_write_4bits(char );
void LCD_clear_4bits();


void LCD_write_str_4bits(char*);
void LCD_write_num_4bits(unsigned int);

void LCD_goto_line1_4bits();
void LCD_goto_line2_4bits();

void LCD_goto_xy_4bits(int , int);
void TIMER0_init(int Mode, int Clock);
void TIMER0_OVIE(int state);
void TIMER0_OCIE (int state);
void TIMER0_setMatchPoint(unsigned char val );
void TIMER0_set0Mode(int Mode);

void ADC_selectVref(int Vref);
void ADC_selectChannel(int ChannelNumber);
void ADC_selectPreescaler(int PreeScaler);
void ADC_ENABLE();
void ADC_DISABLE();
void ADC_Start();
int ADC_Read_R();
int ADC_Read_L();
void wait_ADC();
void ADC_INIT(int ChannelNumber, int Vref, int PreeScaler);

void    UART_init(int BAUDRATE) ;
//Enable TX RX
//CALL UART_SETBAUD
void   UART_setBaudRate (int  BAUDRATE);
//CALCULATE FROM EQUATION 
//UBRL
//UBRH
    
void  UART_SEND(char data);
char UART_recieve();
void UART_Send_str();
void UART_Send_Num();

void SPI_init(int FREQ , int Type );
void SPI_enable();
char SPI_recive(void);
void SPI_send(char data);
void SPI_send_slav(char slavselector ,char data);
void SPI_wait();

#define INT_0  0
#define INT_1  1
#define INT_2  2
// INT_MODE
#define INT_MODE_LOW        0
#define INT_MODE_ANY        1
#define INT_MODE_FALLING    2
#define INT_MODE_RISING     3


void INT_ENABLE(int INT_NAME, int INT_MODE);
void INT_DISABLE(int INT_NAME);

#define LED0    2
#define LED1    7
#define LED2    3
#define RELAY   2
#define BUZZER  3
#define POT     1
#define BTN0    0
#define BTN1    6
#define BTN2    2


void BTNs_init();

void LEDs_init();
void LED0_ON();
void LED1_ON();
void LED2_ON();
void LED0_OFF();
void LED1_OFF();
void LED2_OFF();
void LEDs_OFF();
void LEDs_ON();
void RELAY_init();
void RELAY_ON();
void RELAY_OFF();

void BUZZER_init();
void BUZZER_ON();
void BUZZER_OFF();

int isPressed_BTN0();
int isPressed_BTN1();
int isPressed_BTN2();

void LED0_TOGGLE();
void LED1_TOGGLE();
void LED2_TOGGLE();

void LCD_WRITE_CHAR(char a[100]);

#endif	/* XC_HEADER_TEMPLATE_H */

