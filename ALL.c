
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>   
#include "ALL.h"

#define RW           2
#define RS			 1
#define EN			 3  // Modified TCCR0&
#define ctrl		 PORTB
#define LCD_data	 PORTA

#define ctrl_dir     DDRB
#define LCD_data_dir DDRA


void LCD_cmd(char cmd){
	ctrl &= ~(1<<RS); // access to Command Register
	LCD_data = cmd;
	
	ctrl |= (1<<EN);  // Rising Edge
	_delay_ms(100);
	ctrl &= ~(1<<EN); // Falling Edge
	
}

void LCD_cmd_4bits(char cmd){
	ctrl &= ~(1<<RS); // access to Command Register
	
	LCD_data &= 0x0F;
	LCD_data |= (0xF0&cmd);
	
	ctrl |= (1<<EN);  // Rising Edge
	_delay_us(1);
	ctrl &= ~(1<<EN); // Falling Edge
	
	_delay_us(200);
	LCD_data &= 0x0F;
	LCD_data |= (cmd<<4);
	
	ctrl |= (1<<EN);  // Rising Edge
	_delay_us(1);
	ctrl &= ~(1<<EN); // Falling Edge
	
	_delay_ms(2);
	
}

void LCD_write_4bits(char data){
	ctrl |= (1<<RS); // access to Data Register
	
	LCD_data &= 0x0F;
	LCD_data |= (0xF0&data);
	
	ctrl |= (1<<EN);  // Rising Edge
	_delay_us(1);
	ctrl &= ~(1<<EN); // Falling Edge
	
	_delay_us(200);
	LCD_data &= 0x0F;
	LCD_data |= (data<<4);
	
	ctrl |= (1<<EN);  // Rising Edge
	_delay_us(1);
	ctrl &= ~(1<<EN); // Falling Edge
	
	_delay_ms(2);
	
}

void LCD_init(int mode){
	// Data Direction
	
	ctrl_dir |= (1<<RS)|(1<<EN)|(1<<RW);
	ctrl &= ~(1<<RW);
	
	
	
	
	if(mode == _4BITS){
		LCD_data_dir |= 0xF0;  // PORTx as OUTPUT
		LCD_cmd(0x20);                //Data Mode  4-bits
		_delay_ms(1);
		LCD_cmd_4bits(0x01);          // make clear LCD
		LCD_cmd_4bits(0x0C);          // display on, cursor off
		_delay_ms(100);
	}
	
	else{
		LCD_data_dir = 0xFF;  // PORTx as OUTPUT
		// initiate LCD for 8bit mode.// Data Mode  8-bits
		LCD_cmd(0x38);
		_delay_ms(1);
		LCD_cmd(0x01);          // make clear LCD
		LCD_cmd(0x0C);          // display on, cursor off
		_delay_ms(100);
	}
}

void LCD_write(char data){
	
	LCD_data = data;
	ctrl |= (1<<RS);
	
	
	ctrl |= (1<<EN);  // Rising Edge
	_delay_ms(1);
	ctrl &= ~(1<<EN); // Falling Edge
}

void LCD_clear(){
	_delay_ms(1);
	LCD_cmd(0x01);          // make clear LCD
	_delay_ms(1);
}
void LCD_clear_4bits(){
	_delay_ms(1);
	LCD_cmd_4bits(0x01);          // make clear LCD
	_delay_ms(1);
}

void LCD_write_str(char* str){
	
	for(int i=0; str[i]!= '\0';i++){
		LCD_write(str[i]);
	}
}
void LCD_write_str_4bits(char* str){
	
	for(int i=0; str[i]!= '\0';i++){
		LCD_write_4bits(str[i]);
	}
}
void LCD_WRITE_CHAR(char a[100])
{
   LCD_write_4bits(a[100]);
}
void LCD_write_num(unsigned int num){
	char strNumber[8];
	
	itoa(num, strNumber,10);
	
	LCD_write_str(strNumber);
	
}

void LCD_write_num_4bits(unsigned int num){
	char strNumber[8];
	
	itoa(num, strNumber,10);
	
	LCD_write_str_4bits(strNumber);
	
}


void LCD_goto_line1(){
	LCD_cmd(0X80);
}

void LCD_goto_line2(){
	LCD_cmd(0XC0);
}

void LCD_goto_line1_4bits(){
	LCD_cmd_4bits(0X80);
}

void LCD_goto_line2_4bits(){
	LCD_cmd_4bits(0XC0);
}

void LCD_goto_xy(int row , int col){  // col 0 ~ 15      0 ~ F
	if(row == 0 && col < 16){  // LINE 1
		LCD_cmd(0X80|col);
	}
	else if (row == 1 && col < 16) // LINE 2
	{
		LCD_cmd(0XC0|col);
	}
	else{
		//LCD_write_str("out of dimension");
	}
}

void LCD_goto_xy_4bits(int row , int col){  // col 0 ~ 15      0 ~ F
	if(row == 0 && col < 16){  // LINE 1
		LCD_cmd_4bits(0X80|col);
	}
	else if (row == 1 && col < 16) // LINE 2
	{
		LCD_cmd_4bits(0XC0|col);
	}
	else{
		//LCD_write_str_4bits("out of dimension");
	}
}
void TIMER0_init(int Mode, int Clock) {
    TCCR0 &= 0xE0;
    switch(Mode)
    {
        case normal:
            break;
        case PWM:
            TCCR0 |= (1<<WGM00);
            break;
        case CTC:
             TCCR0 |= (1<<WGM01);
            break;
        case FPWM:
             TCCR0 |= ((1<<WGM00)|(1<<WGM01));
            break;
    }
    
    
    switch (Clock) {
        case No_clock_source:
            break;
        case No_prescaling:
            TCCR0 |= (1 << CS00);
            break;
        case clk8:
            TCCR0 |= (1 << CS01);
            break;
        case clk64:
            TCCR0 |= ((1 << CS00) | (1 << CS01));
            break;
        case clk256:
            TCCR0 |= (1 << CS02);
            break;
        case clk1024:
            TCCR0 |= ((1 << CS02) | (1 << CS00));
            break;
        case falling_edge:
            TCCR0 |= ((1 << CS02) | (1 << CS01));
            break;
        case rising_edge:
            TCCR0 |= ((1 << CS02) | (1 << CS01) | (1 << CS00));
            break;
    }
    
}
void TIMER0_OVIE(int state)
{
    if(state)
    {
        TIMSK |=(1<<TOIE0);
    }
    else
    {
        TIMSK &=~(1<<TOIE0);
    }
}
void TIMER0_OCIE (int state)
{
      if(state)
    {
        TIMSK |=(1<<OCIE0);
    }
    else
    {
        TIMSK &=~(1<<OCIE0);
    }
}
void TIMER0_setMatchPoint(unsigned char val )
{
    OCR0 = val ;
}
void TIMER0_set0Mode(int Mode)
{
    TCCR0 &=~((1<<COM01) | (1<<COM00));
    if(Mode != OC0_Disconnected)
    {
        DDRB |= (1<<3);
    }
    TCCR0  |=(Mode<<4);
}

void ADC_selectVref(int Vref) {
    ADMUX &= ~((1 << REFS1) | (1 << REFS0));
    //ADMUX |=(Vref <<6);
    switch (Vref) {
        case Vref_AREF:
            //no need to do anything 
            break;
        case Vref_AVCC:
            ADMUX |= (1 << REFS0);
            break;
        case Vref_Internal:
            ADMUX |= (1 << REFS1) | (1 << REFS0);
            break;
    }
            

}


void ADC_selectChannel(int ChannelNumber) {
    ADMUX &= ~((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3) | (1 << MUX4) );
    //ADMUX & = 0xE0 ;
     ADMUX |= ChannelNumber ;


    /* switch (ChannelNumber)
         case '0':
         //no thing 
         break;
         case '1':
     ADMUX |=(1<<MUX0);
         break;
       case '2':
     ADMUX |=(1<<MUX1);
         break;
       case '3':
     ADMUX |=((1<<MUX0)|(1<<MUX1));
         break;
       case '4':
     ADMUX |=(1<<MUX2);
         break;
       case '5':
     ADMUX |=((1<<MUX0)|(1<<MUX2));
         break;
       case '6':
     ADMUX |=((1<<MUX2)|(1<<MUX1));
         break;
       case '7':
     ADMUX |=((1<<MUX2)|(1<<MUX1)|(1<<MUX0));
         break;
     
     */


}


void ADC_selectPreescaler(int PreeScaler) {
    ADCSRA &= 0xE0;
    ADCSRA |= PreeScaler;

}

void ADC_ENABLE() {
    ADCSRA |= (1 << ADEN);
}

void ADC_DISABLE() {
    ADCSRA &= ~(1 << ADEN);
}

void ADC_Start() {
    ADCSRA |= (1 << ADSC);
}

void ADC_INIT(int ChannelNumber, int Vref, int PreeScaler) {

    ADC_selectChannel(ChannelNumber);
    ADC_selectPreescaler(PreeScaler);
    ADC_selectVref(Vref);
    ADC_ENABLE();
}

int ADC_Read_R() {
    int data = ADCL;
    data |= (ADCH << 8);
    return data;
}

int ADC_Read_L() {
    int data = ADCL;
    data |= (ADCH << 8);
    return (data >> 6);
}
void wait_ADC()
{
    while(!(ADCSRA & (1<<ADIF)));
}

void    UART_init(int BAUDRATE) 
{
    UCSRB |= ((1<<RXEN) | (1<<TXEN));
    UCSRB |=(1<<RXCIE);
    UART_setBaudRate( BAUDRATE);
}
void   UART_setBaudRate (int  BAUDRATE)
{
   int UBRR= ((F_CPU/16.0)/BAUDRATE)-1;
    UBRRL = (char) UBRR;
    UBRRH = (UBRR >>8);
}
//CALCULATE FROM EQUATION 
//UBRL
//UBRH
    
void  UART_SEND(char data)
{
    while  (!(UCSRA &(1<<UDRE))) ;
    UDR = data ;
}
char UART_recieve()
{
    while (!(UCSRA & (1<<RXC)));
    return UDR;
}
void UART_Send_str(char str[])
{
    int i ;
    for ( i =0 ;str[i]!= '\0' ; i++)
    {
        UART_SEND(str[i]);
    }
    
}
void UART_Send_Num(int number)
{
    char buff[11];
    itoa(number,buff,10);
    UART_Send_str(buff);
}
void SPI_init(int Type, int FREQ) {
    SPCR |= (1<<SPIE);
    switch (Type) {
        case Master:
            SPCR |= (1 << MSTR);
            DDRB |= (1<<MOSI) | (1<<SS) | (1<<SCK);
            break;
        case Sleev:
            SPCR &= ~(1 << MSTR);
            DDRB |= (1<<MISO);
            break;

    }
    switch (FREQ) {
        case f_4:
            break;
        case f_16:
            SPCR |= (1 << SPR0);
            break;
        case f_64:
            SPCR |= (1 << SPR1);
            break;
        case f_128:
            SPCR |= ((1 << SPR1) | (1 << SPR0));
            break;
        case f_2:
            SPSR |= (1 << SPI2X);
            break;
        case f_8:
            SPSR |= (1 << SPI2X);
            SPCR |= (1 << SPR0);
            break;
        case f_32:
            SPSR |= (1 << SPI2X);
            SPCR |= (1 << SPR1);
            break;
//        case f_64:
//            SPSR |= (1 << SPI2X);
//            SPCR |= ((1 << SPR1) | (1 << SPR0));
//            break;
    }
}

void SPI_enable() {
    SPCR |= (1 << SPE);

}
void SPI_send(char data) {
    SPDR = data;
    SPI_wait();


}
void SPI_send_slav(char slavslct ,char data)
{
    PORTA &=~(1<<slavslct);
    SPDR= data;
    SPI_wait();
    PORTA |=(1<<slavslct);
         
    
}
char SPI_recive(void) {
    SPI_wait();
    return SPDR;
}



void SPI_wait() {
    while (!(SPSR & (1 << SPIF)));
}
void INT_ENABLE(int INT_NAME, int INT_MODE){
    if(INT_NAME == INT_0){
        GICR &= ~(1<<INT0); // DISABLE
        switch(INT_MODE){
            case INT_MODE_LOW:
                MCUCR &= ~((1<<ISC01)| (1<<ISC00));
                break;
            case INT_MODE_ANY:
                MCUCR |= (1<<ISC00);
                MCUCR &= ~(1<<ISC01);
                break;
            case INT_MODE_FALLING:
                MCUCR |= (1<<ISC01);
                MCUCR &= ~(1<<ISC00);
                break;
            case INT_MODE_RISING:
                MCUCR |= ((1<<ISC01)| (1<<ISC00));
                break;
        }
        GICR |= (1<<INT0);  // ENABLE
    }else if(INT_NAME == INT_1){
        GICR &= ~(1<<INT1); // DISABLE
        switch(INT_MODE){
            case INT_MODE_LOW:
                MCUCR &= ~((1<<ISC11)| (1<<ISC10));
                break;
            case INT_MODE_ANY:
                MCUCR |= (1<<ISC10);
                MCUCR &= ~(1<<ISC11);
                break;
            case INT_MODE_FALLING:
                MCUCR |= (1<<ISC11);
                MCUCR &= ~(1<<ISC10);
                break;
            case INT_MODE_RISING:
                MCUCR |= ((1<<ISC11)| (1<<ISC10));
                break;
        }
        GICR |= (1<<INT1);  // ENABLE
    }else if(INT_NAME == INT_2){
        GICR &= ~(1<<INT2); // DISABLE
        switch(INT_MODE){
          
            case INT_MODE_FALLING:
                MCUCSR &= ~(1<<ISC2);
                break;
            case INT_MODE_RISING:
                MCUCSR |= (1<<ISC2);
                break;
        }
        GICR |= (1<<INT2);  // ENABLE
    }
    else{
        // DO Nothing
    }
}

void INT_DISABLE(int INT_NAME){
    switch(INT_NAME){
        case INT_0:
            GICR &= ~(1<<INT0);
            break;
        case INT_1:
            GICR &= ~(1<<INT1);
            break;
        case INT_2:
            GICR &= ~(1<<INT2);
            break;
    }
}
int isPressed_BTN0(){
    if(PINB & (1<<BTN0)){
        return 1;
    }
    else{
        return 0;
    }
}

int isPressed_BTN1(){
    if(PIND & (1<<BTN1)){
        return 1;
    }else{
        return 0;
    }
}

int isPressed_BTN2(){
    if(PIND & (1<<BTN2)){
        return 1;
    }else{
        return 0;
    }
}

void BTNs_init(){
    DDRD &= ~((1<<BTN1)|(1<<BTN2));
    DDRB &= ~(1<<BTN0);
}

void BUZZER_init() {
    DDRA |= (1 << BUZZER);
}

void BUZZER_ON() {
    PORTA |= (1 << BUZZER);
}

void BUZZER_OFF() {
    PORTA &= ~(1 << BUZZER);
}

void RELAY_init() {
    DDRA |= (1 << RELAY);
}

void RELAY_ON() {
    PORTA |= (1 << RELAY);
}

void RELAY_OFF() {
    PORTA &= ~(1 << RELAY);
}

void LEDs_init() {
    // Data Direction (input or output)
    DDRC |= (1 << LED1) | (1 << LED0); // PC2,PC7 are output
    DDRD |= (1 << LED2); // PD3 is output
}

void LED0_ON() {
    PORTC |= (1 << LED0);
}

void LED1_ON() {
    PORTC |= (1 << LED1);
}

void LED2_ON() {

    PORTD |= (1 << LED2);
}

void LED0_OFF() {
    PORTC &= ~(1 << LED0);
}

void LED1_OFF() {
    PORTC &= ~(1 << LED1);
}

void LED2_OFF() {
    PORTD &= ~(1 << LED2);
}

void LEDs_OFF() {
    LED0_OFF();
    LED1_OFF();
    LED2_OFF();
}

void LEDs_ON() {
    LED0_ON();
    LED1_ON();
    LED2_ON();
}

void LED0_TOGGLE(){
    
    PORTC ^= (1<<LED0);
    
}
void LED1_TOGGLE(){
    
    PORTC ^= (1<<LED0);
    
}

void LED2_TOGGLE(){
    
    PORTC ^= (1<<LED0);
    
}