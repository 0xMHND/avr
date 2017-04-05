// Mohannad Al Arifi
// 932 09 3718
// 10.6.16

// lab4: Alarm clock
//


# define F_CPU 8000000UL
#include "functions.h"

#define LED_DELAY 50

volatile uint8_t TIME[3] = {0};

void init_tcnt2(){
    TCCR2   = (1<<WGM21)|(1<<WGM20)|(1<<COM21)|(0<<COM20)|(0<<CS22)|(0<<CS21)|(0<<CS20); //Fast PWM,  set OC2 on compare match.
}
void init_adc(){
    ADMUX   = (0<<REFS1) | (1<<REFS0) | (1<<ADLAR); //AVCC refrence, Left justified.
    ADCSRA  = (1<<ADEN) | (0<<ADSC) | (0<<ADFR) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //enable ADC, ADC interrupt, prescale by 128
}

uint8_t read_adc(){
    ADMUX |= (1<<ADSC); //start conversion.
    while(bit_is_clear(ADMUX, ADIF)){}
    return ADHC;
}


/*************************************************************************/
//                           timer/counter0 ISR                          
//When the TCNT0 overflow interrupt occurs, the count_7ms variable is    
//incremented.  Every 7680 interrupts the minutes counter is incremented.
//tcnt0 interrupts come at 7.8125ms internals.
// 1/32768          = 30.517578uS
//(1/32768)*256     = 7.8125ms
//(1/32768)*256*128 = 1 s
/*************************************************************************/
ISR(TIMER0_OVF_vect){
    static uint8_t sec      = 0;
    static uint8_t min      = 0;
    static uint8_t hr       = 0;
    static uint8_t ampm     = 0;

    sec++;
    if( sec>=60 ){
        sec=0;
        min++;
        if( min>=60 ){
            min=0;
            hr++;
            if( hr>=24 ){
                hr=0;
            }
        }
    }

    TIME[0] = sec;
    TIME[1] = min;
    TIME[2] = hr;
}
    
int main()
{

    DDRA    = 0xFF; //set port A to all outputs
    DDRB    = 0xF7; //set port B bits 4-7 B as outputs, Turn on SS, MOSI, SCLK
    DDRD    = 0x04; //set port D bit 2 as output
    DDRE    = 0xff; //set port E bit 6 as output

    PORTA   = 0xFF; //set port A to all ones  (off, active low)

    spi_init();
    spi_read_send(0x00);
    
    init_tcnt0();

    sei();

    uint8_t LED[5]      = {0};
    LED[2]              = 0xff;
    uint8_t i           =  0 ;
    while(1){

/******************************************************************************************************/
       /* 
        LED[0] = int_to_digit(TIME[0]%10);
        LED[1] = int_to_digit((TIME[0]%100)/10);
        if( TIME[0]%2 )    
            LED[2] = 0xfc;
        else
            LED[2] = 0xff;
        LED[3] = int_to_digit(TIME[1]%10);
        LED[4] = int_to_digit((TIME[1]%100)/10);
        */

        LED[0] = int_to_digit(OCR2%10);
        LED[1] = int_to_digit((OCR2%100)/10);
        LED[2] = 0x07;
        LED[3] = int_to_digit((OCR2%1000)/100);
        LED[4] = int_to_digit(OCR2/1000);

        for(i=0; i<5; i++){
            PORTA = 0xFF;           //all segments off
            PORTA = LED[i];         //push button determines which segment is on
            PORTB = ( (i) << 4 );   //digit i  on 
            _delay_us(LED_DELAY);
            PORTA = 0xFF;           //all segments off
            PORTB = 0xF0;           //digit i  off 
        }

      } //while
}  //main



