// build: works. UX not tested.

// Version 5: just to make sure all the features work together.

// features:    - speakers on: music.
//              - dimming using photosensor, ADC, and OC2.
//              - LCD when alarm goes off.
//              - clock.

// TODO v6:
//      user choose music 
//      music goes off when alarm is on
//      convert to 12hr mode when returning the segments rep.
//      
/*************************************************************************/

// Mohannad Al Arifi
// 932 09 3718
// 11.7.16

// lab4: Alarm clock

#include "kellen_music.h"
#include "functions.h"
#include "LCDDriver.h"

#define SET_VOLUME  1
#define LED_DELAY   200 //us

#define ALARM_ON    2 
#define ALARM_NOW       (mode>>ALARM_ON)&0x01
#define SET_CLOCK   4 
#define SET_CLOCK_MODE  (mode>>SET_CLOCK)&0x01
#define AM_PM       5 
#define MODE_12HR       (mode>>AM_PM)&0x01
#define SET_ALARM   6 
#define SET_ALARM_MODE  (mode>>SET_ALARM)&0x01
#define SNOOZE      7

#define SECONDS     0
#define MINUTES     1
#define HOURS       2

volatile uint8_t LED[5]     = {0};
volatile uint8_t CLOCK[3]   = {55,0,0};
volatile uint8_t ALARM[3]   = {0,1,0};
volatile uint8_t mode       = 0;
volatile uint8_t am_pm      = 0;

uint8_t clock_conversion();
uint8_t clock_update();
uint8_t LED_seg(uint8_t mode, uint8_t clock, uint8_t alarm, uint8_t digit);

void init_adc(){
    ADMUX   = (0<<REFS1) | (1<<REFS0) | (1<<ADLAR) |(1<<MUX2)|(1<<MUX1)|(1<<MUX0); //AVCC refrence, Left justified.
    ADCSRA  = (1<<ADEN) | (0<<ADSC) | (0<<ADFR) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //enable ADC, prescale by 128
}
uint8_t read_adc(){
    ADCSRA |= (1<<ADSC); //start conversion.
    while(bit_is_clear(ADCSRA, ADIF)){}
    ADCSRA |= (1<<ADIF);
    return ADCW>>6;
}

// PORTD bit 7 -> alarm signal.
// PORTE bit 3 -> volume.
extern volatile uint16_t beat;
void init_tcnt3(){
    TCCR3A |= (1<<COM3A1) | (0<<COM3A0)  | (1<<WGM31) | (0<<WGM30);
    TCCR3B |= (1<<WGM33) | (1<<WGM32) | (1<<CS30);
    ICR3   = 0x0fff;
    OCR3A  = 0x0fff/3; //Volume duty cycle.
}

void init_tcnt2(){
    TIMSK |= (1<<TOIE2);                             //enable timer overflow interrupt.
    TCCR2 |= (1<<WGM20)|(1<<WGM21)|(1<<COM21)|(1<<COM20)|(0<<CS22)|(0<<CS21)|(1<<CS20); //Fast PWM, (non-inverting), 256 prescale 
}

/*************************************************************************/
//                           timer/counter2 ISR                          
//When the TCNT2 overflow interrupt occurs, the count_7ms variable is    
//incremented.  Every 7680 interrupts the minutes counter is incremented.
//tcnt0 interrupts come at 7.8125ms internals.
// (1/16M)          = 62.5nS
// (1/16M)*256*256  = 4.09mS = ~244 Hz
/*************************************************************************/
ISR(TIMER2_OVF_vect){
    static uint8_t counter = 0;
    counter++;
    
if(counter == 128){
    counter =   0;

    static uint8_t data     = 0;
    static int8_t old_mode  = 0;
    
/******************************************************************************************************/
    OCR2 = 255;
    read_adc();

/******************************************************************************************************/
//TODO: Improvements:   alarm goes off in both 12am or 24hr mode
//                      snoozing push the freaking 10 minutes but doesn't change alarm.             
    if( ( (CLOCK[MINUTES])==(ALARM[MINUTES]) ) &&  ( (CLOCK[HOURS])==(ALARM[HOURS]) ) ){
        if(!ALARM_NOW)
            music_on();   
        mode |= (1<<ALARM_ON);
    }

/******************************************************************************************************/

    static uint8_t ms       = 0;
    if( (mode>>ALARM_ON)&0x01 ){
        ms++;
        if(ms % 16 == 0) {
            //for note duration (64th notes) 
            beat++;
        }
    }

/******************************************************************************************************/

    PORTB   = 0x50; //enable tri-state 
    DDRA    = 0x00; //set port A inputs.
    PORTA   = 0xff; //Enable pull-up resistors
    asm("nop");
    mode    = check_buttons(mode, SET_CLOCK);
    mode    = check_buttons(mode, SET_ALARM);
    mode    = check_buttons(mode, SNOOZE);
    mode    = check_buttons(mode, AM_PM);
    DDRA    = 0xff;  //set port A outputs.
    asm("nop");
    
/******************************************************************************************************/

   if( (mode>>SNOOZE)&0x01 ){ //if snooze button is pressed
       mode &= ~(1<<SNOOZE); 
       mode &= ~(1<<ALARM_ON); //Don't shout.
       ALARM[MINUTES] += 10 ;
       LCD_MovCursor(1,0);                      //~ 0.1ms
       LCD_PutStr("Next Alarm:     ");          //~ 1.6ms
       LCD_MovCursor(2,7);                      //~ 0.1ms
       if(ALARM[HOURS]<10)
           LCD_PutChar('0');
       LCD_PutDec8(ALARM[HOURS]);               //~ 0.2ms 
       LCD_PutChar(':');                        //~ 0.1ms
       LCD_PutDec8(ALARM[MINUTES]);             //~ 0.2ms

       music_off();   
       ms   = 0;
   }

   if( ((mode>>AM_PM)&0x01)!=((old_mode>>AM_PM)&0x01) ) //if AM_PM toggled.
       clock_conversion();

   old_mode = mode;


/******************************************************************************************************/

    static uint8_t L_curr   = 0;
    static uint8_t L_prev   = 0;
    static uint8_t R_curr   = 0;
    static uint8_t R_prev   = 0;
    data = spi_read_send( mode );
    L_curr = (data)&0x03;
    R_curr = (data>>2)&0x03;

/******************************************************************************************************/
//TODO: improvment: skip this whole part if neither set_clock/set_alarm are pressed.
//                  fix setting clock in 12hr mode.

#define CW 1 
#define CCW 2 

    int8_t R_cnt    = 0;
    int8_t L_cnt    = 0;

    static uint8_t sw_table[]   = {0,1,2,0,2,0,0,1,1,0,0,2,0,2,1,0};
    uint8_t sw_index            = 0;
    uint8_t dir                 = 0;
    int8_t enc_mode = 0;

    uint8_t _set_clock_mode = SET_CLOCK_MODE;
    uint8_t _set_alarm_mode = SET_ALARM_MODE;

    static int8_t tick_4R = 0;
    static int8_t tick_4L = 0;

if( _set_alarm_mode || _set_clock_mode || SET_VOLUME){ // if mode is either : set clock, set alarm
        if( _set_clock_mode ){          //priority for setting clock.
            L_cnt   = CLOCK[MINUTES];
            R_cnt   = CLOCK[HOURS];
            enc_mode= SET_CLOCK;
        }
        else if( _set_alarm_mode ){
            L_cnt   = ALARM[MINUTES];
            R_cnt   = ALARM[HOURS];
            enc_mode= SET_ALARM;
        }

        sw_index    = (R_prev<<2)|R_curr;
        R_prev      = R_curr;
        dir         = sw_table[sw_index];
        if(dir == CW) {tick_4R++;}
        if(dir == CCW){tick_4R--;}

        if(tick_4R >= 4){
            tick_4R = 0;
            R_cnt++;
        }
        else if(tick_4R <= -4){
            tick_4R = 0;
            R_cnt--;
        }
        
        sw_index= (L_prev<<2)|L_curr;
        dir     = sw_table[sw_index];
        L_prev = L_curr;
        if(dir == CW) {tick_4L++;}
        if(dir == CCW){tick_4L--;}

        if(tick_4L >= 4){
            tick_4L = 0;
            L_cnt++;
        }
        else if(tick_4L <= -4){
            tick_4L = 0;
            L_cnt--;
        }


        if(enc_mode == SET_ALARM){
            if(L_cnt >= 60)
                L_cnt= L_cnt-60;
            else if(L_cnt < 0)
                L_cnt=60+L_cnt;

            if(R_cnt >= 24)
                R_cnt=0;
            else if(R_cnt < 0)
                R_cnt=23;

            ALARM[MINUTES] = L_cnt;
            ALARM[HOURS]   = R_cnt;
            LED[0] = LED_seg(mode, CLOCK[SECONDS], ALARM[MINUTES], 0);
            LED[1] = LED_seg(mode, CLOCK[SECONDS], ALARM[MINUTES], 1);
            LED[3] = LED_seg(mode, CLOCK[MINUTES], ALARM[HOURS], 3);
            LED[4] = LED_seg(mode, CLOCK[MINUTES], ALARM[HOURS], 4);
        }
        else if(enc_mode == SET_CLOCK){
            if(L_cnt >= 60)
                L_cnt= L_cnt-60;
            else if(L_cnt < 0)
                L_cnt=60+L_cnt;

            if(R_cnt >= 24)
                R_cnt=0;
            else if(R_cnt < 0)
                R_cnt=23;

                CLOCK[SECONDS] = 0;
            CLOCK[MINUTES] = L_cnt;
            CLOCK[HOURS]   = R_cnt;
            LED[0] = LED_seg(mode, CLOCK[MINUTES], ALARM[MINUTES], 0);
            LED[1] = LED_seg(mode, CLOCK[MINUTES], ALARM[MINUTES], 1);
            LED[3] = LED_seg(mode, CLOCK[HOURS], ALARM[HOURS], 3);
            LED[4] = LED_seg(mode, CLOCK[HOURS], ALARM[HOURS], 4);
        }

        else {
            static int16_t volume  = 0x0FFF/2;
            volume += R_cnt*50;
            volume += L_cnt*50;
            if(volume>0x0fff) 
               volume = 0x0fff; 
            if (volume<51) 
                volume = 52;
            
            OCR3A   = volume;
            /*
            static int8_t volume_prescale = 0;
            volume_prescale += R_cnt;
            volume_prescale += L_cnt;
            if(volume_prescale >= 4) 
               volume_prescale = 4; 
            if(volume_prescale <= 0) 
                volume_prescale = 0;
            OCR3A   = 0x0fff/volume_prescale;
            */
        }

    }
}

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
    CLOCK[SECONDS] += 1; //increment seconds.
    clock_update();
    
    if( (mode >> ALARM_ON) & 0x01 ){
        if(CLOCK[SECONDS]%2){
            LCD_MovCursor(1,0);
            LCD_PutStr("   Wake Up !!!  ");
        }
        else{
            LCD_MovCursor(1,0);
            LCD_PutStr(" Dont miss BUS! "); 
        }
    }


    // load the LED with the right segments representation.
    if( CLOCK[0]%2 )    
        LED[2] = 0xfc;
    
    else
        LED[2] = 0xff;
    LED[0] = LED_seg(mode, CLOCK[SECONDS], ALARM[MINUTES], 0);
    LED[1] = LED_seg(mode, CLOCK[SECONDS], ALARM[MINUTES], 1);
    LED[3] = LED_seg(mode, CLOCK[MINUTES], ALARM[HOURS], 3);
    LED[4] = LED_seg(mode, CLOCK[MINUTES], ALARM[HOURS], 4);

}

int main()
{

    DDRA    |= 0xFF;                //A all outputs
    DDRB    |= 0xF7;                //B bits 4-7 B as outputs, Turn on SS, MOSI, SCLK
    DDRD    |= (1<<PD7);            //D bit 7 output: music_signal
    DDRE    |= (1<<PE6) | (1<<PE3); //E bit 6 output: encoder SH/LD , 3 output: volume signal
    DDRF    |= (1<<PF3);                //F bit 3 as output

    PORTA   = 0xFF;                 //A to all ones  (off, active low)

    spi_init();
    
    LCD_Init();
    LCD_Clr();
    LCD_PutStr("      ->!<-     ");
    
    init_tcnt0();
    init_tcnt2();
    init_tcnt3(); 

    init_adc();

    sei();
    OCR2 = 55;

    music_init();
    music_on();   

    while(1){

            PORTA = LED[0];         //push button determines which segment is on
            PORTB = ( (0) << 4 );   //digit i  on 
            _delay_us(LED_DELAY);
            PORTA = 0xFF;           //all segments off
            PORTB = 0xF0;           //digit i  off 
//            _delay_us(LED_DELAY);
            PORTA = LED[1];         //push button determines which segment is on
            PORTB = ( (1) << 4 );   //digit i  on 
            _delay_us(LED_DELAY);
            PORTA = 0xFF;           //all segments off
            PORTB = 0xF0;           //digit i  off 
//            _delay_us(LED_DELAY);
            PORTA = LED[2];         //push button determines which segment is on
            PORTB = ( (2) << 4 );   //digit i  on 
            _delay_us(LED_DELAY);
            PORTA = 0xFF;           //all segments off
            PORTB = 0xF0;           //digit i  off 
//            _delay_us(LED_DELAY);
            PORTA = LED[3];         //push button determines which segment is on
            PORTB = ( (3) << 4 );   //digit i  on 
            _delay_us(LED_DELAY);
            PORTA = 0xFF;           //all segments off
            PORTB = 0xF0;           //digit i  off 
//            _delay_us(LED_DELAY);
            PORTA = LED[4];         //push button determines which segment is on
            PORTB = ( (4) << 4 );   //digit i  on 
            _delay_us(LED_DELAY);
            PORTA = 0xFF;           //all segments off
            PORTB = 0xF0;           //digit i  off 
//            _delay_us(LED_DELAY);

/******************************************************************************************************/

    } //while
}  //main




/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/


uint8_t clock_conversion(){

    if( (mode>>AM_PM) & 0x01 ){ // convert to 12 mode
        if( CLOCK[HOURS] > 12 ){
            CLOCK[HOURS] -= 12;
            am_pm = 1; //pm
        }
        else if(CLOCK[HOURS] == 12)
            am_pm = 1; //pm
        else if(CLOCK[HOURS] == 0){
            CLOCK[HOURS] = 12;
            am_pm = 0; //am
        }
        else
            am_pm = 0; //am
    }
    else{
        if(am_pm){  // if pm 
            if(CLOCK[HOURS] != 12)
                CLOCK[HOURS] += 12;
        }
        else{
            if(CLOCK[HOURS] == 12) //if 12am
                CLOCK[HOURS] = 0;
        }
        am_pm = 0; // turn off
    }

    return 0;
}

uint8_t clock_update(){
    static uint8_t old_sec = 0;
    
    if(CLOCK[SECONDS] != old_sec) //clock ticked. update it. No waste of time.
    {
        if( CLOCK[SECONDS] >= 60 ){
            CLOCK[SECONDS] -= 60;
            CLOCK[MINUTES] += 1;
            if( CLOCK[MINUTES] >= 60 ){
                CLOCK[MINUTES] -= 60;
                CLOCK[HOURS]++;
                switch( (mode>>AM_PM)&0x01 ){   //if 12hr mode or else 24hr mode.
                    case 1:
                        if( CLOCK[HOURS] == 12 ) 
                            am_pm ^= (1<<7);    // flip am_pm.
                        else if( CLOCK[HOURS] > 12 )
                            CLOCK[HOURS] -= 12;
                        break;
                    default:
                        if( CLOCK[HOURS] >= 24 )
                            CLOCK[HOURS] -= 24;
                        break;
                }
            }
        }
        old_sec = CLOCK[SECONDS];
    }

    return 0;
}

uint8_t LED_seg(uint8_t mode, uint8_t clock, uint8_t alarm, uint8_t digit){

#define LETTER_A 0b10001000 
#define LETTER_H 0b10001001


    uint8_t right_digit = clock - 10*(clock/10);
    uint8_t left_digit  = clock / 10;
    switch(digit){
        case 0:
            if( SET_ALARM_MODE )
                return int_to_digit(alarm - 10*(alarm/10));

            else if( ALARM_NOW )
                    return LETTER_H;
            else{
                if( MODE_12HR )     // if display mode is 12 hr, indicate am or pm in digit 0's dot.
                    return int_to_digit(right_digit) & ~(am_pm<<7);

                return int_to_digit(right_digit);
            }

        case 1:
            if( SET_ALARM_MODE )
                return int_to_digit(alarm/10);

            else if( ALARM_NOW )
                return LETTER_A;

            else
                return int_to_digit(left_digit);

        case 3:
            if( SET_ALARM_MODE )
                return int_to_digit(alarm-10*(alarm/10));
            
            else if( ALARM_NOW )
                return LETTER_A;

            else
                return int_to_digit(right_digit);

        case 4:
            if( SET_ALARM_MODE )
                return int_to_digit(alarm/10);

            else if( ALARM_NOW )
                return LETTER_A;

            else
                return int_to_digit(left_digit);

        default:
            return 0b10111111; // return '-'
    }


    //cases: CLOCK(default)
    //          - 24hr mode
    //          - 12hr mode
    //              -- time AM
    //              -- time PM
    //       setting alarm mode
    //          - alarm time in 24hr mode
    //       Alarm off mode
    //          - AAAH
}

