// build: works. 

// Version 7: 

// features:    - speakers on: music.
//              - dimming using photosensor, ADC, and OC2.
//              - LCD when alarm goes off.
//              - clock.
//              - LEDs are in interrupt timer 2.
/*************************************************************************/

// Mohannad Al Arifi
// 932 09 3718
// 11.7.16

// lab4: Alarm clock

#include "kellen_music.h"
#include "functions.h"
#include "LCDDriver.h"

#define SET_VOLUME  1

#define LEFT_DIGITS     MINUTES
#define RIGHT_DIGITS    HOURS

volatile uint8_t LED[5]     = {0};
volatile uint8_t CLOCK[3]   = {55,0,0};
volatile uint8_t ALARM[3]   = {0,1,0};
volatile uint8_t mode       = 0;
volatile uint8_t am_pm      = 0;
extern volatile uint16_t beat;

uint8_t switch_buttons(uint8_t mode);
uint8_t snooze_pushed(uint8_t mode, uint8_t old_mode);
void read_encoders( uint8_t mode);
uint8_t clock_update();
uint8_t clock_conversion();
uint8_t LED_seg(uint8_t mode, uint8_t clock, uint8_t alarm, uint8_t digit);

void init_tcnt2(){
    TIMSK |= (1<<TOIE2);                             //enable timer overflow interrupt.
    TCCR2 |= (1<<WGM20)|(1<<WGM21)|(1<<COM21)|(1<<COM20)|(0<<CS22)|(1<<CS21)|(0<<CS20); //Fast PWM, (non-inverting), 64 prescale 
}
/*************************************************************************/
//                           timer/counter2 ISR                          
//When the TCNT2 overflow interrupt occurs, the count_7ms variable is    
//incremented.  Every 7680 interrupts the minutes counter is incremented.
//tcnt0 interrupts come at 7.8125ms internals.
// (1/16M)          = 62.5nS
// (1/16M)*256*64   = 1.02mS = ~977 Hz
/*************************************************************************/
ISR(TIMER2_OVF_vect){
    static uint8_t i        = 0;
    static uint8_t counter  = 0;
    static uint8_t ms       = 0;


    counter++;
    if( counter == 64){
        counter = 0;    
        if( (mode>>ALARM_ON)&0x01 ){
            ms++;
            if(ms % 8 == 0) {
                //for note duration (64th notes) 
                beat++;
            }
        }
        mode     = switch_buttons(mode); 
    }


    PORTA = 0xff;
    PORTB = 0xF0;           //digit i  off 

    _delay_us(2);

    PORTA = LED[i];         //push button determines which segment is on
    PORTB = ( (i) << 4 );   //digit i  on 

    if(++i >= 5)
        i=0;
        
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
    LED[0] = LED_seg(mode, CLOCK[LEFT_DIGITS], ALARM[MINUTES], 0);
    LED[1] = LED_seg(mode, CLOCK[LEFT_DIGITS], ALARM[MINUTES], 1);
    LED[3] = LED_seg(mode, CLOCK[RIGHT_DIGITS], ALARM[HOURS], 3);
    LED[4] = LED_seg(mode, CLOCK[RIGHT_DIGITS], ALARM[HOURS], 4);

}

int main()
{

    DDRA    |= 0xFF;                //A all outputs
    DDRB    |= 0xF7;                //B bits 4-7 B as outputs, Turn on SS, MOSI, SCLK
    DDRD    |= (1<<PD7);            //D bit 7 output: music_signal
    DDRE    |= (1<<PE6) | (1<<PE3); //E bit 6 output: encoder SH/LD , 3 output: volume signal
    DDRF    |= (1<<PF3);            //F bit 3 as output, bit 7 ADC input.

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

    music_init();

    int8_t old_mode  = 0;
    uint8_t adc_read = 0;
while(1){
	// reading photosensor and dimming appropiatly.
        OCR2     = 255 - read_adc();

	// check alarm
        if( ( (CLOCK[MINUTES])==(ALARM[MINUTES]) ) &&  ( (CLOCK[HOURS])==(ALARM[HOURS]) ) ){
            if(!ALARM_NOW)
                music_on();   
            mode |= (1<<ALARM_ON);
        }

        // check snooze button
        mode     = snooze_pushed(mode, old_mode);
        old_mode = mode;

        // read encoders: set time, set alarm, and set volume.
        read_encoders(mode);

    } //while
}  //main



















/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

uint8_t switch_buttons(uint8_t mode){
    uint8_t local_mode = mode;

    PORTB   = 0x50; //enable tri-state 
    DDRA    = 0x00; //set port A inputs.
    PORTA   = 0xff; //Enable pull-up resistors
    asm("nop");
    local_mode    = check_buttons(local_mode, SET_CLOCK);
    local_mode    = check_buttons(local_mode, SET_ALARM);
    local_mode    = check_buttons(local_mode, SNOOZE);
    local_mode    = check_buttons(local_mode, AM_PM);
    DDRA    = 0xff;  //set port A outputs.
    asm("nop");

    return local_mode;
}

uint8_t snooze_pushed(uint8_t mode, uint8_t old_mode){

    uint8_t  local_mode = mode;

    if( (local_mode>>SNOOZE)&0x01 ){ //if snooze button is pressed
        local_mode &= ~(1<<SNOOZE); 
        local_mode &= ~(1<<ALARM_ON); //Don't shout.
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
    }

    if( (MODE_12HR) != ((old_mode>>AM_PM)&0x01) ) //if AM_PM toggled.
        clock_conversion();

    return local_mode;
}

void read_encoders( uint8_t mode){
    static uint8_t data     = 0;

    static uint8_t L_curr   = 0;
    static uint8_t L_prev   = 0;
    static uint8_t R_curr   = 0;
    static uint8_t R_prev   = 0;

    data = spi_read_send( mode );
    L_curr = (data)&0x03;
    R_curr = (data>>2)&0x03;
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

        if(tick_4R >= 4){   // 4 ticks to make knobs feel more natural.
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

// To make knobs more natural (less sampling)
        if(tick_4L >= 4){
            tick_4L = 0;
            L_cnt++;
        }
        else if(tick_4L <= -4){
            tick_4L = 0;
            L_cnt--;
        }


// Case Set Alarm
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
// Case Set Clock
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
        }
}

}

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

