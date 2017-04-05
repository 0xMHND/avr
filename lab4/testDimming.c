/*************************************************************************/
// test: dimming the LEDs using photosensor.
/*************************************************************************/

// Mohannad Al Arifi
// 932 09 3718
// 10.18.16

// lab4: Alarm clock


#include "functions.h"
#include "LCDDriver.h"
#include <string.h>

// 8us per digit for 256 prescale (interrupts off)
// 4us per digit for 256 prescale (interrupts on)
#define LED_DELAY 2
#define OCR_DELAY 5

#define FREAK_ALARM 2 
#define SET_CLOCK   4 
#define AM_PM       5 
#define SET_ALARM   6 
#define SNOOZE      7

#define SECONDS 0
#define MINUTES 1
#define HOURS   2


volatile uint8_t LED[5]     = {0};
volatile uint8_t CLOCK[3]   = {0,0,0};
volatile uint8_t ALARM[3]   = {0,1,0};
volatile uint8_t mode       = 0;
volatile uint8_t am_pm      = 0;

uint8_t clock_conversion();
uint8_t clock_update();
uint8_t LED_seg(uint8_t mode, uint8_t clock, uint8_t alarm, uint8_t digit);

void init_tcnt2(){
    TIMSK |= (1<<TOIE2);                             //enable timer overflow interrupt.
    TCCR2 |= (1<<WGM20)|(1<<WGM21)|(1<<COM21)|(1<<COM20)|(1<<CS22)|(0<<CS21)|(0<<CS20); //Fast PWM, (non-inverting), 256 prescale 
}

ISR(TIMER2_OVF_vect){
    static uint8_t data     = 0;
    int8_t R_cnt    = 0;
    int8_t L_cnt    = 0;
    static int8_t old_mode  = 0;
    
    OCR2 = 155;

    if( ( (CLOCK[MINUTES])==(ALARM[MINUTES]) ) &&  ( (CLOCK[HOURS])==(ALARM[HOURS]) ) )
        mode |= (1<<FREAK_ALARM);


    PORTA   = 0xff; //Enable pull-up resistors
    PORTB   = 0x50; //enable tri-state 
    DDRA    = 0x00; //set port A inputs.
    asm("nop");
    mode    = check_buttons(mode, SET_CLOCK);
    mode    = check_buttons(mode, SET_ALARM);
    mode    = check_buttons(mode, SNOOZE);
    mode    = check_buttons(mode, AM_PM);
    DDRA    = 0xff;  //set port A outputs.
    asm("nop");
   
   if( (mode>>SNOOZE)&0x01 ){ //if snooze button is pressed
       mode &= ~(1<<SNOOZE); 
       mode &= ~(1<<FREAK_ALARM); //Don't shout.
       ALARM[MINUTES] += 10 ;
       LCD_MovCursor(1,0);                      //~ 0.1ms
       LCD_PutStr("Next Alarm:     ");          //~ 1.6ms
       LCD_MovCursor(2,7);                      //~ 0.1ms
       if(ALARM[HOURS]<10)
           LCD_PutChar('0');
       LCD_PutDec8(ALARM[HOURS]);               //~ 0.2ms 
       LCD_PutChar(':');                        //~ 0.1ms
       LCD_PutDec8(ALARM[MINUTES]);             //~ 0.2ms
   }
   if( ((mode>>AM_PM)&0x01)!=((old_mode>>AM_PM)&0x01) ) //if AM_PM toggled.
       clock_conversion();

   old_mode = mode;


    static uint8_t L_curr   = 0;
    static uint8_t L_prev   = 0;
    static uint8_t R_curr   = 0;
    static uint8_t R_prev   = 0;


    data = spi_read_send( mode );
    L_curr = (data)&0x03;
    R_curr = (data>>2)&0x03;

#define CW 1 
#define CCW 2 
    static uint8_t sw_table[]   = {0,1,2,0,2,0,0,1,1,0,0,2,0,2,1,0};
    uint8_t sw_index            = 0;
    uint8_t dir                 = 0;
    int8_t enc_mode = 0;

    if( (mode>>SET_CLOCK)&0x01 ){
        R_cnt   = CLOCK[MINUTES];
        L_cnt   = CLOCK[HOURS];
        enc_mode= SET_CLOCK;
    }
    else if( (mode>>SET_ALARM)&0x01 ){
        R_cnt   = ALARM[MINUTES];
        L_cnt   = ALARM[HOURS];
        enc_mode= SET_ALARM;
    }

    sw_index    = (R_prev<<2)|R_curr;
    R_prev      = R_curr;
    dir         = sw_table[sw_index];
    switch( (mode&(1<<SET_ALARM)) || (mode&(1<<SET_CLOCK)) ){ // if mode is either : set clock, set alarm
        case 1:
            if(dir == CW) {R_cnt++;}
            if(dir == CCW){R_cnt--;}
            break;
        default:
            break;
    }

    sw_index= (L_prev<<2)|L_curr;
    dir     = sw_table[sw_index];
    L_prev = L_curr;
    switch( (mode&(1<<SET_ALARM)) || (mode&(1<<SET_CLOCK)) ){ // if mode is either : set clock, set alarm
        case 1:
            if(dir == CW) {L_cnt++;}
            if(dir == CCW){L_cnt--;}
            break;
        default:
            break;
    }


    if(R_cnt >= 60)
        R_cnt= R_cnt-60;
    else if(R_cnt < 0)
        R_cnt=60+R_cnt;

    if(L_cnt >= 24)
        L_cnt=0;
    else if(L_cnt < 0)
        L_cnt=23;


    if(enc_mode == SET_ALARM){
        ALARM[MINUTES] = R_cnt;
        ALARM[HOURS]   = L_cnt;
    }
    else if(enc_mode == SET_CLOCK){
        CLOCK[SECONDS] = 0;
        CLOCK[MINUTES] = R_cnt;
        CLOCK[HOURS]   = L_cnt;
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
    
    if( (mode >> FREAK_ALARM) & 0x01 ){
        if(CLOCK[SECONDS]%2){
            LCD_MovCursor(1,0);
            LCD_PutStr("   Wake Up !!!  ");
        }
        else{
            LCD_MovCursor(1,0);
            LCD_PutStr(" Dont miss BUS! "); 
        }
    }

    LED[0] = LED_seg(mode, CLOCK[MINUTES], ALARM[MINUTES], 0);
    LED[1] = LED_seg(mode, CLOCK[MINUTES], ALARM[MINUTES], 1);

    if( CLOCK[0]%2 )    
        LED[2] = 0xfc;
    else
        LED[2] = 0xff;

    LED[3] = LED_seg(mode, CLOCK[HOURS], ALARM[HOURS], 3);
    LED[4] = LED_seg(mode, CLOCK[HOURS], ALARM[HOURS], 4);
}

int main()
{

    DDRA    |= 0xFF; //set port A to all outputs
    DDRB    |= 0xF7; //set port B bits 4-7 B as outputs, Turn on SS, MOSI, SCLK
    DDRD    |= 0x04; //set port D bit 2 as output
    DDRE    |= 0xff; //set port E bit 6 as output
    DDRF    |= 0x08; //set port F bit 3 as output

    PORTA   = 0xFF; //set port A to all ones  (off, active low)

    spi_init();
    
    
    LCD_Init();
    char* LCD_str = "----------------";
    strcpy(LCD_str, "      ->!<-     ");
    LCD_Clr();
    LCD_PutStr(LCD_str);
    

    init_tcnt0();
    init_tcnt2();
    OCR2 = OCR_DELAY;

    sei();

    LED[0] = int_to_digit(0);
    LED[1] = int_to_digit(1);
    LED[2] = 0xfc;
    LED[3] = int_to_digit(3);
    LED[4] = int_to_digit(4);
    uint8_t i           = 0;

    while(1){

/******************************************************************************************************/

        switch(mode){   //Display modes: alarm fraeking, clock, set alarm.
            case (1<<FREAK_ALARM): 
                if(CLOCK[0]%2)
                    break;
            default: 
                for(i=0; i<5; i++){
                    PORTA = LED[i];         //push button determines which segment is on
                    PORTB = ( (i) << 4 );   //digit i  on 
                    _delay_us(LED_DELAY);
                    PORTA = 0xFF;           //all segments off
//                   _delay_us(LED_DELAY);
                    PORTB = 0xF0;           //digit i  off 
                    _delay_us(LED_DELAY);
                }
                break;
        }

/******************************************************************************************************/

      } //while
}  //main

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
    switch(digit){
        case 0:
            if( (mode>>SET_ALARM) & 0x01 )
                return int_to_digit(alarm%10);
            else if( (mode>>FREAK_ALARM) & 0x01 )
                return LETTER_H;
            else{
                if( (mode>>AM_PM) & 0x01)                           // if display mode is 12 hr, indicate am or pm in digit 0's dot.
                    return int_to_digit(clock%10) & ~(am_pm<<7);
                return int_to_digit(clock%10);
            }
        case 1:
            if( (mode>>SET_ALARM) & 0x01 )
                return int_to_digit((alarm%100)/10);
            else if( (mode>>FREAK_ALARM) & 0x01 )
                return LETTER_A;
            else
                return int_to_digit((clock%100)/10);
        case 3:
            if( (mode>>SET_ALARM) & 0x01 )
                return int_to_digit(alarm%10);
            else if( (mode>>FREAK_ALARM) & 0x01 )
                return LETTER_A;
            else
                return int_to_digit(clock%10);
        case 4:
            if( (mode>>SET_ALARM) & 0x01 )
                return int_to_digit((alarm%100)/10);
            else if( (mode>>FREAK_ALARM) & 0x01 )
                return LETTER_A;
            else
                return int_to_digit((clock%100)/10);
        default:
            return 0b10111111; // return '-'
    }
}

