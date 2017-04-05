//    This code keeps clock time in "CLOCK[]". Update seconds using TCNT0 OVR interrupt that occures evey second.
//  In main, convert CLOCK/ALARM to segments and then loop through all four digits to display this value. When 'set 
//  clock mode' is pressed, change the values in CLOCK[] using the encoders. If 'set alarm mode' is pressed, change
//  the values in ALARM[] using encoders. Priority is for setting CLOCK[]. When snooze is pressed, the alarm get
//  pushed for 10 minutes.

//  main display:   - Freak because it is ALARM time.
//                  - Display ALARM if 'set alarm' is on. 
//                  - Display CLOCK.

// TODO:    -

/*************************************************************************/
// test: setting the alarm, snoozing, keeping time.
/*************************************************************************/
// TCNT0 OVR interrupt :
//      update seconds only.
// TCNT2 OVR interrupt :
//      check:   switch button   B4 -> set clock mode
//                               B5 -> set alarm mode
//                               B6 -> set 12 hr mode
//                               B7 -> snooze the alarm for 10 minutes.
//      check:   encoders
/*************************************************************************/

// Mohannad Al Arifi
// 932 09 3718
// 10.18.16

// lab4: Alarm clock


#include "functions.h"

#define LED_DELAY 150

#define FREAK_ALARM 2 
#define SET_CLOCK   4 
#define AM_PM       5 
#define SET_ALARM   6 
#define SNOOZE      7

#define SECONDS 0
#define MINUTES 1
#define HOURS   2


volatile uint8_t CLOCK[3]   = {55,0,0};
volatile uint8_t ALARM[3]   = {0,2,0};
volatile uint8_t mode       = 0;
volatile uint8_t am_pm      = 0;

uint8_t clock_conversion();
uint8_t clock_update();
uint8_t LED_seg(uint8_t mode, uint8_t clock, uint8_t alarm, uint8_t digit);


void init_tcnt2(){
    //enable timer overflow interrupt.
    TIMSK |= (1<<TOIE2);
    //mode: Fast PWM | Compare Output mode: non-inverting | Prescale: 256 
    TCCR2 |= (1<<WGM20)|(1<<WGM21)|(1<<COM21)|(1<<CS22)|(0<<CS21)|(0<<CS20);
}

/*************************************************************************/
//                           timer/counter2 ISR                          
//When the TCNT2 overflow interrupt occurs, the count_7ms variable is    
//incremented.  Every 7680 interrupts the minutes counter is incremented.
//tcnt0 interrupts come at 7.8125ms internals.
// (1/16M)          = 62.5nS
// (1/16M)*256*256  = 4.09mS
/*************************************************************************/
ISR(TIMER2_OVF_vect){
    static uint8_t data     = 0;
    int8_t R_cnt    = 0;
    int8_t L_cnt    = 0;
    static int8_t old_mode  = 0;
    
/******************************************************************************************************/
//TODO: Improvements:   alarm goes off in both 12am or 24hr mode
//                      snoozing push the freaking 10 minutes but doesn't change alarm.             
    if( ( (CLOCK[MINUTES])==(ALARM[MINUTES]) ) &&  ( (CLOCK[HOURS])==(ALARM[HOURS]) ) )
        mode |= (1<<FREAK_ALARM);
    else
        mode &= ~(1<<FREAK_ALARM); 

/******************************************************************************************************/

    PORTB   = 0x50; //enable tri-state 
    DDRA    = 0x00; //set port A inputs.
    PORTA   = 0xff; //Enable pull-up resistors
    asm("nop");
//    asm("nop");
    mode    = check_buttons(mode, SET_CLOCK);
    mode    = check_buttons(mode, SET_ALARM);
    mode    = check_buttons(mode, SNOOZE);
    mode    = check_buttons(mode, AM_PM);
    DDRA    = 0xff;  //set port A outputs.
 //   asm("nop");
    asm("nop");
    
/******************************************************************************************************/

   if( (mode>>SNOOZE)&0x01 ){ //if snooze button is pressed
       mode &= ~(1<<SNOOZE); 
       ALARM[MINUTES] += 10 ;
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
    init_tcnt2();

    sei();

    uint8_t LED[5]      = {0};
    LED[2]              = 0xff;
    uint8_t i           = 0;

    while(1){

        clock_update();
/******************************************************************************************************/

        LED[0] = LED_seg(mode, CLOCK[MINUTES], ALARM[MINUTES], 0);
        LED[1] = LED_seg(mode, CLOCK[MINUTES], ALARM[MINUTES], 1);
        if( CLOCK[0]%2 )    
            LED[2] = 0xfc;
        else
            LED[2] = 0xff;
        LED[3] = LED_seg(mode, CLOCK[HOURS], ALARM[HOURS], 3);
        LED[4] = LED_seg(mode, CLOCK[HOURS], ALARM[HOURS], 4);

/******************************************************************************************************/

        switch(mode){   //Display modes: alarm fraeking, clock, set alarm.
            case (1<<FREAK_ALARM): 
                if(CLOCK[0]%2)
                    break;
            default: 
                for(i=0; i<5; i++){
                    PORTA = 0xFF;           //all segments off
                    PORTA = LED[i];         //push button determines which segment is on
                    PORTB = ( (i) << 4 );   //digit i  on 
                    _delay_us(LED_DELAY);
                    PORTA = 0xFF;           //all segments off
                    PORTB = 0xF0;           //digit i  off 
                }
                break;
        }

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

