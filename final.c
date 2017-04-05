// build: working
  
// Version 5: 

// features:    - speakers on: music.
//              - dimming using photosensor, ADC, and OC2.
//              - LCD when alarm goes off.
//              - clock.
//              - LEDs are in interrupt timer 2.
//              - Local & Outside temperature reading.
//              - user chooses temperature resolution.  switch button 1 from left
//              - user chooses temperature unit.        switch button 0 from left
//              - radio is on by default
//              - user tunes frequency			switch button 3 from left


// Mohannad Al Arifi
// 932 09 3718
// 12.3.16

// lab6: radio 

#include "music/kellen_music.h"
#include "functions/functions.h"
#include "LCD/LCDDriver.h"
#include "lm73/lm73_functions.h"
#include "twi_master/twi_master.h"
#include "uart/uart_functions.h"
#include "si4734_driver/si4734.h"

uint16_t current_fm_freq = 10790;
extern uint8_t si4734_tune_status_buf[8]; //buffer for holding tune_status data  

#define SET_VOLUME  1

#define LEFT_DIGITS     MINUTES
#define RIGHT_DIGITS    HOURS

volatile uint8_t  rcv_rdy;
char              rx_char; 
char              lcd_str_array[16];  //holds string to send to lcd

#define CELSIUS         0
#define FAHRENHEIT      1
volatile uint8_t temp_unit  = CELSIUS;
volatile uint8_t resolution = 0;
char first_line[16];
char second_line[16];
char song_name[5][16];
char outside_temp[16];
char local_temp[16];
void read_tempreture();
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];


volatile uint8_t LED[5]     = {0};
volatile uint8_t CLOCK[3]   = {0,0,0};
volatile uint8_t ALARM[3]   = {0,1,1};
volatile uint8_t mode       = 0;
volatile uint8_t am_pm      = 0;
volatile uint8_t alarm_on   = 0;
extern volatile uint16_t beat;
extern volatile uint8_t song;

void check_alarm();
uint8_t switch_buttons(uint8_t mode);
uint8_t snooze_pushed(uint8_t mode, uint8_t old_mode);
void read_encoders( uint8_t mode);
uint8_t clock_update();
uint8_t clock_conversion();
uint8_t LED_seg(uint8_t mode, uint8_t clock, uint8_t alarm, uint16_t frequency, uint8_t digit);

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
    if( counter == 32){
        counter = 0;    
        if( alarm_on ){
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

    if(LED[4] == 0xc0) // led[5] == 0
        LED[4] = 0xff;
    if(SET_FREQ_MODE)
        LED[2] = 0xff;

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
 
    uart_putc(temp_unit?'F':'C'); //signal start send temp for the mega48.

    // load the LED with the right segments representation.
    if( CLOCK[0]%2 )    
        LED[2] = 0xfc;
    
    else
        LED[2] = 0xff;
    LED[0] = LED_seg(mode, CLOCK[LEFT_DIGITS], ALARM[MINUTES], current_fm_freq, 0);
    LED[1] = LED_seg(mode, CLOCK[LEFT_DIGITS], ALARM[MINUTES], current_fm_freq, 1);
    LED[3] = LED_seg(mode, CLOCK[RIGHT_DIGITS], ALARM[HOURS], current_fm_freq, 3);
    LED[4] = LED_seg(mode, CLOCK[RIGHT_DIGITS], ALARM[HOURS], current_fm_freq, 4);


    if( alarm_on ){
        if(CLOCK[SECONDS] % 2)
            strcpy(first_line, "    Wake UP !   ");
        else{
            switch(song){
                case 0:
                    strcpy(first_line, " Beavers fight !");
                    break;
                case 1:
                    strcpy(first_line, " Tetris Theme  !");
                    break;
                case 2:
                    strcpy(first_line, "Super Mario Bros");
                    break;
                case 3:
                    strcpy(first_line, "-> Zelda Them <-");
                    break;
                case 4:
                    strcpy(first_line, " Beethoven 5th !");
                    break;
                default:
                    strcpy(first_line, " Uknown song   !");
                    break;
            }
        }
    }
    else
        //sprintf(first_line, "nxt Alarm: %02d:%02d", ALARM[HOURS], ALARM[MINUTES]);

#define CELSIUS_SIGN    0x05
#define FAHRENHEIT_SIGN 0x06

    switch(resolution){
        case 0:
            sprintf(second_line, "L>%3.3s O>%3.3s%c    ", local_temp, outside_temp, temp_unit?FAHRENHEIT_SIGN:CELSIUS_SIGN); //copy temp_digits to lcd_outside
            break;
        case 1:
            sprintf(second_line, "L>%5.5s O>%5.5s%c", local_temp, outside_temp, temp_unit?FAHRENHEIT_SIGN:CELSIUS_SIGN); //copy temp_digits to lcd_outside
            break;
        case 2:
            sprintf(second_line, "L%6.6s O%6.6s%c", local_temp, outside_temp, temp_unit?FAHRENHEIT_SIGN:CELSIUS_SIGN); //copy temp_digits to lcd_outside
            break;
        default:
            sprintf(second_line, "L>%3.3s O>%3.3s%c ", local_temp, outside_temp, temp_unit?FAHRENHEIT_SIGN:CELSIUS_SIGN); //copy temp_digits to lcd_outside
            break;
    }

}

void radio_reset(){
    //hardware reset of Si4734
    PORTE &= ~(1<<PE5); //int2 initially low to sense TWI mode
    DDRE  |=  (1<<PE5); //turn on Port E bit 5 to drive it low
    PORTE |=  (1<<PE2); //hardware reset Si4734 
    _delay_us(200);     //hold for 200us, 100us by spec         
    PORTE &= ~(1<<PE2); //release reset 
    _delay_us(30);      //5us required because of my slow I2C translators I suspect
                        //Si code in "low" has 30us delay...no explaination
    DDRE  &= ~(1<<PE5); //now Port E bit 5 becomes input from the radio interrupt
}
void init_radio(){

    EICRB |= (1<<ISC50) | (1<<ISC51);
    EIMSK |= (1<<INT5);

    radio_reset();
}
void radio_signal(){
    fm_rsq_status();
    uint8_t rssi;
    rssi =  si4734_tune_status_buf[4];
#define SIGNAL_SIGN 0x04
    if(rssi < 14)
        sprintf(first_line, "radio       %c   ", SIGNAL_SIGN);
    else if(rssi < 24)
        sprintf(first_line, "radio       %c%c  ", SIGNAL_SIGN,0x03);
    else if(rssi < 32)
        sprintf(first_line, "radio       %c%c%c ", SIGNAL_SIGN,0x03,0x02);
    else
        sprintf(first_line, "radio       %c%c%c%c", SIGNAL_SIGN,0x03,0x02,0x01);
}
int main()
{
    //A all outputs
    DDRA    |= 0xFF;                
    //B bits 4-7 B as outputs, Turn on SS, MOSI, SCLK
    DDRB    |= 0xF7;
    //D bit 7 output: music_signal
    DDRD    |= (1<<PD7);            
    //E bti 7 music_signal, bit 6 encoder SH/LD, bit 5 radio INT, bit 3 volume signal, bit 2 radio reset
    DDRE    |= (1<<PE7) | (1<<PE6) | (1<<PE5) | (1<<PE3) | (1<<PE2);
    //F bit 3 as output < don't know what for. bit 6 ADC photosensor 
    DDRF    |= (1<<PF3);            

    PORTA   = 0xFF;                 //A to all ones  (off, active low)


    uart_init();  
    spi_init();
    
    LCD_Init();
    LCD_Clr();
    LCD_PutStr("      ->!<-     ");
    
    init_tcnt0();
    init_tcnt2();
    init_tcnt3(); 

    init_adc();

    init_twi();   //initalize TWI (twi_master.h)  

    init_radio();

    sei();

    fm_pwr_up(); //powerup the radio as appropriate
    fm_tune_freq(); //tune radio to frequency in current_fm_freq

    init_temp(13); //not timeout, res: 13

    music_init();

    int8_t old_mode  = 0;
    uint8_t counter = 0;
    uint8_t j=0;
    uint8_t line=0;

while(1){
    
        radio_signal();

        // ~104 us
        OCR2     = 255 - read_adc();

        read_encoders(mode);

        if(rcv_rdy==1){
            sprintf(local_temp, "%s", lcd_str_array); //copy temp_digits to lcd_outside
            rcv_rdy=0;
        } 
        
        counter++;
	//check alarm, read temp, set lcd every other 100
        if(counter == 100){
            counter = 0;
            
            check_alarm();
            
            mode     = snooze_pushed(mode, old_mode);
            old_mode = mode;
        

            read_tempreture();

            if(line)
                LCD_PutChar(first_line[j]);
            else
                LCD_PutChar(second_line[j]);
            j++;
            if(j>=16){
                if(line){
                    LCD_MovCursorLn2();
                    line=0;
                }
                else{
                    LCD_MovCursorLn1();
                    line = 1;
                }
                j=0;
            }
        }
    } //while
}  //main

ISR(USART0_RX_vect){
//USART0 RX complete interrupt
static  uint8_t  i;
  rx_char = UDR0;              //get character
  
  lcd_str_array[i++]=rx_char;  //store in array 
 //if entire string has arrived, set flag, reset index
  if(rx_char == '\0'){
    rcv_rdy=1; 
    lcd_str_array[--i]  = (' ');     //clear the count field
    lcd_str_array[i+1]  = (' ');
    lcd_str_array[i+2]  = (' ');
    i=0;  
  }
  
}

void read_tempreture(){
    uint16_t lm73_temp = 0;

    twi_start_rd(LM73_ADDRESS, lm73_rd_buf, 2); //read temperature data from LM73 (2 bytes) 
    while(twi_busy());                    //wait till TWI interrupt is done

    lm73_temp = lm73_rd_buf[0]; //save high temperature byte into lm73_temp
    lm73_temp = lm73_temp << 8; //shift it into upper byte 
    lm73_temp |= lm73_rd_buf[1]; //"OR" in the low temp byte to lm73_temp 

    lm73_temp_convert( outside_temp, lm73_temp, temp_unit);
}














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

void check_alarm(){

    if( ( (CLOCK[MINUTES])==(ALARM[MINUTES]) ) &&  ( (CLOCK[HOURS])==(ALARM[HOURS]) ) && ( (CLOCK[SECONDS])==(ALARM[SECONDS]) ) ){
        if(!alarm_on){
            radio_pwr_dwn();
            music_on();   
        }
        alarm_on = 1;
    }
}

uint8_t switch_buttons(uint8_t mode){
    uint8_t local_mode = mode;

    PORTB   = 0x50; //enable tri-state 
    DDRA    = 0x00; //set port A inputs.
    PORTA   = 0xff; //Enable pull-up resistors
    asm("nop");
    local_mode  = check_buttons(local_mode, SET_SONG);
    if(SET_SONG_MODE){
        local_mode &= ~(1<<SET_SONG); 
        song += 1;
        if(song >= 5)
            song = 0;
    }
    local_mode  = check_buttons(local_mode, SET_FREQ);
    local_mode  = check_buttons(local_mode, SET_CLOCK);
    local_mode  = check_buttons(local_mode, SET_ALARM);
    local_mode  = check_buttons(local_mode, SNOOZE);
    local_mode  = check_buttons(local_mode, AM_PM);
    local_mode  = check_buttons(local_mode, TEMPRATURE);
    temp_unit   = TEMP_UNIT;
    local_mode  = check_buttons(local_mode, RESOLUTION);
    if(RES_CHANGE){
        local_mode &= ~(1<<RESOLUTION); 
        resolution += 1;
        if(resolution >=3)
            resolution = 0;
    }
    
    DDRA    = 0xff;  //set port A outputs.
    asm("nop");

    return local_mode;
}

uint8_t snooze_pushed(uint8_t mode, uint8_t old_mode){

    uint8_t  local_mode = mode;

    if( (local_mode>>SNOOZE)&0x01 ){ //if snooze button is pressed
        local_mode &= ~(1<<SNOOZE); 
        alarm_on = 0; //turn off
        ALARM[SECONDS] = CLOCK[SECONDS] + 10 ;
        music_off();   
        fm_pwr_up(); //powerup the radio as appropriate
        fm_tune_freq(); //tune radio to frequency in current_fm_freq
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
    uint8_t _set_freq_mode  = SET_FREQ_MODE;

    static int8_t tick_4R = 0;
    static int8_t tick_4L = 0;

if( _set_freq_mode || _set_alarm_mode || _set_clock_mode || SET_VOLUME){ // if mode is either : set clock, set alarm

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
        else if( _set_freq_mode ){
            L_cnt   = 0;
            R_cnt   = 0;
            enc_mode= SET_FREQ;
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

// To make knobs more natural (less sampling)
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

            ALARM[SECONDS] = 0;
            ALARM[MINUTES] = L_cnt;
            ALARM[HOURS]   = R_cnt;
            LED[0] = LED_seg(mode, CLOCK[LEFT_DIGITS], ALARM[MINUTES], current_fm_freq, 0);
            LED[1] = LED_seg(mode, CLOCK[LEFT_DIGITS], ALARM[MINUTES], current_fm_freq, 1);
            LED[3] = LED_seg(mode, CLOCK[RIGHT_DIGITS], ALARM[HOURS], current_fm_freq, 3);
            LED[4] = LED_seg(mode, CLOCK[RIGHT_DIGITS], ALARM[HOURS], current_fm_freq, 4);
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
            LED[0] = LED_seg(mode, CLOCK[LEFT_DIGITS], ALARM[MINUTES], current_fm_freq, 0);
            LED[1] = LED_seg(mode, CLOCK[LEFT_DIGITS], ALARM[MINUTES], current_fm_freq, 1);
            LED[3] = LED_seg(mode, CLOCK[RIGHT_DIGITS], ALARM[HOURS], current_fm_freq, 3);
            LED[4] = LED_seg(mode, CLOCK[RIGHT_DIGITS], ALARM[HOURS], current_fm_freq, 4);
        }
        else if(enc_mode == SET_FREQ){
            if(L_cnt==1 || R_cnt==1 )
                current_fm_freq += 20; 
            else if(L_cnt==-1 || R_cnt==-1 )
                current_fm_freq -= 20; 
            else
                return;

            if(current_fm_freq < 8810) // fm range between 88.1 - 107.9 Mhz
                current_fm_freq = 10790;
            if(current_fm_freq > 10790)
                current_fm_freq = 8810;

            fm_tune_freq(); //tune radio to frequency in current_fm_freq
            radio_signal();
            LCD_MovCursorLn1();
            LCD_PutStr(first_line);

            
            LED[0] = LED_seg(mode, CLOCK[LEFT_DIGITS], ALARM[MINUTES], current_fm_freq, 0);
            LED[1] = LED_seg(mode, CLOCK[LEFT_DIGITS], ALARM[MINUTES], current_fm_freq, 1);
            LED[3] = LED_seg(mode, CLOCK[RIGHT_DIGITS], ALARM[HOURS], current_fm_freq, 3);
            LED[4] = LED_seg(mode, CLOCK[RIGHT_DIGITS], ALARM[HOURS], current_fm_freq, 4);
        }

        else {
            static int16_t volume  = 0x0FFF/5;
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

uint8_t LED_seg(uint8_t mode, uint8_t clock, uint8_t alarm, uint16_t frequency, uint8_t digit){

#define LETTER_A 0b10001000 
#define LETTER_H 0b10001001


    uint8_t right_digit = clock - 10*(clock/10);
    uint8_t left_digit  = clock / 10;
    switch(digit){
        case 0:
            if( SET_ALARM_MODE )
                return int_to_digit(alarm - 10*(alarm/10));

            else if( SET_FREQ_MODE )
                return int_to_digit( (frequency%100) / 10);

            else if( alarm_on )
                return LETTER_H;
            else{
                if( MODE_12HR )     // if display mode is 12 hr, indicate am or pm in digit 0's dot.
                    return int_to_digit(right_digit) & ~(am_pm<<7);

                return int_to_digit(right_digit);
            }

        case 1:
            if( SET_ALARM_MODE )
                return int_to_digit(alarm/10);

            else if( SET_FREQ_MODE )
                return int_to_digit( (frequency%1000)/100 ) & ~(1<<7);

            else if( alarm_on )
                return LETTER_A;

            else
                return int_to_digit(left_digit);

        case 3:
            if( SET_ALARM_MODE )
                return int_to_digit(alarm-10*(alarm/10));
            
            else if( SET_FREQ_MODE )
                return int_to_digit( (frequency%10000) / 1000);
            
            else if( alarm_on )
                return LETTER_A;

            else
                return int_to_digit(right_digit);

        case 4:
            if( SET_ALARM_MODE )
                return int_to_digit(alarm/10);

            else if( SET_FREQ_MODE )
                return int_to_digit(frequency/10000);

            else if( alarm_on )
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

