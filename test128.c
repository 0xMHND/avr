// build: draft
/*************************************************************************/

// Mohannad Al Arifi
// 932 09 3718
// 11.7.16

// lab6: test FM Radio

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "./twi_master/twi_master.h" //my defines for TWCR_START, STOP, RACK, RNACK, SEND
#include "./si4734_driver/si4734.h" //my defines for TWCR_START, STOP, RACK, RNACK, SEND
#include "./functions/functions.h" //my defines for TWCR_START, STOP, RACK, RNACK, SEND


//extern enum radio_band{FM, AM, SW};
extern volatile enum radio_band current_radio_band;

extern uint16_t current_fm_freq;
extern uint8_t  current_volume;

uint8_t si4734_tune_status_buf[8]; //buffer for holding tune_status data  
extern volatile uint8_t STC_interrupt;  //flag bit to indicate tune or seek is done
// PE2 -> Si4734 Hardware Reset
// PE3 -> Volume 
// PE7 -> Si4734 int2 (sense  TWI mode)

int main()
{

    DDRB  |= 0xff;
    
    DDRE  |= (1<<PE2) | (1<<PE3); //Port E bit 2 is active high reset for radio 
    PORTE |= (1<<PE2) | (1<<PE3); //radio reset is on at powerup (active high)

    EICRB |= (1<<ISC50) | (1<<ISC51);
    EIMSK |= (1<<INT5);

    init_tcnt3(); 
    init_twi(); 

//    LCD_Init();
 //   LCD_Clr();

    //hardware reset of Si4734
    PORTE &= ~(1<<PE5); //int2 initially low to sense TWI mode
    DDRE  |=  (1<<PE5); //turn on Port E bit 5 to drive it low
    PORTE |=  (1<<PE2); //hardware reset Si4734 
    _delay_us(200);     //hold for 200us, 100us by spec         
    PORTE &= ~(1<<PE2); //release reset 
    _delay_us(30);      //5us required because of my slow I2C translators I suspect
                        //Si code in "low" has 30us delay...no explaination
    DDRE  &= ~(1<<PE5); //now Port E bit 5 becomes input from the radio interrupt

    sei();

    fm_pwr_up(); //powerup the radio as appropriate
    current_fm_freq = 10230; //arg2, arg3: 99.9Mhz, 200khz steps
    fm_tune_freq(); //tune radio to frequency in current_fm_freq
/*
    //to read signal strength...
    while(twi_busy()){} //make sure TWI is not busy 
    fm_rsq_status();

    //save tune status 
    uint8_t rssi =  si4734_tune_status_buf[4];
    char lcd_str[16];
    LCD_PutStr(" rssi = ");
    LCD_PutChar(rssi);
*/
    while(1){
        /*
        while(twi_busy()){} //make sure TWI is not busy 
        fm_rsq_status();
        rssi = si4734_tune_status_buf[4];
        LCD_MovCursorLn1();
        LCD_PutStr(" rssi = ");
        itoa( rssi, lcd_str, 2);
        LCD_MovCursorLn2();
        LCD_PutStr(lcd_str);
        */

        _delay_ms(1000);
        current_fm_freq += 20;
        fm_tune_freq(); //tune radio to frequency in current_fm_freq
        
    } //while
}  //main


//******************************************************************************
// External interrupt 7 is on Port E bit 7. The interrupt is triggered on the
// rising edge of Port E bit 7.  The i/o clock must be running to detect the
// edge (not asynchronouslly triggered)
//******************************************************************************
ISR(INT5_vect){
    STC_interrupt = TRUE;
}
/***********************************************************************/
