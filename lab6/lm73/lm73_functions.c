// lm73_functions.c       
// Roger Traylor 11.28.10

#include <util/twi.h>
#include "lm73_functions.h"
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

//TODO: write functions to change resolution, alarm etc.

uint8_t lm73_wr_buf[2];
uint8_t lm73_rd_buf[2];

//********************************************************************************
uint8_t init_temp(uint8_t res){
#define TIME_OUT 7
#define RESOLUTION 5

    lm73_wr_buf[0] = LM73_PTR_CTRL_STATUS;
    lm73_wr_buf[1] = (1 << TIME_OUT) | ( res%10 << RESOLUTION);
    twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 2);   //start the TWI write process (twi_start_wr())
    _delay_ms(1);

    lm73_wr_buf[0] = LM73_PTR_TEMP;
    twi_start_wr(LM73_ADDRESS, lm73_wr_buf, 1);   //start the TWI write process (twi_start_wr())
    _delay_ms(1);

    return 0;
}
//******************************************************************************
uint8_t lm73_temp_convert(char temp_digits[], uint16_t lm73_temp, uint8_t f_not_c){
//given a temperature reading from an LM73, the address of a buffer
//array, and a format (deg F or C) it formats the temperature into ascii in 

#define CELCIUS 0
#define FAHRENHEIT 1
    int8_t negative_number = (lm73_temp >> 15) & 0x01;
    uint8_t integer = 12;
    uint32_t fraction = 0;

    if(negative_number){
        uint16_t local_temp = (lm73_temp ^ 0xffff) + 1;
        integer  = (local_temp >> 7) & 0xff;
        fraction = (((local_temp>>6)&0x01)*50000) + (((local_temp>>5)&0x01)*25000) +(((local_temp>>4)&0x01)*12500) +(((local_temp>>3)&0x01)*6250) +(((local_temp>>2)&0x01)*3125); 
    }
    else{
        integer  = (lm73_temp >> 7) & 0xff;
        fraction = (((lm73_temp>>6)&0x01)*50000) + (((lm73_temp>>5)&0x01)*25000) +(((lm73_temp>>4)&0x01)*12500) +(((lm73_temp>>3)&0x01)*6250) +(((lm73_temp>>2)&0x01)*3125); 
    }

    if(f_not_c == CELCIUS){
        sprintf(temp_digits, "%c%02d.%02lu", negative_number?'-':' ', integer, fraction);
        return 1;
    }

    // Convert from Celcius to Fahrenheit
    
    fraction = fraction / 1000; 

    int32_t whole = ((int32_t)integer*100) + (int32_t) fraction;
    uint32_t new_fraction = 0;

    if(negative_number){
        whole = ( (-1)*(whole)*9/5 ) + 3200;
        if(whole > 0){
            new_fraction = ( (whole) % 100 );
            negative_number = 0;
        }
        else{
            whole = (-1)*whole;
            new_fraction = ( whole % 100 );
        }

        new_fraction *= 1000;
        whole = (whole/100);
    }
    else{
        whole = ( (whole)*9/5 ) + 3200;
        new_fraction = ( whole % 100 );
        new_fraction *= 1000;
        whole = (whole/100);
    }

    sprintf(temp_digits, "%c%02ld.%02lu", negative_number?'-':' ', whole, new_fraction);

    return 1;
}//lm73_temp_convert
//******************************************************************************

