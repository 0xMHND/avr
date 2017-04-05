#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

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



void init_tcnt0();
void init_tcnt3();

void spi_init();
uint8_t spi_read_send(uint8_t data);

void init_adc();
uint8_t read_adc();

uint8_t debounce_switch(uint8_t button);
uint8_t check_buttons(uint8_t mode, uint8_t pin);
uint8_t int_to_digit(uint8_t num);
