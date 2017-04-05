#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL //16Mhz clock
#include <string.h>

//function prototypes defined here
void song0(uint16_t note); //Beaver Fight Song
void song1(uint16_t note); //Tetris Theme (A)
void song2(uint16_t note); //Mario Bros Theme
void song3(uint16_t note); 
void song4(uint16_t note); //Beethoven's 5th symphony
void play_song(uint8_t song, uint8_t note);
void play_rest(uint8_t duration);
void play_note(char note, uint8_t flat, uint8_t octave, uint8_t duration);
void music_off(void);
void music_on(void);      
void music_init(void);
