#ifndef LCD_H_
#define LCD_H_

#define F_CPU				16000000UL
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>


// Nop
#define __nop() 			__asm__ volatile("nop"::);

void LCD_busy_flag_check();

void LCD_command_write(uint8_t DB);

void LCD_data_write(uint8_t DB);

void LCD_init();

void LCD_text_out(uint8_t const* topline, uint8_t const* botline);

void LCD_masked_text(uint8_t const* topline, uint8_t const* botline, uint16_t topmask, uint16_t botmask);

void Display(void *t, void *b, uint16_t tmsk, uint16_t bmsk);



#endif /* LCD_H_ */