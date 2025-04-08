/*
 * HelloWorld.c
 *
 * Created: 11/9/2023 10:43:27 AM
 * Author : Alin
 */ 



#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"


int main(void) {  

  uart_init(); // open the communication to the micro controller
  io_redirect(); // redirect input and output to the communication
  i2c_init();
  LCD_init();


  DDRD = 0xFF;
  PORTD = 0x00;
  DDRC = 0xF0;
  PORTC = 0x3F;

  double seconds = 0;
  double hbi;
  double HR;

  int index = 0;
  double hbi_list[60];

  while(1) {
    DDRD &= ~(1 << DDD4);     // Clear the PD4 pin
    // PD4 is now an input
    PORTD |= (1 << PORTD4);   // turn On the Pull-up
    // PD4 is now an input with pull-up enabled
    TCCR0B |= (1 << CS02) | (1 << CS01) | (1 << CS00);// Turn on the counter, Clock on Rise
    hbi = TCNT0;
    hbi_list[index % 60] = hbi;
    if (seconds < 60) {
      if (seconds < 10) {
        LCD_set_cursor(0, 0);
        printf("waiting for results");
        seconds++;
        index++;
        _delay_ms(1000);
      } else {
        if(seconds == 10) {
          LCD_clear();
        }
        HR = hbi * (60 / seconds);
        LCD_set_cursor(0, 0);
        printf ("HR: %.0f BPM ", HR);
        seconds++;
        index++;
        _delay_ms(1000);
      }
    } else {
      HR = hbi_list[index % 60] - hbi_list[(index+1) % 60];
      LCD_set_cursor(0, 0);
      printf ("HR: %.0f BPM ", HR);
      index++;
      _delay_ms(1000);
    }
  }
  
  return 0;
}
