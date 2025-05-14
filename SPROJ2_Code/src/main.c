#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"

void timer_init() {
  TCCR1A = 0; // Clear TCCR1A register
  TCCR1B |= (1 << WGM12); // CTC mode
  OCR1A = 15624; // Wait for 15624 ticks (1 second)
  TCCR1B |= (1 << CS12) | (1 << CS10);// Prescaler is set to 1024
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 Compare Match A interrupt
  sei(); // Enable interrupts
}

void interrupt_init() {
  DDRD &= ~(1 << DDD2); // PD2 (INT0) as input
  EIMSK |= (1 << INT0); // Enable INT0
  EICRA |= (1 << ISC01) | (1 << ISC00);  // Rising edge interrupt
}

volatile uint16_t beat_count = 0; // The number heartbeats
volatile uint16_t seconds = 0; // Seconds elapsed since the beginning
volatile uint16_t hbi_list[60] = {0}; // An array to store the number of beats each second for the last 60 seconds
int reset = 1; // Reset button
int flaw = 0; // Flaw detection
int stress = 0; // Stress detection

ISR(INT0_vect) {
  beat_count++;  // One heartbeat detected
}

ISR(TIMER1_COMPA_vect) {
  hbi_list[seconds % 60] = beat_count; // Store the number of beats this second
  beat_count = 0; //clear the beat counter
  seconds++; // Count the seconds
}

int main(void) {
  uart_init();
  io_redirect();
  i2c_init();
  LCD_init();

  interrupt_init();
  timer_init();

  DDRD &= ~(1 << DDD6);           // PD6 as input (reset button)
  PORTD |= (1 << PORTD6);        // Pull-up on PD6

  float HR;
  uint16_t sum;

  while (1) {
    if (!(PIND & (1 << PIND6))) { // Reset button pressed, reset everything
      LCD_clear();
      reset = 1;
      flaw = 0;
      stress = 0;
      seconds = 0;
      beat_count = 0;
      for (int i = 0; i < 60; i++) hbi_list[i] = 0;
    }

    while (reset) {
      if (!(PIND & (1 << PIND6))) { // Just in case we want to reset anytime 
        LCD_clear();
        reset = 1;
        flaw = 0;
        stress = 0;
        seconds = 0;
        beat_count = 0;
        for (int i = 0; i < 60; i++) hbi_list[i] = 0;
      }

      if (seconds < 10) { //For the first ten seconds it doesn't display the HR
        LCD_set_cursor(0, 0);
        printf("waiting... %d s", seconds);
      } else if (seconds < 60) { // After 10 seconds
        sum = 0; // The amount of beats so far
        for (int i = 0; i < seconds; i++) sum += hbi_list[i]; 
        HR = (float)sum * 60.0 / seconds; // The formula to calculate an avg HR
        if (HR < 50 || HR > 150) { // If the HR is off, the device will tell the user
          flaw = 1;
        }
        LCD_set_cursor(0, 0);
        printf("HR: %.0f BPM        ", HR); // Display the HR
      } else { // After 60 seconds, rolling BPM over last 60 seconds
        sum = 0;
        for (int i = 0; i < 60; i++) sum += hbi_list[i]; //The amount of beats in the last 60 seconds
        HR = sum;
        if (HR < 50 || HR > 150) {
          flaw = 1;
        }
        if (HR > 110) {
          stress = 1;
        }
        LCD_set_cursor(0, 0);
        printf("HR: %.0f BPM        ", HR);
      }
      if (stress) { // If stress is detected it stops the loop
        LCD_set_cursor(0, 0); printf("Stress detected     ");
        reset = 0;
      }
      if (flaw) { // If the HR is off it tells the user and stops the loop until we press the reset button
        LCD_set_cursor(0, 0); printf("Wrong placement     ");
        LCD_set_cursor(0, 1); printf("Adjust the device   ");
        LCD_set_cursor(0, 2); printf("And press reset     ");
        reset = 0;
      }
    }
  }

  return 0;
}
