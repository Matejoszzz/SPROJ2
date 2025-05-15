#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"

#define MAX_RR 60

// Variables for RR intervals and time tracking
volatile uint16_t rr_intervals[MAX_RR] = {0};
volatile uint8_t rr_index = 0;
volatile uint32_t last_beat_ms = 0;
volatile uint32_t current_time_ms = 0;

// Heartbeat and general tracking
volatile uint16_t beat_count = 0;
volatile uint16_t seconds = 0;
volatile uint8_t reset = 1;
volatile uint8_t flaw = 0;

void timer_init() {
    TCCR1A = 0;
    TCCR1B |= (1 << WGM12);               // CTC mode
    OCR1A = 249;                          // 1ms at 16MHz with 64 prescaler
    TCCR1B |= (1 << CS11) | (1 << CS10);  // Prescaler 64
    TIMSK1 |= (1 << OCIE1A);              // Enable Compare A Match interrupt
    sei();                                // Enable global interrupts
}

void ext_interrupt_init() {
    DDRD &= ~(1 << DDD2);  // PD2 (INT0) as input
    EIMSK |= (1 << INT0);  // Enable INT0
    EICRA |= (1 << ISC01); // Falling edge generates interrupt
}

ISR(TIMER1_COMPA_vect) {
    current_time_ms++;
    if (current_time_ms % 1000 == 0) {
        seconds++;
    }
}

ISR(INT0_vect) {
    uint16_t rr = current_time_ms - last_beat_ms;

    if (rr_index == MAX_RR)
    {
      rr_index = 0;
    }
    
    if (last_beat_ms != 0) {
        rr_intervals[rr_index] = rr;
    }

    last_beat_ms = current_time_ms;
    beat_count++;
}

float calculate_HRV() {
    if (rr_index < 2) return 0;

    float sum = 0;
    for (uint8_t i = 0; i < rr_index; i++) sum += rr_intervals[i];
    float mean = sum / rr_index;

    float variance = 0;
    for (uint8_t i = 0; i < rr_index; i++) {
        float diff = rr_intervals[i] - mean;
        variance += diff * diff;
    }

    return sqrt(variance / rr_index); // SDNN
}

int main(void) {
    uart_init();
    io_redirect();
    i2c_init();
    LCD_init();

    ext_interrupt_init();
    timer_init();

    DDRD &= ~(1 << DDD6);           // PD6 as input (reset button)
    PORTD |= (1 << PORTD6);        // Pull-up on PD6

    float HR = 0;

    while (1) {
        if (!(PIND & (1 << PIND6))) { // Reset button pressed
            LCD_clear();
            reset = 1;
            flaw = 0;
            seconds = 0;
            beat_count = 0;
            rr_index = 0;
            last_beat_ms = 0;
            current_time_ms = 0;
            for (int i = 0; i < MAX_RR; i++) rr_intervals[i] = 0;
        }

        while (reset) {
            if (!(PIND & (1 << PIND6))) {
                LCD_clear();
                reset = 1;
                flaw = 0;
                seconds = 0;
                beat_count = 0;
                rr_index = 0;
                last_beat_ms = 0;
                current_time_ms = 0;
                for (int i = 0; i < MAX_RR; i++) rr_intervals[i] = 0;
            }

            if (seconds < 10) {
                LCD_set_cursor(0, 0);
                printf("Waiting... %d s", seconds);
            } else {
                if (seconds > 0) {
                    HR = (float)beat_count * 60000.0 / current_time_ms;
                }

                if (HR < 50 || HR > 150) flaw = 1;

                if (flaw) {
                    LCD_set_cursor(0, 0);
                    printf("Wrong placement     ");
                    LCD_set_cursor(0, 1);
                    printf("Adjust the device   ");
                    LCD_set_cursor(0, 2);
                    printf("And press reset     ");
                    reset = 0;
                } else {
                    LCD_set_cursor(0, 0);
                    printf("HR: %.0f BPM       ", HR);
                    LCD_set_cursor(0, 1);
                    printf("Time: %d s         ", seconds);
                    LCD_set_cursor(0, 2);
                    printf("HRV: %.1f ms       ", calculate_HRV());
                }
            }
        }
    }

    return 0;
}