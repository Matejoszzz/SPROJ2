// Library with useful functions for ADC and PWM for the atmega328p microcontroller
// Author: AD-SDU, 2024
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "adcpwm.h"

volatile uint8_t _duty0 = 0, _duty1 = 0, _duty2 = 0, _timer_tick;

void pwm1_init(void){
    cli();
    DDRB |= (1<<PB1);
    // Set Fast PWM mode, non-inverted output on Timer 1
    TCCR1A = (1 << WGM10) | (1 << COM1A1); // Fast PWM, 8-bit
    TCCR1B = (1 << CS11); // Prescaler: 8 > Frequency approx. 4 kHz
}


void pwm3_init(void){
    DDRB |= (1<<PB2)|(1<<PB1)|(1<<PB0);
    // Disable Timer1
   // TCCR1B = 0; // Makes sure timer 1 is not running
    TCCR1A = (1 << WGM11); // Normal Mode 
    TIMSK1 = (1 << OCIE1A);
    OCR1A = 1;
    sei();
    TCCR1B = (1 << CS10); // No prescaler:  Frequency approx. 150Hz
}


void pwm1_set_duty(unsigned char input){
    if (input <= 100){
        OCR1A = input*2.55; // 0 .. 255 range
    }
       
}

void pwm3_set_duty(uint8_t input0, uint8_t input1, uint8_t input2){
    TCNT1 = 0;
    if (input0 <= 100){
        _duty0 = input0;
    }
    if (input1 <= 100){
        _duty1 = input1;
    }
    if (input2 <= 100){
        _duty2 = input2;
    }
       
}

void adc_init(void){
    ADMUX = (1<<REFS0); //set prescaler to 128 and turn on the ADC module
    ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}


uint16_t adc_read(uint8_t adc_channel){
    ADMUX &= 0xf0; // clear any previously used channel, but keep internal reference
    ADMUX |= adc_channel; // set the desired channel
    //start a conversion
    ADCSRA |= (1<<ADSC);
    // now wait for the conversion to complete
    while ((ADCSRA & (1<<ADSC)));
    // now we have the result, so we return it to the calling function as a 16 bit unsigned int
    return ADC;
}


