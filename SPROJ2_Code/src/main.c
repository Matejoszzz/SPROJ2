#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#include "usart.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "adcpwm.h"

#define MAX_RR 30
#define RESP_SIZE 150

typedef struct
{
    float heartRate;
    float hrv;
    int respiratoryRate;
    float bodyTemperature;
} HealthData;

// Variables for RR intervals and time tracking
volatile uint16_t rr_intervals[MAX_RR] = {0};
volatile uint8_t rr_index = 0;
volatile uint32_t last_beat_ms = 0;
volatile uint32_t current_time_ms = 0;

// Heartbeat and general tracking
volatile uint16_t beat_count = 0;
volatile uint16_t seconds = 0;
float HR = 0;
float temp;

// Respiratory Rate variables
volatile uint16_t potPosition[RESP_SIZE] = {0};
volatile uint8_t potIndex = 0;

volatile uint8_t refresh = 0;

void timer_init()
{
    TCCR1A = 0;
    TCCR1B |= (1 << WGM12);              // CTC mode
    OCR1A = 249;                         // 1ms at 16MHz with 64 prescaler
    TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64
    TIMSK1 |= (1 << OCIE1A);             // Enable Compare A Match interrupt
    sei();                               // Enable global interrupts
}

void ext_interrupt_init()
{
    DDRD &= ~(1 << DDD2);  // PD2 (INT0) as input
    EIMSK |= (1 << INT0);  // Enable INT0
    EICRA |= (1 << ISC01); // Falling edge generates interrupt
}

ISR(TIMER1_COMPA_vect)
{
    current_time_ms++;
    if (current_time_ms % 200 == 0)
    {
        potPosition[potIndex] = adc_read(0);
        potIndex++;
        if (potIndex == RESP_SIZE)
        {
            potIndex = 0;
        }
    }

    if ((current_time_ms % 1000) == 0)
    {
        seconds++;
        refresh = 1;
    }
}

ISR(INT0_vect)
{

    if (last_beat_ms && (current_time_ms - last_beat_ms > 300))
    {
        rr_intervals[rr_index] = current_time_ms - last_beat_ms;
        rr_index++;
        if (rr_index == MAX_RR)
        {
            rr_index = 0;
        }
    }

    last_beat_ms = current_time_ms;
    beat_count++;
}

float calculate_HRV()
{
    if (seconds < 15)
        return 0;

    float mean = 0;
    for (uint8_t i = 0; i < MAX_RR; i++)
        mean += rr_intervals[i];
    HR = 60 * MAX_RR / (mean / 1000);
    mean = mean / MAX_RR;

    float variance = 0;
    for (uint8_t i = 0; i < MAX_RR; i++)
    {
        float diff = rr_intervals[i] - mean;
        variance += diff * diff;
    }

    return sqrt(variance / MAX_RR); // SDNN
}

int getRespiratoryRate()
{
    int peaks = 0;
    for (uint8_t i = 1, j = 0; i < 150; i++)
    {
        if (potPosition[i] >= potPosition[i - 1])
        {
            j++;
        }
        else
        {
            if (j > 3)
            {
                peaks++;
            }
            j = 0;
        }
    }

    return peaks * 2;
}

int main(void)
{
    uart_init();
    io_redirect();
    i2c_init();
    LCD_init();
    adc_init();

    ext_interrupt_init();
    timer_init();

    DDRD &= ~(1 << DDD6); // PD6 as input (reset button)
    DDRD |= 1 << 5;
    PORTD |= (1 << PORTD6); // Pull-up on PD6

    HealthData window[60] = {0};
    int wIndex = 0;
    HealthData data[15] = {0};
    int dataIndex = 0;

    while (1)
    {

        if (seconds < 15)
        {
            LCD_set_cursor(0, 0);
            printf("Waiting... %d s", seconds);
        }
        else if (refresh)
        {
            refresh = 0;
            window[wIndex].respiratoryRate = getRespiratoryRate();
            window[wIndex].hrv = calculate_HRV();
            window[wIndex].heartRate = HR;
            window[wIndex].bodyTemperature = (6 * adc_read(1) / 1024) + 34;
            
            LCD_clear();
            LCD_set_cursor(0, 0);
            printf("HRV: %.1f / %.0f ms", window[wIndex].hrv, data[0].hrv);
            LCD_set_cursor(0, 1);
            printf("Time: %d s", seconds);
            LCD_set_cursor(0, 2);
            printf("HR: %.0f / %.0f BPM", window[wIndex].heartRate, data[0].heartRate);
            LCD_set_cursor(0, 3);
            printf("RR: %i / %i RPM", window[wIndex].respiratoryRate, data[0].respiratoryRate);
            wIndex += 1;
            
        }
        if (wIndex == 60)
        {
            for (int i = 0; i < 60; i++)
            {
                data[dataIndex].bodyTemperature += window[i].bodyTemperature;
                data[dataIndex].heartRate += window[i].heartRate;
                data[dataIndex].hrv += window[i].hrv;
                data[dataIndex].respiratoryRate += window[i].respiratoryRate;
            }
            data[dataIndex].bodyTemperature /= 60;
            data[dataIndex].heartRate /= 60;
            data[dataIndex].hrv /= 60;
            data[dataIndex].respiratoryRate /= 60;
            wIndex = 0;
            dataIndex++;
        }
    }

    return 0;
}