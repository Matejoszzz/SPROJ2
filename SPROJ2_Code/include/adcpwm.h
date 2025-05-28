#ifndef ADCPWM_H_INCLUDED
#define ADCPWM_H_INCLUDED

#include <stdint.h>



/**
 * @brief Initialises PWM generation with a frequency of approx. 4 KHz at PB1 (Fast mode, Timer1)
 **/
void pwm1_init(void);

/**
 * @brief Initialises PWM generation with a frequency of approx. 150Hz at PB0, PB1, PB2 (Manual, Timer1)
 **/
void pwm3_init(void);

/**
 * @brief Sets the PWM output at PB1 to a specific duty cycle
 * @param[in] duty Duty cycle in percent [0-100]
 **/
void pwm1_set_duty(unsigned char);


/**
 * @brief Sets the PWM outputs at PB0, PB1, PB2 to specific duty cycles
 * @param[in] dutyPB0 Duty cycle in percent [0-100]
 * @param[in] dutyPB1 Duty cycle in percent [0-100]
 * @param[in] dutyPB2 Duty cycle in percent [0-100]
 **/
void pwm3_set_duty(uint8_t, uint8_t, uint8_t);

/**
 * @brief Initialises the ADC module
 **/
void adc_init(void);

/**
 * @brief Reads the voltage provided at the input channel and returns the adc value as 10 bit
 * @param[in] channel input ADC channel [0-5] corresponding to inputs PC0..PC5
 * @param[out] result 10bit analog to digital conversion result in range [0-1023]
 * @note The corresponding input needs to be configured externally as input in the DDRC register and pullups enabled/disabled if necessary in PORTC
 **/
uint16_t adc_read(uint8_t channel);



#endif