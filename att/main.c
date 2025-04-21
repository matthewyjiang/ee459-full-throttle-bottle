#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h> 
#include "i2c_secondary.c"

#define FOSC 8000000 // Clock Speed
#define BDIV (FOSC / 100000 - 16) / 2 + 1
#define DEVICE_ADDRESS 0x50 // I2C Slave Address

// Define your I2C slave address (7-bit)
#define MY_I2C_ADDRESS 0x42

#define READ_BUFFER_SIZE 5 // Size of the read buffer
#define ADC_CHANNEL 3     // Using ADC0 (can be 0-7 for most AVRs)

unsigned char read_buffer[READ_BUFFER_SIZE]; // Buffer to store read data

// Function to initialize ADC
void adc_init(void) {
    // Set ADC reference voltage to AVCC
    ADMUX = (1 << REFS0);
    
    // Select ADC channel
    ADMUX = (ADMUX & 0xF8) | (ADC_CHANNEL & 0x07);
    
    // Enable ADC and set prescaler to 64 (typical value for 8MHz)
    // This gives an ADC clock of 125kHz which is in the recommended range
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

// Function to read from ADC
uint16_t adc_read(void) {
    // Start conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));
    
    // Return ADC value (10-bit)
    return ADC;
}

// Function to initialize PWM on PB4
void pwm_init(void) {
    // Set PB4 as output
    DDRB |= (1 << PB4);
    
    // Clear output pin initially
    PORTB &= ~(1 << PB4);
}

// Better software PWM implementation using PB4
void better_software_pwm(uint8_t duty_cycle) {
    static uint8_t pwm_counter = 0;
    
    // Increment the counter
    pwm_counter++;
    
    // Compare counter with duty cycle
    if (pwm_counter < duty_cycle) {
        PORTB |= (1 << PB4);  // Set PB4 high
    } else {
        PORTB &= ~(1 << PB4); // Set PB4 low
    }
}


int main(void) {
    // Initialize buffers
    for (int i = 0; i < READ_BUFFER_SIZE; i++) {
        read_buffer[i] = 0x00; // Initialize the read buffer
    }

    // Initialize I2C secondary
    i2c_secondary_init(MY_I2C_ADDRESS, read_buffer, READ_BUFFER_SIZE);
    
    // Initialize ADC
    adc_init();
    
    // Initialize PWM on PB4
    pwm_init();
    
    // Enable global interrupts
    sei();
    
    uint8_t pwm_duty = 128; // Default duty cycle value
    
    while (1) {
        // Read ADC value
        uint16_t adc_value = adc_read();
        
        // Store ADC value in read buffer
        read_buffer[0] = (adc_value >> 8) & 0xFF;  // High byte
        read_buffer[1] = adc_value & 0xFF;         // Low byte
        
        
        pwm_duty = 0x00;
        
        // Perform one step of the PWM cycle
        better_software_pwm(pwm_duty);
        
        // Small delay to control PWM frequency
        // Adjust this value to get desired frequency
        _delay_us(10);
        
        // Other tasks can be performed here without
        // blocking the entire PWM cycle
    }
}