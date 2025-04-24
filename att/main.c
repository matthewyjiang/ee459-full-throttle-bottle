#define F_CPU 8000000UL // 8 MHz Internal Oscillator

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>

#include "usiTwiSlave.h"

// --- Configuration ---
#define SLAVE_ADDR 0x08
#define TEMP_ADC_CHANNEL 3 // ADC3 = PB3
#define HEATER_PIN PB4     // OC1B Output for Timer1 PWM

// I2C Commands (Master to Slave)
#define CMD_SET_SETPOINT 0x01

// PID Parameters (MUST BE TUNED!)
#define PID_KP 6.0
#define PID_KI 0.1
#define PID_KD 0.0
#define PID_SAMPLE_TIME_MS 500

// ADC Config
#define ADC_REF_VOLTAGE 1.1
#define ADC_MAX_VALUE 1023.0

// PID State Variables
volatile int8_t targetTemperature = 20;
volatile int8_t currentTemperature_int = 0;
volatile uint8_t currentTemperature_frac = 0;

float pid_error = 0;
float pid_integral = 0;
float pid_derivative = 0;
float pid_last_error = 0;
int16_t pid_output = 0; // Output for PWM (0-255)

// --- Function Prototypes ---
void setup_adc(void);
uint16_t read_adc(uint8_t channel);
void convert_adc_to_temp(uint16_t adc_value);
void setup_pwm(void); // <<<< Will be modified
void update_pid(void);
void i2c_receive_handler(uint8_t num_bytes);
void i2c_request_handler(void);

// --- Main ---
int main(void) {
    // --- Initialization ---
    setup_adc();
    setup_pwm(); // <<<< Uses updated function

    // Initialize I2C Slave
    usiTwiSlaveInit(SLAVE_ADDR);
    usi_onReceiverPtr = i2c_receive_handler;
    usi_onRequestPtr = i2c_request_handler;

    sei(); // Enable global interrupts

    // --- Main Loop ---
    while (1) {
        uint16_t adc_val = read_adc(TEMP_ADC_CHANNEL);
        convert_adc_to_temp(adc_val);
        update_pid();

        // Apply PID output to PWM (using OCR1B now) <<<< MODIFIED
        if (pid_output < 0) {
            OCR1B = 0;
        } else if (pid_output > 255) {
            OCR1B = 255;
        } else {
            OCR1B = (uint8_t)pid_output;
        }

        _delay_ms(PID_SAMPLE_TIME_MS);
    }
}

// --- Function Definitions ---

void setup_adc(void) {
    ADMUX = (1 << REFS1) | (0 << REFS0) | (0 << ADLAR) | (TEMP_ADC_CHANNEL & 0x0F);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0); // Prescaler 64
}

uint16_t read_adc(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADCW;
}

void convert_adc_to_temp(uint16_t adc_value) {
    float voltage = (adc_value / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    float temp_c_float = (voltage - 0.5) * 100.0;
    currentTemperature_int = (int8_t)temp_c_float;
    float frac_part = temp_c_float - currentTemperature_int;
    if (frac_part < 0) frac_part = 0;
    currentTemperature_frac = (uint8_t)(frac_part * 100.0);
    // Optional bounds check
    // if (currentTemperature_int < -40) currentTemperature_int = -40;
    // if (currentTemperature_int > 125) currentTemperature_int = 125;
}

// --- MODIFIED PWM SETUP FOR TIMER1 ---
void setup_pwm(void) {
    // Configure PB4 (HEATER_PIN / OC1B) as output
    DDRB |= (1 << HEATER_PIN);

    // Configure Timer1 for Fast PWM, 8-bit (Mode 5)
    // OCR1C defines TOP (0xFF = 255)
    // COM1B1=1, COM1B0=0: Clear OC1B on Compare Match, set OC1B at BOTTOM (non-inverting mode)
    // PWM1B = 1: Enable PWM mode for OC1B based on OCR1B
    GTCCR = (1 << PWM1B) | (1 << COM1B1) | (0 << COM1B0);

    // Set TOP value for 8-bit PWM
    OCR1C = 0xFF; // 255

    // Set Timer1 Prescaler to clk/64
    // CS1[3:0] = 0111 (TCCR1 bits CS13=0, CS12=1, CS11=1, CS10=1)
    // Note: Other bits in TCCR1 (CTC1, PWM1A, COM1A[1:0]) are 0 by default, which is fine.
    TCCR1 = (0 << CS13) | (1 << CS12) | (1 << CS11) | (1 << CS10);

    // Start with PWM off (duty cycle 0)
    OCR1B = 0;
}
// --- END MODIFIED PWM SETUP ---

void update_pid(void) {
    float current_temp_combined = currentTemperature_int + (currentTemperature_frac / 100.0);
    pid_error = (float)targetTemperature - current_temp_combined;

    pid_integral += pid_error * (PID_SAMPLE_TIME_MS / 1000.0);
    // Basic integral clamping
    float max_integral = 100.0;
    if (pid_integral > max_integral) pid_integral = max_integral;
    else if (pid_integral < -max_integral) pid_integral = -max_integral;

    pid_derivative = (pid_error - pid_last_error) / (PID_SAMPLE_TIME_MS / 1000.0);
    pid_last_error = pid_error;

    pid_output = (int16_t)( (PID_KP * pid_error) +
                            (PID_KI * pid_integral) +
                            (PID_KD * pid_derivative) );
}

// --- I2C Handlers ---
void i2c_receive_handler(uint8_t num_bytes) {
    if (num_bytes < 1) return;
    uint8_t command = usiTwiReceiveByte();

    if (command == CMD_SET_SETPOINT) {
        if (usiTwiAmountDataInReceiveBuffer() > 0) { // Check if setpoint value exists
             uint8_t received_setpoint = usiTwiReceiveByte();
             if (received_setpoint <= 100) { // Basic validation
                 targetTemperature = (int8_t)received_setpoint;
                 // pid_integral = 0; // Optional: Reset integral on setpoint change
             }
        }
    }
    // Discard extra bytes
    while(usiTwiAmountDataInReceiveBuffer() > 0) {
        usiTwiReceiveByte();
    }
}

void i2c_request_handler(void) {
    usiTwiTransmitByte((uint8_t)currentTemperature_int);
    usiTwiTransmitByte(currentTemperature_frac);
}