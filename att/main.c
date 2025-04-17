#include <avr/io.h>
#include <util/delay.h>
// #include "i2c.h"

#define FOSC 8000000 // Clock Speed
#define BDIV (FOSC / 100000 - 16) / 2 + 1

int main(void) {

    // i2c_init(BDIV); 

    DDRB |= (1 << PB4); 
    
    while (1) {
        PORTB |= (1 << PB4); // Set PB4 high
        _delay_ms(1000); // Wait for 1 second
        PORTB &= ~(1 << PB4); // Set PB4 low
        _delay_ms(1000); // Wait for 1 second
    }

}