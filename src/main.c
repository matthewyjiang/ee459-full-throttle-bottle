#include "esp_common.h"
#include "freertos/task.h"
#include "gpio.h"

// PWM configuration
#define PWM_GPIO 16     // GPIO pin for PWM output
#define PWM_PERIOD 100  // PWM period in timer ticks 
#define PWM_RESOLUTION 100 // PWM resolution (0-100)

// Global PWM variables
static uint8_t pwm_duty = 90;  // Initial duty cycle (50%)
static bool pwm_direction = true; // For breathing effect

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;
    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;
        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;
        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;
        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }
    return rf_cal_sec;
}

// PWM software implementation task
void task_pwm(void* ignore)
{
    // Configure GPIO16 as output
    gpio16_output_conf();
    
    uint8_t tick_count = 0;
    
    while(true) {
        // One PWM cycle
        for (tick_count = 0; tick_count < PWM_RESOLUTION; tick_count++) {
            // Set GPIO high if current tick is less than duty cycle
            if (tick_count < pwm_duty) {
                gpio16_output_set(1);
            } else {
                gpio16_output_set(0);
            }
            
            // Short delay to control PWM frequency
            // Adjust this value to get desired PWM frequency
            os_delay_us(100); 
        }
    }
    
    vTaskDelete(NULL);
}


/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
    // Create PWM task with higher priority
    xTaskCreate(&task_pwm, "pwm_task", 2048, NULL, 3, NULL);
    
}
