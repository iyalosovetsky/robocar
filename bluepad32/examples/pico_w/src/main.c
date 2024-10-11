// Example file - Public Domain
// Need help? http://bit.ly/bluepad32-help

#include <btstack_run_loop.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>


#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include <uni.h>

#include "sdkconfig.h"
#include "my_motors.h"

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Pico W must use BLUEPAD32_PLATFORM_CUSTOM"
#endif


#define WRAPVAL 5000
#define CLKDIV 5.0f

// static const char * controller_addr_string1 = "35:20:3B:54:1A:90"; //dualshock4 #1
// static const char * controller_addr_string2 = "0A:62:AD:23:CC:43"; //dualshock4 #2
// static const char * controller_addr_string3 = "24:A6:FA:1B:E0:8F"; //dualsense5 #serg


// Defined in my_platform.c
struct uni_platform* get_my_platform(void);

void init_pwm_n(uint gpio1,uint gpio2) {
    // Find out which PWM slice is connected to GPIO 16-19 (Slice 0) and GPIO2 (Slice 1)
    uint slice_num = pwm_gpio_to_slice_num(gpio1);  //0A ->0
    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler    
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, false);
    // irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    // irq_set_enabled(PWM_IRQ_WRAP, true);
    // motor1 left as PWM
// PWM 0 Forward PWM, fast decay
// 1 PWM Forward PWM, slow decay
// 0 PWM Reverse PWM, fast decay
// PWM 1 Reverse PWM, slow decay    
    gpio_set_function(gpio1, GPIO_FUNC_PWM); //0A
    // gpio_set_function(gpio2, GPIO_FUNC_PWM); //0A
    // gpio_set_function(gpio2, GPIO_FUNC_PWM); //0B
    gpio_init(gpio2);
    gpio_set_dir(gpio2, GPIO_OUT);
    gpio_put(gpio2, 0);   

    // This section configures the period of the PWM signals
    // pwm_config_set_clkdiv_mode(slice_num, PWM_DIV_FREE_RUNNING);
    pwm_set_clkdiv_mode(slice_num, PWM_DIV_FREE_RUNNING); //Free-running counting at rate dictated by fractional divider
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;
    // +
    pwm_set_phase_correct(slice_num, false);
    // +
    pwm_set_output_polarity(slice_num, false, false);
    // pwm_set_output_polarity(slice_num, true, true);
    // This sets duty cycle
    // pwm_set_chan_level(slice_num, PWM_CHAN_A, 1000);  
    pwm_set_both_levels(slice_num, WRAPVAL/2, 0);
    //+
    pwm_set_enabled(slice_num, false);
     
     // Start the channel
    // pwm_set_mask_enabled((1u << slice_num));  
    
    // // Set Free running, No phase correct, int 1, frac 4 and channel B inverted for Slice 0
    // pwm_set_clkdiv_mode(slice_num, PWM_DIV_FREE_RUNNING); //Free-running counting at rate dictated by fractional divider
    // pwm_set_phase_correct(slice_num, false);
    // pwm_set_clkdiv_int_frac(slice_num, 1, 4);
    // pwm_set_output_polarity(slice_num, false, true);
    // Set the TOP register to 5000, which with the system frequency at 125MHz, corresponds 
    // to a frequency of 20KHz for PWM of Slice 0
    // pwm_set_wrap(slice_num, 5000);
    // pwm_set_both_levels(slice_num, 2500, 2500);
    // Enable PWM running
    // pwm_set_enabled(slice_num, true);

}

int main() {
    stdio_init_all();
    //pwm

 // motor1
    gpio_init(MOTOR1_STBY);
    gpio_set_dir(MOTOR1_STBY, GPIO_OUT);
    gpio_put(MOTOR1_STBY, 0); //to sleep

    gpio_init(MOTOR2_STBY);
    gpio_set_dir(MOTOR2_STBY, GPIO_OUT);
    gpio_put(MOTOR2_STBY, 0); //to sleep
    
    init_pwm_n(LEFT_MOTOR1_PIN_1 ,LEFT_MOTOR1_PIN_2);
    init_pwm_n(RIGHT_MOTOR1_PIN_1,RIGHT_MOTOR1_PIN_2);
    init_pwm_n(LEFT_MOTOR2_PIN_1 ,LEFT_MOTOR2_PIN_2);
    init_pwm_n(RIGHT_MOTOR2_PIN_1,RIGHT_MOTOR2_PIN_2);


   

//     // Print over serial the system clock frequency
    uint32_t clockHz = clock_get_hz(clk_sys);
    logi("clk_sys: %dHz\n", clockHz);      

//     // Flash on board led five times
//     for(int i=0;i<5;i++)
//     {
//         // gpio_put(LED_PIN, 1);
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
//         sleep_ms(250);
//         // gpio_put(LED_PIN, 0);
//         cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
//         sleep_ms(250);
//     }

    // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
    if (cyw43_arch_init()) {
        loge("failed to initialise cyw43_arch\n");
        return -1;
    }

    // Turn-on LED. Turn it off once init is done.
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // Must be called before uni_init()
    uni_platform_set_custom(get_my_platform());

    // Initialize BP32
    uni_init(0, NULL);
uni_bt_allowlist_set_enabled(false);
    // //----------------
    // bd_addr_t controller_addr1;
    // sscanf_bd_addr(controller_addr_string1, controller_addr1);
    // bd_addr_t controller_addr2;
    // sscanf_bd_addr(controller_addr_string2, controller_addr2);
    // bd_addr_t controller_addr3;
    // sscanf_bd_addr(controller_addr_string3, controller_addr3);

    // // Notice that this address will be added in the Non-volatile-storage (NVS).
    // // If the device reboots, the address will still be stored.
    // // Adding a duplicate value will do nothing.
    // // You can add up to four entries in the allowlist.
    // // uni_bt_allowlist_remove_all();
    // uni_bt_allowlist_add_addr(controller_addr1);
    // uni_bt_allowlist_add_addr(controller_addr2);
    // uni_bt_allowlist_add_addr(controller_addr3);
    // uni_bt_allowlist_set_enabled(true);
    // //uni_bt_enable_new_connections_safe(false);
    // logi("robcar_platform: white list added\n");

    ////    ---------- from arduino demo code    



    // Does not return.
    btstack_run_loop_execute();

    return 0;
}
