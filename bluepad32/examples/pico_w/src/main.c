// Example file - Public Domain
// Need help? http://bit.ly/bluepad32-help
//https://github.com/raspberrypi/pico-examples/blob/master/gpio/hello_gpio_irq/hello_gpio_irq.c

#include <btstack_run_loop.h>
#include <pico/cyw43_arch.h>
#include <pico/stdlib.h>


#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

#include <uni.h>

#include "sdkconfig.h"
#include "my_motors.h"

// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Pico W must use BLUEPAD32_PLATFORM_CUSTOM"
#endif


void gpio_callback(uint gpio, uint32_t events) {
    if (gpio==LEFT_MOTOR_FORWARD_PIN_OPTICAL) {
        inc_motors_optical_counter0(MOTOR_FW_L) ; 
    } else if  (gpio==RIGHT_MOTOR_FORWARD_PIN_OPTICAL) {
        inc_motors_optical_counter0(MOTOR_FW_R) ; 
    } else if (gpio==LEFT_MOTOR_BACKWARD_PIN_OPTICAL) {
        inc_motors_optical_counter0(MOTOR_BW_L) ; 
    } else if  (gpio==RIGHT_MOTOR_BACKWARD_PIN_OPTICAL) {
        inc_motors_optical_counter0(MOTOR_BW_R) ; 
    }
    // Put the GPIO event(s) that just happened into event_str
    // so we can print it
    // gpio_event_string(event_str, events);
    // printf("GPIO %d %s\n", gpio, event_str);
}

bool repeating_timer_callback(__unused struct repeating_timer *t) {
    
    ticks(PID_SPEED);
    
   
    return true;
}

// static const char * controller_addr_string1 = "35:20:3B:54:1A:90"; //dualshock4 #1
// static const char * controller_addr_string2 = "0A:62:AD:23:CC:43"; //dualshock4 #2
// static const char * controller_addr_string3 = "24:A6:FA:1B:E0:8F"; //dualsense5 #serg


// Defined in my_platform.c
struct uni_platform* get_my_platform(void);



int main() {
    stdio_init_all();
    //pwm

 
    for (int wheel = 0; wheel < MOTORS_CNT; ++wheel) {
      motor_init ( wheel );
    }
    

    gpio_set_irq_enabled_with_callback(LEFT_MOTOR_FORWARD_PIN_OPTICAL, GPIO_IRQ_EDGE_RISE , true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(RIGHT_MOTOR_FORWARD_PIN_OPTICAL, GPIO_IRQ_EDGE_RISE , true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(LEFT_MOTOR_BACKWARD_PIN_OPTICAL, GPIO_IRQ_EDGE_RISE , true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(RIGHT_MOTOR_BACKWARD_PIN_OPTICAL, GPIO_IRQ_EDGE_RISE , true, &gpio_callback);
    
    
   

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

    // Create a repeating timer that calls repeating_timer_callback.
    // If the delay is > 0 then this is the delay between the previous callback ending and the next starting.
    // If the delay is negative (see below) then the next call to the callback will be exactly 500ms after the
    // start of the call to the last callback
    struct repeating_timer timer;
    add_repeating_timer_ms(TICK_PID_TIMER, repeating_timer_callback, NULL, &timer);

    // Does not return.
    btstack_run_loop_execute();

    return 0;
}
