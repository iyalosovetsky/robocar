// Example file - Public Domain
// Need help? https://tinyurl.com/bluepad32-help
//https://git.kcir.pwr.edu.pl/kdrazyk/sr-manipulator/-/tree/master/software/sr-manipulator
//https://github.com/lohengrin/Bluepad32_PicoW/tree/main
// https://github.com/Freitolini/butterfly
// https://github.com/bvpav/TNR-12
//https://github.com/susesKaninchen/Sumobot2024/tree/main
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"

#include "hardware/clocks.h"

#include <stddef.h>
#include <string.h>

#include <pico/cyw43_arch.h>
#include <pico/time.h>
#include <uni.h>
#include "math.h"

#include "sdkconfig.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#include "my_motors.h"



// Sanity check
#ifndef CONFIG_BLUEPAD32_PLATFORM_CUSTOM
#error "Pico W must use BLUEPAD32_PLATFORM_CUSTOM"
#endif



// Joystick Analog dead zone

#define DEAD_ZONE_XY 8 
#define ZONE1_XY 70 
#define ZONE2_XY 130
#define ZONE3_XY 300
#define ZONE4_XY 470
#define MAX_ZONE_XY 512
#define MIN_ZONE_XY 0
#define DEAD_ZONE_ACCEL 150
#define DEAD_ZONE_GYRO 500


#define UART_ID uart0
#define BAUD_RATE 115200

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define PARITY    UART_PARITY_NONE
#define DATA_BITS 8
#define STOP_BITS 1


#define PROT_SOF 0x01
#define PROT_EOF 0x02
#define UART_BUFF_SIZE 0x08
#define PROT_BUFF_SIZE 0x04

#define FOR_LEFT 0
#define FOR_RIGHT 1
#define BACK_LEFT 2
#define BACK_RIGHT 3

#define CHANGE_DPAD 1
#define CHANGE_BUTTONS 2
#define CHANGE_MISC_BUTTONS 4
#define CHANGE_X 8
#define CHANGE_Y 16
#define CHANGE_RX 32
#define CHANGE_RY 64
#define CHANGE_BRAKE 128
#define CHANGE_THROTTLE 256
#define CHANGE_GYRO 512
#define CHANGE_ACCEL 1024

// a PWM frequency of 5 kHz. (125M/5/5000=5kHz)
#define WRAPVAL 5000
#define MAX_VAL WRAPVAL-100
#define CLKDIV 5.0f

static int chars_rxed = 0;
static uint8_t pos = 0;
static uint8_t ch[UART_BUFF_SIZE] = {0};
static uint8_t modCtrl = 0;
static uint8_t modLed = 0;
static uint8_t modChanged = 0;



enum {
    MOTOR_FW_L = 0,
    MOTOR_FW_R = 1,
    MOTOR_BW_L = 2,
    MOTOR_BW_R = 3
};

#define MOTOR_SPEED_0  0
#define MOTOR_SPEED_1  MAX_VAL-4000
#define MOTOR_SPEED_2  MAX_VAL-3000
#define MOTOR_SPEED_3  MAX_VAL-2000
#define MOTOR_SPEED_4  MAX_VAL-1000
#define MOTOR_SPEED_MAX  MAX_VAL
#define MOTOR_SPEED_TORQUE  2550

enum {
    MOTOR_DIR_BACKWARD = -1,
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD = 1
};


#define JOY_UP 1
#define JOY_DOWN 2
#define JOY_RIGHT 4
#define JOY_LEFT 8

enum {
    /*
      -x(1)
         |
  -y(8)--|---+y(4)
         |
       +x(2)
    */
    CAR_DIR_STOP = 0,
    CAR_DIR_LEFT = JOY_LEFT,
    CAR_DIR_RIGHT = JOY_RIGHT,
    CAR_DIR_FORWARD = JOY_UP,
    CAR_DIR_BACKWARD = JOY_DOWN,
    CAR_DIR_LEFT_FORWARD = JOY_UP|JOY_LEFT,
    CAR_DIR_RIGHT_FORWARD = JOY_UP|JOY_RIGHT,
    CAR_DIR_RIGHT_BACKWARD = JOY_DOWN|JOY_RIGHT,
    CAR_DIR_LEFT_BACKWARD = JOY_DOWN|JOY_LEFT
};

static  int  motors_direction[4]   = {0,0,0,0};
static  uint16_t motors_speed[4]  = {0,0,0,0};
static  uint16_t car_speed  = 0;
static  uint16_t car_motor_n  = 0;
static  int car_motor_dir = 0;
    

#define MONITOR_XY 0x01
#define MONITOR_GYRO 0x02
#define MONITOR_ACCEL 0x02
// static uint8_t mask_monitoring = MONITOR_XY|MONITOR_GYRO|MONITOR_ACCEL; // 1 - xy, butt; 2 - gyro ; 4 - accel/
static uint8_t mask_monitoring = MONITOR_XY; // 1 - xy, butt; 2 - gyro ; 4 - accel/

// prevstate
static uint8_t prev_dpad;
static int32_t prev_axis_x;
static int32_t prev_axis_y;
static int32_t prev_axis_rx;
static int32_t prev_axis_ry;

    // Usage Page: 0x02 (Sim controls)
static int32_t prev_brake;
static int32_t prev_throttle;

    // Usage Page: 0x09 (Button)
static uint16_t prev_buttons;

    // Misc buttons (from 0x0c (Consumer) and others)
static uint8_t prev_misc_buttons;

static int32_t prev_gyro[3];
static int32_t prev_accel[3];


inline static uint8_t decShiftPos(uint8_t pos, uint8_t shift){
	
	return ((pos+UART_BUFF_SIZE-shift)%UART_BUFF_SIZE);
}
inline static uint8_t incShiftPos(uint8_t pos, uint8_t shift){
	
	return ((pos+UART_BUFF_SIZE+shift)%UART_BUFF_SIZE);
}

static void packet_parser(char * data, uint8_t pos){
	if((data[pos] == PROT_SOF) \
	&& (data[incShiftPos(pos, (PROT_BUFF_SIZE-1))] == PROT_EOF) \
	//&& (data[incShiftPos(pos, (1))] == ~data[incShiftPos(pos, (2))])
  )
	{
		modCtrl = data[incShiftPos(pos, (1))] / 0x10;
		modLed = (data[incShiftPos(pos, (1))] % 0x10) & 0x07;
		modChanged = 1;
	}
	
	
}

// Declarations
static void trigger_event_on_gamepad(uni_hid_device_t* d);

//
// Platform Overrides
//

// RX interrupt handler
void on_uart_rx() {

    while (uart_is_readable(UART_ID)) {
        
		ch[pos] = uart_getc(UART_ID);
		
		if(ch[pos] == PROT_EOF){
			packet_parser(&ch[0], decShiftPos(pos, (PROT_BUFF_SIZE-1)));
		}
		
		pos = (pos + 1)%UART_BUFF_SIZE;	
    }
}

static void my_platform_init(int argc, const char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    logi("robcar_platform: init()\n");
    
    // Set up our UART with a basic baud rate.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Actually, we want a different speed
    // The call will return the actual baud rate selected, which will be as close as
    // possible to that requested
    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    // Turn off FIFO's - we want to do this character by character
    uart_set_fifo_enabled(UART_ID, false);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);

    // OK, all set up.
    // Lets send a basic string out, and then run a loop and wait for RX interrupts
    // The handler will count them, but also reflect the incoming data back with a slight change!
    uart_puts(UART_ID, "\nHello, uart interrupts\n");

    logi("robcar_platform: uar init 9600()\n");


#if 0
    uni_gamepad_mappings_t mappings = GAMEPAD_DEFAULT_MAPPINGS;

    // Inverted axis with inverted Y in RY.
    mappings.axis_x = UNI_GAMEPAD_MAPPINGS_AXIS_RX;
    mappings.axis_y = UNI_GAMEPAD_MAPPINGS_AXIS_RY;
    mappings.axis_ry_inverted = true;
    mappings.axis_rx = UNI_GAMEPAD_MAPPINGS_AXIS_X;
    mappings.axis_ry = UNI_GAMEPAD_MAPPINGS_AXIS_Y;

    // Invert A & B
    mappings.button_a = UNI_GAMEPAD_MAPPINGS_BUTTON_B;
    mappings.button_b = UNI_GAMEPAD_MAPPINGS_BUTTON_A;

    uni_gamepad_set_mappings(&mappings);
#endif
}






static void my_platform_on_init_complete(void) {
    logi("robcar_platform: on_init_complete()\n");

    // Safe to call "unsafe" functions since they are called from BT thread

    // Start scanning
    uni_bt_enable_new_connections_unsafe(true);

    // Based on runtime condition, you can delete or list the stored BT keys.
    // if (1)
    if (0)
        uni_bt_del_keys_unsafe();
    else
        uni_bt_list_keys_unsafe();

    // //----------------
    // bd_addr_t controller_addr;

    // sscanf_bd_addr(controller_addr_string, controller_addr);

    // // Notice that this address will be added in the Non-volatile-storage (NVS).
    // // If the device reboots, the address will still be stored.
    // // Adding a duplicate value will do nothing.
    // // You can add up to four entries in the allowlist.
    // uni_bt_allowlist_add_addr(controller_addr);

    // // Finally, enable the allowlist.
    // // Similar to the "add_addr", its value gets stored in the NVS.
    // uni_bt_allowlist_set_enabled(true);

    // ////    ---------- from arduino demo code

    // Turn off LED once init is done.
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    //    uni_bt_service_set_enabled(true);

    uni_property_dump_all();
}

static uni_error_t my_platform_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
    // You can filter discovered devices here. Return any value different from UNI_ERROR_SUCCESS;
    // @param addr: the Bluetooth address
    // @param name: could be NULL, could be zero-length, or might contain the name.
    // @param cod: Class of Device. See "uni_bt_defines.h" for possible values.
    // @param rssi: Received Signal Strength Indicator (RSSI) measured in dBms. The higher (255) the better.

    // As an example, if you want to filter out keyboards, do:
    if (((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD) == UNI_BT_COD_MINOR_KEYBOARD) {
        logi("Ignoring keyboard\n");
        return UNI_ERROR_IGNORE_DEVICE;
    }

    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_device_connected(uni_hid_device_t* d) {
    logi("robcar_platform: device connected: %p\n", d);
}

static void my_platform_on_device_disconnected(uni_hid_device_t* d) {
    logi("robcar_platform: device disconnected: %p\n", d);
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t* d) {
    logi("robcar_platform: device ready: %p\n", d);

    // You can reject the connection by returning an error.
    return UNI_ERROR_SUCCESS;
}
// PWM 0 Forward PWM, fast decay
// 1 PWM Forward PWM, slow decay
// 0 PWM Reverse PWM, fast decay
// PWM 1 Reverse PWM, slow decay
// 0 0 Z Z Coast/fast decay
// 0 1 L H Reverse
// 1 0 H L Forward
// 1 1 L L Brake/slow decay
//Sleep mode nSLEEP pin low
// wheel 0 forward left;  1 forward right;
//       2 backward left;  3 backward right;



// https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf, page 549
      // https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf, page 549
      // https://raspberrypi.github.io/pico-sdk-doxygen/group__hardware__pwm.html
    // if (freq > 2000.0)
    // {
    //   _PWM_config.div = 1;
    // }
    // else if (freq >= 200.0) 
    // {
    //   _PWM_config.div = 10;
    // }
    // else if (freq >= 20.0) 
    // {
    //   _PWM_config.div = 100;
    // }
    // else if (freq >= 10.0) 
    // {
    //   _PWM_config.div = 200;
    // }
    // else if (freq >= ( (float) MIN_PWM_FREQUENCY * freq_CPU / 125000000))
    // {
    //   _PWM_config.div = 255;
    // }
    // _PWM_config.top = ( freq_CPU / freq / _PWM_config.div ) - 1;
        // _actualFrequency = ( freq_CPU  ) / ( (_PWM_config.top + 1) * _PWM_config.div );
    //    if (_phaseCorrect)      _PWM_config.top /= 2;

void motor_drive(uint wheel,uint16_t level, int direction )
{  uint gpio1,gpio2,pin_sleep;
   int motorIndex=-1;
  if (wheel==FOR_LEFT) {
    gpio1 = LEFT_MOTOR1_PIN_1;
    gpio2 = LEFT_MOTOR1_PIN_2;
    pin_sleep = MOTOR1_STBY;
    motorIndex=MOTOR_FW_L;
  } else if (wheel==FOR_RIGHT) {
    gpio1 = RIGHT_MOTOR1_PIN_1;
    gpio2 = RIGHT_MOTOR1_PIN_2;
    pin_sleep = MOTOR1_STBY;
    motorIndex=MOTOR_FW_R;
  } else if (wheel==BACK_LEFT) {
    gpio1 = LEFT_MOTOR2_PIN_1;
    gpio2 = LEFT_MOTOR2_PIN_2;
    pin_sleep = MOTOR2_STBY;
    motorIndex=MOTOR_BW_L;
  } else if (wheel==BACK_RIGHT) {
    gpio1 = RIGHT_MOTOR2_PIN_1;
    gpio2 = RIGHT_MOTOR2_PIN_2;
    pin_sleep = MOTOR2_STBY;
    motorIndex=MOTOR_BW_R;
  } else {
    return;
  }

    if (direction==MOTOR_DIR_BACKWARD) {
        uint gpio_back=gpio1;
        gpio1 =gpio2;
        gpio2 =gpio_back;
    }

  uint slice_num = pwm_gpio_to_slice_num(gpio1);

  

  
  if ((direction==MOTOR_DIR_FORWARD) ||(direction==MOTOR_DIR_BACKWARD)) {
    if (motors_speed[motorIndex] == MIN(level,MAX_VAL)
        && (motors_direction[motorIndex] ==direction)
    ) {
       logi("motor_drive: w/o changes #=%d s=%d dir=%d\n", motorIndex, motors_speed[motorIndex], motors_direction[motorIndex] );
       return;
    } 
    motors_speed[motorIndex] = MIN(level,MAX_VAL);
    motors_direction[motorIndex] =direction;



    pwm_set_enabled(slice_num, false);
    gpio_set_function(gpio1, GPIO_FUNC_PWM); //0A
    // gpio_set_function(gpio2, GPIO_FUNC_PWM); //0B
    gpio_init(gpio2);
    gpio_set_dir(gpio2, GPIO_OUT);
    gpio_put(gpio2, 0);


  
    if (direction==MOTOR_DIR_BACKWARD) {
        pwm_set_both_levels(slice_num, 0, MIN(level,MAX_VAL));
        }
    else {
        pwm_set_both_levels(slice_num, MIN(level,MAX_VAL), 0);
        }

  

    // pwm_set_gpio_level(_pin, PWM_level ); 
    pwm_set_enabled(slice_num, true);
    //  pwm_set_mask_enabled(255);
    gpio_put(pin_sleep, 1); //to run
  }  else {
    gpio_put(pin_sleep, 0); //to sleep
    motors_speed[motorIndex] = 0;
    motors_direction[motorIndex] =0;
    gpio_init(gpio1);
    gpio_set_dir(gpio1, GPIO_OUT);
    gpio_put(gpio1, 0);

    gpio_init(gpio2);
    gpio_set_dir(gpio2, GPIO_OUT);
    gpio_put(gpio2, 0);

    pwm_set_enabled(slice_num, false);
  }
  logi("motor_drive: to gpio1=%d gpio2=%d #=%d s=%d dir=%d \n",gpio1,gpio2, motorIndex, motors_speed[motorIndex], motors_direction[motorIndex] );

}

void car_drive(int direction,uint32_t speed) {
  //    motor_drive(uint wheel,uint32_t level, int direction )
  switch (direction) {
    case CAR_DIR_STOP:
        motor_drive(MOTOR_FW_L,MOTOR_SPEED_0, MOTOR_DIR_STOP );
        motor_drive(MOTOR_FW_R,MOTOR_SPEED_0, MOTOR_DIR_STOP );
        motor_drive(MOTOR_BW_L,MOTOR_SPEED_0, MOTOR_DIR_STOP );
        motor_drive(MOTOR_BW_R,MOTOR_SPEED_0, MOTOR_DIR_STOP );
        break;
    case CAR_DIR_LEFT:
        motor_drive(MOTOR_FW_R,speed, MOTOR_DIR_FORWARD );
        motor_drive(MOTOR_BW_R,speed, MOTOR_DIR_FORWARD );
        motor_drive(MOTOR_FW_L,MOTOR_SPEED_0, MOTOR_DIR_STOP );
        motor_drive(MOTOR_BW_L,MOTOR_SPEED_0, MOTOR_DIR_STOP );
        break;
    case CAR_DIR_RIGHT:
        motor_drive(MOTOR_FW_R,MOTOR_SPEED_0, MOTOR_DIR_FORWARD);
        motor_drive(MOTOR_BW_R,MOTOR_SPEED_0, MOTOR_DIR_FORWARD );
        motor_drive(MOTOR_FW_L,speed, MOTOR_DIR_FORWARD );
        motor_drive(MOTOR_BW_L,speed, MOTOR_DIR_FORWARD );
        break;        
    case CAR_DIR_FORWARD:
        motor_drive(MOTOR_FW_L,speed, MOTOR_DIR_FORWARD );
        motor_drive(MOTOR_BW_L,speed, MOTOR_DIR_FORWARD );
        motor_drive(MOTOR_FW_R,speed, MOTOR_DIR_FORWARD );
        motor_drive(MOTOR_BW_R,speed, MOTOR_DIR_FORWARD );
        break;
    case CAR_DIR_BACKWARD:
        motor_drive(MOTOR_FW_L,speed, MOTOR_DIR_BACKWARD );
        motor_drive(MOTOR_BW_L,speed, MOTOR_DIR_BACKWARD );
        motor_drive(MOTOR_FW_R,speed, MOTOR_DIR_BACKWARD );
        motor_drive(MOTOR_BW_R,speed, MOTOR_DIR_BACKWARD );
        break;
    case CAR_DIR_LEFT_FORWARD :
        motor_drive(MOTOR_FW_L,speed/2, MOTOR_DIR_FORWARD );
        motor_drive(MOTOR_BW_L,speed/2, MOTOR_DIR_FORWARD );
        motor_drive(MOTOR_FW_R,speed  , MOTOR_DIR_FORWARD );
        motor_drive(MOTOR_BW_R,speed  , MOTOR_DIR_FORWARD );
        break;
    case CAR_DIR_RIGHT_FORWARD :
        motor_drive(MOTOR_FW_L,speed  , MOTOR_DIR_FORWARD );
        motor_drive(MOTOR_BW_L,speed  , MOTOR_DIR_FORWARD );
        motor_drive(MOTOR_FW_R,speed/2, MOTOR_DIR_FORWARD );
        motor_drive(MOTOR_BW_R,speed/2, MOTOR_DIR_FORWARD );
        break;
    case CAR_DIR_RIGHT_BACKWARD :
        motor_drive(MOTOR_FW_L,speed  , MOTOR_DIR_BACKWARD );
        motor_drive(MOTOR_BW_L,speed  , MOTOR_DIR_BACKWARD );
        motor_drive(MOTOR_FW_R,speed/2, MOTOR_DIR_BACKWARD );
        motor_drive(MOTOR_BW_R,speed/2, MOTOR_DIR_BACKWARD );
        break;
    case CAR_DIR_LEFT_BACKWARD :
        motor_drive(MOTOR_FW_L,speed/2, MOTOR_DIR_BACKWARD );
        motor_drive(MOTOR_BW_L,speed/2, MOTOR_DIR_BACKWARD );
        motor_drive(MOTOR_FW_R,speed  , MOTOR_DIR_BACKWARD );
        motor_drive(MOTOR_BW_R,speed  , MOTOR_DIR_BACKWARD );
        break;    
    default:
            break;
    }
}   

long mapJ2PWM(long x)
{
  long  res=(MAX_ZONE_XY - MIN_ZONE_XY) * (MOTOR_SPEED_MAX - MOTOR_SPEED_TORQUE) / (MAX_ZONE_XY - MIN_ZONE_XY) + MOTOR_SPEED_TORQUE;
  return res>MOTOR_SPEED_MAX?MOTOR_SPEED_MAX:(res<MOTOR_SPEED_TORQUE+10?0:res);
}

void car_drive2(long speedL,long speedR) {
  
  long motor_speedL=mapJ2PWM(fabs(speedL));
  long motor_speedR=mapJ2PWM(fabs(speedR));
  // stop
  if (motor_speedL<10) {
        motor_drive(MOTOR_FW_L,MOTOR_SPEED_0, MOTOR_DIR_STOP );
        motor_drive(MOTOR_BW_L,MOTOR_SPEED_0, MOTOR_DIR_STOP );
  } else {
        motor_drive(MOTOR_FW_L,motor_speedL, speedL>0?MOTOR_DIR_FORWARD:MOTOR_DIR_BACKWARD );
        motor_drive(MOTOR_BW_L,motor_speedL, speedL>0?MOTOR_DIR_FORWARD:MOTOR_DIR_BACKWARD );
  }
  if (motor_speedR<10) {
        motor_drive(MOTOR_FW_R,MOTOR_SPEED_0, MOTOR_DIR_STOP );
        motor_drive(MOTOR_BW_R,MOTOR_SPEED_0, MOTOR_DIR_STOP );
  } else {
        motor_drive(MOTOR_FW_R,motor_speedR, speedR>0?MOTOR_DIR_FORWARD:MOTOR_DIR_BACKWARD );
        motor_drive(MOTOR_BW_R,motor_speedR, speedR>0?MOTOR_DIR_FORWARD:MOTOR_DIR_BACKWARD);
  
  }   
}


static void my_platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    static uint8_t leds = 0;
    static uint8_t enabled = true;
    static uni_controller_t prev = {0};
    // Find out which PWM slice is connected to GPIO 0 (Slice 0) and GPIO2 (Slice 1)


    uni_gamepad_t* gp;

    // Used to prevent spamming the log, but should be removed in production.
    //    if (memcmp(&prev, ctl, sizeof(*ctl)) == 0) {
    //        return;
    //    }
    prev = *ctl;
    // Print device Id before dumping gamepad.
    // logi("(%p) id=%d ", d, uni_hid_device_get_idx_for_instance(d));
    // uni_controller_dump(ctl);

    switch (ctl->klass) {
        case UNI_CONTROLLER_CLASS_GAMEPAD:
            gp = &ctl->gamepad;
            int32_t flag_change=0;
            //||MONITOR_ACCEL
            if (mask_monitoring & MONITOR_XY) {
                if (prev_dpad!=gp->dpad){
                    flag_change|=CHANGE_DPAD;
                }
                if (prev_buttons!=gp->buttons){
                    flag_change|=CHANGE_BUTTONS;
                }
                if (prev_misc_buttons!=gp->misc_buttons){
                    flag_change|=CHANGE_MISC_BUTTONS;
                }

                if (fabs(prev_axis_x - gp->axis_x)>DEAD_ZONE_XY){
                    flag_change|=CHANGE_X;
                }
                if (fabs(prev_axis_y - gp->axis_y)>DEAD_ZONE_XY){
                    flag_change|=CHANGE_Y;
                    prev_buttons=gp->buttons;
                }

                if (fabs(prev_axis_rx - gp->axis_rx)>DEAD_ZONE_XY){
                    flag_change|=CHANGE_RX;
                }
                if (fabs(prev_axis_ry - gp->axis_ry)>DEAD_ZONE_XY){
                    flag_change|=CHANGE_RY;
                }

                if (fabs(prev_brake - gp->brake)>DEAD_ZONE_XY){
                    flag_change|=CHANGE_BRAKE;
                }
                if (fabs(prev_throttle - gp->throttle)>DEAD_ZONE_XY){
                    flag_change|=CHANGE_THROTTLE;
                }
            }
            if (mask_monitoring & MONITOR_GYRO) {    
                if (fabs(prev_gyro[0] - gp->gyro[0])>DEAD_ZONE_GYRO ||
                    fabs(prev_gyro[1] - gp->gyro[1])>DEAD_ZONE_GYRO ||
                    fabs(prev_gyro[2] - gp->gyro[2])>DEAD_ZONE_GYRO ){
                    flag_change|=CHANGE_GYRO;
                }
            }    
            if (mask_monitoring & MONITOR_ACCEL) {    
                if (fabs(prev_accel[0] - gp->accel[0])>DEAD_ZONE_ACCEL ||
                    fabs(prev_accel[1] - gp->accel[1])>DEAD_ZONE_ACCEL ||
                    fabs(prev_accel[2] - gp->accel[2])>DEAD_ZONE_ACCEL ){
                    flag_change|=CHANGE_ACCEL;
                }
            }    

            //todo
            // static int32_t prev_gyro[3];
            // static int32_t prev_accel[3];

            if (flag_change) {
                logi("fc=%d \n", flag_change);
                
                
                if (flag_change&CHANGE_DPAD) {
                    if (gp->dpad & DPAD_UP) {
                        if (car_motor_n==MOTOR_FW_L) {
                           car_motor_dir++;
                           if (car_motor_dir>MOTOR_DIR_FORWARD) {
                            car_motor_dir=MOTOR_DIR_BACKWARD;
                           }
                        } else {
                           motor_drive(car_motor_n,0, MOTOR_DIR_STOP ); 
                           car_motor_dir=MOTOR_DIR_FORWARD ;
                           car_motor_n=MOTOR_FW_L;
                        }
                        logi("dpad up n=%d s=%d d=%d\n", car_motor_n,car_speed, car_motor_dir);
                        motor_drive(car_motor_n,car_speed, car_motor_dir );
                    }

                    if (gp->dpad & DPAD_DOWN) {
                        if (car_motor_n==MOTOR_FW_R) {
                           car_motor_dir++;
                           if (car_motor_dir>MOTOR_DIR_FORWARD) {
                            car_motor_dir=MOTOR_DIR_BACKWARD;
                           }
                        } else {
                           motor_drive(car_motor_n,0, MOTOR_DIR_STOP ); 
                           car_motor_dir=MOTOR_DIR_FORWARD ;
                           car_motor_n=MOTOR_FW_R;
                        }
                        logi("dpad down n=%d s=%d d=%d\n", car_motor_n,car_speed, car_motor_dir );
                        motor_drive(car_motor_n,car_speed, car_motor_dir );                        
                    }    

                    if (gp->dpad & DPAD_LEFT) {
                        if (car_motor_n==MOTOR_BW_L) {
                           car_motor_dir++;
                           if (car_motor_dir>MOTOR_DIR_FORWARD) {
                            car_motor_dir=MOTOR_DIR_BACKWARD;
                           }
                        } else {
                           motor_drive(car_motor_n,0, MOTOR_DIR_STOP ); 
                           car_motor_dir=MOTOR_DIR_FORWARD ;
                           car_motor_n=MOTOR_BW_L;
                        }
                        logi("dpad left n=%d s=%d d=%d\n", car_motor_n,car_speed, car_motor_dir);
                        motor_drive(car_motor_n,car_speed, car_motor_dir );  
                    }

                    if (gp->dpad & DPAD_RIGHT) {
                        if (car_motor_n==MOTOR_BW_R) {
                           car_motor_dir++;
                           if (car_motor_dir>MOTOR_DIR_FORWARD) {
                            car_motor_dir=MOTOR_DIR_BACKWARD;
                           }
                        } else {
                           motor_drive(car_motor_n,0, MOTOR_DIR_STOP ); 
                           car_motor_dir=MOTOR_DIR_FORWARD ;
                           car_motor_n=MOTOR_BW_R;
                        }
                        logi("dpad right n=%d s=%d d=%d\n", car_motor_n,car_speed, car_motor_dir);
                        motor_drive(car_motor_n,car_speed, car_motor_dir );                     }
                    prev_dpad=gp->dpad;
                } 
                
                if (flag_change&CHANGE_BUTTONS) {
                    if (gp->buttons & BUTTON_Y) {
                        car_speed+=100;
                        if (car_speed>MAX_VAL) {
                           car_speed=0; 
                        }
                        logi("butt Y  n=%d s=%d d=%d\n", car_motor_n,car_speed, car_motor_dir);
                        motor_drive(car_motor_n,car_speed, car_motor_dir );  
                    }
                    prev_buttons=gp->buttons;
                }
                
                if (flag_change&CHANGE_MISC_BUTTONS) prev_misc_buttons=gp->misc_buttons;
                
                if (flag_change&CHANGE_X) prev_axis_x = gp->axis_x;
                
                if (flag_change&CHANGE_Y) prev_axis_y = gp->axis_y;
                
                uint8_t direction_Rxy=0;
                uint32_t speed_Rxy=0;
                long speedX=0;
                long dL=0;
                long dR=0;
                if (flag_change&CHANGE_RX) {
                    // car_drive(uint direction,uint32_t speed);
                    if (fabs(gp->axis_rx)>DEAD_ZONE_XY && gp->axis_rx>0) {
                        direction_Rxy|=JOY_RIGHT;
                    } else if (fabs(gp->axis_rx)>DEAD_ZONE_XY && gp->axis_rx<0) {
                        direction_Rxy|=JOY_LEFT;
                    }
                    if (fabs(gp->axis_rx)>ZONE4_XY) {
                        speed_Rxy=MOTOR_SPEED_MAX;
                    } else if (fabs(gp->axis_rx)>ZONE3_XY) {
                        speed_Rxy=MOTOR_SPEED_4;
                    } else if (fabs(gp->axis_rx)>ZONE2_XY) {
                        speed_Rxy=MOTOR_SPEED_3;
                    } else if (fabs(gp->axis_rx)>ZONE1_XY) {
                        speed_Rxy=MOTOR_SPEED_2;
                    } else if (fabs(gp->axis_rx)>DEAD_ZONE_XY) {
                        speed_Rxy=MOTOR_SPEED_1;
                    } else {
                        speed_Rxy=MOTOR_SPEED_0;
                    }
                    prev_axis_rx=gp->axis_rx;
                }
                
                if (flag_change&CHANGE_RY) {
                    if (fabs(gp->axis_ry)>DEAD_ZONE_XY && gp->axis_ry>0) {
                        direction_Rxy|=JOY_DOWN;
                    } else if (fabs(gp->axis_ry)>DEAD_ZONE_XY && gp->axis_ry<0) {
                        direction_Rxy|=JOY_UP;
                    }
                    if (fabs(gp->axis_ry)>ZONE4_XY) {
                        speed_Rxy=MOTOR_SPEED_MAX;
                    } else if (fabs(gp->axis_ry)>ZONE3_XY) {
                        speed_Rxy=MAX(speed_Rxy,MOTOR_SPEED_4);
                    } else if (fabs(gp->axis_ry)>ZONE2_XY) {
                        speed_Rxy=MAX(speed_Rxy,MOTOR_SPEED_3);
                    } else if (fabs(gp->axis_ry)>ZONE1_XY) {
                        speed_Rxy=MAX(speed_Rxy,MOTOR_SPEED_2);
                    } else if (fabs(gp->axis_ry)>DEAD_ZONE_XY) {
                        speed_Rxy=MAX(speed_Rxy,MOTOR_SPEED_1);
                    } else {
                        speed_Rxy=MAX(speed_Rxy,MOTOR_SPEED_0);
                    }
                    prev_axis_ry=gp->axis_ry;
                }
                // new algo
                if (flag_change&(CHANGE_RY|CHANGE_Y)) {
                    long ry=(gp->axis_ry < -DEAD_ZONE_XY || gp->axis_ry > DEAD_ZONE_XY)? -gp->axis_ry:0;
                    long y=(gp->axis_y < -DEAD_ZONE_XY || gp->axis_y > DEAD_ZONE_XY)? -gp->axis_y:0;

                    
                    if ((y>0) && (ry>0)){ // the same arrow of Y and RY and positive
                       speedX+=MAX(ry,y) ;
                    //    logi("point1 speedX =%d axis_ry=%d axis_y=%d ry=%d y=%d \n", speedX, gp->axis_ry,gp->axis_y,ry,y);
                    } else if ((y<0) && (ry<0)) { // the same arrow of Y and RY and negative
                       speedX+=MIN(ry,y) ;
                    //    logi("point2 speedX =%d axis_ry=%d axis_y=%d ry=%d y=%d \n", speedX, gp->axis_ry,gp->axis_y,ry,y);
                    } else if (((y>0) && (ry<0)) ||((y<0) && (ry>0)))  { // the different arrow of Y and RY
                        speedX+=(ry+y)/2 ;
                    //    logi("point3 speedX =%d axis_ry=%d axis_y=%d ry=%d y=%d \n", speedX, gp->axis_ry,gp->axis_y,ry,y);
                    } else if ((y!=0)&& ry==0){
                       speedX+=y;
                    //    logi("point4 speedX =%d axis_ry=%d axis_y=%d ry=%d y=%d \n", speedX, gp->axis_ry,gp->axis_y,ry,y);
                    } else if ((ry!=0)&& y==0){
                       speedX+=ry;
                    //    logi("point5 speedX =%d axis_ry=%d axis_y=%d ry=%d y=%d \n", speedX, gp->axis_ry,gp->axis_y,ry,y);
                    } else {
                       speedX+=ry+y ;
                    //    logi("point6 speedX =%d ry=%d y=%d \n", speedX, gp->axis_ry,gp->axis_y);
                    }
                    //  logi("speedX =%d\n", speedX);
                }   

                if (flag_change&(CHANGE_RX)) {
                    long rx=(gp->axis_rx < -DEAD_ZONE_XY || gp->axis_rx > DEAD_ZONE_XY)? -gp->axis_rx:0;
                       dL+=rx;
                       dR-=rx;
                }

                if (flag_change&(CHANGE_X)) {
                    long x=(gp->axis_x < -DEAD_ZONE_XY || gp->axis_x > DEAD_ZONE_XY)? -gp->axis_x:0;
                       dL+=x;
                       dR-=x/2;
                    } 
                    //  logi("dL =%d dR =%d \n ", dL, dR);


                // car_drive(direction_Rxy,speed_Rxy);
                if (flag_change&(CHANGE_RY|CHANGE_Y|CHANGE_RX|CHANGE_X)) {
                car_drive2(speedX+dL/3,speedX+dR/3);
                }
                
                if (flag_change&CHANGE_BRAKE) prev_brake=gp->brake;
                
                if (flag_change&CHANGE_THROTTLE) prev_throttle=gp->throttle;
                
                if (flag_change&CHANGE_GYRO) {
                    prev_gyro[0] = gp->gyro[0];
                    prev_gyro[1] = gp->gyro[1];
                    prev_gyro[2] = gp->gyro[2];
                }

                if (flag_change&CHANGE_ACCEL) {
                    prev_accel[0] = gp->accel[0];
                    prev_accel[1] = gp->accel[1];
                    prev_accel[2] = gp->accel[2];
                }

                uni_controller_dump(ctl);
                flag_change=0;
                prev_dpad=gp->dpad;
            }
 


            // Debugging
            // Axis ry: control rumble
            if ((gp->buttons & BUTTON_A) && d->report_parser.play_dual_rumble != NULL) {
                d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 250 /* duration ms */,
                                                  128 /* weak magnitude */, 0 /* strong magnitude */);
            }

            if ((gp->buttons & BUTTON_B) && d->report_parser.play_dual_rumble != NULL) {
                d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 250 /* duration ms */,
                                                  0 /* weak magnitude */, 128 /* strong magnitude */);
            }
            // Buttons: Control LEDs On/Off
            if ((gp->buttons & BUTTON_X) && d->report_parser.set_player_leds != NULL) {
                d->report_parser.set_player_leds(d, leds++ & 0x0f);
            }
            // Axis: control RGB color
            if ((gp->buttons & BUTTON_Y) && d->report_parser.set_lightbar_color != NULL) {
                uint8_t r = (gp->axis_x * 256) / 512;
                uint8_t g = (gp->axis_y * 256) / 512;
                uint8_t b = (gp->axis_rx * 256) / 512;
                d->report_parser.set_lightbar_color(d, r, g, b);
            }

            // Toggle Bluetooth connections
            if ((gp->buttons & BUTTON_SHOULDER_L) && enabled) {
                logi("*** Disabling Bluetooth connections\n");
                uni_bt_enable_new_connections_safe(false);
                enabled = false;
            }
            if ((gp->buttons & BUTTON_SHOULDER_R) && !enabled) {
                logi("*** Enabling Bluetooth connections\n");
                uni_bt_enable_new_connections_safe(true);
                enabled = true;
            }
            break;
        case UNI_CONTROLLER_CLASS_BALANCE_BOARD:
            // Do something
            uni_balance_board_dump(&ctl->balance_board);
            break;
        case UNI_CONTROLLER_CLASS_MOUSE:
            // Do something
            uni_mouse_dump(&ctl->mouse);
            break;
        case UNI_CONTROLLER_CLASS_KEYBOARD:
            // Do something
            uni_keyboard_dump(&ctl->keyboard);
            break;
        default:
            loge("Unsupported controller class: %d\n", ctl->klass);
            break;
    }
}

 const uni_property_t* my_platform_get_property(uni_property_idx_t idx) {
    ARG_UNUSED(idx);
    return NULL;
}

static void my_platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
    switch (event) {
        case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON:
            // Optional: do something when "system" button gets pressed.
            trigger_event_on_gamepad((uni_hid_device_t*)data);
            break;

        case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
            // When the "bt scanning" is on / off. Could be triggered by different events
            // Useful to notify the user
            logi("my_platform_on_oob_event: Bluetooth enabled: %d\n", (bool)(data));
            break;

        default:
            logi("my_platform_on_oob_event: unsupported event: 0x%04x\n", event);
    }
}

//
// Helpers
//
static void trigger_event_on_gamepad(uni_hid_device_t* d) {
    if (d->report_parser.play_dual_rumble != NULL) {
        d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 50 /* duration ms */, 128 /* weak magnitude */,
                                          40 /* strong magnitude */);
    }

    if (d->report_parser.set_player_leds != NULL) {
        static uint8_t led = 0;
        led += 1;
        led &= 0xf;
        d->report_parser.set_player_leds(d, led);
    }

    if (d->report_parser.set_lightbar_color != NULL) {
        static uint8_t red = 0x10;
        static uint8_t green = 0x20;
        static uint8_t blue = 0x40;

        red += 0x10;
        green -= 0x20;
        blue += 0x40;
        d->report_parser.set_lightbar_color(d, red, green, blue);
    }
}



//
// Entry Point
//
struct uni_platform* get_my_platform(void) {
    static struct uni_platform plat = {
        .name = "Igors Robocar Platform",
        .init = my_platform_init,
        .on_init_complete = my_platform_on_init_complete,
        .on_device_discovered = my_platform_on_device_discovered,
        .on_device_connected = my_platform_on_device_connected,
        .on_device_disconnected = my_platform_on_device_disconnected,
        .on_device_ready = my_platform_on_device_ready,
        .on_oob_event = my_platform_on_oob_event,
        .on_controller_data = my_platform_on_controller_data,
        .get_property = my_platform_get_property,
    };

    return &plat;
}