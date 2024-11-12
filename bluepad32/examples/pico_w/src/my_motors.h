/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MY_MOTORS
#define _MY_MOTORS



#ifdef __cplusplus
extern "C" {
#endif

struct motor_encoder_s {
    int  index;
    int  mode;
    int32_t speed;
    uint32_t  abs_encoder_counter;
    int32_t  dir_encoder_counter;
    uint64_t  encoder_time   ;
    uint32_t  sysTimer   ;
    uint32_t _buf[3];
    uint_fast8_t _filter_buf_point ;  
    float _middle_f ;  

    // коэффициенты ПИД регулятора
    // пропорциональный - от него зависит агрессивность управления, нужно увеличивать kp
    // при увеличении нагрузки на вал, чтобы регулятор подавал больший управляющий ШИМ сигнал
    float kp ;		// (знач. по умолчанию)
    
    // интегральный - позволяет нивелировать ошибку со временем, имеет накопительный эффект
    float ki ;		// (знач. по умолчанию)
    
    // дифференциальный. Позволяет чуть сгладить рывки, но при большом значении
    // сам становится причиной рывков и раскачки системы!
    float kd ;		// (знач. по умолчанию)    

    uint32_t _min , _max ;
    float integral ; // интегральная сумма
    int32_t _dt ;
    float _dts ;
    int32_t posLast , posCurrent , posTarget ;
    int32_t speedMax , speedTarget,speedCurrent,_lastSpeed ;
    int32_t _prevInput;
    float pwm;
    int32_t pwmMin ; //0
    int32_t pwmMax ; //255
    uint gpio1,gpio2,pin_sleep,pwm_decay;
    int32_t _accel ;
    int8_t runMode ;
    int32_t controlPos ;	// для отладки
    float controlSpeed ;
    int32_t _stopzone ;
    
};
typedef struct motor_encoder_s motor_encoder_t;


// https://arduino.ua/prod3697-draiver-dvigatelei-dvyhkanalnii-drv8833

// Логіка роботи драйвера:
// IN1	        IN2	   STBY	OUT1	    OUT2	    Опис режиму
// Високий	Високий	Високий	Низький	    Низький	    Гальмування
// Низький	Високий	Високий	Низький	    Високий	    Зворотне обертання
// Низький	Високий	Високий	Низький	    Низький	    Гальмування
// Високий	Низький	Високий	Високий	    Низький	    Пряме обертання
// Високий	Низький	Високий	Низький	    Низький	    Гальмування
// Низький	Низький	Високий	Виключений	Виключений	Стоп
//    -	   -	Низький	Виключений	Виключений	Очікування

//https://www.ti.com/lit/ds/symlink/drv8833.pdf?ts=1728735941257&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FDRV8833%253Futm_source%253Dgoogle%2526utm_medium%253Dcpc%2526utm_campaign%253Dasc-mdbu-null-44700045336317173_prodfolderdynamic-cpc-pf-google-ww_en_int%2526utm_content%253Dprodfolddynamic%2526ds_k%253DDYNAMIC+SEARCH+ADS%2526DCM%253Dyes%2526gad_source%253D1%2526gclid%253DCjwKCAjwvKi4BhABEiwAH2gcwwgBp0jcbk6fpD_rP5sUf1QDQ5ERuCeUmAdO25ZHMQFmUAZjAfEdsRoCmigQAvD_BwE%2526gclsrc%253Daw.ds
// The current regulation is a fixed frequency PWM slow decay.
// xIN1 xIN2 FUNCTION
// PWM 0 Forward PWM, fast decay
// 1 PWM Forward PWM, slow decay
// 0 PWM Reverse PWM, fast decay
// PWM 1 Reverse PWM, slow decay
// ƒPWM Current control PWM frequency Internal PWM frequency 50 kHz

#define DRV8833_SLOW_DECAY 1 // pwm inverted in this case 
#define DRV8833_FAST_DECAY 0 // pwm not inverted in this case
#define DRV8833_MODE_DECAY DRV8833_FAST_DECAY // pwm not inverted in this case
#define DRV8833_SLEEP 0
#define DRV8833_RUN 1
// Coast/fast decay
#define DRV8833_Z_STATE 0
//Brake/slow decay
#define DRV8833_STOP_STATE 1 
// 1 PWM Forward PWM, slow decay
// 0 PWM Reverse PWM, fast decay

// The inputs can also be used for PWM control of the motor speed. When controlling a winding with PWM, when
// the drive current is interrupted, the inductive nature of the motor requires that the current must continue to flow.
// This is called recirculation current. To handle this recirculation current, the H-bridge can operate in two different
// states: fast decay or slow decay. In fast decay mode, the H-bridge is disabled and recirculation current flows
// through the body diodes; in slow decay, the motor winding is shorted.
// To PWM using fast decay, the PWM signal is applied to one xIN pin while the other is held low; to use slow
// decay, one xIN pin is held high.


//      GPIO	0	1	2	3	4	5	6	7	8	9	10	11	12	13	14	15
// PWM Channel	0A	0B	1A	1B	2A	2B	3A	3B	4A	4B	5A	5B	6A	6B	7A	7B
//       GPIO	16	17	18	19	20	21	22	23	24	25	26	27	28	29		
// PWM Channel	0A	0B	1A	1B	2A	2B	3A	3B	4A	4B	5A	5B	6A	6B		







// 16 17 gnd 18 19 20
// forward nc  AIN2 AIN1 STBY BIN1 BIN2 NC GND
//         -    17   16   20   18   19  -  GND
//         -    blue mag  red  yel  ora -  green  
// 16 17 -> 0A 0B -> AIN1 AIN2 ->magenta blue
// 18 19 -> 1A 1B -> BIN1 BIN2 ->yellow orange
// 20 stand by
// #0A
#define LEFT_MOTOR_FORWARD_PIN_1 18
// #0B
#define LEFT_MOTOR_FORWARD_PIN_2 19
// #1A
#define RIGHT_MOTOR_FORWARD_PIN_1 16
// #1B
#define RIGHT_MOTOR_FORWARD_PIN_2 17
#define MOTOR_FORWARD_STBY 20





/* MOTOR_BACKWARD*/
// 15 14 gnd  13 12 11
// bacward nc  AIN2 AIN1 STBY BIN1 BIN2 NC GND
//         -    15   14   11    12   13  -  GND
//         -    mage blue red oran yell  -  green  
// 14 15 -> 7A 7B -> AIN1 AIN2 -> blue magenta
// 12 13 -> 6A 6B -> BIN1 BIN2 -> orange yellow
// 11 stand by red
// #7A
#define LEFT_MOTOR_BACKWARD_PIN_1 14
// #7B
#define LEFT_MOTOR_BACKWARD_PIN_2 15
// #6A
#define RIGHT_MOTOR_BACKWARD_PIN_1 12
// #6B
#define RIGHT_MOTOR_BACKWARD_PIN_2 13
#define MOTOR_BACKWARD_STBY 11

//optical counters
#define LEFT_MOTOR_FORWARD_PIN_OPTICAL 0
#define RIGHT_MOTOR_FORWARD_PIN_OPTICAL 1
#define LEFT_MOTOR_BACKWARD_PIN_OPTICAL 2
#define RIGHT_MOTOR_BACKWARD_PIN_OPTICAL 3


#define UART_TX_PIN 4
#define UART_RX_PIN 5

#define UART_ID uart1
#define BAUD_RATE 115200

#define PARITY    UART_PARITY_NONE
#define DATA_BITS 8
#define STOP_BITS 1


#define PROT_SOF 0x01
#define PROT_EOF 0x02
#define UART_BUFF_SIZE 0x08
#define PROT_BUFF_SIZE 0x04





// power 40     39    38   37  36
//       vbus vsys   gnd enab  3v3
//        -    red black   -   pink red  
 


// pwm
// a PWM frequency of 5 kHz. (125M/5/5000=5kHz)
#define WRAPVAL 5000
#define MAX_VAL WRAPVAL-100
#define CLKDIV 5.0f
// коэффициенты ПИД
#define PID_P 2.2
#define PID_I 0.4
#define PID_D 0.01


enum {
    MOTOR_FW_L = 0,
    MOTOR_FW_R = 1,
    MOTOR_BW_L = 2,
    MOTOR_BW_R = 3
};

enum AM_runMode {	
    IDLE_RUN=0,
    ACCEL_POS=1,
    PID_POS,
    ACCEL_SPEED,
    PID_SPEED
};

#define MOTORS_CNT 4
#define TICK_PID_TIMER 40



#define MOTOR_SPEED_0  0
#define MOTOR_SPEED_DEADZONE  10
#define MOTOR_SPEED_1  MAX_VAL-4000
#define MOTOR_SPEED_2  MAX_VAL-3000
#define MOTOR_SPEED_3  MAX_VAL-2000
#define MOTOR_SPEED_4  MAX_VAL-1000
#define MOTOR_SPEED_MAX  MAX_VAL
#define MOTOR_SPEED_TORQUE  2550
#define MOTOR_SPEED_TPS_MAX 200
#define MOTOR_SPEED_TPS_MIN 10

#define MOTOR_MODE_BACKWARD   -1
#define MOTOR_MODE_AUTO  0
#define MOTOR_MODE_STOP  2
#define MOTOR_MODE_FORWARD  1


 
 


motor_encoder_t* get_motor (uint wheel);
void md_motor_drive2(motor_encoder_t* motor,uint16_t speed, int mode);
void motor_drive0(uint wheel,uint16_t level, int mode);
void motor_drive(motor_encoder_t* motor);
void car_drive(uint32_t speedL,uint32_t speedR);
void car_drive_PID(int32_t speedL,int32_t speedR);
uint16_t inc_motors_optical_counter0(uint wheel )  ;
void motor_init (uint wheel );
void tickOne(uint wheel) ;
void ticks(int defaultMode) ;
void tick(motor_encoder_t* motor) ;

void incTargetPos(uint32_t posFL, uint32_t posFR,  uint32_t posBL, uint32_t posBR);
void setTargetSpeed(uint32_t speedFL, uint32_t speedFR,  uint32_t speedBL, uint32_t speedBR);

#ifdef __cplusplus
}
#endif
#endif
