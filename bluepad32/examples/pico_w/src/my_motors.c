#include <stddef.h>
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <uni.h>
#include "math.h"
#include "my_joystick.h"
#include "my_motors.h"
#include "pico/time.h"

#define _sign(x) ((x) > 0 ? 1 : -1)
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

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

// static  int  motors_direction[4]   = {0,0,0,0};
// static  uint16_t motors_speed[4]  = {0,0,0,0};
// static  uint16_t  motors_optical_counter[4]   = {0,0,0,0};
// static  uint32_t  motors_optical_times[4]   = {0,0,0,0};


motor_encoder_t motors[MOTORS_CNT] ={0} ;

motor_encoder_t* get_motor (uint wheel )
{
  return &motors[wheel];
}

static int carMode = -1 ;
static uint pwm_decay = DRV8833_MODE_DECAY ;

void init_pwm_gpio(uint gpio1,uint gpio2, uint pin_sleep) {

    gpio_init(pin_sleep);
    gpio_set_dir(pin_sleep, GPIO_OUT);
    gpio_put(pin_sleep, 0); //to sleep

    // Find out which PWM slice is connected to GPIO 16-19 (Slice 0) and GPIO2 (Slice 1)
    uint slice_num = pwm_gpio_to_slice_num(gpio1);  //0A ->0
    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler    
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, false);
    // irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    // irq_set_enabled(PWM_IRQ_WRAP, true);
    // MOTOR_FORWARD left as PWM
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


void motor_init (uint wheel )
{
  if (wheel>=MOTORS_CNT) {
    return;
  }
  motor_encoder_t* motor=get_motor(wheel);
  motor->index=wheel;
  if (wheel==MOTOR_FW_L) {
    motor->gpio1 = LEFT_MOTOR_FORWARD_PIN_1;
    motor->gpio2 = LEFT_MOTOR_FORWARD_PIN_2;
    motor->pin_sleep = MOTOR_FORWARD_STBY;
  } else if (wheel==MOTOR_FW_R) {
    motor->gpio1 = RIGHT_MOTOR_FORWARD_PIN_1;
    motor->gpio2 = RIGHT_MOTOR_FORWARD_PIN_2;
    motor->pin_sleep = MOTOR_FORWARD_STBY;
  } else if (wheel==MOTOR_BW_L) {
    motor->gpio1 = LEFT_MOTOR_BACKWARD_PIN_1;
    motor->gpio2 = LEFT_MOTOR_BACKWARD_PIN_2;
    motor->pin_sleep = MOTOR_BACKWARD_STBY;
  } else if (wheel==MOTOR_BW_R) {
    motor->gpio1 = RIGHT_MOTOR_BACKWARD_PIN_1;
    motor->gpio2 = RIGHT_MOTOR_BACKWARD_PIN_2;
    motor->pin_sleep = MOTOR_BACKWARD_STBY;
  } else {
    return;
  }
  motor->pwm_decay=pwm_decay;
  init_pwm_gpio(motor->gpio1 , motor->gpio2, motor->pin_sleep );



 
 

  motor->speed=0;
  motor->mode=0;
  motor->abs_encoder_counter=0;
  motor->dir_encoder_counter=0;

  //pid
  motor->kp = PID_P;		// (знач. по умолчанию)
    
  // интегральный - позволяет нивелировать ошибку со временем, имеет накопительный эффект
  motor->ki = PID_I;		// (знач. по умолчанию)
    
  // дифференциальный. Позволяет чуть сгладить рывки, но при большом значении
  // сам становится причиной рывков и раскачки системы!
  motor->kd = PID_D;	

  //system timer
  motor->sysTimer = time_us_32();
  motor->pwmMax = MOTOR_SPEED_TPS_MAX;
  motor->pwmMin = MOTOR_SPEED_TPS_MIN;
  motor->_min = 0;
  motor->_max = 0;
  motor->_lastSpeed = 0;	
  motor->integral = 0;
  // motor->_dt = 20;
  motor->_dts = 0.02;
  motor->posLast = 0;
  motor->posCurrent = 0;
  motor->posTarget = 0;
  motor->speedCurrent = 0;
  motor->speedMax = 170; // default 300/opto 500
  motor->speedTarget = 0;
  // motor->_ratio = 1;
    // uint32_t sysTimer = 0;
  // motor->_accel = 300;
  motor->pwm = 0;	
  // motor->controlSpeed = 0;
  // motor->_stopzone = 8;
  motor->_prevInput = 0;

  // for calculate median
  motor->_buf[0]=0;
  motor->_buf[1]=0;
  motor->_buf[2]=0;
  motor->_filter_buf_point=0;
  motor->_middle_f=0.0;

  motor->_accel = 7;  //default 300
  motor->runMode = PID_SPEED; //IDLE_RUN default
  motor->controlPos = 0;	// для отладки
  motor->controlSpeed = 0;
  motor->_stopzone = 8; //default 7/ opto 8
  carMode=PID_SPEED;

// #define MAX_SPEED 70    // максимальная скорость моторов, в тиках в секунду! 160
// #define MIN_DUTY 50     // мин. сигнал, при котором мотор начинает движение
// #define STEP_SIZE 50    // перемещение по кнопкам крестовины (в тиках)
// #define ACCEL 7         // ускорение
// #define MAX_FOLLOW_SPEED 500  // макс. скорость

}


 

void incTargetPos(uint32_t posFL, uint32_t posFR,  uint32_t posBL, uint32_t posBR) {
    motors[MOTOR_FW_L].posTarget += posFL;
    motors[MOTOR_FW_R].posTarget += posFR;
    motors[MOTOR_BW_L].posTarget += posBL;
    motors[MOTOR_BW_R].posTarget += posBR;
}

void setTargetSpeed(uint32_t speedFL, uint32_t speedFR,  uint32_t speedBL, uint32_t speedBR) {
    motors[MOTOR_FW_L].speedTarget = speedFL;
    motors[MOTOR_FW_R].speedTarget = speedFR;
    motors[MOTOR_BW_L].speedTarget = speedBL;
    motors[MOTOR_BW_R].speedTarget = speedBR;
}



// void  setTargetDeg(long pos) {
//     posTarget = (long)pos * _ratio / 360.0;
//     _mode = AUTO;
// }


 

uint16_t inc_motors_optical_counter0(uint wheel )  
{ 
  if (motors[wheel].mode==MOTOR_MODE_BACKWARD) {
    --motors[wheel].dir_encoder_counter;  
  } else {
    ++motors[wheel].dir_encoder_counter;  
  }
  ++motors[wheel].abs_encoder_counter;  
  return motors[wheel].dir_encoder_counter;
}


// change mode between target pos and accelerated speed with PID
void changeCarMode(int newCarMode) {
  if (carMode!=newCarMode){
  for (int wheel = 0; wheel < MOTORS_CNT; ++wheel) {
      motors[wheel].mode=newCarMode;
      motors[wheel].posCurrent=motors[wheel].abs_encoder_counter;
      motors[wheel].posTarget=motors[wheel].posCurrent;
    }
  }
}
 









int32_t filter(motor_encoder_t* motor ) {
    motor->_buf[motor->_filter_buf_point] = motor->speedCurrent;
    if (++motor->_filter_buf_point >= 3) motor->_filter_buf_point = 0;
    int32_t middle = 0;
    if ((motor->_buf[0] <= motor->_buf[1]) && (motor->_buf[0] <= motor->_buf[2])) {
        middle = (motor->_buf[1] <= motor->_buf[2]) ? motor->_buf[1] : motor->_buf[2];
    } else {
        if ((motor->_buf[1] <= motor->_buf[0]) && (motor->_buf[1] <= motor->_buf[2])) {
            middle = (motor->_buf[0] <= motor->_buf[2]) ? motor->_buf[0] : motor->_buf[2];
        }
        else {
            middle = (motor->_buf[0] <= motor->_buf[1]) ? motor->_buf[0] : motor->_buf[1];
        }
    }
    motor->_middle_f += (middle-motor->_middle_f) * 0.7;
    return motor->_middle_f;
}

// from this video
// void  PIDcontrol(motor_encoder_t* motor, int32_t target, int32_t current, bool cutoff) {
// 	// cutoff нужен только для стабилизации позиции, обнуляет integral и учитывает мёртвую зону мотора
// 	int32_t err = target - current;
// 	motor->pwm = err * motor->kp;										// P составляющая	
// 	motor->pwm += (float)(motor->_prevInput - current) * motor->kd / motor->_dts;	// D составляющая
// 	motor->_prevInput = current;
// 	motor->integral += (float)err * motor->ki * motor->_dts;
// 	motor->pwm += motor->integral;										// I составляющая	
// 	if (cutoff) {		// отсечка
// 		if (abs(err) > motor->_stopzone) 
//     motor->pwm += _sign(err)*motor->pwmMin;	// учитываем pwmMin - мёртвую зону мотора (метод setMinDuty)
// 		else motor->integral = 0;		
// 	} else {
// 		if (err == 0 && target == 0) motor->integral = 0;
// 		else motor->pwm += _sign(err)*motor->pwmMin;
// 	}
// 	motor->pwm = constrain(motor->pwm, -motor->pwmMax, motor->pwmMax);	// ограничиваем по 8 или 10 бит
// 	motor->speed=motor->pwm;									// и поехали
// }


void  PIDcontrol(motor_encoder_t* motor, int32_t target, int32_t current, bool cutoff) {
    // cutoff нужен только для стабилизации позиции, обнуляет integral и учитывает мёртвую зону мотора
    int32_t err = target - current;				// ошибка регулирования
    int32_t deltaInput = motor->_prevInput - current;		// изменение входного сигнала
    motor->pwm = 0;
    if (!cutoff) motor->pwm = err * motor->kp;				// P составляющая для режимов скорости	
    //  logi("tick[PIDcontrol 1]:  motor#%d target=  %d  current=%d -> pwm=%f \n",motor->index,  target,current,  motor->pwm);

    motor->pwm += (float)deltaInput * motor->kd / motor->_dts;	// D составляющая
    //  logi("tick[PIDcontrol 2]:  motor#%d target=  %d  current=%d -> pwm=%f \n",motor->index,  target,current,  motor->pwm);
    motor->_prevInput = current;						// запомнили текущий
    motor->integral += (float)err * motor->ki * motor->_dts;			// интегральная сумма
    if (cutoff) motor->integral += deltaInput * motor->kp;	// +P по скорости изменения для режимов позиции
    motor->integral = constrain(motor->integral, -motor->pwmMax, motor->pwmMax);	// ограничили
    motor->pwm += motor->integral;							// I составляющая	
    // logi("tick[PIDcontrol 3]:  motor#%d target=  %d  current=%d -> pwm=%f \n",motor->index,  target,current,  motor->pwm);

    // if (cutoff) {								// отсечка (для режимов позиции)
    //     if (abs(err) < motor->_stopzone) {
    //       motor->integral = 0; 
    //       motor->pwm = 0;
    //       }
    // } else {									// для скорости
    //     if (err == 0 && target == 0) motor->integral = 0;
    // }
    if (cutoff) {		// отсечка
      if (abs(err) > motor->_stopzone) 
      motor->pwm += _sign(err)*motor->pwmMin;	// учитываем pwmMin - мёртвую зону мотора (метод setMinDuty)
      else motor->integral = 0;		
    } else {
      if (err == 0 && target == 0) motor->integral = 0;
      else motor->pwm += _sign(err)*motor->pwmMin;
    }    
    // motor->pwm = constrain(motor->pwm, -motor->pwmMax, motor->pwmMax);	// ограничиваем по разрешению
    if (_sign(target)==1 ||(target==0)){
      motor->pwm = constrain(motor->pwm, 0, motor->pwmMax);	// ограничиваем по разрешению
    } else if (_sign(target)==-1 ){
      motor->pwm = constrain(motor->pwm, -motor->pwmMax, 0);	// ограничиваем по разрешению
    } else {
      motor->pwm = constrain(motor->pwm, -motor->pwmMax, motor->pwmMax);	// ограничиваем по разрешению
    }
    
    if (cutoff && motor->_min != 0 && motor->_max != 0 && (current <= motor->_min || current >= motor->_max)) {
        motor->speed=0;	// вырубаем, если вышли за диапазон
    } else {
      motor->speed=motor->pwm;								// и поехали
    }
}





 

void tick(motor_encoder_t* motor) {
       // printf("Repeat at %lld\n", time_us_64());
        // motor->posCurrent = motor->abs_encoder_counter;
        motor->posCurrent = motor->dir_encoder_counter;
        // uint64_t t =time_us_64();
        uint32_t t = time_us_32();
        motor->_dt = t - motor->sysTimer;
        motor->_dts = (float) motor->_dt/1000000.0 ;
        motor->sysTimer = t;
        // motor->_dts = (float)(t /1000000.0) ;
        motor->speedCurrent = (int32_t)(motor->posCurrent - motor->posLast) / motor->_dts;	// ищем скорость в тиках/секунду
        
        motor->speedCurrent = filter(motor);  // медиана + RA

        if (motor->index==1) {
        // logi("tick: delta #%d = %d dts = %f \n",motor->index, motor->posCurrent - motor->posLast, motor->_dts);
        // logi("tick[IN]:  motor#%d speed=  %d  target=%d -> current=%d \n",motor->index,  motor->speed,motor->speedTarget,  motor->speedCurrent);

        }
        motor->posLast = motor->posCurrent;
        switch (motor->runMode) {	
          

        case ACCEL_POS: //we use it // Режим переміщення на позицію
            {
                uint32_t err = motor->posTarget - motor->controlPos;												// "ошибка" позиции
                if (err != 0) {
                    if (motor->_accel != 0) {
                        bool thisDir = (motor->controlSpeed * motor->controlSpeed / motor->_accel / 2.0 >= abs(err));  // пора тормозить
                        motor->controlSpeed += motor->_accel * motor->_dts * (thisDir ? -_sign(motor->controlSpeed) : _sign(err));
                    } else {
                        motor->controlSpeed = err / motor->_dts;	// профиль постоянной скорости
                    }
                    motor->controlSpeed = constrain(motor->controlSpeed, -motor->speedMax, motor->speedMax);
                    motor->controlPos += motor->controlSpeed * motor->_dts;
                }
                PIDcontrol(motor,motor->controlPos, motor->posCurrent, true);
            }
            break;
        case PID_POS:				
            PIDcontrol(motor, motor->posTarget, motor->posCurrent, true);
            break;
        case ACCEL_SPEED: 
            {				
                int err = motor->speedTarget - motor->speedCurrent;						// помилка швидкості
                //float reducer = min(abs(err) / _accel*10.0, 1.0);		// уменьшает ускорение, если шаг будет дальше чем установка
                motor->pwm += (float)_sign(err) * motor->_accel/10 * motor->_dts;			// ускоряем/замедляем
                motor->pwm = constrain(motor->pwm, -motor->pwmMax, motor->pwmMax);		// ограничитель 8/10 бит
                motor->speed = motor->pwm;
            }			
            break;
        case PID_SPEED:	// we use it	and it is default	 // режим утримання швидкості
            PIDcontrol(motor,motor->speedTarget, motor->speedCurrent, false);			
            break;
        }		
        // if (motor->index==1) {
        // logi("tick[PID]:  motor#%d speed=  %d  target=%d -> current=%d \n",motor->index,  motor->speed,motor->speedTarget,  motor->speedCurrent);
        // }
        // if (runMode > 1) return (getState() != 0); --ACCEL_SPEED,    PID_SPEED,
        // else return (getState() != 0 || abs(posTarget - posCurrent) > _stopzone); //ACCEL_POS, PID_POS
 
}


void ticks(int defaultMode) {
    int cntPosDone=0;
    for (int wheel = 0; wheel < MOTORS_CNT; ++wheel) {
      tick(&motors[wheel]);
      if ((defaultMode>0) && ((motors[wheel].runMode==ACCEL_POS) ||(motors[wheel].runMode==PID_POS))
        && ((motors[wheel].posTarget - motors[wheel].posCurrent) && (abs(motors[wheel].posTarget - motors[wheel].posCurrent) > motors[wheel]._stopzone)))
       {
        cntPosDone++;
      }
    }
    // the target of pos is reached
    if ((defaultMode>0) &&(cntPosDone==MOTORS_CNT)) {
      for (int wheel = 0; wheel < MOTORS_CNT; ++wheel) {
        motors[wheel].runMode=defaultMode;
      }
    }
    //apply there
    for (int wheel = 0; wheel < MOTORS_CNT; ++wheel) {
      motor_drive(&motors[wheel]);
    }
    


}

void tickOne(uint wheel) {
  tick(&motors[wheel]);
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




void motor_drive0(uint wheel,uint16_t level, int mode ) {
  md_motor_drive2(&motors[wheel],  level,   mode );
}



void md_motor_drive2(motor_encoder_t* motor,uint16_t speed, int mode )
{  

  if ((mode==MOTOR_MODE_FORWARD) ||(mode==MOTOR_MODE_BACKWARD)) {
    if (motor->speed == speed && (motor->mode ==mode) ) {
      //  logi("md_motor_drive2: w/o changes #=%d s=%d dir=%d\n", motor->index, motor->speed, motor->mode );
       return;
    } 
  uint gpio_pwm_val,gpio_static_val;
  if (mode==MOTOR_MODE_BACKWARD) {
        gpio_pwm_val =motor->gpio2;
        gpio_static_val =motor->gpio1;
    } else {
        gpio_pwm_val =motor->gpio1;
        gpio_static_val =motor->gpio2;

    }
    uint slice_num = pwm_gpio_to_slice_num(gpio_pwm_val);    
    motor->speed = speed;
    motor->mode =mode;



    pwm_set_enabled(slice_num, false);
    gpio_set_function(gpio_pwm_val, GPIO_FUNC_PWM); //0A
    gpio_init(gpio_static_val);
    gpio_set_dir(gpio_static_val, GPIO_OUT);
    gpio_put(gpio_static_val, motor->pwm_decay); // pwm will be  norm

  
    if (mode==MOTOR_MODE_BACKWARD) {
        pwm_set_both_levels(slice_num, 0, MIN((pwm_decay == DRV8833_SLOW_DECAY)?WRAPVAL-speed:speed,MAX_VAL));
        }
    else {
        pwm_set_both_levels(slice_num, MIN((pwm_decay == DRV8833_SLOW_DECAY)?WRAPVAL-speed:speed,MAX_VAL), 0);
        }

  

    // pwm_set_gpio_level(_pin, PWM_level ); 
    pwm_set_enabled(slice_num, true);
    //  pwm_set_mask_enabled(255);
    gpio_put(motor->pin_sleep, DRV8833_RUN); //to run
    logi("md_motor_drive2: changed to #=%d s=%d dir=%d\n", motor->index, motor->speed, motor->mode );
  }  else {
    gpio_put(motor->pin_sleep, DRV8833_SLEEP); //to sleep
    motor->speed = 0;
    motor->mode = mode;
    gpio_init(motor->gpio1);
    gpio_set_dir(motor->gpio1, GPIO_OUT);
    gpio_put(motor->gpio1, DRV8833_Z_STATE);

    gpio_init(motor->gpio2);
    gpio_set_dir(motor->gpio2, GPIO_OUT);
    gpio_put(motor->gpio2, DRV8833_Z_STATE);
    uint slice_num = pwm_gpio_to_slice_num(motor->gpio1);    
    pwm_set_enabled(slice_num, false);
  }
}


void motor_drive(motor_encoder_t* motor )
{  

  if (motor->_lastSpeed ==motor->speed ) {
    
    return;
  }
  
  uint gpio_pwm_val,gpio_static_val;
  if ((fabs(motor->speed)-MOTOR_SPEED_DEADZONE)<0) {
     motor->mode = MOTOR_MODE_STOP;
  } else if (motor->speed<0) {
    motor->mode = MOTOR_MODE_BACKWARD;
  } else {
    motor->mode = MOTOR_MODE_FORWARD;
  }
  motor->_lastSpeed = motor->speed;
  // if (motor->index==1) {
  // logi("motor_drive[2]: changed #=%d s=%d dir=%d\n", motor->index, motor->speed, motor->mode );   
  // }
  if ((motor->mode==MOTOR_MODE_BACKWARD)  ||(motor->mode==MOTOR_MODE_FORWARD)) {
  // if (motor->index==1) {
  // logi("motor_drive[3]: changed #=%d s=%d dir=%d\n", motor->index, motor->speed, motor->mode );   
  // }
    if (motor->mode==MOTOR_MODE_BACKWARD) {
        gpio_pwm_val =motor->gpio2;
        gpio_static_val =motor->gpio1;
    } else {
        gpio_pwm_val = motor->gpio1;
        gpio_static_val =motor->gpio2;
    }
    uint slice_num = pwm_gpio_to_slice_num(gpio_pwm_val);    
    pwm_set_enabled(slice_num, false);
    gpio_set_function(gpio_pwm_val, GPIO_FUNC_PWM); //0A
    gpio_init(gpio_static_val);
    gpio_set_dir(gpio_static_val, GPIO_OUT);
    gpio_put(gpio_static_val, motor->pwm_decay); // pwm will be  norm
    // uint32_t sp1=    mapTickPerSec2PWM((int32_t)fabs(motor->speed));
    uint32_t sp=fabs((motor->speed<0?-motor->speed:motor->speed) * (MOTOR_SPEED_MAX - MOTOR_SPEED_TORQUE) / (MOTOR_SPEED_TPS_MAX - MOTOR_SPEED_TPS_MIN) + MOTOR_SPEED_TORQUE);
    
    
    if (motor->pwm_decay == DRV8833_SLOW_DECAY) {
       sp= WRAPVAL-sp;
    }
    // if (motor->index==1) {
    //  logi("motor_drive[4]: changed #=%d sp=%d dir=%d pwm=%d \n", motor->index, sp, motor->mode, motor->pwm );   
    // }
    if (motor->mode==MOTOR_MODE_BACKWARD) {
        pwm_set_both_levels(slice_num, 0, fabs(sp));
        }
    else {
        pwm_set_both_levels(slice_num, fabs(sp), 0);
        }

  

    pwm_set_enabled(slice_num, true);
    //  pwm_set_mask_enabled(255);
    gpio_put(motor->pin_sleep, DRV8833_RUN); //to run
    // logi("motor_drive2  changed #=%d s=%d dir=%d\n", motor->index, motor->speed, motor->mode );
  }  else {
    gpio_put(motor->pin_sleep, DRV8833_SLEEP); //to sleep
    gpio_init(motor->gpio1);
    gpio_set_dir(motor->gpio1, GPIO_OUT);
    gpio_put(motor->gpio1, DRV8833_Z_STATE);

    gpio_init(motor->gpio2);
    gpio_set_dir(motor->gpio2, GPIO_OUT);
    gpio_put(motor->gpio2, DRV8833_Z_STATE);
    uint slice_num = pwm_gpio_to_slice_num(motor->gpio1);    
    pwm_set_enabled(slice_num, false);
  }
}


uint32_t mapJ2PWM(uint32_t x)
{
  uint32_t  res=(x - MIN_ZONE_XY) * (MOTOR_SPEED_MAX - MOTOR_SPEED_TORQUE) / (MAX_ZONE_XY - MIN_ZONE_XY) + MOTOR_SPEED_TORQUE;
  res= (res>MOTOR_SPEED_MAX)?MOTOR_SPEED_MAX:(res<MOTOR_SPEED_TORQUE+10?0:res);
  return (res>MOTOR_SPEED_MAX)?MOTOR_SPEED_MAX:(res<MOTOR_SPEED_TORQUE+10?0:res);
}

int32_t mapJ2TickPerSec(int32_t x)
{
  if (fabs(x )< MIN_ZONE_XY) { return 0;}
  int32_t  res=x * (MOTOR_SPEED_TPS_MAX ) / MAX_ZONE_XY  ;
  return res;
}

 


void car_drive_PID(int32_t speedL,int32_t speedR) {
  int32_t motor_speedL=mapJ2TickPerSec(speedL);
  int32_t motor_speedR=mapJ2TickPerSec(speedR);
  logi("car_drive_PID:  speedL=%d speedR=%d motor_speedL=%d motor_speedR=%d\n", speedL, speedR, motor_speedL, motor_speedR );
  motors[MOTOR_FW_L].speedTarget=motor_speedL;
  motors[MOTOR_BW_L].speedTarget=motor_speedL;
  motors[MOTOR_FW_R].speedTarget=motor_speedR;
  motors[MOTOR_BW_R].speedTarget=motor_speedR;
}

void car_drive(uint32_t speedL,uint32_t speedR) {
   logi("car_drive:  speedL=%d speedR=%d\n", speedL, speedR );

  uint32_t motor_speedL=mapJ2PWM(fabs(speedL));
  uint32_t motor_speedR=mapJ2PWM(fabs(speedR));
  

  // stop
  if (fabs(speedL)<DEAD_ZONE_XY) {
        md_motor_drive2(&motors[MOTOR_FW_L],MOTOR_SPEED_0, MOTOR_MODE_STOP );
        md_motor_drive2(&motors[MOTOR_BW_L],MOTOR_SPEED_0, MOTOR_MODE_STOP );
  } else {
        md_motor_drive2(&motors[MOTOR_FW_L],motor_speedL, speedL>0?MOTOR_MODE_FORWARD:MOTOR_MODE_BACKWARD );
        md_motor_drive2(&motors[MOTOR_BW_L],motor_speedL, speedL>0?MOTOR_MODE_FORWARD:MOTOR_MODE_BACKWARD );
  }

  if (fabs(speedR)<DEAD_ZONE_XY) {
        md_motor_drive2(&motors[MOTOR_FW_R],MOTOR_SPEED_0, MOTOR_MODE_STOP );
        md_motor_drive2(&motors[MOTOR_BW_R],MOTOR_SPEED_0, MOTOR_MODE_STOP );
  } else {
        md_motor_drive2(&motors[MOTOR_FW_R],motor_speedR, speedR>0?MOTOR_MODE_FORWARD:MOTOR_MODE_BACKWARD );
        md_motor_drive2(&motors[MOTOR_BW_R],motor_speedR, speedR>0?MOTOR_MODE_FORWARD:MOTOR_MODE_BACKWARD);
  }   
}
