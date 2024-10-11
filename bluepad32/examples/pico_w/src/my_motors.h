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





// PICO_CONFIG: PARAM_ASSERTIONS_ENABLED_HARDWARE_TIMER, Enable/disable assertions in the hardware_timer module, type=bool, default=0, group=hardware_timer

//      GPIO	0	1	2	3	4	5	6	7	8	9	10	11	12	13	14	15
// PWM Channel	0A	0B	1A	1B	2A	2B	3A	3B	4A	4B	5A	5B	6A	6B	7A	7B
//       GPIO	16	17	18	19	20	21	22	23	24	25	26	27	28	29		
// PWM Channel	0A	0B	1A	1B	2A	2B	3A	3B	4A	4B	5A	5B	6A	6B		

// #0A
#define LEFT_MOTOR1_PIN_1 16
// #0B
#define LEFT_MOTOR1_PIN_2 17
// #1A
#define RIGHT_MOTOR1_PIN_1 18
// #1B
#define RIGHT_MOTOR1_PIN_2 19
#define MOTOR1_STBY 20


/* motor2*/
// #7A
#define LEFT_MOTOR2_PIN_1 14
// #7B
#define LEFT_MOTOR2_PIN_2 15
// #6A
#define RIGHT_MOTOR2_PIN_1 12
// #6B
#define RIGHT_MOTOR2_PIN_2 13
#define MOTOR2_STBY 11



#ifdef __cplusplus
}
#endif
#endif
