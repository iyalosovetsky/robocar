/*
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MY_JOYSTICK
#define _MY_JOYSTICK



#ifdef __cplusplus
extern "C" {
#endif

#define DEAD_ZONE_XY 20 
#define ZONE1_XY 70 
#define ZONE2_XY 130
#define ZONE3_XY 300
#define ZONE4_XY 470

#define DEAD_ZONE_ACCEL 150
#define DEAD_ZONE_GYRO 500


#define MAX_ZONE_XY 512
#define MIN_ZONE_XY 0

#define JOY_UP 1
#define JOY_DOWN 2
#define JOY_RIGHT 4
#define JOY_LEFT 8


#ifdef __cplusplus
}
#endif
#endif
