/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/time.h"

#define TASK_ALTITUDE_RATE_HZ 100

typedef struct positionConfig_s {
    uint8_t altitude_source;
    uint8_t altitude_prefer_baro;
    uint16_t altitude_lpf;                // lowpass cutoff (value / 100) Hz for altitude smoothing
    uint16_t altitude_d_lpf;              // lowpass for (value / 100) Hz for altitude derivative smoothing
} positionConfig_t;

typedef struct position_msp_debug_s {
    /*uint8_t wp_no;
    uint32_t lat;
    uint32_t lon;
    uint32_t AltHold;
    uint16_t heading;
    uint16_t time_to_stay;
    uint8_t nav_flag;*/
    int32_t msg1;
    int32_t msg2;
    int32_t msg3;
    int32_t msg4;
    int32_t msg5;
    int32_t msg6;
    int32_t msg7;
} position_msp_debug;

extern position_msp_debug position_msp;
extern position_msp_debug setWP_msp;

PG_DECLARE(positionConfig_t, positionConfig);

void calculateEstimatedAltitude(void);
void positionInit(void);
int32_t getEstimatedAltitudeCm(void);
float getAltitude(void);
int16_t getEstimatedVario(void);
