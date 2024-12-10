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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/pwm_output.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"

#include "io/gimbal.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"
#include "build/debug.h" // 为了记录servo 240802 jsl
/*
设计三个宏，分别用来单独分析左/右舵机的细分数据、和拿左右舵机的粗分数据
三者同时只能取一个，因为debug变量总数不够
240803 jsl
*/
// #define SERVO_LOG_DEBUG_LEFT // 单独记录debug左舵机 240802 jsl
// #define SERVO_LOG_DEBUG_RIGHT // 单独记录debug右舵机 240803 jsl
#define SERVO_LOG_BI // 记录双旋翼的左右舵机 240803 jsl


PG_REGISTER_WITH_RESET_FN(servoConfig_t, servoConfig, PG_SERVO_CONFIG, 0);

void pgResetFn_servoConfig(servoConfig_t *servoConfig)
{
    servoConfig->dev.servoCenterPulse = 1500;
    servoConfig->dev.servoPwmRate = 50;
    servoConfig->tri_unarmed_servo = 1;
    servoConfig->servo_lowpass_freq = 0;
    servoConfig->channelForwardingStartChannel = AUX1;

#ifdef SERVO1_PIN
    servoConfig->dev.ioTags[0] = IO_TAG(SERVO1_PIN);
#endif
#ifdef SERVO2_PIN
    servoConfig->dev.ioTags[1] = IO_TAG(SERVO2_PIN);
#endif
#ifdef SERVO3_PIN
    servoConfig->dev.ioTags[2] = IO_TAG(SERVO3_PIN);
#endif
#ifdef SERVO4_PIN
    servoConfig->dev.ioTags[3] = IO_TAG(SERVO4_PIN);
#endif
}

PG_REGISTER_ARRAY(servoMixer_t, MAX_SERVO_RULES, customServoMixers, PG_SERVO_MIXER, 0);

PG_REGISTER_ARRAY_WITH_RESET_FN(servoParam_t, MAX_SUPPORTED_SERVOS, servoParams, PG_SERVO_PARAMS, 0);

void pgResetFn_servoParams(servoParam_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        RESET_CONFIG(servoParam_t, &instance[i],
            .min = DEFAULT_SERVO_MIN,
            .max = DEFAULT_SERVO_MAX,
            .middle = DEFAULT_SERVO_MIDDLE,
            .rate = 100,
            .forwardFromChannel = CHANNEL_FORWARDING_DISABLED
        );
    }
}

// no template required since default is zero
PG_REGISTER(gimbalConfig_t, gimbalConfig, PG_GIMBAL_CONFIG, 0);

int16_t servo[MAX_SUPPORTED_SERVOS];

static uint8_t servoRuleCount = 0;
static servoMixer_t currentServoMixer[MAX_SERVO_RULES];
static int useServo;


#define COUNT_SERVO_RULES(rules) (sizeof(rules) / sizeof(servoMixer_t))
// mixer rule format servo, input, rate, speed, min, max, box
static const servoMixer_t servoMixerAirplane[] = {
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_RUDDER,      INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_ELEVATOR,    INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_THROTTLE,    INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerFlyingWing[] = {
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL, -100, 0, 0, 100, 0 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_THROTTLE,    INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerTri[] = {
    { SERVO_RUDDER, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
};

#if defined(USE_UNCOMMON_MIXERS)
/* servoMixerBi 是rule的数组
   rule是servoMixer_t类型的结构体
240731 jsl*/  
static const servoMixer_t servoMixerBI[] = {
    /* input source共14个、双旋翼只用2个：stabilized yaw + pitch
       从servo的角度看过去，yaw是两个都cw转、pitch是一个cw一个ccw -> 对yaw都+1、对pitch+1-1 
       左舵 偏航输入 +100%响应 不限速 MIN0 MAX100 无box 240730 jsl */
    /*
    上拉
    { SERVO_BICOPTER_LEFT, INPUT_STABILIZED_YAW,     100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_LEFT, INPUT_STABILIZED_PITCH,  -100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT, INPUT_STABILIZED_YAW,    100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    下推
    { SERVO_BICOPTER_LEFT, INPUT_STABILIZED_YAW,     100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_LEFT, INPUT_STABILIZED_PITCH,  -100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT, INPUT_STABILIZED_YAW,    100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    */
    { SERVO_BICOPTER_LEFT, INPUT_STABILIZED_YAW,     100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_LEFT, INPUT_STABILIZED_PITCH,  -100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT, INPUT_STABILIZED_YAW,    100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
#ifdef CONFIGURATION_TAILSITTER
    { SERVO_BICOPTER_LEFT_ELEVON, INPUT_STABILIZED_YAW,    -100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_LEFT_ELEVON, INPUT_STABILIZED_PITCH,   100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT_ELEVON, INPUT_STABILIZED_YAW,   -100, 0, 0, 100, 0 },
    { SERVO_BICOPTER_RIGHT_ELEVON, INPUT_STABILIZED_PITCH, -100, 0, 0, 100, 0 },
#endif
};

static const servoMixer_t servoMixerDual[] = {
    { SERVO_DUALCOPTER_LEFT, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_DUALCOPTER_RIGHT, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerSingle[] = {
    { SERVO_SINGLECOPTER_1, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_1, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_2, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_2, INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_3, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_3, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_4, INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
    { SERVO_SINGLECOPTER_4, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
};

static const servoMixer_t servoMixerHeli[] = {
    { SERVO_HELI_LEFT, INPUT_STABILIZED_PITCH,   -50, 0, 0, 100, 0 },
    { SERVO_HELI_LEFT, INPUT_STABILIZED_ROLL,    -87, 0, 0, 100, 0 },
    { SERVO_HELI_LEFT, INPUT_RC_AUX1,    100, 0, 0, 100, 0 },
    { SERVO_HELI_RIGHT, INPUT_STABILIZED_PITCH,  -50, 0, 0, 100, 0 },
    { SERVO_HELI_RIGHT, INPUT_STABILIZED_ROLL,  87, 0, 0, 100, 0 },
    { SERVO_HELI_RIGHT, INPUT_RC_AUX1,    100, 0, 0, 100, 0 },
    { SERVO_HELI_TOP, INPUT_STABILIZED_PITCH,   100, 0, 0, 100, 0 },
    { SERVO_HELI_TOP, INPUT_RC_AUX1,    100, 0, 0, 100, 0 },
    { SERVO_HELI_RUD, INPUT_STABILIZED_YAW, 100, 0, 0, 100, 0 },
};
#else
#define servoMixerBI NULL
#define servoMixerDual NULL
#define servoMixerSingle NULL
#define servoMixerHeli NULL
#endif // USE_UNCOMMON_MIXERS

static const servoMixer_t servoMixerGimbal[] = {
    { SERVO_GIMBAL_PITCH, INPUT_GIMBAL_PITCH, 125, 0, 0, 100, 0 },
    { SERVO_GIMBAL_ROLL, INPUT_GIMBAL_ROLL,  125, 0, 0, 100, 0 },
};

const mixerRules_t servoMixers[] = {
    /* servoMixers 是数组
       每个元素是 mixerRules_t 类型的结构体、
       包括rulecount数和指向rule(s)的指针(数组首地址)、
       加起来就能访问所有rule 
       240731 jsl*/
    { 0, NULL },                // entry 0
    { COUNT_SERVO_RULES(servoMixerTri), servoMixerTri },       // MULTITYPE_TRI
    { 0, NULL },                // MULTITYPE_QUADP
    { 0, NULL },                // MULTITYPE_QUADX
    { COUNT_SERVO_RULES(servoMixerBI), servoMixerBI },        // MULTITYPE_BI
    // 双旋翼有4条规则：左右各2条：1偏航1俯仰 240731 jsl
    { COUNT_SERVO_RULES(servoMixerGimbal), servoMixerGimbal },    // * MULTITYPE_GIMBAL
    { 0, NULL },                // MULTITYPE_Y6
    { 0, NULL },                // MULTITYPE_HEX6
    { COUNT_SERVO_RULES(servoMixerFlyingWing), servoMixerFlyingWing },// * MULTITYPE_FLYING_WING
    { 0, NULL },                // MULTITYPE_Y4
    { 0, NULL },                // MULTITYPE_HEX6X
    { 0, NULL },                // MULTITYPE_OCTOX8
    { 0, NULL },                // MULTITYPE_OCTOFLATP
    { 0, NULL },                // MULTITYPE_OCTOFLATX
    { COUNT_SERVO_RULES(servoMixerAirplane), servoMixerAirplane },  // * MULTITYPE_AIRPLANE
    { COUNT_SERVO_RULES(servoMixerHeli), servoMixerHeli },                // * MULTITYPE_HELI_120_CCPM
    { 0, NULL },                // * MULTITYPE_HELI_90_DEG
    { 0, NULL },                // MULTITYPE_VTAIL4
    { 0, NULL },                // MULTITYPE_HEX6H
    { 0, NULL },                // * MULTITYPE_PPM_TO_SERVO
    { COUNT_SERVO_RULES(servoMixerDual), servoMixerDual },      // MULTITYPE_DUALCOPTER
    { COUNT_SERVO_RULES(servoMixerSingle), servoMixerSingle },    // MULTITYPE_SINGLECOPTER
    { 0, NULL },                // MULTITYPE_ATAIL4
    { 0, NULL },                // MULTITYPE_CUSTOM
    { 0, NULL },                // MULTITYPE_CUSTOM_PLANE
    { 0, NULL },                // MULTITYPE_CUSTOM_TRI
    { 0, NULL },
};

int16_t determineServoMiddleOrForwardFromChannel(servoIndex_e servoIndex)
{
    const uint8_t channelToForwardFrom = servoParams(servoIndex)->forwardFromChannel;

    if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED && channelToForwardFrom < rxRuntimeState.channelCount) {
        return scaleRangef(constrainf(rcData[channelToForwardFrom], PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, servoParams(servoIndex)->min, servoParams(servoIndex)->max);
    }

    return servoParams(servoIndex)->middle;
}

int servoDirection(int servoIndex, int inputSource)
{
    // determine the direction (reversed or not) from the direction bitfield of the servo
    if (servoParams(servoIndex)->reversedSources & (1 << inputSource)) {
        return -1;
    } else {
        return 1;
    }
}

void loadCustomServoMixer(void)
{
    // reset settings
    servoRuleCount = 0;
    memset(currentServoMixer, 0, sizeof(currentServoMixer));

    // load custom mixer into currentServoMixer
    for (int i = 0; i < MAX_SERVO_RULES; i++) {
        // check if done
        if (customServoMixers(i)->rate == 0) {
            break;
        }
        currentServoMixer[i] = *customServoMixers(i);
        servoRuleCount++;
    }
}

static void servoConfigureOutput(void)
{
    if (useServo) {
        servoRuleCount = servoMixers[getMixerMode()].servoRuleCount;
        if (servoMixers[getMixerMode()].rule) {
            for (int i = 0; i < servoRuleCount; i++)
                currentServoMixer[i] = servoMixers[getMixerMode()].rule[i];
                /* currentServoMixer 是双旋翼servo rules的数组
                   只包括了双旋翼的rules
                   240731 jsl*/
        }
    }

    switch (getMixerMode()) {
    case MIXER_CUSTOM_AIRPLANE:
    case MIXER_CUSTOM_TRI:
        loadCustomServoMixer();
        break;
    default:
        break;
    }
}


void servosInit(void)
{
    // enable servos for mixes that require them. note, this shifts motor counts.
    useServo = mixers[getMixerMode()].useServo;
    // if we want camstab/trig, that also enables servos, even if mixer doesn't
    if (featureIsEnabled(FEATURE_SERVO_TILT) || featureIsEnabled(FEATURE_CHANNEL_FORWARDING)) {
        useServo = 1; 
    }
    // servo tilt、channel forward、mixer需要舵机都会导致useServo为真 240730 jsl

    // give all servos a default command
    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = DEFAULT_SERVO_MIDDLE;
    }

    if (mixerIsTricopter()) {
        servosTricopterInit();
    }

    servoConfigureOutput();
}

void servoMixerLoadMix(int index)
{
    // we're 1-based
    index++;
    // clear existing
    for (int i = 0; i < MAX_SERVO_RULES; i++) {
        customServoMixersMutable(i)->targetChannel = customServoMixersMutable(i)->inputSource = customServoMixersMutable(i)->rate = customServoMixersMutable(i)->box = 0;
    }
    for (int i = 0; i < servoMixers[index].servoRuleCount; i++) {
        *customServoMixersMutable(i) = servoMixers[index].rule[i];
    }
}

STATIC_UNIT_TESTED void forwardAuxChannelsToServos(uint8_t firstServoIndex)
{
    // start forwarding from this channel
    int channelOffset = servoConfig()->channelForwardingStartChannel;
    const int maxAuxChannelCount = MIN(MAX_AUX_CHANNEL_COUNT, rxConfig()->max_aux_channel);
    for (int servoOffset = 0; servoOffset < maxAuxChannelCount && channelOffset < MAX_SUPPORTED_RC_CHANNEL_COUNT; servoOffset++) {
        pwmWriteServo(firstServoIndex + servoOffset, rcData[channelOffset++]);
    }
}

// Write and keep track of written servos

static uint32_t servoWritten;

STATIC_ASSERT(sizeof(servoWritten) * 8 >= MAX_SUPPORTED_SERVOS, servoWritten_is_too_small);

static void writeServoWithTracking(uint8_t index, servoIndex_e servoname)
{
    pwmWriteServo(index, servo[servoname]);
    servoWritten |= (1 << servoname);
}

static void updateGimbalServos(uint8_t firstServoIndex)
{
    writeServoWithTracking(firstServoIndex + 0, SERVO_GIMBAL_PITCH);
    writeServoWithTracking(firstServoIndex + 1, SERVO_GIMBAL_ROLL);
}

static void servoTable(void);
static void filterServos(void);

void writeServos(void)
{
    servoTable(); 
    // 这里调用servoTable()，Table先调用servoMixer()记录控制指令+方向+中值，再由Table限幅, 拿到servo[]数据 240802 jsl
    // 1. 记录控制指令+中值+限幅的servo值 240803 jsl
#ifdef SERVO_LOG_BI
    DEBUG_SET(DEBUG_ANGLE_MODE, 4, servo[SERVO_BICOPTER_LEFT]);
    DEBUG_SET(DEBUG_ANGLE_MODE, 5, servo[SERVO_BICOPTER_RIGHT]);
#endif

    filterServos(); // 对servo[]数据滤波 240802 jsl

    // 2. 这里log的是+滤波的最终servo值 240802 jsl
#ifdef SERVO_LOG_BI
    DEBUG_SET(DEBUG_ANGLE_MODE, 6, servo[SERVO_BICOPTER_LEFT]);
    DEBUG_SET(DEBUG_ANGLE_MODE, 7, servo[SERVO_BICOPTER_RIGHT]);
#endif

    uint8_t servoIndex = 0;
    switch (getMixerMode()) {
    case MIXER_TRI:
    case MIXER_CUSTOM_TRI:
        // We move servo if unarmed flag set or armed
        if (!(servosTricopterIsEnabledServoUnarmed() || ARMING_FLAG(ARMED))) {
            servo[SERVO_RUDDER] = 0; // kill servo signal completely.
        }
        writeServoWithTracking(servoIndex++, SERVO_RUDDER);
        break;

    case MIXER_FLYING_WING:
        writeServoWithTracking(servoIndex++, SERVO_FLAPPERON_1);
        writeServoWithTracking(servoIndex++, SERVO_FLAPPERON_2);
        break;

    case MIXER_CUSTOM_AIRPLANE:
    case MIXER_AIRPLANE:
        for (int i = SERVO_PLANE_INDEX_MIN; i <= SERVO_PLANE_INDEX_MAX; i++) {
            writeServoWithTracking(servoIndex++, i);
        }
        break;

#ifdef USE_UNCOMMON_MIXERS
    case MIXER_BICOPTER:
#ifdef FOLDABLE_WING
        writeServoWithTracking(servoIndex++, SERVO_BICOPTER_SWINGBAT);
        writeServoWithTracking(servoIndex++, SERVO_BICOPTER_FOLDWING);
#endif
        writeServoWithTracking(servoIndex++, SERVO_BICOPTER_LEFT);
        writeServoWithTracking(servoIndex++, SERVO_BICOPTER_RIGHT);
#ifdef CONFIGURATION_TAILSITTER
        writeServoWithTracking(servoIndex++, SERVO_BICOPTER_LEFT_ELEVON);
        writeServoWithTracking(servoIndex++, SERVO_BICOPTER_RIGHT_ELEVON);
#endif
        // 3. 最终写入双旋翼舵机角度指令 servo last 240730 jsl
        break;

    case MIXER_HELI_120_CCPM:
        writeServoWithTracking(servoIndex++, SERVO_HELI_LEFT);
        writeServoWithTracking(servoIndex++, SERVO_HELI_RIGHT);
        writeServoWithTracking(servoIndex++, SERVO_HELI_TOP);
        writeServoWithTracking(servoIndex++, SERVO_HELI_RUD);
        break;

    case MIXER_DUALCOPTER:
        writeServoWithTracking(servoIndex++, SERVO_DUALCOPTER_LEFT);
        writeServoWithTracking(servoIndex++, SERVO_DUALCOPTER_RIGHT);
        break;

    case MIXER_SINGLECOPTER:
        for (int i = SERVO_SINGLECOPTER_INDEX_MIN; i <= SERVO_SINGLECOPTER_INDEX_MAX; i++) {
            writeServoWithTracking(servoIndex++, i);
        }
        break;
#endif // USE_UNCOMMON_MIXERS

    default:
        break;
    }

    // Two servos for SERVO_TILT, if enabled
    if (featureIsEnabled(FEATURE_SERVO_TILT) || getMixerMode() == MIXER_GIMBAL) {
        updateGimbalServos(servoIndex);
        servoIndex += 2;
    }

    // Scan servos and write those marked forwarded and not written yet
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        const uint8_t channelToForwardFrom = servoParams(i)->forwardFromChannel;
        if ((channelToForwardFrom != CHANNEL_FORWARDING_DISABLED) && !(servoWritten & (1 << i))) {
            pwmWriteServo(servoIndex++, servo[i]);
        }
    }

    // forward AUX to remaining servo outputs (not constrained)
    if (featureIsEnabled(FEATURE_CHANNEL_FORWARDING)) {
        forwardAuxChannelsToServos(servoIndex);
        servoIndex += MAX_AUX_CHANNEL_COUNT;
    }
}

void servoMixer(void) 
/* servo.c最关键的函数 把输入分配到舵机上 240730 jsl */ 
{
    int16_t input[INPUT_SOURCE_COUNT]; // Range [-500:+500]
    static int16_t currentOutput[MAX_SERVO_RULES];

    if (FLIGHT_MODE(PASSTHRU_MODE)) {   
        /* 手飞叫Passthrough 注意我们用的不是这个、是angle mode 240730 jsl */
        // Direct passthru from RX
        input[INPUT_STABILIZED_ROLL] = rcCommand[ROLL];
        input[INPUT_STABILIZED_PITCH] = rcCommand[PITCH];
        input[INPUT_STABILIZED_YAW] = rcCommand[YAW];
    } else {
        // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
        input[INPUT_STABILIZED_ROLL] = pidData[FD_ROLL].Sum * PID_SERVO_MIXER_SCALING;
        input[INPUT_STABILIZED_PITCH] = pidData[FD_PITCH].Sum * PID_SERVO_MIXER_SCALING;
        input[INPUT_STABILIZED_YAW] = pidData[FD_YAW].Sum * PID_SERVO_MIXER_SCALING;
        /* 1. 求舵机输入（pidData.Sum in -> scale -> input source out)
           变量：input source共14个，其中RPY3个来自pidData.Sum（角速度环输出）、双旋翼只用PY两个
           问题：舵机最大角度是angleLimit/2、设为180才能达到90度
           思路：增大input可以1解禁pidSum 2放大scale
           解决：没改pidSumLimit、把scale从0.7改为3、丝滑到90度（不带桨）
           240731 jsl */
        /*
           新问题：不上桨一切OK、上桨后舵机在0指令时自激振荡发散
           原因：反扭力矩较大
           思路：加rate：小数值还乘0.7、大数值才乘3
           尝试1：bangbang，会在切换处振荡
           尝试2：线性0.7-3，发现振荡
           240731 jsl */
        // input[INPUT_STABILIZED_PITCH] = ABS(pidData[FD_PITCH].Sum) < PIDSUM_LIMIT * 0.7f? pidData[FD_PITCH].Sum * PID_SERVO_MIXER_SCALING : pidData[FD_PITCH].Sum * 3.0f;
        // input[INPUT_STABILIZED_PITCH] = pidData[FD_PITCH].Sum * (0.7f + 2.3f * ABS(pidData[FD_PITCH].Sum) / PIDSUM_LIMIT);

        // Reverse yaw servo when inverted in 3D mode
        if (featureIsEnabled(FEATURE_3D) && (rcData[THROTTLE] < rxConfig()->midrc)) {
            input[INPUT_STABILIZED_YAW] *= -1;
        }
    }

    input[INPUT_GIMBAL_PITCH] = scaleRange(attitude.values.pitch, -1800, 1800, -500, +500);
    input[INPUT_GIMBAL_ROLL] = scaleRange(attitude.values.roll, -1800, 1800, -500, +500);

    input[INPUT_STABILIZED_THROTTLE] = motor[0] - 1000 - 500;  // Since it derives from rcCommand or mincommand and must be [-500:+500]

    // center the RC input value around the RC middle value
    // by subtracting the RC middle value from the RC input value, we get:
    // data - middle = input
    // 2000 - 1500 = +500
    // 1500 - 1500 = 0
    // 1000 - 1500 = -500
    input[INPUT_RC_ROLL]     = rcData[ROLL]     - rxConfig()->midrc;
    input[INPUT_RC_PITCH]    = rcData[PITCH]    - rxConfig()->midrc;
    input[INPUT_RC_YAW]      = rcData[YAW]      - rxConfig()->midrc;
    input[INPUT_RC_THROTTLE] = rcData[THROTTLE] - rxConfig()->midrc;
    input[INPUT_RC_AUX1]     = rcData[AUX1]     - rxConfig()->midrc;
    input[INPUT_RC_AUX2]     = rcData[AUX2]     - rxConfig()->midrc;
    input[INPUT_RC_AUX3]     = rcData[AUX3]     - rxConfig()->midrc;
    input[INPUT_RC_AUX4]     = rcData[AUX4]     - rxConfig()->midrc;

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = 0;
    }

    // mix servos according to rules
    for (int i = 0; i < servoRuleCount; i++) {
        // consider rule if no box assigned or box is active
        if (currentServoMixer[i].box == 0 || IS_RC_MODE_ACTIVE(BOXSERVO1 + currentServoMixer[i].box - 1)) {
            /* currentServoMixer 是servo rules的数组
               每个元素是一个servo rule、
               每个rule又是个结构体、包含 box, inputSource, targetChannel等
               240731 jsl*/
            uint8_t target = currentServoMixer[i].targetChannel;
            uint8_t from = currentServoMixer[i].inputSource;
            uint16_t servo_width = servoParams(target)->max - servoParams(target)->min;
            int16_t min = currentServoMixer[i].min * servo_width / 100 - servo_width / 2;
            int16_t max = currentServoMixer[i].max * servo_width / 100 - servo_width / 2;

            if (currentServoMixer[i].speed == 0)
                currentOutput[i] = input[from];
                /* 2. 按rule循环、得出rule对应的输出
                input in -> currentOutput out 
                240731 jsl*/
            else {
                if (currentOutput[i] < input[from])
                    currentOutput[i] = constrain(currentOutput[i] + currentServoMixer[i].speed, currentOutput[i], input[from]);
                else if (currentOutput[i] > input[from])
                    currentOutput[i] = constrain(currentOutput[i] - currentServoMixer[i].speed, input[from], currentOutput[i]);
            }

            /* 3. 按rule循环，把rule的输出给target servo
            双旋翼每个舵机有2个rule
            currentOutput in -> * rate * direction -> servo[target] out 
            target指左舵/右舵
            240731 jsl*/
            servo[target] += servoDirection(target, from) * constrain(((int32_t)currentOutput[i] * currentServoMixer[i].rate) / 100, min, max);
            // position_msp.msg2 = servo[SERVO_BICOPTER_LEFT]*100.0f;
            // position_msp.msg3 = servo[SERVO_BICOPTER_RIGHT]*100.0f;
            
            /*
            计划：直接在servo上加前馈，使它达到90度、不影响pid
            给400就很接近90度了
            240802 jsl
            */
            //servo[target] -= 500;

            /*
            记录log servo值备查
            发现中点是600而不是1500
            这里记录的是未加中值、未滤波的原始指令
            240802 jsl
            */
            // if (target == SERVO_BICOPTER_LEFT){
            //     DEBUG_SET(DEBUG_ANGLE_MODE, 4, servo[target]);
            // }
            // else if(target == SERVO_BICOPTER_RIGHT){
            //     DEBUG_SET(DEBUG_ANGLE_MODE, 5, servo[target]);
            // }
            


        } else {
            currentOutput[i] = 0;
        }
    }
    /*
    logdebug 1. 记录未加中值、未滤波的原始指令 (700 800) 240802 jsl
    */
#ifdef SERVO_LOG_DEBUG_LEFT
    DEBUG_SET(DEBUG_ANGLE_MODE, 4, servo[SERVO_BICOPTER_LEFT]);
#endif
#ifdef SERVO_LOG_DEBUG_RIGHT
    DEBUG_SET(DEBUG_ANGLE_MODE, 4, servo[SERVO_BICOPTER_RIGHT]);
#endif

    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = ((int32_t)servoParams(i)->rate * servo[i]) / 100L; // 有个放大0-1.25倍，但一般就是1倍，240802 jsl

        /*logdebug 2. 记录有方向的指令*/
#ifdef SERVO_LOG_DEBUG_LEFT
        if (i == SERVO_BICOPTER_LEFT){
            DEBUG_SET(DEBUG_ANGLE_MODE, 5, servo[i]);
        }
#endif
#ifdef SERVO_LOG_DEBUG_RIGHT
        if (i == SERVO_BICOPTER_RIGHT){
            DEBUG_SET(DEBUG_ANGLE_MODE, 5, servo[i]);
        }
#endif

        /*logdebug 3. 记录中值*/
#ifdef SERVO_LOG_DEBUG_LEFT
        if (i == SERVO_BICOPTER_LEFT){
            DEBUG_SET(DEBUG_ANGLE_MODE, 6, determineServoMiddleOrForwardFromChannel(i));
        }
#endif
#ifdef SERVO_LOG_DEBUG_RIGHT
        if (i == SERVO_BICOPTER_RIGHT){
            // 确认了debug是16位、数据都不用缩小 240803 jsl
            DEBUG_SET(DEBUG_ANGLE_MODE, 6, determineServoMiddleOrForwardFromChannel(i));
        }
#endif

        servo[i] += determineServoMiddleOrForwardFromChannel(i);

        /*logdebug 4. 记录加中值后的指令*/
#ifdef SERVO_LOG_DEBUG_LEFT
        if (i == SERVO_BICOPTER_LEFT){
            DEBUG_SET(DEBUG_ANGLE_MODE, 7, servo[i]);
        }
#endif
#ifdef SERVO_LOG_DEBUG_RIGHT
        if (i == SERVO_BICOPTER_RIGHT){
            DEBUG_SET(DEBUG_ANGLE_MODE, 7, servo[i]);
        }
#endif
    }
}


static void servoTable(void)
{
    // airplane / servo mixes
    switch (getMixerMode()) {
    case MIXER_CUSTOM_TRI:
    case MIXER_TRI:
        servosTricopterMixer();
        break;
    case MIXER_CUSTOM_AIRPLANE:
    case MIXER_FLYING_WING:
    case MIXER_AIRPLANE:
    case MIXER_BICOPTER:
    case MIXER_DUALCOPTER:
    case MIXER_SINGLECOPTER:
    case MIXER_HELI_120_CCPM:
    case MIXER_GIMBAL:
        servoMixer();
        break;
    /*
    servoTable函数判断机型，如果是三旋翼用xx、其他都用servoMixer()
    240802 jsl
    */

    /*
    case MIXER_GIMBAL:
        servo[SERVO_GIMBAL_PITCH] = (((int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate * attitude.values.pitch) / 50) + determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_PITCH);
        servo[SERVO_GIMBAL_ROLL] = (((int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll) / 50) + determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_ROLL);
        break;
    */

    default:
        break;
    }

    // camera stabilization 没操作双旋翼、无关 240803 jsl
    if (featureIsEnabled(FEATURE_SERVO_TILT)) {
        // center at fixed position, or vary either pitch or roll by RC channel
        servo[SERVO_GIMBAL_PITCH] = determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_PITCH);
        servo[SERVO_GIMBAL_ROLL] = determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_ROLL);

        if (IS_RC_MODE_ACTIVE(BOXCAMSTAB)) {
            if (gimbalConfig()->mode == GIMBAL_MODE_MIXTILT) {
                servo[SERVO_GIMBAL_PITCH] -= (-(int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate) * attitude.values.pitch / 50 - (int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll / 50;
                servo[SERVO_GIMBAL_ROLL] += (-(int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate) * attitude.values.pitch / 50 + (int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll / 50;
            } else {
                servo[SERVO_GIMBAL_PITCH] += (int32_t)servoParams(SERVO_GIMBAL_PITCH)->rate * attitude.values.pitch / 50;
                servo[SERVO_GIMBAL_ROLL] += (int32_t)servoParams(SERVO_GIMBAL_ROLL)->rate * attitude.values.roll  / 50;
            }
        }
    }

    // constrain servos 做了舵机限幅、和双旋翼有关
    /*logdebug 5. 限幅后的指令  240803 jsl*/
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = constrain(servo[i], servoParams(i)->min, servoParams(i)->max); // limit the values
    }
}

bool isMixerUsingServos(void)
{
    return useServo;
}

static biquadFilter_t servoFilter[MAX_SUPPORTED_SERVOS];

void servosFilterInit(void)
{
    if (servoConfig()->servo_lowpass_freq) {
        for (int servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            biquadFilterInitLPF(&servoFilter[servoIdx], servoConfig()->servo_lowpass_freq, targetPidLooptime);
        }
    }

}
static void filterServos(void)
{
#if defined(MIXER_DEBUG)
    uint32_t startTime = micros();
#endif
    if (servoConfig()->servo_lowpass_freq) {
        for (int servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            servo[servoIdx] = lrintf(biquadFilterApply(&servoFilter[servoIdx], (float)servo[servoIdx]));
            // 默认没用滤波，可以直接用这个做预测 240803 jsl
            // Sanity check
            servo[servoIdx] = constrain(servo[servoIdx], servoParams(servoIdx)->min, servoParams(servoIdx)->max);
        }
    }
#if defined(MIXER_DEBUG)
    debug[0] = (int16_t)(micros() - startTime);
#endif
}
#endif // USE_SERVOS
