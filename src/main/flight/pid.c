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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h> //20250424 for FLT_EPSILON in quat2eulZXY

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/config_reset.h"
#include "config/simplified_tuning.h"

#include "drivers/pwm_output.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/controlrate_profile.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/rpm_filter.h"
#include "flight/position.h"

#include "io/gps.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "pid.h"

typedef enum {
    LEVEL_MODE_OFF = 0,
    LEVEL_MODE_R,
    LEVEL_MODE_RP,
} levelMode_e;

const char pidNames[] =
    "ROLL;"
    "PITCH;"
    "YAW;"
    "LEVEL;"
    "MAG;";

FAST_DATA_ZERO_INIT uint32_t targetPidLooptime;
FAST_DATA_ZERO_INIT pidAxisData_t pidData[XYZ_AXIS_COUNT];
FAST_DATA_ZERO_INIT pidRuntime_t pidRuntime;

#ifdef QUATERNION_CONTROL
    float angularRateDesired[XYZ_AXIS_COUNT];
    float Yaw_desire;
    bool arming_state=false, previous_arming_state=false;
#endif

#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED FAST_DATA_ZERO_INIT float axisError[XYZ_AXIS_COUNT];
#endif

#if defined(USE_THROTTLE_BOOST)
FAST_DATA_ZERO_INIT float throttleBoost;
pt1Filter_t throttleLpf;
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 3);

#ifndef DEFAULT_PID_PROCESS_DENOM
#define DEFAULT_PID_PROCESS_DENOM       1
#endif

#ifdef USE_RUNAWAY_TAKEOFF
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = DEFAULT_PID_PROCESS_DENOM,
    .runaway_takeoff_prevention = true,
    .runaway_takeoff_deactivate_throttle = 20,  // throttle level % needed to accumulate deactivation time
    .runaway_takeoff_deactivate_delay = 500,    // Accumulated time (in milliseconds) before deactivation in successful takeoff
);
#else
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = DEFAULT_PID_PROCESS_DENOM,
);
#endif

#ifdef USE_ACRO_TRAINER
#define ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT 500.0f  // Max gyro rate for lookahead time scaling
#define ACRO_TRAINER_SETPOINT_LIMIT       1000.0f // Limit the correcting setpoint
#endif // USE_ACRO_TRAINER

#ifdef ROTORDISK_FEEDBACK
// JJJJJJJack 20240822 sim servo with controller
float servoAngleFeedback;
float servoAngularRateFeedback;
float servoRateIntegral;
float rotorDiskAngleFeedback;
#define motorInertia 6e-5f
float PitchTarget;
#ifdef CONFIGURATION_QUADTILT
float PitchTarget_rad;
float bodyPitchTarget;
#endif
float servo2RotorDiskMap_K = 1;
float servo2RotorDiskMap_B = 0.0;
#ifdef CONFIGURATION_TAILSITTER
// For tailsitter (with linkage)
float servoPWMRange = 1000.0f;
float servoAngleRange = 180.0f;
#elif defined BI_DOCK
// For bicopter docking (servo driving motor directly) 270 deg servo
float servoPWMRange = 1000.0f;
float servoAngleRange = 270.0f;
#else
// For miniBi (servo driving motor directly)
float servoPWMRange = 2000.0f;
float servoAngleRange = 180.0f;
#endif

// JJJJJJJack& Jsl 20240910 sim a lowpass filter
float LP_A = 50, LP_last_out = 0, LP_last_in = 0;
float servoLastDesiredAngle = 0.0;
float diskDiffRate = 0.0, diskAngularRate = 0.0;
#endif

#define CRASH_RECOVERY_DETECTION_DELAY_US 1000000  // 1 second delay before crash recovery detection is active after entering a self-level mode

#define LAUNCH_CONTROL_YAW_ITERM_LIMIT 50 // yaw iterm windup limit when launch mode is "FULL" (all axes)

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 8);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .pid = {
            [PID_ROLL] =  PID_ROLL_DEFAULT,
            [PID_PITCH] = PID_PITCH_DEFAULT,
            [PID_YAW] =   PID_YAW_DEFAULT,
            [PID_LEVEL] = { 50, 75, 75, 50 },
            [PID_MAG] =   { 40, 0, 0, 0 },
        },
        .pidSumLimit = PIDSUM_LIMIT,
        .pidSumLimitYaw = PIDSUM_LIMIT_YAW,
        .yaw_lowpass_hz = 100,
        .dterm_notch_hz = 0,
        .dterm_notch_cutoff = 0,
        .itermWindupPointPercent = 85,
        .pidAtMinThrottle = PID_STABILISATION_ON,
        .angle_limit = 60,
        .feedforward_transition = 0,
        .yawRateAccelLimit = 0,
        .rateAccelLimit = 0,
        .anti_gravity_gain = 80,
        .crash_time = 500,          // ms
        .crash_delay = 0,           // ms
        .crash_recovery_angle = 10, // degrees
        .crash_recovery_rate = 100, // degrees/second
        .crash_dthreshold = 50,     // degrees/second/second
        .crash_gthreshold = 400,    // degrees/second
        .crash_setpoint_threshold = 350, // degrees/second
        .crash_recovery = PID_CRASH_RECOVERY_OFF, // off by default
        .horizon_limit_degrees = 135,
        .horizon_ignore_sticks = false,
        .crash_limit_yaw = 200,
        .itermLimit = 400,
        .throttle_boost = 5,
        .throttle_boost_cutoff = 15,
        .iterm_rotation = false,
        .iterm_relax = ITERM_RELAX_RP,
        .iterm_relax_cutoff = ITERM_RELAX_CUTOFF_DEFAULT,
        .iterm_relax_type = ITERM_RELAX_SETPOINT,
        .acro_trainer_angle_limit = 20,
        .acro_trainer_lookahead_ms = 50,
        .acro_trainer_debug_axis = FD_ROLL,
        .acro_trainer_gain = 75,
        .abs_control_gain = 0,
        .abs_control_limit = 90,
        .abs_control_error_limit = 20,
        .abs_control_cutoff = 11,
        .dterm_lpf1_static_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
            // NOTE: dynamic lpf is enabled by default so this setting is actually
            // overridden and the static lowpass 1 is disabled. We can't set this
            // value to 0 otherwise Configurator versions 10.4 and earlier will also
            // reset the lowpass filter type to PT1 overriding the desired BIQUAD setting.
        .dterm_lpf2_static_hz = DTERM_LPF2_HZ_DEFAULT,   // second Dterm LPF ON by default
        .dterm_lpf1_type = FILTER_PT1,
        .dterm_lpf2_type = FILTER_PT1,
        .dterm_lpf1_dyn_min_hz = DTERM_LPF1_DYN_MIN_HZ_DEFAULT,
        .dterm_lpf1_dyn_max_hz = DTERM_LPF1_DYN_MAX_HZ_DEFAULT,
        .launchControlMode = LAUNCH_CONTROL_MODE_NORMAL,
        .launchControlThrottlePercent = 20,
        .launchControlAngleLimit = 0,
        .launchControlGain = 40,
        .launchControlAllowTriggerReset = true,
        .use_integrated_yaw = false,
        .integrated_yaw_relax = 200,
        .thrustLinearization = 0,
        .d_min = D_MIN_DEFAULT,
        .d_min_gain = 37,
        .d_min_advance = 20,
        .motor_output_limit = 100,
        .auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY,
        .transient_throttle_limit = 0,
        .profileName = { 0 },
        .dyn_idle_min_rpm = 0,
        .dyn_idle_p_gain = 50,
        .dyn_idle_i_gain = 50,
        .dyn_idle_d_gain = 50,
        .dyn_idle_max_increase = 150,
        .dyn_idle_start_increase = 50,
        .feedforward_averaging = FEEDFORWARD_AVERAGING_OFF,
        .feedforward_max_rate_limit = 90,
        .feedforward_smooth_factor = 25,
        .feedforward_jitter_factor = 7,
        .feedforward_boost = 15,
        .dterm_lpf1_dyn_expo = 5,
        .level_race_mode = false,
        .vbat_sag_compensation = 0,
        .simplified_pids_mode = PID_SIMPLIFIED_TUNING_RPY,
        .simplified_master_multiplier = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_roll_pitch_ratio = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_i_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_d_gain = SIMPLIFIED_TUNING_D_DEFAULT,
        .simplified_pi_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_dmin_ratio = SIMPLIFIED_TUNING_D_DEFAULT,
        .simplified_feedforward_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_pitch_pi_gain = SIMPLIFIED_TUNING_DEFAULT,
        .simplified_dterm_filter = true,
        .simplified_dterm_filter_multiplier = SIMPLIFIED_TUNING_DEFAULT,
        .anti_gravity_cutoff_hz = 5,
        .anti_gravity_p_gain = 100,
        .tpa_mode = TPA_MODE_D,
        .tpa_rate = 65,
        .tpa_breakpoint = 1350,
        .angle_feedforward_smoothing_ms = 80,
        .angle_earth_ref = 100,
        .horizon_delay_ms = 500, // 500ms time constant on any increase in horizon strength
        .tpa_low_rate = 20,
        .tpa_low_breakpoint = 1050,
        .tpa_low_always = 0,
        .ez_landing_threshold = 25,
        .ez_landing_limit = 15,
        .ez_landing_speed = 50,
    );

#ifndef USE_D_MIN
    pidProfile->pid[PID_ROLL].D = 30;
    pidProfile->pid[PID_PITCH].D = 32;
#endif
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

// Scale factors to make best use of range with D_LPF debugging, aiming for max +/-16K as debug values are 16 bit
#define D_LPF_RAW_SCALE 25
#define D_LPF_PRE_TPA_SCALE 10


void pidSetItermAccelerator(float newItermAccelerator)
{
    pidRuntime.itermAccelerator = newItermAccelerator;
}

bool pidOsdAntiGravityActive(void)
{
    return (pidRuntime.itermAccelerator > pidRuntime.antiGravityOsdCutoff);
}

void pidStabilisationState(pidStabilisationState_e pidControllerState)
{
    pidRuntime.pidStabilisationEnabled = (pidControllerState == PID_STABILISATION_ON) ? true : false;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

void pidResetIterm(void)
{
    for (int axis = 0; axis < 3; axis++) {
        //if(!(mixerConfig()->mixerMode == MIXER_BICOPTER && axis == FD_PITCH))
        pidData[axis].I = 0.0f;
#if defined(USE_ABSOLUTE_CONTROL)
        axisError[axis] = 0.0f;
#endif
    }
}

void pidUpdateTpaFactor(float throttle)
{
    static bool isTpaLowFaded = false;
    // don't permit throttle > 1 & throttle < 0 ? is this needed ? can throttle be > 1 or < 0 at this point
    throttle = constrainf(throttle, 0.0f, 1.0f);
    bool isThrottlePastTpaLowBreakpoint = (throttle < pidRuntime.tpaLowBreakpoint && pidRuntime.tpaLowBreakpoint > 0.01f) ? false : true;
    float tpaRate = 0.0f;
    if (isThrottlePastTpaLowBreakpoint || isTpaLowFaded) {
        tpaRate = pidRuntime.tpaMultiplier * fmaxf(throttle - pidRuntime.tpaBreakpoint, 0.0f);
        if (!pidRuntime.tpaLowAlways && !isTpaLowFaded) {
            isTpaLowFaded = true;
        }
    } else {
        tpaRate = pidRuntime.tpaLowMultiplier * (pidRuntime.tpaLowBreakpoint - throttle);
    }
    pidRuntime.tpaFactor = 1.0f - tpaRate;
}

void pidUpdateAntiGravityThrottleFilter(float throttle)
{
    static float previousThrottle = 0.0f;
    const float throttleInv = 1.0f - throttle;
    float throttleDerivative = fabsf(throttle - previousThrottle) * pidRuntime.pidFrequency;
    DEBUG_SET(DEBUG_ANTI_GRAVITY, 0, lrintf(throttleDerivative * 100));
    throttleDerivative *= throttleInv * throttleInv;
    // generally focus on the low throttle period
    if (throttle > previousThrottle) {
        throttleDerivative *= throttleInv * 0.5f;
        // when increasing throttle, focus even more on the low throttle range
    }
    previousThrottle = throttle;
    throttleDerivative = pt2FilterApply(&pidRuntime.antiGravityLpf, throttleDerivative);
    // lower cutoff suppresses peaks relative to troughs and prolongs the effects
    // PT2 smoothing of throttle derivative.
    // 6 is a typical value for the peak boost factor with default cutoff of 6Hz
    DEBUG_SET(DEBUG_ANTI_GRAVITY, 1, lrintf(throttleDerivative * 100));
    pidRuntime.antiGravityThrottleD = throttleDerivative;
}

#ifdef USE_ACRO_TRAINER
void pidAcroTrainerInit(void)
{
    pidRuntime.acroTrainerAxisState[FD_ROLL] = 0;
    pidRuntime.acroTrainerAxisState[FD_PITCH] = 0;
}
#endif // USE_ACRO_TRAINER

#ifdef USE_THRUST_LINEARIZATION
float pidCompensateThrustLinearization(float throttle)
{
    if (pidRuntime.thrustLinearization != 0.0f) {
        // for whoops where a lot of TL is needed, allow more throttle boost
        const float throttleReversed = (1.0f - throttle);
        throttle /= 1.0f + pidRuntime.throttleCompensateAmount * sq(throttleReversed);
    }
    return throttle;
}

float pidApplyThrustLinearization(float motorOutput)
{
    motorOutput *= 1.0f + pidRuntime.thrustLinearization * sq(1.0f - motorOutput);
    return motorOutput;
}
#endif

#if defined(USE_ACC)
// Calculate strength of horizon leveling; 0 = none, 1.0 = most leveling
STATIC_UNIT_TESTED FAST_CODE_NOINLINE float calcHorizonLevelStrength(void)
{
    const float currentInclination = MAX(abs(attitude.values.roll), abs(attitude.values.pitch)) * 0.1f;
    // 0 when level, 90 when vertical, 180 when inverted (degrees):
    float absMaxStickDeflection = MAX(fabsf(getRcDeflection(FD_ROLL)), fabsf(getRcDeflection(FD_PITCH)));
    // 0-1, smoothed if RC smoothing is enabled

    float horizonLevelStrength = MAX((pidRuntime.horizonLimitDegrees - currentInclination) * pidRuntime.horizonLimitDegreesInv, 0.0f);
    // 1.0 when attitude is 'flat', 0 when angle is equal to, or greater than, horizonLimitDegrees
    horizonLevelStrength *= MAX((pidRuntime.horizonLimitSticks - absMaxStickDeflection) * pidRuntime.horizonLimitSticksInv, pidRuntime.horizonIgnoreSticks);
    // use the value of horizonIgnoreSticks to enable/disable this effect.
    // value should be 1.0 at center stick, 0.0 at max stick deflection:
    horizonLevelStrength *= pidRuntime.horizonGain;

    if (pidRuntime.horizonDelayMs) {
        const float horizonLevelStrengthSmoothed = pt1FilterApply(&pidRuntime.horizonSmoothingPt1, horizonLevelStrength);
        horizonLevelStrength = MIN(horizonLevelStrength, horizonLevelStrengthSmoothed);
    }
    return horizonLevelStrength;
    // 1 means full levelling, 0 means none
}

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
STATIC_UNIT_TESTED FAST_CODE_NOINLINE float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim,
                                                        float currentPidSetpoint, float horizonLevelStrength)
{
    // Applies only to axes that are in Angle mode
    // We now use Acro Rates, transformed into the range +/- 1, to provide setpoints
    const float angleLimit = pidProfile->angle_limit;
    float angleFeedforward = 0.0f;

#ifdef USE_FEEDFORWARD
    angleFeedforward = angleLimit * getFeedforward(axis) * pidRuntime.angleFeedforwardGain * pidRuntime.maxRcRateInv[axis];
    //  angle feedforward must be heavily filtered, at the PID loop rate, with limited user control over time constant
    // it MUST be very delayed to avoid early overshoot and being too aggressive
    angleFeedforward = pt3FilterApply(&pidRuntime.angleFeedforwardPt3[axis], angleFeedforward);
#endif

    // // 记录进来的sp，240728 jsl
    // if (axis == FD_PITCH){
    //     DEBUG_SET(DEBUG_ANGLE_MODE, 7, currentPidSetpoint); 
    // }

    // 240728 jsl, 1 line no setpoint
    float angleTarget = angleLimit * currentPidSetpoint * pidRuntime.maxRcRateInv[axis];

    //float angleTarget = angleLimit * currentPidSetpoint * pidRuntime.maxRcRateInv[axis];
    // use acro rates for the angle target in both horizon and angle modes, converted to -1 to +1 range using maxRate

#ifdef USE_GPS_RESCUE
    angleTarget += gpsRescueAngle[axis] / 100.0f; // Angle is in centidegrees, stepped on roll at 10Hz but not on pitch
#endif
    float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f; // stepped at 500hz with some 4ms flat spots

#ifdef ROTORDISK_FEEDBACK
    // Save the pitch target direct feedforward
    if(axis == FD_PITCH){
        PitchTarget = currentAngle - angleTarget;
#ifdef CONFIGURATION_QUADTILT
        PitchTarget_rad = DEGREES_TO_RADIANS(PitchTarget);
#endif
    }
#endif

#ifdef ROTORDISK_FEEDBACK
    // Mapping servo angle to motor pitch angle
    rotorDiskAngleFeedback = servoAngleFeedback * servo2RotorDiskMap_K + servo2RotorDiskMap_B;
    position_msp.msg5 = rotorDiskAngleFeedback * 100.0f;
    // Enable rotor disk angle feedback, only in pitch axis for bicopter
    if(mixerConfig()->mixerMode == MIXER_BICOPTER && axis == FD_PITCH)
        currentAngle += rotorDiskAngleFeedback;
#endif

#ifdef CONFIGURATION_QUADTILT
    if(axis == FD_PITCH)
        angleTarget = -bodyPitchTarget;
#endif
    const float errorAngle = angleTarget - currentAngle;
    // 角度环=反馈+前馈  240730 jsl
    float angleRate = errorAngle * pidRuntime.angleGain; //+ angleFeedforward; 240728 jsl
    // if(axis == FD_PITCH)
    //     position_msp.msg5 =  currentAngle*100.0f;
    // minimise cross-axis wobble due to faster yaw responses than roll or pitch, and make co-ordinated yaw turns
    // by compensating for the effect of yaw on roll while pitched, and on pitch while rolled
    // earthRef code here takes about 76 cycles, if conditional on angleEarthRef it takes about 100.  sin_approx costs most of those cycles.
    // 240728 jsl shut down earth ref, comment 5 lines
    //float sinAngle = sin_approx(DEGREES_TO_RADIANS(pidRuntime.angleTarget[axis == FD_ROLL ? FD_PITCH : FD_ROLL]));
    //sinAngle *= (axis == FD_ROLL) ? -1.0f : 1.0f; // must be negative for Roll
    //const float earthRefGain = FLIGHT_MODE(GPS_RESCUE_MODE) ? 1.0f : pidRuntime.angleEarthRef;
    //angleRate += pidRuntime.angleYawSetpoint * sinAngle * earthRefGain;
    //pidRuntime.angleTarget[axis] = angleTarget;  // set target for alternate axis to current axis, for use in preceding calculation

    // smooth final angle rate output to clean up attitude signal steps (500hz), GPS steps (10 or 100hz), RC steps etc
    // this filter runs at ATTITUDE_CUTOFF_HZ, currently 50hz, so GPS roll may be a bit steppy
    // 240728 jsl comment filter 
    // angleRate = pt3FilterApply(&pidRuntime.attitudeFilter[axis], angleRate);

    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) {
        currentPidSetpoint = angleRate;
    } else {
        // can only be HORIZON mode - crossfade Angle rate and Acro rate
        currentPidSetpoint = currentPidSetpoint * (1.0f - horizonLevelStrength) + angleRate * horizonLevelStrength;
    }
    // add debug 240728 jsl
    if (axis == FD_PITCH){
        DEBUG_SET(DEBUG_ANGLE_MODE, 0, angleTarget * 10.0f);//前4位x10在bfl中看起来是真值
        DEBUG_SET(DEBUG_ANGLE_MODE, 1, currentAngle * 10.0f);
        DEBUG_SET(DEBUG_ANGLE_MODE, 2, currentPidSetpoint * 10.0f);
        DEBUG_SET(DEBUG_ANGLE_MODE, 3, angleRate * 10.0f);
        //DEBUG_SET(DEBUG_ANGLE_MODE, 4, currentPidSetpoint); // 后4位不乘在bfl中看起来是真值
        //DEBUG_SET(DEBUG_ANGLE_MODE, 4, getSetpointRate(axis)); 
        //DEBUG_SET(DEBUG_ANGLE_MODE, 5, getMaxRcRate(axis));
    }
    //logging
    if (axis == FD_ROLL) {
        // comment 4 lines, replacing debug angle mode 240728 jsl
        //DEBUG_SET(DEBUG_ANGLE_MODE, 0, lrintf(angleTarget * 10.0f)); // target angle
        //DEBUG_SET(DEBUG_ANGLE_MODE, 1, lrintf(errorAngle * pidRuntime.angleGain * 10.0f)); // un-smoothed error correction in degrees
        //DEBUG_SET(DEBUG_ANGLE_MODE, 2, lrintf(angleFeedforward * 10.0f)); // feedforward amount in degrees
        //DEBUG_SET(DEBUG_ANGLE_MODE, 3, lrintf(currentAngle * 10.0f)); // angle returned

        DEBUG_SET(DEBUG_ANGLE_TARGET, 0, lrintf(angleTarget * 10.0f));
        // 240728 jsl comment 1 line
        //DEBUG_SET(DEBUG_ANGLE_TARGET, 1, lrintf(sinAngle * 10.0f)); // modification factor from earthRef
        // debug ANGLE_TARGET 2 is yaw attenuation
        DEBUG_SET(DEBUG_ANGLE_TARGET, 3, lrintf(currentAngle * 10.0f)); // angle returned
    }

    DEBUG_SET(DEBUG_CURRENT_ANGLE, axis, lrintf(currentAngle * 10.0f)); // current angle

    
    #ifdef QUATERNION_CONTROL
        return angularRateDesired[axis];
    #endif

    return currentPidSetpoint;
}

// Quaternion control added by JJJJJJJack
// Date created: 12/10/2022
#ifdef QUATERNION_CONTROL
void angularRateFromQuaternionError(const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim){
    float quat_des[4], quat_ang[4], quat_ang_inv[4], quat_diff[4], axisAngle[4];
    float eulerAngleYRP[3], temp[4];
    quaternion q_ang = QUATERNION_INITIALIZE;
    arming_state = ARMING_FLAG(ARMED);
    calc_psi_des(getRcDeflection(FD_YAW), DEGREES_TO_RADIANS(-attitude.raw[2]/10.0f), arming_state, previous_arming_state, &Yaw_desire);
    previous_arming_state = arming_state;
    /*
    eulerAngleYPR[0] = conv2std(Yaw_desire);
    eulerAngleYPR[1] = (pidProfile->levelAngleLimit * getRcDeflection(FD_PITCH))/57.3f;
    eulerAngleYPR[2] = (pidProfile->levelAngleLimit * getRcDeflection(FD_ROLL))/57.3f;
    */
   
    eulerAngleYRP[0] = conv2std(Yaw_desire);
    eulerAngleYRP[1] = (pidProfile->angle_limit * getRcDeflection(FD_ROLL))/57.3f; //levelAngleLimit -> angle_limit 
    eulerAngleYRP[2] = (pidProfile->angle_limit * getRcDeflection(FD_PITCH))/57.3f; // 55 -> 60, may be official update
    
    float currentPitch = (attitude.raw[FD_PITCH] - angleTrim->raw[FD_PITCH]) / 10.0f; // stepped at 500hz with some 4ms flat spots
    float joystickPitchTarget = RADIANS_TO_DEGREES(eulerAngleYRP[2]);
    #ifdef ROTORDISK_FEEDBACK
    // Save the pitch target direct feedforward
    PitchTarget = currentPitch - joystickPitchTarget;
    #ifdef CONFIGURATION_QUADTILT
    PitchTarget_rad = DEGREES_TO_RADIANS(PitchTarget);
    #endif
    #endif



    #ifdef CONFIGURATION_QUADTILT
    eulerAngleYRP[2] = DEGREES_TO_RADIANS(-bodyPitchTarget);
    #endif
    //eulerAngleYPR[2] = attitudeUpright() ? eulerAngleYPR[2] : 0 - eulerAngleYPR[2];
    //eul2quatZYX(eulerAngleYPR, quat_des_RC);

    //eul2quatZYX(eulerAngleYPR, quat_des);
    eul2quatZXY(eulerAngleYRP, quat_des);   //20250424 ZXY 
    
    quaternionNormalize(quat_des, quat_des);
    getQuaternion(&q_ang);
    quat_ang[0] = q_ang.w; quat_ang[1] = q_ang.x; quat_ang[2] = q_ang.y; quat_ang[3] = q_ang.z;

    #ifdef ROTORDISK_FEEDBACK
        // Mapping servo angle to motor pitch angle
        rotorDiskAngleFeedback = servoAngleFeedback * servo2RotorDiskMap_K + servo2RotorDiskMap_B;
        float rotorDiskAngleFeedback_Rad = DEGREES_TO_RADIANS(rotorDiskAngleFeedback); 
        // position_msp.msg5 = rotorDiskAngleFeedback * 100.0f;
        // Enable rotor disk angle feedback, only in pitch axis for bicopter
        if(mixerConfig()->mixerMode == MIXER_BICOPTER) {
            // Get rotor disk quaternion from body quaternion and tilt angle
            float q_disk_tilt[4] = {cos(0.5f*rotorDiskAngleFeedback_Rad), 0, sin(0.5f*rotorDiskAngleFeedback_Rad), 0};
            quaternionMultiply(quat_ang, q_disk_tilt, temp);
            memcpy(quat_ang, temp, sizeof(temp) );
            //q_ang = q_ang * q_disk_tilt;
        }
    #endif
    

    // Logging quaternion through blackbox debug
    // Date created 05/17/2023
    debug[0] = q_ang.w*1e4f;
    debug[1] = q_ang.x*1e4f;
    debug[2] = q_ang.y*1e4f;
    debug[3] = q_ang.z*1e4f;
    //position_msp.msg3 = quat_des[0]*100.0f;
    //position_msp.msg4 = quat_des[1]*100.0f;
    //position_msp.msg5 = quat_des[2]*100.0f;
    //position_msp.msg6 = quat_des[3]*100.0f;
    //position_msp.msg3 = q_ang.w*100.0f;
    //position_msp.msg4 = q_ang.x*100.0f;
    //position_msp.msg5 = q_ang.y*100.0f;
    //position_msp.msg6 = q_ang.z*100.0f;

    quaternionInverse(quat_ang, quat_ang_inv);
    quaternionMultiply(quat_ang_inv, quat_des, quat_diff);

    // Get body quaternion from rotor disk quaternion, inverse rotation, 20250528 lwj
    float q_disk_tilt_inverse[4] = {cos(-0.5f*rotorDiskAngleFeedback_Rad), 0, sin(-0.5f*rotorDiskAngleFeedback_Rad), 0};
    quaternionMultiply(quat_diff, q_disk_tilt_inverse, temp);
    memcpy(quat_diff, temp, sizeof(temp));

    quaternionToAxisAngle(quat_diff, axisAngle);


    angularRateDesired[FD_ROLL] = RADIANS_TO_DEGREES(axisAngle[0] * axisAngle[3])* pidRuntime.angleGain;

    angularRateDesired[FD_PITCH] = RADIANS_TO_DEGREES(axisAngle[1] * axisAngle[3]) * pidRuntime.angleGain;

    //Reduce the yaw gain for bicopter
    angularRateDesired[FD_YAW] = RADIANS_TO_DEGREES(axisAngle[2] * axisAngle[3]) * 1.2f;

}
#endif


static FAST_CODE_NOINLINE void handleCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const rollAndPitchTrims_t *angleTrim,
    const int axis, const timeUs_t currentTimeUs, const float gyroRate, float *currentPidSetpoint, float *errorRate)
{
    if (pidRuntime.inCrashRecoveryMode && cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) > pidRuntime.crashTimeDelayUs) {
        if (crash_recovery == PID_CRASH_RECOVERY_BEEP) {
            BEEP_ON;
        }
        if (axis == FD_YAW) {
            *errorRate = constrainf(*errorRate, -pidRuntime.crashLimitYaw, pidRuntime.crashLimitYaw);
        } else {
            // on roll and pitch axes calculate currentPidSetpoint and errorRate to level the aircraft to recover from crash
            if (sensors(SENSOR_ACC)) {
                // errorAngle is deviation from horizontal
                const float errorAngle =  -(attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
                *currentPidSetpoint = errorAngle * pidRuntime.angleGain;
                *errorRate = *currentPidSetpoint - gyroRate;
            }
        }
        // reset iterm, since accumulated error before crash is now meaningless
        // and iterm windup during crash recovery can be extreme, especially on yaw axis
        pidData[axis].I = 0.0f;
        if (cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) > pidRuntime.crashTimeLimitUs
            || (getMotorMixRange() < 1.0f
                   && fabsf(gyro.gyroADCf[FD_ROLL]) < pidRuntime.crashRecoveryRate
                   && fabsf(gyro.gyroADCf[FD_PITCH]) < pidRuntime.crashRecoveryRate
                   && fabsf(gyro.gyroADCf[FD_YAW]) < pidRuntime.crashRecoveryRate)) {
            if (sensors(SENSOR_ACC)) {
                // check aircraft nearly level
                if (abs(attitude.raw[FD_ROLL] - angleTrim->raw[FD_ROLL]) < pidRuntime.crashRecoveryAngleDeciDegrees
                   && abs(attitude.raw[FD_PITCH] - angleTrim->raw[FD_PITCH]) < pidRuntime.crashRecoveryAngleDeciDegrees) {
                    pidRuntime.inCrashRecoveryMode = false;
                    BEEP_OFF;
                }
            } else {
                pidRuntime.inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        }
    }
}

static FAST_CODE_NOINLINE void detectAndSetCrashRecovery(
    const pidCrashRecovery_e crash_recovery, const int axis,
    const timeUs_t currentTimeUs, const float delta, const float errorRate)
{
    // if crash recovery is on and accelerometer enabled and there is no gyro overflow, then check for a crash
    // no point in trying to recover if the crash is so severe that the gyro overflows
    if ((crash_recovery || FLIGHT_MODE(GPS_RESCUE_MODE)) && !gyroOverflowDetected()) {
        if (ARMING_FLAG(ARMED)) {
            if (getMotorMixRange() >= 1.0f && !pidRuntime.inCrashRecoveryMode
                && fabsf(delta) > pidRuntime.crashDtermThreshold
                && fabsf(errorRate) > pidRuntime.crashGyroThreshold
                && fabsf(getSetpointRate(axis)) < pidRuntime.crashSetpointThreshold) {
                if (crash_recovery == PID_CRASH_RECOVERY_DISARM) {
                    setArmingDisabled(ARMING_DISABLED_CRASH_DETECTED);
                    disarm(DISARM_REASON_CRASH_PROTECTION);
                } else {
                    pidRuntime.inCrashRecoveryMode = true;
                    pidRuntime.crashDetectedAtUs = currentTimeUs;
                }
            }
            if (pidRuntime.inCrashRecoveryMode && cmpTimeUs(currentTimeUs, pidRuntime.crashDetectedAtUs) < pidRuntime.crashTimeDelayUs && (fabsf(errorRate) < pidRuntime.crashGyroThreshold
                || fabsf(getSetpointRate(axis)) > pidRuntime.crashSetpointThreshold)) {
                pidRuntime.inCrashRecoveryMode = false;
                BEEP_OFF;
            }
        } else if (pidRuntime.inCrashRecoveryMode) {
            pidRuntime.inCrashRecoveryMode = false;
            BEEP_OFF;
        }
    }
}
#endif // USE_ACC

#ifdef USE_ACRO_TRAINER

int acroTrainerSign(float x)
{
    return x > 0 ? 1 : -1;
}

// Acro Trainer - Manipulate the setPoint to limit axis angle while in acro mode
// There are three states:
// 1. Current angle has exceeded limit
//    Apply correction to return to limit (similar to pidLevel)
// 2. Future overflow has been projected based on current angle and gyro rate
//    Manage the setPoint to control the gyro rate as the actual angle  approaches the limit (try to prevent overshoot)
// 3. If no potential overflow is detected, then return the original setPoint

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM. We accept the
// performance decrease when Acro Trainer mode is active under the assumption that user is unlikely to be
// expecting ultimate flight performance at very high loop rates when in this mode.
static FAST_CODE_NOINLINE float applyAcroTrainer(int axis, const rollAndPitchTrims_t *angleTrim, float setPoint)
{
    float ret = setPoint;

    if (!FLIGHT_MODE(ANGLE_MODE) && !FLIGHT_MODE(HORIZON_MODE) && !FLIGHT_MODE(GPS_RESCUE_MODE)) {
        bool resetIterm = false;
        float projectedAngle = 0;
        const int setpointSign = acroTrainerSign(setPoint);
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        const int angleSign = acroTrainerSign(currentAngle);

        if ((pidRuntime.acroTrainerAxisState[axis] != 0) && (pidRuntime.acroTrainerAxisState[axis] != setpointSign)) {  // stick has reversed - stop limiting
            pidRuntime.acroTrainerAxisState[axis] = 0;
        }

        // Limit and correct the angle when it exceeds the limit
        if ((fabsf(currentAngle) > pidRuntime.acroTrainerAngleLimit) && (pidRuntime.acroTrainerAxisState[axis] == 0)) {
            if (angleSign == setpointSign) {
                pidRuntime.acroTrainerAxisState[axis] = angleSign;
                resetIterm = true;
            }
        }

        if (pidRuntime.acroTrainerAxisState[axis] != 0) {
            ret = constrainf(((pidRuntime.acroTrainerAngleLimit * angleSign) - currentAngle) * pidRuntime.acroTrainerGain, -ACRO_TRAINER_SETPOINT_LIMIT, ACRO_TRAINER_SETPOINT_LIMIT);
        } else {

        // Not currently over the limit so project the angle based on current angle and
        // gyro angular rate using a sliding window based on gyro rate (faster rotation means larger window.
        // If the projected angle exceeds the limit then apply limiting to minimize overshoot.
            // Calculate the lookahead window by scaling proportionally with gyro rate from 0-500dps
            float checkInterval = constrainf(fabsf(gyro.gyroADCf[axis]) / ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT, 0.0f, 1.0f) * pidRuntime.acroTrainerLookaheadTime;
            projectedAngle = (gyro.gyroADCf[axis] * checkInterval) + currentAngle;
            const int projectedAngleSign = acroTrainerSign(projectedAngle);
            if ((fabsf(projectedAngle) > pidRuntime.acroTrainerAngleLimit) && (projectedAngleSign == setpointSign)) {
                ret = ((pidRuntime.acroTrainerAngleLimit * projectedAngleSign) - projectedAngle) * pidRuntime.acroTrainerGain;
                resetIterm = true;
            }
        }

        if (resetIterm) {
            pidData[axis].I = 0;
        }

        if (axis == pidRuntime.acroTrainerDebugAxis) {
            DEBUG_SET(DEBUG_ACRO_TRAINER, 0, lrintf(currentAngle * 10.0f));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 1, pidRuntime.acroTrainerAxisState[axis]);
            DEBUG_SET(DEBUG_ACRO_TRAINER, 2, lrintf(ret));
            DEBUG_SET(DEBUG_ACRO_TRAINER, 3, lrintf(projectedAngle * 10.0f));
        }
    }

    return ret;
}
#endif // USE_ACRO_TRAINER

static float accelerationLimit(int axis, float currentPidSetpoint)
{
    static float previousSetpoint[XYZ_AXIS_COUNT];
    const float currentVelocity = currentPidSetpoint - previousSetpoint[axis];

    if (fabsf(currentVelocity) > pidRuntime.maxVelocity[axis]) {
        currentPidSetpoint = (currentVelocity > 0) ? previousSetpoint[axis] + pidRuntime.maxVelocity[axis] : previousSetpoint[axis] - pidRuntime.maxVelocity[axis];
    }

    previousSetpoint[axis] = currentPidSetpoint;
    return currentPidSetpoint;
}

static void rotateVector(float v[XYZ_AXIS_COUNT], float rotation[XYZ_AXIS_COUNT])
{
    // rotate v around rotation vector rotation
    // rotation in radians, all elements must be small
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        int i_1 = (i + 1) % 3;
        int i_2 = (i + 2) % 3;
        float newV = v[i_1] + v[i_2] * rotation[i];
        v[i_2] -= v[i_1] * rotation[i];
        v[i_1] = newV;
    }
}

STATIC_UNIT_TESTED void rotateItermAndAxisError(void)
{
    if (pidRuntime.itermRotation
#if defined(USE_ABSOLUTE_CONTROL)
        || pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR
#endif
        ) {
        const float gyroToAngle = pidRuntime.dT * RAD;
        float rotationRads[XYZ_AXIS_COUNT];
        for (int i = FD_ROLL; i <= FD_YAW; i++) {
            rotationRads[i] = gyro.gyroADCf[i] * gyroToAngle;
        }
#if defined(USE_ABSOLUTE_CONTROL)
        if (pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR) {
            rotateVector(axisError, rotationRads);
        }
#endif
        if (pidRuntime.itermRotation) {
            float v[XYZ_AXIS_COUNT];
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                v[i] = pidData[i].I;
            }
            rotateVector(v, rotationRads );
            for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
                pidData[i].I = v[i];
            }
        }
    }
}

#if defined(USE_ITERM_RELAX)
#if defined(USE_ABSOLUTE_CONTROL)
STATIC_UNIT_TESTED void applyAbsoluteControl(const int axis, const float gyroRate, float *currentPidSetpoint, float *itermErrorRate)
{
    if (pidRuntime.acGain > 0 || debugMode == DEBUG_AC_ERROR) {
        const float setpointLpf = pt1FilterApply(&pidRuntime.acLpf[axis], *currentPidSetpoint);
        const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);
        float acErrorRate = 0;
        const float gmaxac = setpointLpf + 2 * setpointHpf;
        const float gminac = setpointLpf - 2 * setpointHpf;
        if (gyroRate >= gminac && gyroRate <= gmaxac) {
            const float acErrorRate1 = gmaxac - gyroRate;
            const float acErrorRate2 = gminac - gyroRate;
            if (acErrorRate1 * axisError[axis] < 0) {
                acErrorRate = acErrorRate1;
            } else {
                acErrorRate = acErrorRate2;
            }
            if (fabsf(acErrorRate * pidRuntime.dT) > fabsf(axisError[axis]) ) {
                acErrorRate = -axisError[axis] * pidRuntime.pidFrequency;
            }
        } else {
            acErrorRate = (gyroRate > gmaxac ? gmaxac : gminac ) - gyroRate;
        }

        if (isAirmodeActivated()) {
            axisError[axis] = constrainf(axisError[axis] + acErrorRate * pidRuntime.dT,
                -pidRuntime.acErrorLimit, pidRuntime.acErrorLimit);
            const float acCorrection = constrainf(axisError[axis] * pidRuntime.acGain, -pidRuntime.acLimit, pidRuntime.acLimit);
            *currentPidSetpoint += acCorrection;
            *itermErrorRate += acCorrection;
            DEBUG_SET(DEBUG_AC_CORRECTION, axis, lrintf(acCorrection * 10));
            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 3, lrintf(acCorrection * 10));
            }
        }
        DEBUG_SET(DEBUG_AC_ERROR, axis, lrintf(axisError[axis] * 10));
    }
}
#endif

STATIC_UNIT_TESTED void applyItermRelax(const int axis, const float iterm,
    const float gyroRate, float *itermErrorRate, float *currentPidSetpoint)
{
    const float setpointLpf = pt1FilterApply(&pidRuntime.windupLpf[axis], *currentPidSetpoint);
    const float setpointHpf = fabsf(*currentPidSetpoint - setpointLpf);

    if (pidRuntime.itermRelax) {
        if (axis < FD_YAW || pidRuntime.itermRelax == ITERM_RELAX_RPY || pidRuntime.itermRelax == ITERM_RELAX_RPY_INC) {
            float itermRelaxThreshold = ITERM_RELAX_SETPOINT_THRESHOLD;
            if (FLIGHT_MODE(ANGLE_MODE)) {
                itermRelaxThreshold *= 0.2f;
            }
            const float itermRelaxFactor = MAX(0, 1 - setpointHpf / itermRelaxThreshold);
            const bool isDecreasingI =
                ((iterm > 0) && (*itermErrorRate < 0)) || ((iterm < 0) && (*itermErrorRate > 0));
            if ((pidRuntime.itermRelax >= ITERM_RELAX_RP_INC) && isDecreasingI) {
                // Do Nothing, use the precalculed itermErrorRate
            } else if (pidRuntime.itermRelaxType == ITERM_RELAX_SETPOINT) {
                *itermErrorRate *= itermRelaxFactor;
            } else if (pidRuntime.itermRelaxType == ITERM_RELAX_GYRO ) {
                *itermErrorRate = fapplyDeadband(setpointLpf - gyroRate, setpointHpf);
            } else {
                *itermErrorRate = 0.0f;
            }

            if (axis == FD_ROLL) {
                DEBUG_SET(DEBUG_ITERM_RELAX, 0, lrintf(setpointHpf));
                DEBUG_SET(DEBUG_ITERM_RELAX, 1, lrintf(itermRelaxFactor * 100.0f));
                DEBUG_SET(DEBUG_ITERM_RELAX, 2, lrintf(*itermErrorRate));
            }
        }

#if defined(USE_ABSOLUTE_CONTROL)
        applyAbsoluteControl(axis, gyroRate, currentPidSetpoint, itermErrorRate);
#endif
    }
}
#endif

#ifdef USE_AIRMODE_LPF
void pidUpdateAirmodeLpf(float currentOffset)
{
    if (pidRuntime.airmodeThrottleOffsetLimit == 0.0f) {
        return;
    }

    float offsetHpf = currentOffset * 2.5f;
    offsetHpf = offsetHpf - pt1FilterApply(&pidRuntime.airmodeThrottleLpf2, offsetHpf);

    // During high frequency oscillation 2 * currentOffset averages to the offset required to avoid mirroring of the waveform
    pt1FilterApply(&pidRuntime.airmodeThrottleLpf1, offsetHpf);
    // Bring offset up immediately so the filter only applies to the decline
    if (currentOffset * pidRuntime.airmodeThrottleLpf1.state >= 0 && fabsf(currentOffset) > pidRuntime.airmodeThrottleLpf1.state) {
        pidRuntime.airmodeThrottleLpf1.state = currentOffset;
    }
    pidRuntime.airmodeThrottleLpf1.state = constrainf(pidRuntime.airmodeThrottleLpf1.state, -pidRuntime.airmodeThrottleOffsetLimit, pidRuntime.airmodeThrottleOffsetLimit);
}

float pidGetAirmodeThrottleOffset(void)
{
    return pidRuntime.airmodeThrottleLpf1.state;
}
#endif

#ifdef USE_LAUNCH_CONTROL
#define LAUNCH_CONTROL_MAX_RATE 100.0f
#define LAUNCH_CONTROL_MIN_RATE 5.0f
#define LAUNCH_CONTROL_ANGLE_WINDOW 10.0f  // The remaining angle degrees where rate dampening starts

// Use the FAST_CODE_NOINLINE directive to avoid this code from being inlined into ITCM RAM to avoid overflow.
// The impact is possibly slightly slower performance on F7/H7 but they have more than enough
// processing power that it should be a non-issue.
static FAST_CODE_NOINLINE float applyLaunchControl(int axis, const rollAndPitchTrims_t *angleTrim)
{
    float ret = 0.0f;

    // Scale the rates based on stick deflection only. Fixed rates with a max of 100deg/sec
    // reached at 50% stick deflection. This keeps the launch control positioning consistent
    // regardless of the user's rates.
    if ((axis == FD_PITCH) || (pidRuntime.launchControlMode != LAUNCH_CONTROL_MODE_PITCHONLY)) {
        const float stickDeflection = constrainf(getRcDeflection(axis), -0.5f, 0.5f);
        ret = LAUNCH_CONTROL_MAX_RATE * stickDeflection * 2;
    }

#if defined(USE_ACC)
    // If ACC is enabled and a limit angle is set, then try to limit forward tilt
    // to that angle and slow down the rate as the limit is approached to reduce overshoot
    if ((axis == FD_PITCH) && (pidRuntime.launchControlAngleLimit > 0) && (ret > 0)) {
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        if (currentAngle >= pidRuntime.launchControlAngleLimit) {
            ret = 0.0f;
        } else {
            //for the last 10 degrees scale the rate from the current input to 5 dps
            const float angleDelta = pidRuntime.launchControlAngleLimit - currentAngle;
            if (angleDelta <= LAUNCH_CONTROL_ANGLE_WINDOW) {
                ret = scaleRangef(angleDelta, 0, LAUNCH_CONTROL_ANGLE_WINDOW, LAUNCH_CONTROL_MIN_RATE, ret);
            }
        }
    }
#else
    UNUSED(angleTrim);
#endif

    return ret;
}
#endif

// Betaflight pid controller, which will be maintained in the future with additional features specialised for current (mini) multirotor usage.
// Based on 2DOF reference design (matlab)
void FAST_CODE pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    static float previousGyroRateDterm[XYZ_AXIS_COUNT];
    static float previousRawGyroRateDterm[XYZ_AXIS_COUNT];

#ifdef USE_TPA_MODE
    const float tpaFactorKp = (pidProfile->tpa_mode == TPA_MODE_PD) ? pidRuntime.tpaFactor : 1.0f;
#else
    const float tpaFactorKp = pidRuntime.tpaFactor;
#endif

#ifdef USE_YAW_SPIN_RECOVERY
    const bool yawSpinActive = gyroYawSpinDetected();
#endif

    const bool launchControlActive = isLaunchControlActive();

#if defined(USE_ACC)
    static timeUs_t levelModeStartTimeUs = 0;
    static bool gpsRescuePreviousState = false;
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
    float horizonLevelStrength = 0.0f;

    const bool gpsRescueIsActive = FLIGHT_MODE(GPS_RESCUE_MODE);
    levelMode_e levelMode;
    if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE) || gpsRescueIsActive) {
        if (pidRuntime.levelRaceMode && !gpsRescueIsActive) {
            levelMode = LEVEL_MODE_R; // level race mode是放开pitch、但会保持roll水平，240730 jsl
        } else {
            levelMode = LEVEL_MODE_RP;
        }

        // Keep track of when we entered a self-level mode so that we can
        // add a guard time before crash recovery can activate.
        // Also reset the guard time whenever GPS Rescue is activated.
        if ((levelModeStartTimeUs == 0) || (gpsRescueIsActive && !gpsRescuePreviousState)) {
            levelModeStartTimeUs = currentTimeUs;
        }

        // Calc horizonLevelStrength if needed
        if (FLIGHT_MODE(HORIZON_MODE)) {
            horizonLevelStrength = calcHorizonLevelStrength();
        }
    } else {
        levelMode = LEVEL_MODE_OFF;
        levelModeStartTimeUs = 0;
    }

    gpsRescuePreviousState = gpsRescueIsActive;
#else
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);
#endif

    // Anti Gravity
    if (pidRuntime.antiGravityEnabled) {
        pidRuntime.antiGravityThrottleD *= pidRuntime.antiGravityGain;
        // used later to increase pTerm
        pidRuntime.itermAccelerator = pidRuntime.antiGravityThrottleD * ANTIGRAVITY_KI;
    } else {
        pidRuntime.antiGravityThrottleD = 0.0f;
        pidRuntime.itermAccelerator = 0.0f;
    }
    DEBUG_SET(DEBUG_ANTI_GRAVITY, 2, lrintf((1 + (pidRuntime.itermAccelerator / pidRuntime.pidCoefficient[FD_PITCH].Ki)) * 1000));
    // amount of antigravity added relative to user's pitch iTerm coefficient
    // used later to increase iTerm

    // iTerm windup (attenuation of iTerm if motorMix range is large)
    float dynCi = 1.0;
    if (pidRuntime.itermWindupPointInv > 1.0f && mixerConfig()->mixerMode != MIXER_BICOPTER) {
        dynCi = constrainf((1.0f - getMotorMixRange()) * pidRuntime.itermWindupPointInv, 0.0f, 1.0f);
    }
    // Precalculate gyro delta for D-term here, this allows loop unrolling
    float gyroRateDterm[XYZ_AXIS_COUNT];
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
        gyroRateDterm[axis] = gyro.gyroADCf[axis];

        // Log the unfiltered D for ROLL and PITCH
        if (debugMode == DEBUG_D_LPF && axis != FD_YAW) {
            const float delta = (previousRawGyroRateDterm[axis] - gyroRateDterm[axis]) * pidRuntime.pidFrequency / D_LPF_RAW_SCALE;
            previousRawGyroRateDterm[axis] = gyroRateDterm[axis];
            DEBUG_SET(DEBUG_D_LPF, axis, lrintf(delta)); // debug d_lpf 2 and 3 used for pre-TPA D
        }

        gyroRateDterm[axis] = pidRuntime.dtermNotchApplyFn((filter_t *) &pidRuntime.dtermNotch[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = pidRuntime.dtermLowpassApplyFn((filter_t *) &pidRuntime.dtermLowpass[axis], gyroRateDterm[axis]);
        gyroRateDterm[axis] = pidRuntime.dtermLowpass2ApplyFn((filter_t *) &pidRuntime.dtermLowpass2[axis], gyroRateDterm[axis]);
    }

    rotateItermAndAxisError();

#ifdef USE_RPM_FILTER
    rpmFilterUpdate();
#endif


    
    #ifdef QUATERNION_CONTROL
    angularRateFromQuaternionError(pidProfile, angleTrim);
    #endif
    //
    // ----------PID controller----------
    for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) { // 超大循环905-1155，把角度、角速度都算完 240730 jsl
        // 1. 接收RC指令（currentPidSetpoint out） 240730 jsl
        float currentPidSetpoint = getSetpointRate(axis); // 这里curSP是摇杆指令、没确定是期望角度还是期望角速度 240730 jsl
        if (pidRuntime.maxVelocity[axis]) {
            currentPidSetpoint = accelerationLimit(axis, currentPidSetpoint);
        }
        // Yaw control is GYRO based, direct sticks control is applied to rate PID
        // yaw 只有角速度控制 240730 jsl
        // When Race Mode is active PITCH control is also GYRO based in level or horizon mode
#if defined(USE_ACC)
        pidRuntime.axisInAngleMode[axis] = false;
        if (axis < FD_YAW) { // 在angle模式下，roll和pitch是角度（yaw还是角速度） 240730 jsl
            if (levelMode == LEVEL_MODE_RP || (levelMode == LEVEL_MODE_R && axis == FD_ROLL)) {
                pidRuntime.axisInAngleMode[axis] = true;
                // 2. Roll Pitch角度环（currentPidSetpoint in, currentPidSetpoint out） 240730 jsl
                currentPidSetpoint = pidLevel(axis, pidProfile, angleTrim, currentPidSetpoint, horizonLevelStrength); // curSP经过角度环、是期望角速度 240730 jsl
            }
        } else { // yaw axis only
            if (levelMode == LEVEL_MODE_RP) {
                // if earth referencing is requested, attenuate yaw axis setpoint when pitched or rolled
                // and send yawSetpoint to Angle code to modulate pitch and roll
                // code cost is 107 cycles when earthRef enabled, 20 otherwise, nearly all in cos_approx
                const float earthRefGain = FLIGHT_MODE(GPS_RESCUE_MODE) ? 1.0f : pidRuntime.angleEarthRef;
                if (earthRefGain) {
                    pidRuntime.angleYawSetpoint = currentPidSetpoint;
                    float maxAngleTargetAbs = earthRefGain * fmaxf( fabsf(pidRuntime.angleTarget[FD_ROLL]), fabsf(pidRuntime.angleTarget[FD_PITCH]) );
                    maxAngleTargetAbs *= (FLIGHT_MODE(HORIZON_MODE)) ? horizonLevelStrength : 1.0f;
                    // reduce compensation whenever Horizon uses less levelling
                    currentPidSetpoint *= cos_approx(DEGREES_TO_RADIANS(maxAngleTargetAbs));
                    DEBUG_SET(DEBUG_ANGLE_TARGET, 2, currentPidSetpoint); // yaw setpoint after attenuation
                }
            }
        }
#endif

/*
#ifdef CONFIGURATION_TAILSITTER
        // Coordinate flight in rate mode for tailsitter fixed wing configuration
        // Calculate currentPidSetpoint for yaw (roll in vtol mode)
        if (axis == FD_ROLL && !FLIGHT_MODE(ANGLE_MODE)){
            // get current attitude quaternion
            float quat_ang[4], quat_90pitch[4], eulerYPR_90Pitch[3], quat_ang_fixedwing[4], eulerYPR_fixedwing[3];
            quaternion q_ang = QUATERNION_INITIALIZE;
            getQuaternion(&q_ang);
            quat_ang[0] = q_ang.w; quat_ang[1] = q_ang.x; quat_ang[2] = q_ang.y; quat_ang[3] = q_ang.z;
            // set 90 deg rotate quaternion for fixed wing mode
            eulerYPR_90Pitch[0] = 0; eulerYPR_90Pitch[1] = -M_PIf/2; eulerYPR_90Pitch[2] = 0;
            eul2quatZYX(eulerYPR_90Pitch, quat_90pitch);
            // get fixed wing mode YPR euler angle
            quaternionMultiply(quat_ang, quat_90pitch, quat_ang_fixedwing);
            quat2eulZYX(quat_ang_fixedwing, eulerYPR_fixedwing);
            // estimate airspeed
            const float airspeed = 2.0f;
            // calculate coordinate turn yaw setpoint (saturate input and output)
            // limit roll and pitch to +-60
            float RP_saturate = M_PIf / 7;
            if(ABS(eulerYPR_fixedwing[1]) < RP_saturate ){
                currentPidSetpoint = RADIANS_TO_DEGREES(9.8f / airspeed * tanf(constrainf(eulerYPR_fixedwing[2], -RP_saturate, RP_saturate)) * cosf(eulerYPR_fixedwing[1]));
            }
            position_msp.msg6 = currentPidSetpoint*100.0f;
        }
#endif
*/

#ifdef USE_ACRO_TRAINER
        if ((axis != FD_YAW) && pidRuntime.acroTrainerActive && !pidRuntime.inCrashRecoveryMode && !launchControlActive) {
            currentPidSetpoint = applyAcroTrainer(axis, angleTrim, currentPidSetpoint);
        }
#endif // USE_ACRO_TRAINER

#ifdef USE_LAUNCH_CONTROL
        if (launchControlActive) {
#if defined(USE_ACC)
            currentPidSetpoint = applyLaunchControl(axis, angleTrim);
#else
            currentPidSetpoint = applyLaunchControl(axis, NULL);
#endif
        }
#endif

        // Handle yaw spin recovery - zero the setpoint on yaw to aid in recovery
        // It's not necessary to zero the set points for R/P because the PIDs will be zeroed below
#ifdef USE_YAW_SPIN_RECOVERY
        if ((axis == FD_YAW) && yawSpinActive) {
            currentPidSetpoint = 0.0f;
        }
#endif // USE_YAW_SPIN_RECOVERY

        // -----calculate error rate
        // 3.1 RPY角速度误差 (currentPidSetpoint in, errorRate out) 240730 jsl
        // const float gyroRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec
        float gyroRate = gyro.gyroADCf[axis]; // Process variable from gyro output in deg/sec
        // 20240910 Enable rotor disk angular rate feedback, only in pitch axis for bicopter
        // if (mixerConfig()->mixerMode == MIXER_BICOPTER && axis == FD_PITCH) {
        //    gyroRate += diskAngularRate;//lowPassFilterUpdate(servoAngularRateFeedback, pidRuntime.dT);
        // }
        // position_msp.msg4 = gyro.gyroADCf[FD_PITCH] * 100.0;

#ifdef ROTORDISK_FEEDBACK
        // Rotate gyro feedback to rotor disk frame.
        // Solve the control allocation problem in feedback instead of controller
        // From a rotation matrix based on rotorDisk angle
        const float rotorDiskRadians = DEGREES_TO_RADIANS(rotorDiskAngleFeedback);
        switch(axis) {
            case FD_ROLL:{
                gyroRate = cosf(rotorDiskRadians) * gyro.gyroADCf[FD_ROLL] - sinf(rotorDiskRadians) * gyro.gyroADCf[FD_YAW];
                break;
            }
            case FD_PITCH: {
                gyroRate = gyro.gyroADCf[FD_PITCH];
                break;
            }
            case FD_YAW: {
                gyroRate = sinf(rotorDiskRadians) * gyro.gyroADCf[FD_ROLL] + cosf(rotorDiskRadians) * gyro.gyroADCf[FD_YAW];
                break;
            }
        }
#endif


        float errorRate = currentPidSetpoint - gyroRate; // r - y
#if defined(USE_ACC)
        handleCrashRecovery(
            pidProfile->crash_recovery, angleTrim, axis, currentTimeUs, gyroRate,
            &currentPidSetpoint, &errorRate);
#endif

        const float previousIterm = pidData[axis].I; // I 项 240730 jsl
        float itermErrorRate = errorRate; // I 项累积速率 240730 jsl
#ifdef USE_ABSOLUTE_CONTROL
        const float uncorrectedSetpoint = currentPidSetpoint;
#endif

#if defined(USE_ITERM_RELAX)
        if (!launchControlActive && !pidRuntime.inCrashRecoveryMode) {
            applyItermRelax(axis, previousIterm, gyroRate, &itermErrorRate, &currentPidSetpoint);
            errorRate = currentPidSetpoint - gyroRate;
        }
#endif
#ifdef USE_ABSOLUTE_CONTROL
        const float setpointCorrection = currentPidSetpoint - uncorrectedSetpoint;
#endif

        // --------low-level gyro-based PID based on 2DOF PID controller. ----------

        // -----calculate P component
        // 3.2 P控制（errorRate in, pidData[axis].P out）
        pidData[axis].P = pidRuntime.pidCoefficient[axis].Kp * errorRate * tpaFactorKp; // P 和 tpa 有关 240730 jsl
        if (axis == FD_YAW) {
            pidData[axis].P = pidRuntime.ptermYawLowpassApplyFn((filter_t *) &pidRuntime.ptermYawLowpass, pidData[axis].P);
        }


        // -----calculate I component
        float Ki = pidRuntime.pidCoefficient[axis].Ki;
#ifdef USE_LAUNCH_CONTROL
        // if launch control is active override the iterm gains and apply iterm windup protection to all axes
        if (launchControlActive) {
            Ki = pidRuntime.launchControlKi;
        } else
#endif
        {
            if (axis == FD_YAW) {
                pidRuntime.itermAccelerator = 0.0f; // no antigravity on yaw iTerm
            }
        }
        // 3.3 I控制（itermErrorRate in, pidData[axis].I out）
        const float iTermChange = (Ki + pidRuntime.itermAccelerator) * dynCi * pidRuntime.dT * itermErrorRate;
        pidData[axis].I = constrainf(previousIterm + iTermChange, -pidRuntime.itermLimit, pidRuntime.itermLimit);
#ifdef CONFIGURATION_TAILSITTER
        // limit roll(yaw for fixed wing) integral in coordinate flight
        if(!FLIGHT_MODE(ANGLE_MODE)){
            pidData[FD_ROLL].I = constrainf(pidData[FD_ROLL].I, -1.0f, 1.0f)*0.5f;
        }
#endif
        // -----calculate D component

        float pidSetpointDelta = 0;
        //position_msp.msg1 = setWP_msp.msg1;
#ifdef USE_FEEDFORWARD
        if (FLIGHT_MODE(ANGLE_MODE) && pidRuntime.axisInAngleMode[axis]) {
            // this axis is fully under self-levelling control
            // it will already have stick based feedforward applied in the input to their angle setpoint
            // a simple setpoint Delta can be used to for PID feedforward element for motor lag on these axes
            // however RC steps come in, via angle setpoint
            // and setpoint RC smoothing must have a cutoff half normal to remove those steps completely
            // the RC stepping does not come in via the feedforward, which is very well smoothed already
            // if uncommented, and the forcing to zero is removed, the two following lines will restore PID feedforward to angle mode axes
            // but for now let's see how we go without it (which was the case before 4.5 anyway)
//            pidSetpointDelta = currentPidSetpoint - pidRuntime.previousPidSetpoint[axis];
//            pidSetpointDelta *= pidRuntime.pidFrequency * pidRuntime.angleFeedforwardGain;
            // 看起来roll和pitch没加前馈 240730 jsl
            pidSetpointDelta = 0.0f;
        } else {
            // the axis is operating as a normal acro axis, so use normal feedforard from rc.c
            pidSetpointDelta = getFeedforward(axis);
        }
#endif
        pidRuntime.previousPidSetpoint[axis] = currentPidSetpoint; // this is the value sent to blackbox, and used for Dmin setpoint

        // disable D if launch control is active
        if ((pidRuntime.pidCoefficient[axis].Kd > 0) && !launchControlActive) {
            // Divide rate change by dT to get differential (ie dr/dt).
            // dT is fixed and calculated from the target PID loop time
            // This is done to avoid DTerm spikes that occur with dynamically
            // calculated deltaT whenever another task causes the PID
            // loop execution to be delayed.
            const float delta = - (gyroRateDterm[axis] - previousGyroRateDterm[axis]) * pidRuntime.pidFrequency;
            float preTpaD = pidRuntime.pidCoefficient[axis].Kd * delta;

#if defined(USE_ACC)
            if (cmpTimeUs(currentTimeUs, levelModeStartTimeUs) > CRASH_RECOVERY_DETECTION_DELAY_US) {
                detectAndSetCrashRecovery(pidProfile->crash_recovery, axis, currentTimeUs, delta, errorRate);
            }
#endif

#if defined(USE_D_MIN)
            float dMinFactor = 1.0f;
            if (pidRuntime.dMinPercent[axis] > 0) {
                float dMinGyroFactor = pt2FilterApply(&pidRuntime.dMinRange[axis], delta);
                dMinGyroFactor = fabsf(dMinGyroFactor) * pidRuntime.dMinGyroGain;
                const float dMinSetpointFactor = (fabsf(pidSetpointDelta)) * pidRuntime.dMinSetpointGain;
                dMinFactor = MAX(dMinGyroFactor, dMinSetpointFactor);
                dMinFactor = pidRuntime.dMinPercent[axis] + (1.0f - pidRuntime.dMinPercent[axis]) * dMinFactor;
                dMinFactor = pt2FilterApply(&pidRuntime.dMinLowpass[axis], dMinFactor);
                dMinFactor = MIN(dMinFactor, 1.0f);
                if (axis == FD_ROLL) {
                    DEBUG_SET(DEBUG_D_MIN, 0, lrintf(dMinGyroFactor * 100));
                    DEBUG_SET(DEBUG_D_MIN, 1, lrintf(dMinSetpointFactor * 100));
                    DEBUG_SET(DEBUG_D_MIN, 2, lrintf(pidRuntime.pidCoefficient[axis].Kd * dMinFactor * 10 / DTERM_SCALE));
                } else if (axis == FD_PITCH) {
                    DEBUG_SET(DEBUG_D_MIN, 3, lrintf(pidRuntime.pidCoefficient[axis].Kd * dMinFactor * 10 / DTERM_SCALE));
                }
            }

            // Apply the dMinFactor
            preTpaD *= dMinFactor;
#endif
            // 3.4 D控制（Tpa in pidData.D out） 240730 jsl
            pidData[axis].D = preTpaD * pidRuntime.tpaFactor;

            // Log the value of D pre application of TPA
            if (axis != FD_YAW) {
                DEBUG_SET(DEBUG_D_LPF, axis - FD_ROLL + 2, lrintf(preTpaD * D_LPF_PRE_TPA_SCALE));
            }
        } else {
            pidData[axis].D = 0; // yaw把D置零？
            if (axis != FD_YAW) {
                DEBUG_SET(DEBUG_D_LPF, axis - FD_ROLL + 2, 0);
            }
        }

        previousGyroRateDterm[axis] = gyroRateDterm[axis];

        // -----calculate feedforward component

#ifdef USE_ABSOLUTE_CONTROL
        // include abs control correction in feedforward
        pidSetpointDelta += setpointCorrection - pidRuntime.oldSetpointCorrection[axis];
        pidRuntime.oldSetpointCorrection[axis] = setpointCorrection;
#endif
        // no feedforward in launch control
        const float feedforwardGain = launchControlActive ? 0.0f : pidRuntime.pidCoefficient[axis].Kf;
        // 3.5 F控制（pidData.F out） 240730 jsl
        pidData[axis].F = feedforwardGain * pidSetpointDelta;
#ifdef ROTORDISK_FEEDBACK
        // Set the pitch forward
        if(axis == FD_PITCH){
            pidData[axis].F = (-PitchTarget - servo2RotorDiskMap_B) / servo2RotorDiskMap_K * servoPWMRange / servoAngleRange;
            position_msp.msg1 = pidData[axis].F * 100.0f;
        }
#endif        
#ifdef USE_YAW_SPIN_RECOVERY
        if (yawSpinActive) {
            pidData[axis].I = 0;  // in yaw spin always disable I
            if (axis <= FD_PITCH)  {
                // zero PIDs on pitch and roll leaving yaw P to correct spin
                pidData[axis].P = 0;
                pidData[axis].D = 0;
                pidData[axis].F = 0;
            }
        }
#endif // USE_YAW_SPIN_RECOVERY

#ifdef USE_LAUNCH_CONTROL
        // Disable P/I appropriately based on the launch control mode
        if (launchControlActive) {
            // if not using FULL mode then disable I accumulation on yaw as
            // yaw has a tendency to windup. Otherwise limit yaw iterm accumulation.
            const int launchControlYawItermLimit = (pidRuntime.launchControlMode == LAUNCH_CONTROL_MODE_FULL) ? LAUNCH_CONTROL_YAW_ITERM_LIMIT : 0;
            pidData[FD_YAW].I = constrainf(pidData[FD_YAW].I, -launchControlYawItermLimit, launchControlYawItermLimit);

            // for pitch-only mode we disable everything except pitch P/I
            if (pidRuntime.launchControlMode == LAUNCH_CONTROL_MODE_PITCHONLY) {
                pidData[FD_ROLL].P = 0;
                pidData[FD_ROLL].I = 0;
                pidData[FD_YAW].P = 0;
                // don't let I go negative (pitch backwards) as front motors are limited in the mixer
                pidData[FD_PITCH].I = MAX(0.0f, pidData[FD_PITCH].I);
            }
        }
#endif

        // Add P boost from antiGravity when sticks are close to zero
        if (axis != FD_YAW) {
            float agSetpointAttenuator = fabsf(currentPidSetpoint) / 50.0f;
            agSetpointAttenuator = MAX(agSetpointAttenuator, 1.0f);
            // attenuate effect if turning more than 50 deg/s, half at 100 deg/s
            const float antiGravityPBoost = 1.0f + (pidRuntime.antiGravityThrottleD / agSetpointAttenuator) * pidRuntime.antiGravityPGain;
            pidData[axis].P *= antiGravityPBoost;
            if (axis == FD_PITCH) {
                DEBUG_SET(DEBUG_ANTI_GRAVITY, 3, lrintf(antiGravityPBoost * 1000));
            }
        }

#ifdef CONFIGURATION_QUADTILT
        // Do not pass in feedforward to SUM since it affects motor mix
        const float pidSum = pidData[axis].P + pidData[axis].I + pidData[axis].D;
#else
        // 3.6 calculating the PID sum (pidData.Sum out)
        const float pidSum = pidData[axis].P + pidData[axis].I + pidData[axis].D + pidData[axis].F;
#endif
#ifdef USE_INTEGRATED_YAW_CONTROL
        if (axis == FD_YAW && pidRuntime.useIntegratedYaw) {
            pidData[axis].Sum += pidSum * pidRuntime.dT * 100.0f;
            pidData[axis].Sum -= pidData[axis].Sum * pidRuntime.integratedYawRelax / 100000.0f * pidRuntime.dT / 0.000125f;
        } else
#endif
        {
            pidData[axis].Sum = pidSum;
        }
    }
#ifdef ROTORDISK_FEEDBACK
    // JJJJJJJack 20240822
    // Estimate servo output
    float servoDesiredAngle = pidData[FD_PITCH].Sum*PID_SERVO_MIXER_SCALING/servoPWMRange*servoAngleRange;
    estimateServoAngle(servoDesiredAngle, pidRuntime.dT);
    // JJJJJJJack & Jsl 20240910
    // Estimate disk angular rate using diff + lowpass
    estimateDiskAngularRate(servoDesiredAngle, pidRuntime.dT);
#endif
    

    // 零油门pidData全置零？ 240730 jsl
    // Disable PID control if at zero throttle or if gyro overflow detected
    // This may look very innefficient, but it is done on purpose to always show real CPU usage as in flight
    if (!pidRuntime.pidStabilisationEnabled || gyroOverflowDetected()) {
        for (int axis = FD_ROLL; axis <= FD_YAW; ++axis) {
            pidData[axis].P = 0;
            pidData[axis].I = 0;
            pidData[axis].D = 0;
            pidData[axis].F = 0;

            pidData[axis].Sum = 0;
        }
    } else if (pidRuntime.zeroThrottleItermReset) {
        pidResetIterm();
    }
}

bool crashRecoveryModeActive(void)
{
    return pidRuntime.inCrashRecoveryMode;
}

#ifdef USE_ACRO_TRAINER
void pidSetAcroTrainerState(bool newState)
{
    if (pidRuntime.acroTrainerActive != newState) {
        if (newState) {
            pidAcroTrainerInit();
        }
        pidRuntime.acroTrainerActive = newState;
    }
}
#endif // USE_ACRO_TRAINER

void pidSetAntiGravityState(bool newState)
{
    if (newState != pidRuntime.antiGravityEnabled) {
        // reset the accelerator on state changes
        pidRuntime.itermAccelerator = 0.0f;
    }
    pidRuntime.antiGravityEnabled = newState;
}

bool pidAntiGravityEnabled(void)
{
    return pidRuntime.antiGravityEnabled;
}

#ifdef USE_DYN_LPF
void dynLpfDTermUpdate(float throttle)
{
    if (pidRuntime.dynLpfFilter != DYN_LPF_NONE) {
        float cutoffFreq;
        if (pidRuntime.dynLpfCurveExpo > 0) {
            cutoffFreq = dynLpfCutoffFreq(throttle, pidRuntime.dynLpfMin, pidRuntime.dynLpfMax, pidRuntime.dynLpfCurveExpo);
        } else {
            cutoffFreq = fmaxf(dynThrottle(throttle) * pidRuntime.dynLpfMax, pidRuntime.dynLpfMin);
        }

        switch (pidRuntime.dynLpfFilter) {
        case DYN_LPF_PT1:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt1FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt1Filter, pt1FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        case DYN_LPF_BIQUAD:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                biquadFilterUpdateLPF(&pidRuntime.dtermLowpass[axis].biquadFilter, cutoffFreq, targetPidLooptime);
            }
            break;
        case DYN_LPF_PT2:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt2FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt2Filter, pt2FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        case DYN_LPF_PT3:
            for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                pt3FilterUpdateCutoff(&pidRuntime.dtermLowpass[axis].pt3Filter, pt3FilterGain(cutoffFreq, pidRuntime.dT));
            }
            break;
        }
    }
}
#endif

float dynLpfCutoffFreq(float throttle, uint16_t dynLpfMin, uint16_t dynLpfMax, uint8_t expo)
{
    const float expof = expo / 10.0f;
    const float curve = throttle * (1 - throttle) * expof + throttle;
    return (dynLpfMax - dynLpfMin) * curve + dynLpfMin;
}

void pidSetItermReset(bool enabled)
{
    pidRuntime.zeroThrottleItermReset = enabled;
}

float pidGetPreviousSetpoint(int axis)
{
    return pidRuntime.previousPidSetpoint[axis];
}

float pidGetDT(void)
{
    return pidRuntime.dT;
}

float pidGetPidFrequency(void)
{
    return pidRuntime.pidFrequency;
}

#ifdef ROTORDISK_FEEDBACK
// JJJJJJJack 20240822
// Simulate the OMG servo with controller
float estimateServoAngle(float inputAngle, float DT)
{
    // Define Parameters
    const float simServoAngleKP = 60.0f;
    const float simServoMaxRate = 500.0f;
    const float simServoRateKP = 0.008f;
    const float simServoRateKI = 0.008f;
    //const float simServoRateKD = 0.008f;
    // Calculate angle error
    const float servoAngleError = inputAngle - servoAngleFeedback;
    // Get desired angular rate via angle P
    float servoDesiredAngularRate = servoAngleError * simServoAngleKP;
    servoDesiredAngularRate = constrainf(servoDesiredAngularRate, -simServoMaxRate, simServoMaxRate);
    // Calculate angular rate error
    const float servoAngularRateError = servoDesiredAngularRate - servoAngularRateFeedback;
    // Angular rate PID
    const float servoTorqueP = servoAngularRateError * simServoRateKP;
    servoRateIntegral += servoAngularRateError;
    const float servoTorqueI = servoRateIntegral * simServoRateKI;
    //float servoTorqueD = -servoAngularRateFeedback/ * simServoRateKD;
    const float servoTorque = servoTorqueP + servoTorqueI;
    const float servoAngularAcc = servoTorque / motorInertia;
    // Intergrating acc to rate and rate to angle
    servoAngularRateFeedback += servoAngularAcc * DT;
    servoAngleFeedback += servoAngularRateFeedback * DT;
    return servoAngleFeedback;
}

// JJJJJJJack & Jsl 20240910
// Simulate a low pass filter for servo angular rate
float lowPassFilterUpdate(float input, float DT){
  float filtered_out = ((2.0f-LP_A*DT)*LP_last_out + LP_A*DT*(input+LP_last_in))/(LP_A*DT+2.0f);
  LP_last_in = input;
  LP_last_out = filtered_out;
  return filtered_out;
}

// JJJJJJJack & Jsl 20240910
// Estimate disk angular rate using diff + lowpass
void estimateDiskAngularRate(float servoDesiredAngle, float DT){
    // diff
    diskDiffRate = (servoDesiredAngle - servoLastDesiredAngle) / DT; 
    diskDiffRate = fabs(diskDiffRate) > 1500 ? 0 : diskDiffRate;
    servoLastDesiredAngle = servoDesiredAngle;  
    // lowpass
    diskAngularRate = lowPassFilterUpdate(diskDiffRate, DT);
}
#endif

#ifdef QUATERNION_CONTROL

float sign(float x)
{
  return x > 0 ? 1 : -1;
}

// Wrap angle in radians to [-pi pi]
float conv2std(float input_angle)
{
    while(input_angle < 0)
        input_angle += 2*M_PI;
    input_angle += M_PI;
    input_angle = fmod(input_angle, 2*M_PI);
    input_angle -= M_PI;
    return input_angle;
}

// Integration of psi stick input to the desired psi
// Input:
//     psi_sp_diff: Stick input
//     psi_current: Current yaw
//     armed: Current arm state
//     armed_prev: Last arm state
// Output:
//     psi_des: Pointer to the desired psi
void calc_psi_des(float psi_sp_diff, float psi_current,  bool armed, bool  armed_prev, float * psi_des)
{
  float psi = psi_current;
  
  if(!armed)
    *psi_des = psi + psi_sp_diff;

  if((armed_prev != armed) && armed) {
    /*  if ARMed */
    *psi_des = psi;
    #ifdef INVERTED_FLIGHT
    if(!attitudeUpright())
        *psi_des = conv2std(*psi_des + M_PI);
    #endif
  }
  
  if (fabsf(psi_sp_diff) >= 0.01){
    /*  if rudder stick is at not at center, then change psi_sp. Else do not modify psi_sp */
    //float psi_integral = psi + psi_sp_diff;
    *psi_des += psi_sp_diff*0.005f;
  }
}

void eul2quatZYX(float eulerAngle[3], float * quaternion)
{
    // from Wiki: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // Abbreviations for the various angular functions
    float yaw = eulerAngle[0], pitch = eulerAngle[1], roll = eulerAngle[2];
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    quaternion[0] = cr * cp * cy + sr * sp * sy;
    quaternion[1] = sr * cp * cy - cr * sp * sy;
    quaternion[2] = cr * sp * cy + sr * cp * sy;
    quaternion[3] = cr * cp * sy - sr * sp * cy;
}

void quat2eulZYX(float quaternion[4], float * eulerAngle)
{
    // from Wiki: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (quaternion[0] * quaternion[1] + quaternion[2] * quaternion[3]);
    float cosr_cosp = 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]);
    eulerAngle[2] = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = sqrt(1 + 2 * (quaternion[0] * quaternion[2] - quaternion[1] * quaternion[3]));
    float cosp = sqrt(1 - 2 * (quaternion[0] * quaternion[2] - quaternion[1] * quaternion[3]));
    eulerAngle[1] = 2 * atan2f(sinp, cosp) - M_PIf / 2;

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (quaternion[0] * quaternion[3] + quaternion[1] * quaternion[2]);
    float cosy_cosp = 1 - 2 * (quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
    eulerAngle[0] = atan2f(siny_cosp, cosy_cosp);

}
//20250423, rotation in ZXY order
//------------------- ZXY ----------------------------
void eul2quatZXY(float eulerAngle[3], float * quaternion) {
    // ZXY顺序：绕Z轴(yaw) -> 绕X轴(roll) -> 绕Y轴(pitch)
    float yaw = eulerAngle[0];    // Z轴旋转角（单位：弧度）
    float roll = eulerAngle[1];   // X轴旋转角（单位：弧度）
    float pitch = eulerAngle[2];  // Y轴旋转角（单位：弧度）

    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);

    // 四元数乘法顺序：q_z * q_x * q_y
    quaternion[0] = cy * cr * cp - sy * sr * sp; // w
    quaternion[1] = cy * sr * cp - sy * cr * sp; // x
    quaternion[2] = cy * cr * sp + sy * sr * cp; // y
    quaternion[3] = cy * sr * sp + sy * cr * cp; // z
}

void quat2eulZXY(float quaternion[4], float * eulerAngle) {
    float qw = quaternion[0];
    float qx = quaternion[1];
    float qy = quaternion[2];
    float qz = quaternion[3];

    float tmp = 2 * (qw * qx + qy * qz);
    // 截断到[-1, 1]范围
    if (tmp > 1.0f) tmp = 1.0f;
    else if (tmp < -1.0f) tmp = -1.0f;

    eulerAngle[1] = asinf(tmp); // Roll ∈ [-π/2, π/2]
    float tolA = 0.5f * M_PIf - 10 * FLT_EPSILON;
    float tolB = -0.5f * M_PIf + 10 * FLT_EPSILON;
    if (eulerAngle[1] >= tolA){
        eulerAngle[0] = 2 * atan2f(qy, qw);
        eulerAngle[2] = 0.0f;
    }
    else if (eulerAngle[1] <= tolB){
        eulerAngle[0] = -2 * atan2f(qy, qw);
        eulerAngle[2] = 0.0f;
    }
    else{
        eulerAngle[0] = atan2f((2 * qw * qz - 2 * qx * qy ), (2 * sqrtf(qw) + 2 * sqrtf(qy) - 1) );
        eulerAngle[2] = atan2f((2 * qw * qy - 2 * qx * qz ), (2 * sqrtf(qw) + 2 * sqrtf(qz) - 1) );
    }
}
//------------------------------------------------------

float quaternionNorm(float quat[4])
{
    return quat[0]*quat[0] + quat[1]*quat[1] + quat[2]*quat[2] + quat[3]*quat[3];
}

void quaternionNormalize(float quat[4], float * result)
{
    float quaternion_norm = quaternionNorm(quat);
    result[0] = quat[0] / sqrtf(quaternion_norm);
    result[1] = quat[1] / sqrtf(quaternion_norm);
    result[2] = quat[2] / sqrtf(quaternion_norm);
    result[3] = quat[3] / sqrtf(quaternion_norm);
}

void quaternionMultiply(float q1[4], float q2[4], float * result)
{
    result[1] =  q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2] + q1[0] * q2[1];
    result[2] = -q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1] + q1[0] * q2[2];
    result[3] =  q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0] + q1[0] * q2[3];
    result[0] = -q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3] + q1[0] * q2[0];
}

void quaternionConjugate(float quaternion[4], float * result)
{
    result[1] = -quaternion[1];
    result[2] = -quaternion[2];
    result[3] = -quaternion[3];
    result[0] = quaternion[0];
}

void quaternionInverse(float quaternion[4], float * result)
{
    float quaternion_conjugate[4];
    quaternionConjugate(quaternion, quaternion_conjugate);
    float quaternion_norm = quaternionNorm(quaternion);
    result[0] = quaternion_conjugate[0] / quaternion_norm;
    result[1] = quaternion_conjugate[1] / quaternion_norm;
    result[2] = quaternion_conjugate[2] / quaternion_norm;
    result[3] = quaternion_conjugate[3] / quaternion_norm;
    //qinv  = quatconj( q )./(quatnorm( q )*ones(1,4));
}

void quaternionToAxisAngle(float quaternion[4], float * axisAngle)
{
    //Normalize the quaternions
    float quat_norm[4];
    quaternionNormalize(quaternion, quat_norm);

    //Normalize and generate the rotation vector and angle sequence
    //For a single quaternion q = [w x y z], the formulas are as follows:
    //(axis) v = [x y z] / norm([x y z]);
    //(angle) theta = 2 * acos(w)
    float axis[3], axis_norm, angle;
    axis_norm = sqrtf(quat_norm[1]*quat_norm[1] + quat_norm[2]*quat_norm[2] + quat_norm[3]*quat_norm[3]);
    if(axis_norm != 0){
        axis[0] = quat_norm[1] / axis_norm;
        axis[1] = quat_norm[2] / axis_norm;
        axis[2] = quat_norm[3] / axis_norm;
    }else{
        axis[0] = 0; axis[1] = 0; axis[2] = 1;
    }
    angle = conv2std(2.0f*acosf(quat_norm[0]));

    axisAngle[0] = axis[0];
    axisAngle[1] = axis[1];
    axisAngle[2] = axis[2];
    axisAngle[3] = angle;
}

#endif
