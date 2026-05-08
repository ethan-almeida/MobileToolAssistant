#include "motor_hw.h"
#include "obst_det.h"
#include "dwm1000.h"
#include "dwm_triangulation.h"
#include <stdlib.h>

/* External timers */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim12;

/* From main.c */
extern RemotePosition remote_pos;
extern bool standby;

/* From obst_det.c */
extern ToF_Data_t ToF_DATA[TOF_SENSOR_COUNT];

/* ================= Robot Command ================= */

RobotCommand_t robotCommand;

/* ================= Tunable Motion Values ================= */

/* Obstacle avoidance speeds */
#define MOTOR_PWM_FORWARD   55000
#define MOTOR_PWM_TURN      52000
#define MOTOR_PWM_REVERSE   55000

/* Follow / steering */
#define FOLLOW_PWM_BASE     47000
#define FOLLOW_PWM_MIN      35000
#define FOLLOW_PWM_MAX      55000

/* Distance thresholds */
#define FOLLOW_STOP_CM      100
#define FOLLOW_RESUME_CM    115

/* Raw UWB validity */
#define UWB_MIN_VALID_CM     25
#define UWB_MAX_VALID_CM    600

/* Angle model: 90 = center, 180 = left, 0 = right */
#define ANGLE_CENTER_DEG     90
#define ANGLE_DEADBAND_DEG    5
#define ANGLE_MAX_ERROR_DEG  60

/* Differential steering */
#define STEER_GAIN_BASE     260
#define STEER_GAIN_EXTRA    220
#define STEER_EXTRA_START    15
#define STEER_MAX_PWM      18000

/* Obstacle pacing */
#define OBST_STOP_BEFORE_MS   250
#define OBST_ACTION_MS        280
#define OBST_STOP_AFTER_MS    250

/* ================= Helper ================= */

static inline uint16_t clamp_pwm(uint32_t pwm)
{
    if (pwm > PWM_MAX)
        return PWM_MAX;
    return (uint16_t)pwm;
}

static inline uint16_t safe_distance(uint16_t d)
{
    if (d == 0)
        return 4000;
    return d;
}

static inline int16_t abs_i16(int16_t x)
{
    return (x < 0) ? -x : x;
}

/* ================= Follow State ================= */

static uint8_t follow_stop_latched = 0;

/* ================= Obstacle Action State ================= */

typedef enum
{
    OBST_STATE_IDLE = 0,
    OBST_STATE_STOP_BEFORE,
    OBST_STATE_ACTION,
    OBST_STATE_STOP_AFTER
} ObstacleState_t;

typedef enum
{
    OBST_CMD_NONE = 0,
    OBST_CMD_REVERSE,
    OBST_CMD_TURN_LEFT,
    OBST_CMD_TURN_RIGHT
} ObstacleCommand_t;

static ObstacleState_t obst_state = OBST_STATE_IDLE;
static ObstacleCommand_t obst_cmd = OBST_CMD_NONE;
static uint32_t obst_state_start_ms = 0;

/* ================= Initialization ================= */

void MotorHW_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   /* Right Fwd */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   /* unused */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);   /* Left Fwd */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);   /* Left Rev */
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);  /* Right Rev */

    Drive_Stop();
}

/* ================= Basic Drive Functions ================= */

void Drive_Forward(uint16_t pwm)
{
    pwm = clamp_pwm(pwm);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm);  /* Left Fwd */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);    /* Left Rev */

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);  /* Right Fwd */
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);   /* Right Rev */
}

void Drive_Reverse(uint16_t pwm)
{
    pwm = clamp_pwm(pwm);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);    /* Left Fwd */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm);  /* Left Rev */

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);    /* Right Fwd */
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pwm); /* Right Rev */
}

void Drive_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);    /* Left Fwd */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);    /* Left Rev */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);    /* Right Fwd */
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);   /* Right Rev */
}

void Drive_Left(uint16_t pwm)
{
    pwm = clamp_pwm(pwm);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);    /* Left Fwd */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm);  /* Left Rev */

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);  /* Right Fwd */
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);   /* Right Rev */
}

void Drive_Right(uint16_t pwm)
{
    pwm = clamp_pwm(pwm);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm);  /* Left Fwd */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);    /* Left Rev */

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);    /* Right Fwd */
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pwm); /* Right Rev */
}

void Turn_Left(uint16_t pwm)
{
    Drive_Left(pwm);
}

void Turn_Right(uint16_t pwm)
{
    Drive_Right(pwm);
}

void Drive_Differential(uint16_t left_pwm, uint16_t right_pwm)
{
    left_pwm  = clamp_pwm(left_pwm);
    right_pwm = clamp_pwm(right_pwm);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, left_pwm);   /* Left Fwd */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);          /* Left Rev */

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, right_pwm);  /* Right Fwd */
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);         /* Right Rev */
}

void Drive_Angle(int16_t angle_deg, uint16_t base_pwm)
{
    if (angle_deg > ANGLE_DEADBAND_DEG)
        Drive_Left(base_pwm);
    else if (angle_deg < -ANGLE_DEADBAND_DEG)
        Drive_Right(base_pwm);
    else
        Drive_Forward(base_pwm);
}

/* ================= UWB Helper ================= */

static uint8_t UWB_ReadRaw(uint16_t *dist_cm, int16_t *angle_error_deg)
{
    uint16_t raw_dist_cm   = remote_pos.distance_cm;
    int16_t  raw_angle_deg = remote_pos.angle_deg;
    uint8_t  raw_valid     = remote_pos.valid;

    uint8_t measurement_ok =
        (raw_valid != 0) &&
        (raw_dist_cm >= UWB_MIN_VALID_CM) &&
        (raw_dist_cm <= UWB_MAX_VALID_CM) &&
        (raw_angle_deg >= 0) &&
        (raw_angle_deg <= 180);

    if (!measurement_ok)
        return 0;

    *dist_cm = raw_dist_cm;
    *angle_error_deg = raw_angle_deg - ANGLE_CENTER_DEG;

    return 1;
}

/* ================= Obstacle Action Helpers ================= */

static void Obstacle_StartAction(ObstacleCommand_t cmd)
{
    obst_cmd = cmd;
    obst_state = OBST_STATE_STOP_BEFORE;
    obst_state_start_ms = HAL_GetTick();
    Drive_Stop();
}

static uint8_t Obstacle_RunStateMachine(void)
{
    uint32_t now = HAL_GetTick();

    if (obst_state == OBST_STATE_IDLE)
        return 0;

    switch (obst_state)
    {
        case OBST_STATE_STOP_BEFORE:
            Drive_Stop();
            if ((now - obst_state_start_ms) >= OBST_STOP_BEFORE_MS)
            {
                obst_state = OBST_STATE_ACTION;
                obst_state_start_ms = now;
            }
            return 1;

        case OBST_STATE_ACTION:
            if (obst_cmd == OBST_CMD_REVERSE)
            {
                Drive_Reverse(MOTOR_PWM_REVERSE);
            }
            else if (obst_cmd == OBST_CMD_TURN_LEFT)
            {
                Drive_Left(MOTOR_PWM_TURN);
            }
            else if (obst_cmd == OBST_CMD_TURN_RIGHT)
            {
                Drive_Right(MOTOR_PWM_TURN);
            }
            else
            {
                Drive_Stop();
            }

            if ((now - obst_state_start_ms) >= OBST_ACTION_MS)
            {
                obst_state = OBST_STATE_STOP_AFTER;
                obst_state_start_ms = now;
                Drive_Stop();
            }
            return 1;

        case OBST_STATE_STOP_AFTER:
            Drive_Stop();
            if ((now - obst_state_start_ms) >= OBST_STOP_AFTER_MS)
            {
                obst_state = OBST_STATE_IDLE;
                obst_cmd = OBST_CMD_NONE;
            }
            return 1;

        default:
            obst_state = OBST_STATE_IDLE;
            obst_cmd = OBST_CMD_NONE;
            Drive_Stop();
            return 0;
    }
}

/* ================= Normal follow ================= */

static void MotorHW_FollowUserStep(void)
{
    uint16_t dist_cm;
    int16_t angle_error_deg;
    int16_t abs_err;
    int32_t base_pwm;
    int32_t steer_pwm;
    int32_t extra_err;
    int32_t left_pwm;
    int32_t right_pwm;

    if (!UWB_ReadRaw(&dist_cm, &angle_error_deg))
    {
        Drive_Stop();
        return;
    }

    abs_err = abs_i16(angle_error_deg);

    if (dist_cm <= FOLLOW_STOP_CM)
    {
        follow_stop_latched = 1;
        Drive_Stop();
        return;
    }

    if (follow_stop_latched)
    {
        if (dist_cm <= FOLLOW_RESUME_CM)
        {
            Drive_Stop();
            return;
        }
        follow_stop_latched = 0;
    }

    base_pwm = FOLLOW_PWM_BASE;

    if (dist_cm < 150)
        base_pwm = 34000;
    else if (dist_cm < 220)
        base_pwm = 39000;

    if (abs_err <= ANGLE_DEADBAND_DEG)
    {
        Drive_Differential((uint16_t)base_pwm, (uint16_t)base_pwm);
        return;
    }

    if (angle_error_deg > ANGLE_MAX_ERROR_DEG) angle_error_deg = ANGLE_MAX_ERROR_DEG;
    if (angle_error_deg < -ANGLE_MAX_ERROR_DEG) angle_error_deg = -ANGLE_MAX_ERROR_DEG;
    abs_err = abs_i16(angle_error_deg);

    steer_pwm = (int32_t)abs_err * STEER_GAIN_BASE;

    if (abs_err > STEER_EXTRA_START)
    {
        extra_err = abs_err - STEER_EXTRA_START;
        steer_pwm += extra_err * STEER_GAIN_EXTRA;
    }

    if (steer_pwm > STEER_MAX_PWM)
        steer_pwm = STEER_MAX_PWM;

    if (angle_error_deg > 0)
    {
        left_pwm  = base_pwm - steer_pwm;
        right_pwm = base_pwm + steer_pwm;
    }
    else
    {
        left_pwm  = base_pwm + steer_pwm;
        right_pwm = base_pwm - steer_pwm;
    }

    if (left_pwm < FOLLOW_PWM_MIN)   left_pwm = FOLLOW_PWM_MIN;
    if (right_pwm < FOLLOW_PWM_MIN)  right_pwm = FOLLOW_PWM_MIN;

    if (left_pwm > FOLLOW_PWM_MAX)   left_pwm = FOLLOW_PWM_MAX;
    if (right_pwm > FOLLOW_PWM_MAX)  right_pwm = FOLLOW_PWM_MAX;

    Drive_Differential((uint16_t)left_pwm, (uint16_t)right_pwm);
}

/* ================= Main Motor Logic ================= */

void MotorHW_TaskStep(void)
{
    uint16_t left   = safe_distance(ToF_DATA[0].distance);
    uint16_t center = safe_distance(ToF_DATA[1].distance);
    uint16_t right  = safe_distance(ToF_DATA[2].distance);

    uint8_t left_blocked   = (left   < SIDE_THRESHOLD_FAR);
    uint8_t center_blocked = (center < OBSTACLE_THRESHOLD_FAR);
    uint8_t right_blocked  = (right  < SIDE_THRESHOLD_FAR);

    robotCommand.dodgeAngle = 90;

    /* standby mode = know position but don't move */
    if (standby)
    {
        Drive_Stop();
        obst_state = OBST_STATE_IDLE;
        obst_cmd = OBST_CMD_NONE;
        return;
    }

    /* If already in the middle of a stop/action/stop obstacle sequence, keep running it */
    if (Obstacle_RunStateMachine())
        return;

    /* ===== ToF obstacle avoidance gets priority, but now paced ===== */

    if (center < OBSTACLE_THRESHOLD_CLOSE)
    {
        robotCommand.dodgeAngle = 180;
        Obstacle_StartAction(OBST_CMD_REVERSE);
        return;
    }

    if (center_blocked)
    {
        if (left > right)
        {
            robotCommand.dodgeAngle = 135;
            Obstacle_StartAction(OBST_CMD_TURN_LEFT);
        }
        else
        {
            robotCommand.dodgeAngle = 45;
            Obstacle_StartAction(OBST_CMD_TURN_RIGHT);
        }
        return;
    }

    if (left_blocked && !right_blocked)
    {
        robotCommand.dodgeAngle = 45;
        Obstacle_StartAction(OBST_CMD_TURN_RIGHT);
        return;
    }

    if (right_blocked && !left_blocked)
    {
        robotCommand.dodgeAngle = 135;
        Obstacle_StartAction(OBST_CMD_TURN_LEFT);
        return;
    }

    if (left_blocked && right_blocked && !center_blocked)
    {
        Drive_Forward(MOTOR_PWM_FORWARD);
        robotCommand.dodgeAngle = 90;
        return;
    }

    /* ===== Normal follow ===== */
    MotorHW_FollowUserStep();
}
