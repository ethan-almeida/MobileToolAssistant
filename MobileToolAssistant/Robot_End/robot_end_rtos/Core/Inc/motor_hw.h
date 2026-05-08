#ifndef MOTOR_HW_H
#define MOTOR_HW_H

#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>

/* ================= PWM Limits ================= */

/* Must match your timer period = 65535 */
#define PWM_MAX 65535

/* ================= Robot Command ================= */

typedef struct
{
    int16_t dodgeAngle;
    int16_t followAngle;
    bool dodge;
    bool standby;
    uint16_t speed;
} RobotCommand_t;

extern RobotCommand_t robotCommand;

/* ================= Motor Control API ================= */

void MotorHW_Init(void);
void MotorHW_StopAll(void);

/* ================= High Level Drive ================= */

void Drive_Forward(uint16_t pwm);
void Drive_Reverse(uint16_t pwm);
void Drive_Stop(void);
void Drive_Left(uint16_t pwm);
void Drive_Right(uint16_t pwm);
void Turn_Left(uint16_t pwm);
void Turn_Right(uint16_t pwm);
void Drive_Angle(int16_t angle_deg, uint16_t base_pwm);
void Drive_Differential(uint16_t left_pwm, uint16_t right_pwm);

/* ================= Task Step ================= */

void MotorHW_TaskStep(void);

#endif
