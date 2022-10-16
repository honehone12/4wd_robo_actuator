#ifndef SERVO_4WD_CONTROL_H_
#define SERVO_4WD_CONTROL_H_

#include "Servo.h"

#define SERVO_PULSE_MAX_FR 2300
#define SERVO_PULSE_MIN_FR 700
#define SERVO_PULSE_MAX_FL 2300
#define SERVO_PULSE_MIN_FL 700
#define SERVO_PULSE_MAX_RR 2300
#define SERVO_PULSE_MIN_RR 700
#define SERVO_PULSE_MAX_RL 2300
#define SERVO_PULSE_MIN_RL 700

#define PWM_OUT_FR 9
#define PWM_OUT_FL 10
#define PWM_OUT_RR 5
#define PWM_OUT_RL 6

#define WHEEL_DIR_R -1.0f
#define WHEEL_DIR_L 1.0f
#define ROTATE_DIR -1.0f

#define STOPPING 90
#define SERVO_WRITE_RANGE 180

#define MAX_LINEAR_VELOCITY_ABS 1.0f // always > 0.0
#define MAX_ANGULAR_VELOCITY_ABS 2.0f //always > 0.0

#define NUETRAL_SPIN_WEIGHT 0.5f //0.0~1.0
#define ORBITAL_SPIN_WEIGHT 0.5f //0.0~1.0

#define SAFETY_LINEAR 0.5f //0.0~1.0
#define SAFETY_ANGULAR 0.5f //0.0~1.0

struct Servo4WD
{
  Servo* fr;
  Servo* fl;
  Servo* rr;
  Servo* rl;
};

inline float clamp(float value, float min, float max)
{
  value = value > max ? max : value;
  value = value < min ? min : value;
  return value;
}

void servoInit(const Servo4WD* servo4wd)
{
  servo4wd->fr->attach(
    PWM_OUT_FR,
    SERVO_PULSE_MIN_FR,
    SERVO_PULSE_MAX_FR
  );
  servo4wd->fr->write(STOPPING);
  servo4wd->fl->attach(
    PWM_OUT_FL,
    SERVO_PULSE_MIN_FL,
    SERVO_PULSE_MAX_FL
  );
  servo4wd->fl->write(STOPPING);
  servo4wd->rr->attach(
    PWM_OUT_RR,
    SERVO_PULSE_MIN_RR,
    SERVO_PULSE_MAX_RR
  );
  servo4wd->rr->write(STOPPING);
  servo4wd->rl->attach(
    PWM_OUT_RL,
    SERVO_PULSE_MIN_RL,
    SERVO_PULSE_MAX_RL
  );
  servo4wd->rl->write(STOPPING);
}

void prepareExit(const Servo4WD* servo4wd)
{
  servo4wd->fr->write(STOPPING);
  servo4wd->rr->write(STOPPING);
  servo4wd->fl->write(STOPPING);
  servo4wd->rl->write(STOPPING);
}

inline void normalizedWrite(Servo* servo_ptr, float normalized_value)
{
  int write_value(
    (normalized_value + 1) / 2 * SERVO_WRITE_RANGE
  );
  servo_ptr->write(write_value);
}

void straight(const Servo4WD* servo4wd, float cmd_linear)
{
  cmd_linear = clamp(
    cmd_linear,
    -MAX_LINEAR_VELOCITY_ABS,
    MAX_LINEAR_VELOCITY_ABS
  ) / MAX_LINEAR_VELOCITY_ABS;
  float cmd_r(cmd_linear * WHEEL_DIR_R);
  float cmd_l(cmd_linear * WHEEL_DIR_L);

  normalizedWrite(servo4wd->fr, cmd_r);
  normalizedWrite(servo4wd->fl, cmd_l);
  normalizedWrite(servo4wd->rr, cmd_r);
  normalizedWrite(servo4wd->rl, cmd_l);
}

void neutralTurn(const Servo4WD* servo4wd, float cmd_angular)
{
  cmd_angular = clamp(
    cmd_angular,
    -MAX_ANGULAR_VELOCITY_ABS,
    MAX_ANGULAR_VELOCITY_ABS
  ) / MAX_ANGULAR_VELOCITY_ABS * NUETRAL_SPIN_WEIGHT * ROTATE_DIR;

  normalizedWrite(servo4wd->fr, cmd_angular);
  normalizedWrite(servo4wd->fl, cmd_angular);
  normalizedWrite(servo4wd->rr, cmd_angular);
  normalizedWrite(servo4wd->rl, cmd_angular);
}

void orbitalMove(const Servo4WD* servo4wd, float cmd_linear, float cmd_angular)
{
  cmd_linear = clamp(
    cmd_linear,
    -MAX_LINEAR_VELOCITY_ABS,
    MAX_LINEAR_VELOCITY_ABS
  ) / MAX_LINEAR_VELOCITY_ABS * SAFETY_LINEAR;
  cmd_angular = clamp(
    cmd_angular,
    -MAX_ANGULAR_VELOCITY_ABS,
    MAX_ANGULAR_VELOCITY_ABS
  ) / MAX_ANGULAR_VELOCITY_ABS * SAFETY_ANGULAR * ORBITAL_SPIN_WEIGHT * ROTATE_DIR;
  float cmd_r((cmd_linear - cmd_angular) * WHEEL_DIR_R);
  float cmd_l((cmd_linear + cmd_angular) * WHEEL_DIR_L);

  normalizedWrite(servo4wd->fr, cmd_r);
  normalizedWrite(servo4wd->fl, cmd_l);
  normalizedWrite(servo4wd->rr, cmd_r);
  normalizedWrite(servo4wd->rl, cmd_l);
}

#endif
