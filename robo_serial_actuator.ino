#include "servo_4wd_control.h"
#include "serial_endpoint.h"

// is 1 sec too long ?? 
#define CMD_VEL_TIMEOUT 1000 //millsec

Servo fr;
Servo fl;
Servo rr;
Servo rl;
struct Servo4WD servo4wd;
struct Twist cmd;
unsigned long time_last_cmd;  

void setup()
{
  servo4wd = { &fr, &fl, &rr, &rl };
  cmd = { 0.0f, 0.0f };
  time_last_cmd = 0;
  
  servoInit(&servo4wd);
  serialInit();
}

void loop()
{
  if(isSerialDataAvailable())
  {
    unsigned long time_now(millis());
    //just in case of overflow
    if(time_now < time_last_cmd)
    {
      time_last_cmd = 0;
    }

    if(readCMDVEL(&cmd))
    {
      if(
        abs(cmd.linear) <= SERIAL_ENDPOINT_MAX_CMD && 
        abs(cmd.angular) <= SERIAL_ENDPOINT_MAX_CMD
      ) 
      {
        if(cmd.linear != 0.0f && cmd.angular == 0.0f)
        {
          straight(&servo4wd, cmd.linear);
        }
        else if(cmd.linear == 0.0f && cmd.angular != 0.0f)
        {
          neutralTurn(&servo4wd, cmd.angular);
        }
        else if (cmd.linear != 0.0f && cmd.angular != 0.0f)
        {
          orbitalMove(&servo4wd, cmd.linear, cmd.angular);
        }
        else
        {
          prepareExit(&servo4wd);
        }

        time_last_cmd = time_now;
      }
    }

    // for safety
    if(time_now - time_last_cmd > CMD_VEL_TIMEOUT)
    {
      prepareExit(&servo4wd);
    }
  }

  if(isFeedbackAvailable())
  {
    if(!writeFeedback())
    {
      prepareExit(&servo4wd);
    }
  }

  delay(SERIAL_ENDPOINT_TARGET_FPS_DELAY);
}
