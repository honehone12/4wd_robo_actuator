#ifndef SERIAL_ENDPOINT_H_
#define SERIAL_ENDPOINT_H_

#include "Arduino.h" //arduino core libralies for develop

#define SERIAL_ENDPOINT_MAX_FALSE_COUNT 255
#define SERIAL_ENDPOINT_HEADER_LEN 8
#define SERIAL_ENDPOINT_HEADER_BYTE 0xff
#define SERIAL_ENDPOINT_REQUIRE_BYTES 20
#define SERIAL_ENDPOINT_TARGET_FPS_DELAY /*67*/ 33 /*16*/ // 0.016sec=60FPS
#define SERIAL_ENDPOINT_MAX_CMD 2.99f

struct Twist
{
  float linear;
  float angular;
};

static union 
{
  const struct 
  {
    byte byte_array_len;
    byte type_size;
    byte data_len;
    byte data_type;
  };
  byte bin[4] 
  {
    0x08, // float * 2
    0x04, // size of float
    0x02, // data len = 2
    0x01 // float type
  };
} target_data_description;

static const byte header_array[]
{
  0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff
};

static union
{
  struct Twist twist;
  byte bin[8]; //float * 2
} cmd_vel_exchanger;

extern HardwareSerial Serial;

// call this end of setup()
void serialInit(unsigned long baud_rate = 115200)
{
  Serial.begin(baud_rate);

  while (!Serial)
  {
    delay(SERIAL_ENDPOINT_TARGET_FPS_DELAY);
  }
}

inline bool isSerialDataAvailable(void)
{
  if(Serial)
  {
    return Serial.available() >= SERIAL_ENDPOINT_REQUIRE_BYTES;
  }

  return false;
}

inline bool isFeedbackAvailable(void)
{
  if(Serial)
  {
    return Serial.availableForWrite() >= SERIAL_ENDPOINT_REQUIRE_BYTES;
  }

  return false;
}

bool readCMDVEL(Twist* cmd)
{
  // read header 0xff * 8
  byte header_count(0);
  byte false_count(0);
  while (header_count < SERIAL_ENDPOINT_HEADER_LEN)
  {
    // mind raw is -1 if Serial is not available.
    if(Serial.read() == SERIAL_ENDPOINT_HEADER_BYTE)
    {
      header_count++;
    }
    else
    {
      header_count = 0;
      false_count++;
      if(false_count >= SERIAL_ENDPOINT_MAX_FALSE_COUNT)
      {
        return false;
      }
    }
  }

  // read data description
  if( 
    (byte)Serial.read() == target_data_description.byte_array_len && 
    (byte)Serial.read() == target_data_description.type_size &&
    (byte)Serial.read() == target_data_description.data_len &&
    (byte)Serial.read() == target_data_description.data_type
  )
  {
    // read CMD
    Serial.readBytes(cmd_vel_exchanger.bin, 8);
    if(
      !isfinite(cmd_vel_exchanger.twist.linear) || 
      !isfinite(cmd_vel_exchanger.twist.angular)
    )
    {
      // or 9.99f ??
      cmd->linear = 0.0f;
      cmd->angular = 0.0f;
    }
    else
    {
      cmd->linear = cmd_vel_exchanger.twist.linear;
      cmd->angular = cmd_vel_exchanger.twist.angular;
    }
    return true;
  }

  return false;
}

bool writeFeedback(void)
{
  size_t total(0);
  total += Serial.write(
    header_array,
    8
  );
  total += Serial.write(
    target_data_description.bin,
    4
  );
  total += Serial.write(
    cmd_vel_exchanger.bin,
    8
  );
  if(total == SERIAL_ENDPOINT_REQUIRE_BYTES)
  {
    Serial.flush();
    return true;
  }

  return false;
}

#endif