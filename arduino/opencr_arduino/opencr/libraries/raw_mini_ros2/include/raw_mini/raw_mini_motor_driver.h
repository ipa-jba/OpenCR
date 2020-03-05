/*******************************************************************************
 * Copyright 2016 ROBOTIS CO., LTD.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

#ifndef RAW_MINI_MOTOR_DRIVER_H_
#define RAW_MINI_MOTOR_DRIVER_H_

#include <Dynamixel2Arduino.h>

enum MotorLocation
{
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  REAR_LEFT = 2,
  REAR_RIGHT = 3,
  MOTOR_NUM_MAX
};

enum VelocityType
{
  LINEAR_X = 0,
  LINEAR_Y = 1,
  ANGULAR_Z = 2,
  TYPE_NUM_MAX
};

class RawMiniMotorDriver
{
public:
  RawMiniMotorDriver();
  ~RawMiniMotorDriver();

  bool init(void);
  void close(void);

  bool is_connected();

  bool set_torque(bool onoff);
  bool get_torque();

  bool read_present_position(int32_t& front_left_value,
                             int32_t& front_right_value,
                             int32_t& rear_left_value,
                             int32_t& rear_right_value);
  bool read_present_velocity(int32_t& front_left_value,
                             int32_t& front_right_value,
                             int32_t& rear_left_value,
                             int32_t& rear_right_value);
  bool read_present_current(int16_t& front_left_value,
                            int16_t& front_right_value,
                            int16_t& rear_left_value,
                            int16_t& rear_right_value);
  bool read_profile_acceleration(uint32_t& front_left_value,
                                 uint32_t& front_right_value,
                                 uint32_t& rear_left_value,
                                 uint32_t& rear_right_value);

  bool write_velocity(int32_t front_left_value,
                      int32_t front_right_value,
                      int32_t rear_left_value,
                      int32_t rear_right_value);
  bool write_profile_acceleration(uint32_t front_left_value,
                                  uint32_t front_right_value,
                                  uint32_t rear_left_value,
                                  uint32_t rear_right_value);

  bool control_motors(float front_left_command,
                      float front_right_command,
                      float rear_left_command,
                      float rear_right_command);

private:
  uint8_t front_left_wheel_id_;
  uint8_t front_right_wheel_id_;
  uint8_t rear_left_wheel_id_;
  uint8_t rear_right_wheel_id_;
  bool torque_;
};

#endif  // RAW_MINI_MOTOR_DRIVER_H_
