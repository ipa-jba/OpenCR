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

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho, Valentin Starlinger */

#ifndef TURTLEBOT3_MECANUM_CORE_MOTOR_DRIVER_H_
#define TURTLEBOT3_MECANUM_CORE_MOTOR_DRIVER_H_

#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE 64
#define ADDR_X_GOAL_VELOCITY 104
#define ADDR_X_GOAL_POSITION 116
#define ADDR_X_REALTIME_TICK 120
#define ADDR_X_PRESENT_VELOCITY 128
#define ADDR_X_PRESENT_POSITION 132

// Limit values (XM430-W210-T)
#define LIMIT_X_MAX_VELOCITY 240

// Data Byte Length
#define LEN_X_TORQUE_ENABLE 1
#define LEN_X_GOAL_VELOCITY 4
#define LEN_X_GOAL_POSITION 4
#define LEN_X_REALTIME_TICK 2
#define LEN_X_PRESENT_VELOCITY 4
#define LEN_X_PRESENT_POSITION 4

#define PROTOCOL_VERSION 2.0  // Dynamixel protocol version 2.0

#define DXL_LEFT_REAR_ID 3    // ID of left rear motor
#define DXL_RIGHT_REAR_ID 4   // ID of right rear motor
#define DXL_LEFT_FRONT_ID 1   // ID of left front motor
#define DXL_RIGHT_FRONT_ID 2  // ID of right front motor
#define BAUDRATE 57600       // baurd rate of Dynamixel
#define DEVICENAME ""         // no need setting on OpenCR

#define TORQUE_ENABLE 1   // Value for enabling the torque
#define TORQUE_DISABLE 0  // Value for disabling the torque

#define LEFT_FRONT 0
#define RIGHT_FRONT 1
#define LEFT_REAR 2
#define RIGHT_REAR 3

#define VELOCITY_CONSTANT_VALUE                                                                                        \
  1263.632956882  // V = r * w = r * RPM * 0.10472
                  //   = 0.033 * 0.229 * Goal RPM * 0.10472
                  // Goal RPM = V * 1263.632956882

class Turtlebot3MecanumCoreMotorDriver
{
public:
  Turtlebot3MecanumCoreMotorDriver();
  Turtlebot3MecanumCoreMotorDriver(uint8_t left_front_motor_id,
                                   uint8_t right_front_motor_id,
                                   uint8_t left_rear_motor_id,
                                   uint8_t right_rear_motor_id);
  ~Turtlebot3MecanumCoreMotorDriver();
  bool init(void);
  void closeDynamixel(void);
  bool setTorque(uint8_t id, bool onoff);
  bool getTorque();
  bool readEncoder(int32_t& left_front_value,
                   int32_t& right_front_value,
                   int32_t& left_rear_value,
                   int32_t& right_rear_value);
  bool
  writeVelocity(int64_t left_front_value, int64_t right_front_value, int64_t left_rear_value, int64_t right_rear_value);
  bool controlMotor(const float r, const float wheel_separation_x, const float wheel_separation_y, float* value);

private:
  uint32_t baudrate_;
  float protocol_version_;
  uint8_t left_rear_wheel_id_;
  uint8_t right_rear_wheel_id_;
  uint8_t left_front_wheel_id_;
  uint8_t right_front_wheel_id_;
  bool torque_;

  dynamixel::PortHandler* portHandler_;
  dynamixel::PacketHandler* packetHandler_;

  dynamixel::GroupSyncWrite* groupSyncWriteVelocity_;
  dynamixel::GroupSyncRead* groupSyncReadEncoder_;
};

#endif  // TURTLEBOT3_MECANUM_CORE_MOTOR_DRIVER_H_
