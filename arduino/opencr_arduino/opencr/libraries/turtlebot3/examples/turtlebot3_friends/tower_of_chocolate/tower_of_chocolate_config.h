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

#ifndef TURTLEBOT3_MECANUM_CORE_CONFIG_H_
#define TURTLEBOT3_MECANUM_CORE_CONFIG_H_

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <TurtleBot3.h>

#include <math.h>

#include "tower_of_chocolate_motor_driver.h"

#define INIT_LOG_DATA "This core is adjusted to serve the Chocolate Overlord."

#define HARDWARE_VER "1.0.0"
#define SOFTWARE_VER "1.0.0"
#define FIRMWARE_VER "0.0.1"

#define CONTROL_MOTOR_SPEED_PERIOD 30  // hz
#define CONTROL_MOTOR_TIMEOUT 100      // ms

#define ENCODER_MIN -2147483648  // raw
#define ENCODER_MAX 2147483648   // raw

#define MAX_WHEEL_VEL 300  // 0.229 rev/60s

#define TICK2RAD 0.001533981            // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
#define RAD2VEL 41.6999850896           //
#define DEG2RAD(x) (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x) (x * 57.2957795131)  // *180/PI

// Callback function prototypes
void dispenseTower1Callback(const std_msgs::Bool& dispense_message);
void dispenseTower2Callback(const std_msgs::Bool& dispense_message);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time& t, uint32_t _micros);

void updateMotorInfo();
void updateTime(void);

void sendLogMsg(void);

int32_t motor_ids_[2] = { 1, 2 };

int32_t n_ejections_[2] = { 0, 0 };

/*******************************************************************************
 * ROS NodeHandle
 *******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
 * Subscriber
 *******************************************************************************/
ros::Subscriber<std_msgs::Bool> dispense_tower_1_cb_("gib_chocolate_1", dispenseTower1Callback);
ros::Subscriber<std_msgs::Bool> dispense_tower_2_cb_("gib_chocolate_2", dispenseTower2Callback);

/*******************************************************************************
 * Declaration for motor
 *******************************************************************************/
TowerOfChocolateMotorDriver motor_driver_;

/*******************************************************************************
 * Declaration for SLAM and navigation
 *******************************************************************************/
unsigned long prev_update_time;

/*******************************************************************************
 * Declaration for Battery
 *******************************************************************************/
bool setup_end = false;

bool button_state_[2] = {false, false};

#endif  // TURTLEBOT3_MECANUM_CORE_CONFIG_H_
