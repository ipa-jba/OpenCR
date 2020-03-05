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

/* Authors: Jannik Abbenseth*/

#include "tower_of_chocolate_config.h"

/*******************************************************************************
 * Setup function
 *******************************************************************************/
void setup()
{
  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  // nh.getHardware()->setBaud(115200);
  nh.subscribe(dispense_tower_1_cb_);
  nh.subscribe(dispense_tower_2_cb_);

  // Setting for Dynamixel motors
  const char* log_msg;

  if (motor_driver_.init(&log_msg))
  {
    nh.loginfo(log_msg);
  }
  else
  {
    nh.logerror(log_msg);
  }

  prev_update_time = millis();

  pinMode(LED_WORKING_CHECK, OUTPUT);
  pinMode(BDPIN_LED_USER_3, OUTPUT);

  pinMode(BDPIN_PUSH_SW_1, INPUT);
  pinMode(BDPIN_PUSH_SW_2, INPUT);

  // SerialBT2.begin(57600);
  setup_end = true;
}

/*******************************************************************************
 * Loop function
 *******************************************************************************/
void loop()
{
  uint32_t t = millis();
  updateTime();

  // Send log message after ROS connection
  sendLogMsg();

  // Show LED status
  // diagnosis.showLedStatus(nh.connected());

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  if (!button_state_[0] && digitalRead(BDPIN_PUSH_SW_1))
  {
    button_state_[1] = true;
    std_msgs::Bool msg;
    msg.data = true;
    dispenseTower1Callback(msg);
  }
  else
  {
    button_state_[0] = digitalRead(BDPIN_PUSH_SW_1);
  }

  if (!button_state_[1] && digitalRead(BDPIN_PUSH_SW_2))
  {
    button_state_[1] = true;
    std_msgs::Bool msg;
    msg.data = true;
    dispenseTower2Callback(msg);
  }
  else
  {
    button_state_[1] = digitalRead(BDPIN_PUSH_SW_2);
  }

  // give the serial link time to process
  delay(10);
}

void dispenseTower1Callback(const std_msgs::Bool& dispense_message)
{
  const char* log;
  char log_msg[60];

  if (!dispense_message.data)
  {
    return;
  }

  if (motor_driver_.dropItLikeItsHot(motor_ids_[0], &log))
  {
    n_ejections_[0]++;
    sprintf(log_msg, "Tower 1 ejected %d pieces of chocolate.", n_ejections_[0]);
    nh.loginfo(log_msg);
  }
  else
  {
    nh.logerror(log);
  }
}
void dispenseTower2Callback(const std_msgs::Bool& dispense_message)
{
  const char* log;
  char log_msg[60];

  if (!dispense_message.data)
  {
    return;
  }

  if (motor_driver_.dropItLikeItsHot(motor_ids_[1], &log))
  {
    n_ejections_[1]++;
    sprintf(log_msg, "Tower 2 ejected %d pieces of chocolate.", n_ejections_[1]);
    nh.loginfo(log_msg);
  }
  else
  {
    nh.logerror(log);
  }
}

/*******************************************************************************
 * Update the base time for interpolation
 *******************************************************************************/
void updateTime()
{
  current_offset = micros();
  current_time = nh.now();
}

/*******************************************************************************
 * ros::Time::now() implementation
 *******************************************************************************/
ros::Time rosNow()
{
  return addMicros(current_time, micros() - current_offset);
}

/*******************************************************************************
 * Send log message
 *******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[50];
  const char* init_log_data = INIT_LOG_DATA;

  if (nh.connected())
  {
    if (log_flag == false)
    {
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      sprintf(log_msg, "Connected to OpenCR board!");
      nh.loginfo(log_msg);

      sprintf(log_msg, init_log_data);
      nh.loginfo(log_msg);

      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      log_flag = true;
    }
  }
  else
  {
    log_flag = false;
  }
}
