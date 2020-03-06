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

#include "../../include/raw_mini/raw_mini.h"

/*******************************************************************************
 * Definition of dependency data according to RawMini model.
 *******************************************************************************/
typedef struct RawMiniModelInfo
{
  const char* model_str;
  uint32_t model_info;
  float wheel_radius;
  float wheelbase;  // for mecanum control: wheelbase is from one wheel to the other (mecanum formula uses lx =
                    // wheelbase/2)
  float track;      // for mecanum control: track is from one wheel to the other (mecanum formula uses ly = track/2)
} RawMiniModelInfo;

static const RawMiniModelInfo gen_zero_info = {
  "GenZero", 1, 0.045, 0.23, 0.18,
};

/*******************************************************************************
 * Declaration for motors
 *******************************************************************************/
static RawMiniMotorDriver motor_driver;

static const RawMiniModelInfo* p_model_info;
static float max_rot_vel;

static float goal_velocity[MotorLocation::MOTOR_NUM_MAX] = { 0.0, 0.0, 0.0, 0.0 };

static void update_goal_velocity_from_3values(void);
static bool get_connection_state_with_motors();
static void set_connection_state_with_motors(bool is_connected);

/*******************************************************************************
 * Declaration for sensors
 *******************************************************************************/
static Turtlebot3Sensor sensors;

/*******************************************************************************
 * Declaration for diagnosis
 *******************************************************************************/
static Turtlebot3Diagnosis diagnosis;

/*******************************************************************************
 * Declaration for DYNAMIXEL Slave Function
 *******************************************************************************/
#define SERIAL_DXL_SLAVE Serial
const uint8_t ID_DXL_SLAVE = 200;
const uint16_t MODEL_NUM_DXL_SLAVE = 0x5042;
const float PROTOCOL_VERSION_DXL_SLAVE = 2.0;
const uint32_t HEARTBEAT_TIMEOUT_MS = 500;

static void dxl_slave_write_callback_func(uint16_t addr, uint8_t& dxl_err_code, void* arg);

static bool get_connection_state_with_ros2_node();
static void set_connection_state_with_ros2_node(bool is_connected);
static void update_connection_state_with_ros2_node();

static void update_imu(uint32_t interval_ms);
static void update_times(uint32_t interval_ms);
static void update_gpios(uint32_t interval_ms);
static void update_motor_status(uint32_t interval_ms);
static void update_battery_status(uint32_t interval_ms);
static void update_analog_sensors(uint32_t interval_ms);

DYNAMIXEL::USBSerialPortHandler port_dxl_slave(SERIAL_DXL_SLAVE);
DYNAMIXEL::Slave dxl_slave(port_dxl_slave, MODEL_NUM_DXL_SLAVE);

enum ControlTableItemAddr
{

  ADDR_MILLIS = 10,
  ADDR_MICROS = 14,

  ADDR_DEVICE_STATUS = 18,
  ADDR_HEARTBEAT = 19,

  ADDR_USER_LED_1 = 20,
  ADDR_USER_LED_2 = 21,
  ADDR_USER_LED_3 = 22,
  ADDR_USER_LED_4 = 23,
  ADDR_BUTTON_1 = 26,
  ADDR_BUTTON_2 = 27,
  ADDR_BUMPER_1 = 28,
  ADDR_BUMPER_2 = 29,

  ADDR_ILLUMINATION = 30,
  ADDR_IR = 34,
  ADDR_SORNA = 38,

  ADDR_BATTERY_VOLTAGE = 42,
  ADDR_BATTERY_PERCENT = 46,

  ADDR_SOUND = 50,

  ADDR_IMU_RECALIBRATION = 59,
  ADDR_ANGULAR_VELOCITY_X = 60,
  ADDR_ANGULAR_VELOCITY_Y = 64,
  ADDR_ANGULAR_VELOCITY_Z = 68,
  ADDR_LINEAR_ACC_X = 72,
  ADDR_LINEAR_ACC_Y = 76,
  ADDR_LINEAR_ACC_Z = 80,
  ADDR_MAGNETIC_X = 84,
  ADDR_MAGNETIC_Y = 88,
  ADDR_MAGNETIC_Z = 92,
  ADDR_ORIENTATION_W = 96,
  ADDR_ORIENTATION_X = 100,
  ADDR_ORIENTATION_Y = 104,
  ADDR_ORIENTATION_Z = 108,

  ADDR_PRESENT_CURRENT_FL = 120,
  ADDR_PRESENT_CURRENT_FR = 124,
  ADDR_PRESENT_CURRENT_RL = 128,
  ADDR_PRESENT_CURRENT_RR = 132,
  ADDR_PRESENT_VELOCITY_FL = 136,
  ADDR_PRESENT_VELOCITY_FR = 140,
  ADDR_PRESENT_VELOCITY_RL = 144,
  ADDR_PRESENT_VELOCITY_RR = 148,
  ADDR_PRESENT_POSITION_FL = 152,
  ADDR_PRESENT_POSITION_FR = 156,
  ADDR_PRESENT_POSITION_RL = 160,
  ADDR_PRESENT_POSITION_RR = 164,

  ADDR_MOTOR_TORQUE = 169,
  ADDR_CMD_VEL_FRONT_LEFT = 170,
  ADDR_CMD_VEL_FRONT_RIGHT = 174,
  ADDR_CMD_VEL_REAR_LEFT = 178,
  ADDR_CMD_VEL_REAR_RIGHT = 182,

  ADDR_PROFILE_ACC_FL = 190,
  ADDR_PROFILE_ACC_FR = 194,
  ADDR_PROFILE_ACC_RL = 198,
  ADDR_PROFILE_ACC_RR = 202
};

typedef struct ControlItemVariables
{
  uint32_t model_inform;

  uint32_t dev_time_millis;
  uint32_t dev_time_micros;

  int8_t device_status;
  uint8_t heart_beat;

  bool user_led[4];
  bool push_button[2];
  bool bumper[2];

  uint16_t illumination;
  uint32_t ir_sensor;
  float sornar;

  uint32_t bat_voltage_x100;
  uint32_t bat_percent_x100;

  uint8_t buzzer_sound;

  bool imu_recalibration;
  float angular_vel[3];
  float linear_acc[3];
  float magnetic[3];
  float orientation[4];

  int32_t present_position[MotorLocation::MOTOR_NUM_MAX];
  int32_t present_velocity[MotorLocation::MOTOR_NUM_MAX];
  int32_t present_current[MotorLocation::MOTOR_NUM_MAX];

  bool motor_torque_enable_state;
  int32_t cmd_vel_rot[MotorLocation::MOTOR_NUM_MAX];
  uint32_t profile_acceleration[MotorLocation::MOTOR_NUM_MAX];
} ControlItemVariables;

static ControlItemVariables control_items;

/*******************************************************************************
 * Definition for RawMiniCore 'begin()' function
 *******************************************************************************/
void RawMiniCore::begin(const char* model_name)
{
  uint16_t model_motor_rpm;

  p_model_info = &gen_zero_info;
  model_motor_rpm = 77;

  max_rot_vel = 2 * PI * model_motor_rpm / 60;
  port_dxl_slave.begin();
  bool ret;
  (void)ret;
  DEBUG_SERIAL_BEGIN(57600);
  // Setting for Dynamixel motors
  ret = motor_driver.init();
  DEBUG_PRINTLN(ret == true ? "Motor driver setup completed." : "Motor driver setup failed.");
  // Setting for IMU
  ret = sensors.init();
  DEBUG_PRINTLN(ret == true ? "Sensors setup completed." : "Sensors setup failed.");
  // Init diagnosis
  ret = diagnosis.init();
  DEBUG_PRINTLN(ret == true ? "Diagnosis setup completed." : "Diagnosis setup failed.");

  // Init DXL Slave function
  dxl_slave.setPortProtocolVersion(PROTOCOL_VERSION_DXL_SLAVE);
  dxl_slave.setFirmwareVersion(FIRMWARE_VER);
  dxl_slave.setID(ID_DXL_SLAVE);

  /* Add control items for Slave */
  // Items for model information of device
  control_items.model_inform = p_model_info->model_info;
  // dxl_slave.addControlItem(ADDR_MODEL_INFORM, control_items.model_inform);
  // Items for Timer of device
  dxl_slave.addControlItem(ADDR_MILLIS, control_items.dev_time_millis);
  dxl_slave.addControlItem(ADDR_MICROS, control_items.dev_time_micros);
  // Items to inform device status
  dxl_slave.addControlItem(ADDR_DEVICE_STATUS, control_items.device_status);
  // Items to check connection state with node
  dxl_slave.addControlItem(ADDR_HEARTBEAT, control_items.heart_beat);
  // Items for GPIO
  dxl_slave.addControlItem(ADDR_USER_LED_1, control_items.user_led[0]);
  dxl_slave.addControlItem(ADDR_USER_LED_2, control_items.user_led[1]);
  dxl_slave.addControlItem(ADDR_USER_LED_3, control_items.user_led[2]);
  dxl_slave.addControlItem(ADDR_USER_LED_4, control_items.user_led[3]);
  dxl_slave.addControlItem(ADDR_BUTTON_1, control_items.push_button[0]);
  dxl_slave.addControlItem(ADDR_BUTTON_2, control_items.push_button[1]);
  dxl_slave.addControlItem(ADDR_BUMPER_1, control_items.bumper[0]);
  dxl_slave.addControlItem(ADDR_BUMPER_2, control_items.bumper[1]);
  // Items for Analog sensors
  dxl_slave.addControlItem(ADDR_ILLUMINATION, control_items.illumination);
  dxl_slave.addControlItem(ADDR_IR, control_items.ir_sensor);
  dxl_slave.addControlItem(ADDR_SORNA, control_items.sornar);
  // Items for Battery
  dxl_slave.addControlItem(ADDR_BATTERY_VOLTAGE, control_items.bat_voltage_x100);
  dxl_slave.addControlItem(ADDR_BATTERY_PERCENT, control_items.bat_percent_x100);
  // Items for Buzzer
  dxl_slave.addControlItem(ADDR_SOUND, control_items.buzzer_sound);
  // Items for IMU
  dxl_slave.addControlItem(ADDR_IMU_RECALIBRATION, control_items.imu_recalibration);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_X, control_items.angular_vel[0]);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_Y, control_items.angular_vel[1]);
  dxl_slave.addControlItem(ADDR_ANGULAR_VELOCITY_Z, control_items.angular_vel[2]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_X, control_items.linear_acc[0]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_Y, control_items.linear_acc[1]);
  dxl_slave.addControlItem(ADDR_LINEAR_ACC_Z, control_items.linear_acc[2]);
  dxl_slave.addControlItem(ADDR_MAGNETIC_X, control_items.magnetic[0]);
  dxl_slave.addControlItem(ADDR_MAGNETIC_Y, control_items.magnetic[1]);
  dxl_slave.addControlItem(ADDR_MAGNETIC_Z, control_items.magnetic[2]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_W, control_items.orientation[0]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_X, control_items.orientation[1]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_Y, control_items.orientation[2]);
  dxl_slave.addControlItem(ADDR_ORIENTATION_Z, control_items.orientation[3]);
  // Items to check status of motors
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_FL, control_items.present_position[MotorLocation::FRONT_LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_FR, control_items.present_position[MotorLocation::FRONT_RIGHT]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_RL, control_items.present_position[MotorLocation::REAR_LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_CURRENT_RR, control_items.present_position[MotorLocation::REAR_RIGHT]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_FL, control_items.present_velocity[MotorLocation::FRONT_LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_FR, control_items.present_velocity[MotorLocation::FRONT_RIGHT]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_RL, control_items.present_velocity[MotorLocation::REAR_LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_VELOCITY_RR, control_items.present_velocity[MotorLocation::REAR_RIGHT]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_FL, control_items.present_current[MotorLocation::FRONT_LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_FR, control_items.present_current[MotorLocation::FRONT_RIGHT]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_RL, control_items.present_current[MotorLocation::REAR_LEFT]);
  dxl_slave.addControlItem(ADDR_PRESENT_POSITION_RR, control_items.present_current[MotorLocation::REAR_RIGHT]);
  // Items to control motors
  dxl_slave.addControlItem(ADDR_MOTOR_TORQUE, control_items.motor_torque_enable_state);

  dxl_slave.addControlItem(ADDR_CMD_VEL_FRONT_LEFT, control_items.cmd_vel_rot[MotorLocation::FRONT_LEFT]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_FRONT_RIGHT, control_items.cmd_vel_rot[MotorLocation::FRONT_RIGHT]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_REAR_LEFT, control_items.cmd_vel_rot[MotorLocation::REAR_LEFT]);
  dxl_slave.addControlItem(ADDR_CMD_VEL_REAR_RIGHT, control_items.cmd_vel_rot[MotorLocation::REAR_RIGHT]);

  dxl_slave.addControlItem(ADDR_PROFILE_ACC_FL, control_items.profile_acceleration[MotorLocation::FRONT_LEFT]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_FR, control_items.profile_acceleration[MotorLocation::FRONT_RIGHT]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_RL, control_items.profile_acceleration[MotorLocation::REAR_LEFT]);
  dxl_slave.addControlItem(ADDR_PROFILE_ACC_RR, control_items.profile_acceleration[MotorLocation::REAR_RIGHT]);

  // Set user callback function for processing write command from master.
  dxl_slave.setWriteCallbackFunc(dxl_slave_write_callback_func);

  // Check connection state with motors.
  if (motor_driver.is_connected() == true)
  {
    motor_driver.set_torque(true);
    control_items.device_status = STATUS_RUNNING;
    set_connection_state_with_motors(true);
  }
  else
  {
    control_items.device_status = STATUS_NOT_CONNECTED_MOTORS;
    set_connection_state_with_motors(false);
    DEBUG_PRINTLN("Can't communicate with the motor!");
    DEBUG_PRINTLN("  Please check the connection to the motor and the power supply.");
    DEBUG_PRINTLN();
    return;
  }

  // Init IMU
  sensors.initIMU();
  sensors.calibrationGyro();

  // To indicate that the initialization is complete.
  sensors.makeMelody(1);
}

/*******************************************************************************
 * Definition for RawMiniCore 'run()' function
 *******************************************************************************/
void RawMiniCore::run()
{
  static uint32_t pre_time_to_control_motor;

  // Check connection state with ROS2 node
  update_connection_state_with_ros2_node();

  /* For diagnosis */
  // Show LED status
  diagnosis.showLedStatus(get_connection_state_with_ros2_node());
  // Update Voltage
  diagnosis.updateVoltageCheck(true);

  /* For sensing and run buzzer */
  // Update the IMU unit
  sensors.updateIMU();
  // Update sonar data
  // TODO: sensors.updateSonar(t);
  // Run buzzer if there is still melody to play.
  sensors.onMelody();

  /* For processing DYNAMIXEL slave function */
  // Update control table of OpenCR to communicate with ROS2 node
  update_imu(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_times(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_gpios(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_motor_status(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_battery_status(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);
  update_analog_sensors(INTERVAL_MS_TO_UPDATE_CONTROL_ITEM);

  // Packet processing with ROS2 Node.
  dxl_slave.processPacket();

  /* For controlling DYNAMIXEL motors (Wheels) */
  if (millis() - pre_time_to_control_motor >= INTERVAL_MS_TO_CONTROL_MOTOR)
  {
    pre_time_to_control_motor = millis();
    if (get_connection_state_with_ros2_node() == false)
    {
      memset(goal_velocity, 0, sizeof(goal_velocity));
    }
    if (get_connection_state_with_motors() == true)
    {
      motor_driver.control_motors(goal_velocity[MotorLocation::FRONT_LEFT],
                                  goal_velocity[MotorLocation::FRONT_RIGHT],
                                  goal_velocity[MotorLocation::REAR_LEFT],
                                  goal_velocity[MotorLocation::REAR_RIGHT]);
    }
  }
}

/*******************************************************************************
 * Function definition for updating velocity values
 * to be used for control of DYNAMIXEL(motors).
 *******************************************************************************/
void update_goal_velocity_from_3values(void)
{
  sensors.setLedPattern(goal_velocity[VelocityType::LINEAR_X], goal_velocity[VelocityType::ANGULAR_Z]);
}

/*******************************************************************************
 * Function definition for updating control items in RawMini.
 *******************************************************************************/
float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void update_times(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if (millis() - pre_time >= interval_ms)
  {
    pre_time = millis();

    control_items.dev_time_millis = millis();
    control_items.dev_time_micros = micros();
  }
}

void update_gpios(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if (millis() - pre_time >= interval_ms)
  {
    pre_time = millis();

    control_items.user_led[0] = digitalRead(BDPIN_GPIO_4);
    control_items.user_led[1] = digitalRead(BDPIN_GPIO_6);
    control_items.user_led[2] = digitalRead(BDPIN_GPIO_8);
    control_items.user_led[3] = digitalRead(BDPIN_GPIO_10);

    control_items.push_button[0] = digitalRead(BDPIN_PUSH_SW_1);
    control_items.push_button[1] = digitalRead(BDPIN_PUSH_SW_2);

    control_items.bumper[0] = sensors.getBumper1State();
    control_items.bumper[1] = sensors.getBumper2State();
  }
}

void update_battery_status(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;
  float bat_voltage, bat_percent;

  if (millis() - pre_time >= interval_ms)
  {
    pre_time = millis();

    bat_voltage = sensors.checkVoltage();
    control_items.bat_voltage_x100 = (uint32_t)(bat_voltage * 100);

    if (bat_voltage >= 3.5 * 3)
    {
      bat_percent = map_float(bat_voltage, 3.5 * 3, 4.1 * 3, 0.0, 100.0);
      control_items.bat_percent_x100 = (uint32_t)(bat_percent * 100);
    }
  }
}

void update_analog_sensors(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;

  if (millis() - pre_time >= interval_ms)
  {
    pre_time = millis();

    control_items.illumination = (uint16_t)sensors.getIlluminationData();
    control_items.ir_sensor = (uint32_t)sensors.getIRsensorData();
    control_items.sornar = (float)sensors.getSonarData();
  }
}

void update_imu(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;
  float* p_imu_data;

  if (millis() - pre_time >= interval_ms)
  {
    pre_time = millis();

    p_imu_data = sensors.getImuAngularVelocity();
    memcpy(control_items.angular_vel, p_imu_data, sizeof(control_items.angular_vel));

    p_imu_data = sensors.getImuLinearAcc();
    memcpy(control_items.linear_acc, p_imu_data, sizeof(control_items.linear_acc));

    p_imu_data = sensors.getImuMagnetic();
    memcpy(control_items.magnetic, p_imu_data, sizeof(control_items.magnetic));

    p_imu_data = sensors.getOrientation();
    memcpy(control_items.orientation, p_imu_data, sizeof(control_items.orientation));
  }
}

void update_motor_status(uint32_t interval_ms)
{
  static uint32_t pre_time;
  int16_t current_fl, current_fr, current_rl, current_rr;

  if (millis() - pre_time >= interval_ms)
  {
    pre_time = millis();

    if (get_connection_state_with_motors() == true)
    {
      motor_driver.read_present_position(control_items.present_position[MotorLocation::FRONT_LEFT],
                                         control_items.present_position[MotorLocation::FRONT_RIGHT],
                                         control_items.present_position[MotorLocation::REAR_LEFT],
                                         control_items.present_position[MotorLocation::REAR_RIGHT]);
      motor_driver.read_present_velocity(control_items.present_velocity[MotorLocation::FRONT_LEFT],
                                         control_items.present_velocity[MotorLocation::FRONT_RIGHT],
                                         control_items.present_velocity[MotorLocation::REAR_LEFT],
                                         control_items.present_velocity[MotorLocation::REAR_RIGHT]);
      if (motor_driver.read_present_current(current_fl, current_fr, current_rl, current_rr) == true)
      {
        control_items.present_current[MotorLocation::FRONT_LEFT] = current_fl;
        control_items.present_current[MotorLocation::FRONT_RIGHT] = current_fr;
        control_items.present_current[MotorLocation::REAR_LEFT] = current_rl;
        control_items.present_current[MotorLocation::REAR_RIGHT] = current_rr;
      }

      control_items.motor_torque_enable_state = motor_driver.get_torque();
    }
  }
}

/*******************************************************************************
 * Callback function definition to be used in communication with the ROS2 node.
 *******************************************************************************/
static void dxl_slave_write_callback_func(uint16_t item_addr, uint8_t& dxl_err_code, void* arg)
{
  (void)arg;

  switch (item_addr)
  {
      // case ADDR_MODEL_INFORM:
      // control_items.model_inform = p_model_info->model_info;
      // dxl_err_code = 0x07;
      // break;

    case ADDR_SOUND:
      sensors.makeMelody(control_items.buzzer_sound);
      break;

    case ADDR_IMU_RECALIBRATION:
      if (control_items.imu_recalibration == true)
      {
        sensors.calibrationGyro();
        control_items.imu_recalibration = false;
      }
      break;

    case ADDR_MOTOR_TORQUE:
      if (get_connection_state_with_motors() == true)
        motor_driver.set_torque(control_items.motor_torque_enable_state);
      break;

    case ADDR_CMD_VEL_FRONT_LEFT:
      goal_velocity[MotorLocation::FRONT_LEFT] =
          constrain((float)(control_items.cmd_vel_rot[MotorLocation::FRONT_LEFT]), -max_rot_vel, max_rot_vel);
      break;

    case ADDR_CMD_VEL_FRONT_RIGHT:
      goal_velocity[MotorLocation::FRONT_RIGHT] =
          constrain((float)(control_items.cmd_vel_rot[MotorLocation::FRONT_RIGHT]), -max_rot_vel, max_rot_vel);
      break;
    case ADDR_CMD_VEL_REAR_LEFT:
      goal_velocity[MotorLocation::REAR_LEFT] =
          constrain((float)(control_items.cmd_vel_rot[MotorLocation::REAR_LEFT]), -max_rot_vel, max_rot_vel);
      break;

    case ADDR_CMD_VEL_REAR_RIGHT:
      goal_velocity[MotorLocation::REAR_RIGHT] =
          constrain((float)(control_items.cmd_vel_rot[MotorLocation::REAR_RIGHT]), -max_rot_vel, max_rot_vel);
      break;

    case ADDR_PROFILE_ACC_FL:
    case ADDR_PROFILE_ACC_FR:
    case ADDR_PROFILE_ACC_RL:
    case ADDR_PROFILE_ACC_RR:
      if (get_connection_state_with_motors() == true)
        motor_driver.write_profile_acceleration(control_items.profile_acceleration[MotorLocation::FRONT_LEFT],
                                                control_items.profile_acceleration[MotorLocation::FRONT_RIGHT],
                                                control_items.profile_acceleration[MotorLocation::REAR_LEFT],
                                                control_items.profile_acceleration[MotorLocation::REAR_RIGHT]);
      break;
  }
}

/*******************************************************************************
 * Function definition to check the connection status with the ROS2 node.
 *******************************************************************************/
static bool connection_state_with_ros2_node = false;

static bool get_connection_state_with_ros2_node()
{
  return connection_state_with_ros2_node;
}

static void set_connection_state_with_ros2_node(bool is_connected)
{
  connection_state_with_ros2_node = is_connected;
}

void update_connection_state_with_ros2_node()
{
  static uint32_t pre_time;
  static uint8_t pre_data;
  static bool pre_state;

  // To wait for IMU Calibration
  if (pre_state != get_connection_state_with_ros2_node())
  {
    pre_state = get_connection_state_with_ros2_node();
    pre_time = millis();
    return;
  }

  if (pre_data != control_items.heart_beat)
  {
    pre_time = millis();
    pre_data = control_items.heart_beat;
    set_connection_state_with_ros2_node(true);
  }
  else
  {
    if (millis() - pre_time >= HEARTBEAT_TIMEOUT_MS)
    {
      pre_time = millis();
      set_connection_state_with_ros2_node(false);
    }
  }
}

/*******************************************************************************
 * Function definition to check the connection with the motor.
 *******************************************************************************/
static bool is_connected_motors = false;

static bool get_connection_state_with_motors()
{
  return is_connected_motors;
}

static void set_connection_state_with_motors(bool is_connected)
{
  is_connected_motors = is_connected;
}

/*******************************************************************************
 * Function definition to test motors using the built-in buttons of OpenCR.
 *******************************************************************************/
const float TICK2RAD = 0.001533981;  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
const float TEST_DISTANCE = 0.300;   // meter
const float TEST_RADIAN = 3.14;      // 180 degree
