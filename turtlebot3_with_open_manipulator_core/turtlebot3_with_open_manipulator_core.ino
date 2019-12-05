/*******************************************************************************
* sanchuan dao ci yiyou
2019-12-05
2019-11-28
* remote control service
* Copyright 2018 ROBOTIS CO., LTD.
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
******************************************************************************
*/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

#include "turtlebot3_with_open_manipulator_core_config.h"
#include "pitches.h"
#define DEG2RAD 0.017453293
#include <TurtleBot3.h>
#include <RC100.h>
/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  DEBUG_SERIAL.begin(57600);

  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);
  nh.subscribe(joint_position_sub);
  nh.subscribe(joint_move_time_sub);
  nh.subscribe(gripper_position_sub);
  nh.subscribe(gripper_move_time_sub);
  nh.subscribe(sound_sub);
  nh.subscribe(motor_power_sub);
  nh.subscribe(reset_sub);

  nh.advertise(sensor_state_pub);  
  nh.advertise(version_info_pub);
  nh.advertise(imu_pub);
  nh.advertise(cmd_vel_rc100_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(battery_state_pub);
  nh.advertise(mag_pub);

  rc_patrol_msg.data='a';
  nh.advertise(rc_partol_pub);

  tf_broadcaster.init(nh);

  // Setting for Dynamixel motors
  motor_driver.init(NAME);
  manipulator_driver.init(&joint_id[0], joint_cnt, &gripper_id[0], gripper_cnt);

  // Setting for IMU
  sensors.init();

  // Init diagnosis
  diagnosis.init();

  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  // controllers.init(MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  connectRC100();
  // Setting for SLAM and navigation (odometry, joint states, TF)
  initOdom();

  initJointStates();

  prev_update_time = millis();

  pinMode(LED_WORKING_CHECK, OUTPUT);

  setup_end = true;
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  uint32_t t = millis();
  updateTime();
  updateVariable(nh.connected());
  updateTFPrefix(nh.connected());

  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    updateGoalVelocity();
    motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);
    tTime[0] = t;
  }

  if ((t-tTime[1]) >= (1000 / CMD_VEL_PUBLISH_FREQUENCY))
  {
    publishCmdVelFromRC100Msg();
    tTime[1] = t;
  }

  if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishSensorStateMsg();
    publishBatteryStateMsg();
    publishDriveInformation();
    tTime[2] = t;
  }

  if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
    publishImuMsg();
    publishMagMsg();
    tTime[3] = t;
  }

  if ((t-tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_FREQUENCY))
  {
    publishVersionInfoMsg();
    tTime[4] = t;
  }

#ifdef DEBUG
  if ((t-tTime[5]) >= (1000 / DEBUG_LOG_FREQUENCY))
  {
    sendDebuglog();
    tTime[5] = t;
  }
#endif

  if ((t-tTime[6]) >= (1000 / JOINT_CONTROL_FREQEUNCY))
  {
    jointControl();
    tTime[6] = t;
  }

  // Send log message after ROS connection
  sendLogMsg();

  // Receive data from RC100 
  //controllers.getRCdata(goal_velocity_from_rc100);
  getData(100);
  // Check push button pressed for simple test drive
  driveTest(diagnosis.getButtonPress(3000));

  // Update the IMU unit
  sensors.updateIMU();

  // TODO
  // Update sonar data
  // sensors.updateSonar(t);

  // Start Gyro Calibration after ROS connection
  updateGyroCali();

  // Show LED status
  diagnosis.showLedStatus(nh.connected());

  // Update Voltage
  battery_state = diagnosis.updateVoltageCheck(setup_end);

  // Call all the callbacks waiting to be called at that point in time
  nh.spinOnce();

  // Wait the serial link time to process
  waitForSerialLink(nh.connected());
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;

  goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
}

 /*
// 先借用以下这个callback函数，
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  double pose[]={cmd_vel_msg.linear.x, cmd_vel_msg.linear.y, cmd_vel_msg.linear.z , 0};
  manipulator_driver.currentBasedPos(pose);
 
  char log_msg[50];
  // 读取当前位置值
  float man_ctrl_pos[10]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  nh.loginfo("A");
  // man_ctrl_pos[0]=0.0;
  // joint_states.position;
  for(int i=0;i<4;i++){
    man_ctrl_pos[i+1]=joint_states.position[2+i];
    man_ctrl_pos[i+6]=joint_states.position[2+i];
  }
  man_ctrl_pos[5]=0.2;
  nh.loginfo("B");
  // man_ctrl_pos[6]+=cmd_vel_msg.linear.x/100.0;
  man_ctrl_pos[6]+= 0.27;
  
  sprintf(log_msg,"%3.8lf",man_ctrl_pos[6]);
  // sprintf(log_msg, "");
  nh.loginfo(log_msg);
  // man_ctrl_pos[1]+=cmd_vel_msg.angular.z/100.0;
  // goal_velocity_from_cmd[LINEAR]  = cmd_vel_msg.linear.x;
  // goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;
  
  // joint_trajectory_point.data.clear();
  // joint_trajectory_point.dim.
  joint_trajectory_point.data=man_ctrl_pos;
  nh.loginfo("C");
  // joint_trajectory_point.layout.joidata_offset=0;
  
  // atos()
  
    sprintf(log_msg,"%3.8lf",joint_trajectory_point.data[6]);
  // sprintf(log_msg, "");
  nh.loginfo(log_msg);
  // goal_velocity_from_cmd[LINEAR]  = constrain(goal_velocity_from_cmd[LINEAR],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  // goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  
}
*/
/*******************************************************************************
* Callback function for joint trajectory msg
*******************************************************************************/
void jointTrajectoryPointCallback(const std_msgs::Float64MultiArray& joint_trajectory_point_msg)
{
  if (is_moving == false)
  {
    joint_trajectory_point = joint_trajectory_point_msg;
    is_moving = true;
  }
}

/*******************************************************************************
* Callback function for joint move time msg
*******************************************************************************/
void jointMoveTimeCallback(const std_msgs::Float64& time_msg)
{
  double data = time_msg.data;

  manipulator_driver.writeJointProfileControlParam(data);
}

/*******************************************************************************
* Callback function for gripper position msg
*******************************************************************************/
void gripperPositionCallback(const std_msgs::Float64MultiArray& gripper_msg)
{
  double goal_gripper_position[5] = {0.0, };
  const double OPEN_MANIPULATOR_GRIPPER_OFFSET = -0.015f;

  for (int index = 0; index < gripper_cnt; index++)
    goal_gripper_position[index] = gripper_msg.data[index] / OPEN_MANIPULATOR_GRIPPER_OFFSET;

  manipulator_driver.writeGripperPosition(goal_gripper_position);
}

/*******************************************************************************
* Callback function for gripper move time msg
*******************************************************************************/
void gripperMoveTimeCallback(const std_msgs::Float64& time_msg)
{
  double data = time_msg.data;
  
  manipulator_driver.writeGripperProfileControlParam(data);
}

/*******************************************************************************
* Callback function for sound msg
*******************************************************************************/
void soundCallback(const turtlebot3_msgs::Sound& sound_msg)
{
  sensors.makeSound(sound_msg.value);
}

/*******************************************************************************
* Callback function for motor_power msg
*******************************************************************************/
void motorPowerCallback(const std_msgs::Bool& power_msg)
{
  bool dxl_power = power_msg.data;

  motor_driver.setTorque(dxl_power);
  manipulator_driver.setTorque(dxl_power);
}

/*******************************************************************************
* Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty& reset_msg)
{
  char log_msg[50];

  (void)(reset_msg);

  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);

  sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);

  initOdom();

  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);  
}

/*******************************************************************************
* Publish msgs (CMD Velocity data from RC100 : angular velocity, linear velocity)
*******************************************************************************/
void publishCmdVelFromRC100Msg(void)
{
  cmd_vel_rc100_msg.linear.x  = goal_velocity_from_rc100[LINEAR];
  cmd_vel_rc100_msg.angular.z = goal_velocity_from_rc100[ANGULAR];

  cmd_vel_rc100_pub.publish(&cmd_vel_rc100_msg);
}
void getData(uint32_t wait_time)
{
  static uint8_t state = 0;
  static uint32_t tick = 0;

  bool rc100_state = false;
  // bool processing_state = false;

  uint16_t get_rc100_data = 0;
  // String get_processing_data = "";

  if (availableRC100())
  {
    get_rc100_data = readRC100Data();
    rc100_state = true;
  }

  switch (state)
  {
    case 0:
      if (rc100_state)
      {
        fromRC100(&manipulator_driver,goal_velocity_from_rc100, goal_velocity_from_cmd,get_rc100_data,&rc_partol_pub,&rc_patrol_msg);
        tick = millis();
        state = 1;
      }
     break;

    case 1:
      if ((millis() - tick) >= wait_time)
      {
        state = 0;
      }
     break;

    default:
      state = 0;
     break;
  }
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();

  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;

  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
* Publish msgs (Magnetic data)
*******************************************************************************/
void publishMagMsg(void)
{
  mag_msg = sensors.getMag();

  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = mag_frame_id;

  mag_pub.publish(&mag_msg);
}

/*******************************************************************************
* Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void)
{
  bool dxl_comm_result = false;

  sensor_state_msg.header.stamp = rosNow();
  sensor_state_msg.battery = sensors.checkVoltage();

  dxl_comm_result = motor_driver.readEncoder(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);

  if (dxl_comm_result == true)
    updateMotorInfo(sensor_state_msg.left_encoder, sensor_state_msg.right_encoder);
  else
    return;

  sensor_state_msg.bumper = sensors.checkPushBumper();

  sensor_state_msg.cliff = sensors.getIRsensorData();

  // TODO
  // sensor_state_msg.sonar = sensors.getSonarData();

  sensor_state_msg.illumination = sensors.getIlluminationData();
  
  sensor_state_msg.button = sensors.checkPushButton();

  sensor_state_msg.torque = motor_driver.getTorque();

  sensor_state_pub.publish(&sensor_state_msg);
}

/*******************************************************************************
* Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = "0.0.0";
  version_info_msg.software = "0.0.0";
  version_info_msg.firmware = FIRMWARE_VER;

  version_info_pub.publish(&version_info_msg);
}

/*******************************************************************************
* Publish msgs (battery_state)
*******************************************************************************/
void publishBatteryStateMsg(void)
{
  battery_state_msg.header.stamp = rosNow();
  battery_state_msg.design_capacity = 1.8f; //Ah
  battery_state_msg.voltage = sensors.checkVoltage();
  battery_state_msg.percentage = (float)(battery_state_msg.voltage / 11.1f);

  if (battery_state == 0)
    battery_state_msg.present = false;
  else
    battery_state_msg.present = true;  

  battery_state_pub.publish(&battery_state_msg);
}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}

/*******************************************************************************
* Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");  

        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/base_link");
      }

      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      nh.loginfo(log_msg); 

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg); 

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}

/*******************************************************************************
* Update the odometry
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;

  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);

  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
* Update the joint states 
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[20] = {0.0, };
  static float joint_states_vel[20] = {0.0, };
  static float joint_states_eff[20] = {0.0, };

  const double OPEN_MANIPULATOR_GRIPPER_OFFSET = -0.015f;

  double get_joint_position[joint_cnt + gripper_cnt];
  double get_joint_velocity[joint_cnt + gripper_cnt];
  double get_joint_current[joint_cnt + gripper_cnt];

  manipulator_driver.syncReadDynamixelInfo();
  manipulator_driver.getPosition(get_joint_position);
  manipulator_driver.getVelocity(get_joint_velocity);
  manipulator_driver.getCurrent(get_joint_current);

  joint_states_pos[LEFT]  = last_rad[LEFT];
  joint_states_pos[RIGHT] = last_rad[RIGHT];

  joint_states_vel[LEFT]  = last_velocity[LEFT];
  joint_states_vel[RIGHT] = last_velocity[RIGHT];

  for (uint8_t num = 0; num < (joint_cnt + gripper_cnt); num++)
  {
    if (num > joint_cnt)
      get_joint_position[num] = get_joint_position[num] * OPEN_MANIPULATOR_GRIPPER_OFFSET;

    joint_states_pos[WHEEL_NUM + num] = get_joint_position[num];
    joint_states_vel[WHEEL_NUM + num] = get_joint_velocity[num];
    joint_states_eff[WHEEL_NUM + num] = get_joint_current[num];
  }

  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
  joint_states.effort = joint_states_eff;
}

/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

/*******************************************************************************
* Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_tick, int32_t right_tick)
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};
  
  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0.0;
      last_tick[index]      = 0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }  

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  step_time = diff_time;

  if (step_time == 0)
    return false;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (isnan(wheel_l))
    wheel_l = 0.0;

  if (isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  // theta = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION;  
  orientation = sensors.getOrientation();
  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT]  = wheel_l / step_time;
  last_velocity[RIGHT] = wheel_r / step_time;
  last_theta = theta;

  return true;
}

/*******************************************************************************
* Manipulator's joint control
*******************************************************************************/
void jointControl(void)
{
  const uint8_t POINT_SIZE = joint_cnt + 1; // Add time parameter
  const double JOINT_CONTROL_PERIOD = 1.0f / (double)JOINT_CONTROL_FREQEUNCY;
  static uint32_t points = 0;

  static uint8_t wait_for_write = 0;
  static uint8_t loop_cnt = 0;

  if (is_moving == true)
  {
    uint32_t all_points_cnt = joint_trajectory_point.data_length;
    uint8_t write_cnt = 0;

    if (loop_cnt < (wait_for_write))
    {
      loop_cnt++;
      return;
    }
    else
    {
      double goal_joint_position[joint_cnt];
      double move_time = 0.0f;

      if (points == 0) move_time = joint_trajectory_point.data[points + POINT_SIZE] - joint_trajectory_point.data[points];
      else if ((points + POINT_SIZE) >= all_points_cnt) move_time = joint_trajectory_point.data[points] / 2.0f;
      else  move_time = joint_trajectory_point.data[points] - joint_trajectory_point.data[points - POINT_SIZE];

      for (uint32_t positions = points + 1; positions < (points + POINT_SIZE); positions++)
      {        
        if ((points + POINT_SIZE) >= all_points_cnt)
        {
          goal_joint_position[write_cnt] = joint_trajectory_point.data[positions];
        }
        else
        {
          double offset = 2.0f * (joint_trajectory_point.data[positions + POINT_SIZE] - joint_trajectory_point.data[positions]);
          goal_joint_position[write_cnt] = joint_trajectory_point.data[positions] + offset;
        }
        write_cnt++;
      }

      manipulator_driver.writeJointProfileControlParam(move_time * 2.0f);
      manipulator_driver.writeJointPosition(goal_joint_position);

      wait_for_write = move_time / JOINT_CONTROL_PERIOD;
      points = points + POINT_SIZE;

      if (points >= all_points_cnt)
      {
        points = 0;
        wait_for_write = 0;
        is_moving = false;
      }
      else
      {
        loop_cnt = 0;
      }
    }
  }
}
void get_RC100_and_ctrl_Opnemain(){
  double goal_joint_position[joint_cnt];

  manipulator_driver.writeJointPosition(goal_joint_position);
}
/*******************************************************************************
* Turtlebot3 test drive using push buttons
*******************************************************************************/
void driveTest(uint8_t buttons)
{
  static bool move[2] = {false, false};
  static int32_t saved_tick[2] = {0, 0};
  static double diff_encoder = 0.0;

  int32_t current_tick[2] = {0, 0};

  motor_driver.readEncoder(current_tick[LEFT], current_tick[RIGHT]);

  if (buttons & (1<<0))  
  {
    move[LINEAR] = true;
    saved_tick[RIGHT] = current_tick[RIGHT];

    diff_encoder = TEST_DISTANCE / (0.207 / 4096); // (Circumference of Wheel) / (The number of tick per revolution)
  }
  else if (buttons & (1<<1))
  {
    move[ANGULAR] = true;
    saved_tick[RIGHT] = current_tick[RIGHT];

    diff_encoder = (TEST_RADIAN * TURNING_RADIUS) / (0.207 / 4096);
  }

  if (move[LINEAR])
  {    
    if (abs(saved_tick[RIGHT] - current_tick[RIGHT]) <= diff_encoder)
    {
      goal_velocity_from_button[LINEAR]  = 0.05;
    }
    else
    {
      goal_velocity_from_button[LINEAR]  = 0.0;
      move[LINEAR] = false;
    }
  }
  else if (move[ANGULAR])
  {   
    if (abs(saved_tick[RIGHT] - current_tick[RIGHT]) <= diff_encoder)
    {
      goal_velocity_from_button[ANGULAR]= -0.7;
    }
    else
    {
      goal_velocity_from_button[ANGULAR]  = 0.0;
      move[ANGULAR] = false;
    }
  }
}

/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;
  
  if (isConnected)
  {
    if (variable_flag == false)
    {      
      sensors.initIMU();
      initOdom();

      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  
  if (isConnected)
  {
    if (wait_flag == false)
    {      
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}

/*******************************************************************************
* Time Interpolation function (deprecated)
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros)
{
  uint32_t sec, nsec;

  sec  = _micros / 1000 + t.sec;
  nsec = _micros % 1000000000 + t.nsec;

  return ros::Time(sec, nsec);
}

/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(void)
{
  static bool isEnded = false;
  char log_msg[50];

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}

/*******************************************************************************
* Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];  

  String firmware_version = FIRMWARE_VER;
  String bringup_log      = "This core(v" + firmware_version + ") is compatible with TB3 with OpenManipulator";
   
  const char* init_log_data = bringup_log.c_str();

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

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {"wheel_left_joint", "wheel_right_joint", "joint1", "joint2", "joint3", "joint4", "gripper"};

  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;

  joint_states.name_length     = WHEEL_NUM + joint_cnt + gripper_cnt;
  joint_states.position_length = WHEEL_NUM + joint_cnt + gripper_cnt;
  joint_states.velocity_length = WHEEL_NUM + joint_cnt + gripper_cnt;
  joint_states.effort_length   = WHEEL_NUM + joint_cnt + gripper_cnt;
}

/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/
void updateGoalVelocity(void)
{
  goal_velocity[LINEAR]  = goal_velocity_from_button[LINEAR]  + goal_velocity_from_cmd[LINEAR]  + goal_velocity_from_rc100[LINEAR];
  goal_velocity[ANGULAR] = goal_velocity_from_button[ANGULAR] + goal_velocity_from_cmd[ANGULAR] + goal_velocity_from_rc100[ANGULAR];
}

/*******************************************************************************
* map (return double)
*******************************************************************************/
double mapd(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*******************************************************************************
* Send Debug data
*******************************************************************************/
void sendDebuglog(void)
{
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("EXTERNAL SENSORS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.print("Bumper : "); DEBUG_SERIAL.println(sensors.checkPushBumper());
  DEBUG_SERIAL.print("Cliff : "); DEBUG_SERIAL.println(sensors.getIRsensorData());
  DEBUG_SERIAL.print("Sonar : "); DEBUG_SERIAL.println(sensors.getSonarData());
  DEBUG_SERIAL.print("Illumination : "); DEBUG_SERIAL.println(sensors.getIlluminationData());

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("OpenCR SENSORS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.print("Battery : "); DEBUG_SERIAL.println(sensors.checkVoltage());
  DEBUG_SERIAL.println("Button : " + String(sensors.checkPushButton()));

  float* quat = sensors.getOrientation();

  DEBUG_SERIAL.println("IMU : ");
  DEBUG_SERIAL.print("    w : "); DEBUG_SERIAL.println(quat[0]);
  DEBUG_SERIAL.print("    x : "); DEBUG_SERIAL.println(quat[1]);
  DEBUG_SERIAL.print("    y : "); DEBUG_SERIAL.println(quat[2]);
  DEBUG_SERIAL.print("    z : "); DEBUG_SERIAL.println(quat[3]);
  
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("DYNAMIXELS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Torque(wheel) : " + String(motor_driver.getTorque()));
  DEBUG_SERIAL.println("Torque(joint) : " + String(manipulator_driver.getTorqueState()));

  int32_t encoder[WHEEL_NUM] = {0, 0};
  motor_driver.readEncoder(encoder[LEFT], encoder[RIGHT]);
  
  DEBUG_SERIAL.println("Encoder(left) : " + String(encoder[LEFT]));
  DEBUG_SERIAL.println("Encoder(right) : " + String(encoder[RIGHT]));

  double present_position[joint_cnt + gripper_cnt];
  manipulator_driver.getPosition(present_position);

  for (uint8_t num = 0; num < joint_cnt + gripper_cnt; num++)
  {
    DEBUG_SERIAL.print("Present Position(Joint_");
    DEBUG_SERIAL.print(num);
    DEBUG_SERIAL.print(") : ");
    DEBUG_SERIAL.println(present_position[num]);
  }

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("TurtleBot3");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Odometry : ");   
  DEBUG_SERIAL.print("         x : "); DEBUG_SERIAL.println(odom_pose[0]);
  DEBUG_SERIAL.print("         y : "); DEBUG_SERIAL.println(odom_pose[1]);
  DEBUG_SERIAL.print("     theta : "); DEBUG_SERIAL.println(odom_pose[2]);
}
