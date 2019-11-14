/*******************************************************************************
  Copyright 2018 ROBOTIS CO., LTD.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef REMOTE_CONTROLLER_H_
#define REMOTE_CONTROLLER_H_
// #define DEG2RAD(x) ((x)*0.017453293)
#define DEG2RAD 0.017453293
#include <TurtleBot3.h>
#include <RC100.h>

RC100 rc100;
double grip_value = 0.0;
uint16_t old_data=0;
void connectRC100()
{
  rc100.begin(1);
}

int availableRC100()
{
  return rc100.available();
}

uint16_t readRC100Data()
{
  return rc100.readData();
}

void fromRC100(OpenManipulatorDriver* open_manipulator,float *goal_velocity_from_rc100, uint16_t data)
{
  if(data!=old_data){
    old_data=data;
    return;
  }
  //void RobotisManipulator::makeJointTrajectoryFromPresentPosition(std::vector<double> delta_goal_joint_position, double move_time, std::vector<JointValue> present_joint_value)
  double pose[]={0.0, 0.0, 0.0, 0.0, 0.0};

  if (!(data & RC100_BTN_6)) {
    if (data & RC100_BTN_L){
      pose[0]=5*DEG2RAD;
      open_manipulator->currentBasedPos(pose);
    }
    else if (data & RC100_BTN_R){
      pose[0]=-5*DEG2RAD;
      open_manipulator->currentBasedPos(pose);
    }else if (data & RC100_BTN_D){
      pose[1]=-5*DEG2RAD;
      open_manipulator->currentBasedPos(pose);
    }else if (data & RC100_BTN_U){
      pose[1]=5*DEG2RAD;
      open_manipulator->currentBasedPos(pose);
    }else if (data & RC100_BTN_3){
      pose[2]=5*DEG2RAD;
      open_manipulator->currentBasedPos(pose);
    }else if (data & RC100_BTN_1){
      pose[2]=-5*DEG2RAD;
      open_manipulator->currentBasedPos(pose);
    }else if (data & RC100_BTN_4)
    {
      pose[3]=5*DEG2RAD;
      open_manipulator->currentBasedPos(pose);
    }else if (data & RC100_BTN_2)
    {
      pose[3]=-5*DEG2RAD;
      open_manipulator->currentBasedPos(pose);
    }else if(data & RC100_BTN_5){
      goal_velocity_from_rc100[0]=0;
      goal_velocity_from_rc100[1]=0;
    }
  } else {
    // float goal_velocity_from_rc100[WHEEL_NUM] = {0.0, 0.0};
    if (data & RC100_BTN_U)
      goal_velocity_from_rc100[0]=0.2;
    else if (data & RC100_BTN_D)
      goal_velocity_from_rc100[0]=-0.2;      
    else if (data & RC100_BTN_L)
      goal_velocity_from_rc100[1]=0.2;
    else if (data & RC100_BTN_R)
      goal_velocity_from_rc100[1]=-0.2;
    else if(data & RC100_BTN_5){
      goal_velocity_from_rc100[0]=0;
      goal_velocity_from_rc100[1]=0;
    }else if(data & RC100_BTN_2){
      pose[4]=0.02;
      open_manipulator->currentBasedPos(pose);
    }else if(data & RC100_BTN_4){
      pose[4]=-0.02;
      open_manipulator->currentBasedPos(pose);
    }
    
  }
}
#endif
