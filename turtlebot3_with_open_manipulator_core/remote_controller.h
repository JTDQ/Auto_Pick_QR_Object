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
#include "turtlebot3_with_open_manipulator_core_config.h"
#include "pitches.h"
RC100 rc100;
double grip_value = 0.0;
void sing_melody(int);
void fromRC100(OpenManipulatorDriver* open_manipulator,float *goal_velocity_from_rc100, uint16_t data,ros::Publisher* rc_partol_pub,std_msgs::Int32* rc_patrol_msg);
void connectRC100();
uint16_t readRC100Data();
int availableRC100();


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

void fromRC100(OpenManipulatorDriver* open_manipulator,float *goal_velocity_from_rc100,float *goal_velocity_from_cmd, uint16_t data,ros::Publisher* rc_partol_pub,std_msgs::Int32* rc_patrol_msg)
{
  // if(data!=old_data){
  //   old_data=data;
  //   return;
  // }
  //void RobotisManipulator::makeJointTrajectoryFromPresentPosition(std::vector<double> delta_goal_joint_position, double move_time, std::vector<JointValue> present_joint_value)
  double pose[]={0.0, 0.0, 0.0, 0.0, 0.0};
  static uint32_t last_partol_time=0;
  bool isControl=false; 
  uint32_t t;
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
      goal_velocity_from_cmd[0]=0;
      goal_velocity_from_cmd[1]=0;
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
    }else if((data&RC100_BTN_1)){
      t = millis();
      if(t-last_partol_time>5000){
        isControl=true;
        rc_patrol_msg->data = 1;        
      }
    }else if(data & RC100_BTN_2){
      t = millis();
      if(t-last_partol_time>5000){
        isControl=true;
        rc_patrol_msg->data = 2;        
      }
    }else if((data & RC100_BTN_3)){
      t = millis();
      if(t-last_partol_time>5000){
        isControl=true;
        rc_patrol_msg->data = 3;        
      }
    }else if(data & RC100_BTN_4){
      t = millis();
      if(t-last_partol_time>5000){
        isControl=true;
        rc_patrol_msg->data = 4;        
      }
    }
    if (isControl==true)
    {
        sing_melody(rc_patrol_msg->data);
        last_partol_time=t;
        for (size_t i = 0; i < 5; i++)
        {
          rc_partol_pub->publish(rc_patrol_msg);
        }
    }
    
    
  }
}
void sing_melody(int index){
    int melody1[] = {
      NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
    };

    // note durations: 4 = quarter note, 8 = eighth note, etc.:
    int noteDurations1[] = {
      4, 8, 8, 4, 4, 4, 4, 4
    };
    int melody[] = {
      // NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
      NOTE_G3, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_C4
    };

    // note durations: 4 = quarter note, 8 = eighth note, etc.:
    int noteDurations[] = {
      16, 16, 16, 16, 8, 16, 16
    };

  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 2000 / noteDurations[thisNote];
    tone(BDPIN_BUZZER, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(BDPIN_BUZZER);
  }
}
#endif
