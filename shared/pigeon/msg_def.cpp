// Copyright <2020> [Copyright rossihwang@gmail.com]

#include "pigeon/msg_def.hpp"

std::unordered_map<MessageId, const pb_msgdesc_t*> fields_map({
  {MessageId::WHL_CNTR, WheelsCounter_fields},
  {MessageId::IMU, Imu_fields},
  {MessageId::TWIST, Twist_fields},
  {MessageId::PWM_CTRL, PwmCtrl_fields},
  {MessageId::LOG, Log_fields},
});