// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include <pb.h>
#include <unordered_map>
#include <msg/counter2.pb.h>
#include <msg/imu.pb.h>
#include <msg/twist.pb.h>
#include <msg/logging.pb.h>
#include <msg/pwm_ctrl.pb.h>

enum class MessageId {
  CNTR2 = 0,
  IMU,
  TWIST,
  PWM_CTRL,
  LOG
};

extern std::unordered_map<MessageId, const pb_msgdesc_t*> fields_map;