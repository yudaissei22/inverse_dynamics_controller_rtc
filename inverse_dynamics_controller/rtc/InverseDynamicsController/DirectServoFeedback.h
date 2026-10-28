#pragma once
#include <stdexcept>
#include <string>

// Select the meaning of the hardware-enabled feedback for the direct profile.
// In hrpsys_trans_bridge/iob-shm.cpp, SERVO_STATE additionally requires pgain>0.5;
// POWER_STATE instead reports the motor's raw control status == 0x0000.
// This use of POWER_STATE is specific to that backend, not generic power presence.
class DirectServoFeedback {
public:
  explicit DirectServoFeedback(const std::string& source = "servo") {
    if (source == "servo") mask_ = 0x02;
    else if (source == "trans_power") mask_ = 0x04;
    else throw std::invalid_argument("direct_servo_feedback_source must be servo or trans_power");
  }
  long mask() const { return mask_; }
  bool enabled(long state) const { return (state & mask_) != 0; }

private:
  long mask_;
};
