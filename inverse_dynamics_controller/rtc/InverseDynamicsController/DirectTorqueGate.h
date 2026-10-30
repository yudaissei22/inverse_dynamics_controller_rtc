#pragma once
#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

// All times are monotonic seconds, supplied by the RTC.
// This class gates torque commands; RobotHardware still performs gain ramps.
class DirectTorqueGate {
public:
  enum Mode { POSITION, TO_TORQUE, TORQUE, TO_POSITION, FAULT };

  DirectTorqueGate(std::vector<double> limits, std::vector<double> rates,
                  double feedback_timeout, double alignment_tolerance)
      : limits_(limits), rates_(rates), command_(limits.size(), 0.0),
        output_(limits.size(), 0.0), feedback_timeout_(feedback_timeout),
        alignment_tolerance_(alignment_tolerance) {
    if (limits.empty() || limits.size() != rates.size() ||
        !positive(feedback_timeout) || !positive(alignment_tolerance))
      throw std::invalid_argument("invalid torque gate configuration");
    for (size_t i = 0; i < limits.size(); ++i)
      if (!std::isfinite(limits[i]) || limits[i] < 0 || !positive(rates[i]))
        throw std::invalid_argument("invalid torque limit or slew rate");
  }

  void updateActual(const std::vector<double>& q, double now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!valid(q) || !std::isfinite(now)) { fail("invalid qAct"); return; }
    actual_ = q; actual_time_ = now; have_actual_ = true;
  }
  void updatePositionReference(const std::vector<double>& q, double now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!valid(q) || !std::isfinite(now)) { fail("invalid qServoRef"); return; }
    reference_ = q; reference_time_ = now; have_reference_ = true;
  }

  void updateServoState(const std::vector<bool>& enabled, double now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (enabled.size() != limits_.size() || !std::isfinite(now)) { fail("invalid servoState"); return; }
    all_servos_on_ = std::all_of(enabled.begin(), enabled.end(), [](bool on) { return on; });
    all_servos_off_ = std::none_of(enabled.begin(), enabled.end(), [](bool on) { return on; });
    servo_time_ = now;
    have_servo_state_ = true;
  }

  // Called BEFORE issuing any RobotHardware gain changes.
  bool begin(bool torque, double duration, double now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!positive(duration) || duration > 30.0 || !freshActual(now) ||
        !freshServoState(now) || !all_servos_on_) return false;
    if (torque) {
      if (mode_ != POSITION || !aligned(now)) return false;
      mode_ = TO_TORQUE;
    } else {
      if (mode_ != TORQUE) return false;
      mode_ = TO_POSITION;
    }
    if (!gravity_command_) clearCommand();
    duration_ = duration;
    transition_deadline_ = now + duration + 5.0;
    gains_started_ = false;
    return true;
  }

  // A snapshot check; caller must also stop competing position-command producers.
  bool readyForPosition(double now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_ == TO_POSITION && (zeroOutput() || freshGravityCommand(now)) && aligned(now) &&
           freshServoState(now) && all_servos_on_;
  }

  // Called AFTER both RobotHardware gain service calls returned successfully.
  // It is a software scheduling acknowledgement, NOT measured gain feedback.
  bool markGainsStarted(double now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!transitioning() || gains_started_ || now > transition_deadline_ ||
        !freshActual(now) || !freshServoState(now) || !all_servos_on_) return false;
    if (mode_ == TO_POSITION &&
        ((!zeroOutput() && !freshGravityCommand(now)) || !aligned(now))) return false;
    gain_finish_time_ = now + duration_;
    transition_deadline_ = gain_finish_time_ + 2.0;
    gains_started_ = true;
    return true;
  }

  // gravity=true is reserved for the RTC's opt-in g(q) producer. Manual
  // commands remain TORQUE-only. Gravity can support the arm during gain ramps.
  bool setCommand(const std::vector<double>& tau, double ttl, double now, bool gravity = false) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == FAULT || (!gravity && mode_ != TORQUE)) return false;
    if (!checkHardware(now)) return false;
    if (!valid(tau) || !positive(ttl) || ttl > 1.0) { fail("invalid torque command or TTL"); return false; }
    for (size_t i = 0; i < tau.size(); ++i)
      if (std::abs(tau[i]) > limits_[i]) { fail("torque limit exceeded"); return false; }
    command_ = tau;
    command_expiry_ = now + ttl;
    have_command_ = true;
    gravity_command_ = gravity;
    return true;
  }

  // Call EVERY RTC cycle, including cycles without a new qRef.
  std::vector<double> step(double now, double dt) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!positive(dt) || !std::isfinite(now)) fail("invalid control time");
    if ((mode_ != POSITION || gravity_command_) && mode_ != FAULT) checkHardware(now);
    if (gravity_command_ && now >= command_expiry_) fail("gravity command timeout");
    if (transitioning() && !freshReference(now)) fail("qServoRef timeout");
    if (transitioning() && now > transition_deadline_) fail("control transition timeout");
    if (transitioning() && gains_started_ && now >= gain_finish_time_) {
      mode_ = mode_ == TO_TORQUE ? TORQUE : POSITION;
      if (!gravity_command_) clearCommand(); // Manual commands never survive mode changes.
    }
    if (mode_ == FAULT) {
      std::fill(output_.begin(), output_.end(), 0.0);
      return output_;
    }
    if (have_command_ && now >= command_expiry_) clearCommand();
    for (size_t i = 0; i < output_.size(); ++i) {
      const double target = (mode_ == TORQUE || gravity_command_) && have_command_ ? command_[i] : 0.0;
      const double change = std::max(-rates_[i] * dt,
                                    std::min(rates_[i] * dt, target - output_[i]));
      output_[i] += change;
    }
    return output_;
  }

  Mode mode() const { std::lock_guard<std::mutex> lock(mutex_); return mode_; }
  bool torqueIsZero() const {
    std::lock_guard<std::mutex> lock(mutex_); return zeroOutput();
  }
  // Software command readiness, NOT measured motor torque. Before removing
  // position gains, wait until the bounded output has caught up with g(q).
  bool gravityOutputReady(double now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == FAULT || !freshGravityCommand(now) || !freshActual(now) ||
        !freshServoState(now) || !all_servos_on_) return false;
    for (size_t i = 0; i < output_.size(); ++i)
      if (std::abs(output_[i] - command_[i]) > 1e-6) return false;
    return true;
  }
  void abort() {
    std::lock_guard<std::mutex> lock(mutex_);
    fail("aborted by client or RTC lifecycle");
  }
  void fault(const std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    fail(reason.c_str());
  }
  // Revoke a producer's pending command; step() slews the output to zero.
  void cancelCommand() {
    std::lock_guard<std::mutex> lock(mutex_);
    clearCommand();
  }
  std::string faultReason() const {
    std::lock_guard<std::mutex> lock(mutex_); return fault_reason_;
  }

  // The RTC independently checks fresh hardware feedback; a client assertion
  // that the servos are off is not enough to reset a latched fault.
  bool resetAfterServoOff(double now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!freshServoState(now) || !all_servos_off_) return false;
    clearCommand();
    std::fill(output_.begin(), output_.end(), 0.0);
    mode_ = POSITION;
    fault_reason_.clear();
    have_actual_ = have_reference_ = gains_started_ = false;
    return true;
  }

private:
  static bool positive(double x) { return std::isfinite(x) && x > 0.0; }
  bool valid(const std::vector<double>& x) const {
    if (x.size() != limits_.size()) return false;
    for (double value : x) if (!std::isfinite(value)) return false;
    return true;
  }
  bool freshActual(double now) const {
    return std::isfinite(now) && have_actual_ && now >= actual_time_ &&
           now - actual_time_ < feedback_timeout_;
  }
  bool freshReference(double now) const {
    return std::isfinite(now) && have_reference_ && now >= reference_time_ &&
           now - reference_time_ < feedback_timeout_;
  }
  bool freshServoState(double now) const {
    return std::isfinite(now) && have_servo_state_ && now >= servo_time_ &&
           now - servo_time_ < feedback_timeout_;
  }
  bool aligned(double now) const {
    if (!freshActual(now) || !freshReference(now)) return false;
    for (size_t i = 0; i < actual_.size(); ++i)
      if (std::abs(actual_[i] - reference_[i]) > alignment_tolerance_) return false;
    return true;
  }
  bool transitioning() const { return mode_ == TO_TORQUE || mode_ == TO_POSITION; }
  bool zeroOutput() const {
    for (double x : output_) if (std::abs(x) > 1e-12) return false;
    return true;
  }
  void clearCommand() {
    have_command_ = false;
    gravity_command_ = false;
    std::fill(command_.begin(), command_.end(), 0.0);
  }
  bool freshGravityCommand(double now) const {
    return gravity_command_ && have_command_ && now < command_expiry_;
  }
  bool checkHardware(double now) {
    if (!freshActual(now)) fail("qAct timeout");
    else if (!freshServoState(now)) fail("servoState timeout");
    else if (!all_servos_on_) fail("motor disabled");
    return mode_ != FAULT;
  }
  void fail(const char* reason) {
    // A client's cleanup abort must not overwrite the original failure.
    if (mode_ != FAULT) fault_reason_ = reason;
    mode_ = FAULT;
    clearCommand();
  }

  mutable std::mutex mutex_;
  Mode mode_ = POSITION;
  std::string fault_reason_;
  std::vector<double> limits_, rates_, command_, output_, actual_, reference_;
  double feedback_timeout_, alignment_tolerance_;
  double actual_time_ = 0, reference_time_ = 0, command_expiry_ = 0;
  double servo_time_ = 0;
  double duration_ = 0, transition_deadline_ = 0, gain_finish_time_ = 0;
  bool have_actual_ = false, have_reference_ = false, have_command_ = false;
  bool gains_started_ = false;
  bool gravity_command_ = false;
  bool have_servo_state_ = false, all_servos_on_ = false, all_servos_off_ = false;
};
