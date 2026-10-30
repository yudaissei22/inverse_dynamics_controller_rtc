// Standalone tests: no ROS, CORBA, shared memory or motor connection.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "DirectTorqueGate.h"
#include "DirectServoFeedback.h"
#include <cassert>
#include <iostream>
#include <limits>

using Gate = DirectTorqueGate;

void feedback(Gate& gate, double now, bool on = true,
              double actual = 0.0, double reference = 0.0) {
  gate.updateActual({actual}, now);
  gate.updatePositionReference({reference}, now);
  gate.updateServoState({on}, now);
}
void torqueMode(Gate& gate) {
  feedback(gate, 0.0);
  assert(gate.begin(true, 0.1, 0.0));
  assert(gate.markGainsStarted(0.0));
  feedback(gate, 0.1);
  assert(gate.step(0.1, 0.01)[0] == 0.0);
  assert(gate.mode() == Gate::TORQUE);
}

int main() {
  // Gravity preloading is bounded and supports BOTH gain transitions without
  // a zero-torque gap. Manual commands still cannot run in POSITION.
  {
    Gate gate({2}, {1}, 0.2, 0.01);
    feedback(gate, 0);
    assert(!gate.gravityOutputReady(0));
    assert(gate.setCommand({0.1}, 0.2, 0, true));
    assert(!gate.gravityOutputReady(0));
    assert(gate.step(0.05, 0.05)[0] == 0.05);
    assert(!gate.gravityOutputReady(0.05));
    assert(gate.step(0.1, 0.05)[0] == 0.1);
    assert(gate.gravityOutputReady(0.1));
    // Command readiness is revoked by expiry, not only a subsequent step().
    assert(!gate.gravityOutputReady(0.2));
    gate.cancelCommand();
    assert(!gate.gravityOutputReady(0.1));
  }
  {
    Gate gate({2}, {100}, 0.2, 0.01);
    feedback(gate, 0);
    assert(!gate.setCommand({1}, 0.1, 0));
    assert(gate.setCommand({1}, 0.1, 0, true));
    assert(gate.step(0.01, 0.01)[0] == 1);
    assert(gate.begin(true, 0.02, 0.01));
    assert(gate.markGainsStarted(0.01));
    assert(gate.step(0.04, 0.01)[0] == 1);
    assert(gate.mode() == Gate::TORQUE);
    assert(gate.begin(false, 0.02, 0.04));
    assert(gate.readyForPosition(0.04));
    assert(gate.markGainsStarted(0.04));
    assert(gate.step(0.07, 0.01)[0] == 1);
    assert(gate.mode() == Gate::POSITION);
    // Expiring gravity must not be replayed, even in POSITION.
    assert(gate.step(0.11, 0.01)[0] == 0);
    assert(gate.mode() == Gate::FAULT);
    assert(gate.faultReason() == "gravity command timeout");
  }
  {
    Gate gate({2}, {100}, 0.2, 0.01);
    feedback(gate, 0);
    assert(!gate.setCommand({3}, 0.1, 0, true));
    assert(gate.mode() == Gate::FAULT);
    assert(gate.step(0.01, 0.01)[0] == 0);
  }
  // Regression: trans iob reports SERVO_STATE only while pgain > 0.5.
  // POWER_STATE reports raw motor enable even at zero P/D gains.
  {
    DirectServoFeedback motorFeedback("trans_power");
    DirectServoFeedback legacyFeedback("servo");
    assert(motorFeedback.mask() == 4 && legacyFeedback.mask() == 2);
    auto status = [](double pgain, bool enabled) {
      return 1L | (enabled ? 4L : 0L) | (enabled && pgain > 0.5 ? 2L : 0L);
    };
    Gate gate({2}, {10}, 0.2, 0.01);
    auto feed = [&](double now, double pgain, bool enabled) {
      gate.updateActual({0}, now);
      gate.updatePositionReference({0}, now);
      gate.updateServoState({motorFeedback.enabled(status(pgain, enabled))}, now);
    };
    feed(0, 1, true);
    assert(gate.begin(true, 2.0, 0));
    assert(gate.markGainsStarted(0));
    for (int i = 1; i <= 20; ++i) {
      const double gain = 1.0 - i / 20.0;
      feed(i * 0.1, gain, true);
      gate.step(i * 0.1, 0.1);
      assert(gate.mode() != Gate::FAULT);
      if (gain <= 0.5) assert(!legacyFeedback.enabled(status(gain, true)));
    }
    assert(gate.mode() == Gate::TORQUE);
    assert(gate.faultReason().empty());
    assert(!gate.resetAfterServoOff(2.0)); // Low gain is NOT motor OFF.
    feed(2.1, 0, true);
    assert(gate.begin(false, 0.2, 2.1)); // Return is allowed at P=0.
    assert(gate.readyForPosition(2.1));
    assert(gate.markGainsStarted(2.1));
    feed(2.4, 1, true);
    gate.step(2.4, 0.1);
    assert(gate.mode() == Gate::POSITION);
    feed(2.5, 1, true);
    assert(gate.begin(true, 0.1, 2.5));
    assert(gate.markGainsStarted(2.5));
    feed(2.65, 0, true);
    gate.step(2.65, 0.1);
    assert(gate.mode() == Gate::TORQUE);
    feed(2.7, 0, false);
    assert(gate.step(2.7, 0.1)[0] == 0);
    assert(gate.mode() == Gate::FAULT);
    assert(gate.faultReason() == "motor disabled");
    gate.abort();
    assert(gate.faultReason() == "motor disabled"); // Preserve first cause.
    assert(gate.resetAfterServoOff(2.7));
    assert(gate.faultReason().empty());
  }
  {
    bool rejectedSource = false;
    try { DirectServoFeedback invalid("unknown"); }
    catch (const std::invalid_argument&) { rejectedSource = true; }
    assert(rejectedSource);
  }
  // Synthetic units/values, not hardware tuning parameters.
  {
    Gate gate({2.0}, {10.0}, 0.2, 0.01);
    assert(!gate.begin(true, 0.1, 0));
    feedback(gate, 0, false);
    assert(!gate.begin(true, 0.1, 0));
    feedback(gate, 0, true, 0, 0.5);
    assert(!gate.begin(true, 0.1, 0));
    feedback(gate, 0);
    assert(gate.begin(true, 0.1, 0));
    assert(!gate.setCommand({1}, 0.1, 0));
    assert(!gate.begin(false, 0.1, 0));
    assert(gate.markGainsStarted(0));
    assert(!gate.markGainsStarted(0));
    assert(gate.step(0.1, 0.1)[0] == 0);
    assert(gate.mode() == Gate::TORQUE);
    assert(gate.step(0.11, 0.01)[0] == 0); // No command -> zero every cycle.
    assert(gate.setCommand({1.0}, 0.05, 0.11));
    assert(std::abs(gate.step(0.12, 0.01)[0] - 0.1) < 1e-12);
    assert(gate.step(0.17, 0.05)[0] == 0); // Expired command -> zero target.
    feedback(gate, 0.18, true, 0.3, 0.0);
    assert(gate.begin(false, 0.1, 0.18));
    assert(!gate.readyForPosition(0.18));
    assert(!gate.markGainsStarted(0.18));
    feedback(gate, 0.18, true, 0.3, 0.3);
    assert(gate.readyForPosition(0.18));
    assert(gate.markGainsStarted(0.18));
    gate.step(0.29, 0.01);
    assert(gate.mode() == Gate::POSITION);
    assert(!gate.setCommand({1}, 0.1, 0.29));
  }
  {
    Gate gate({2}, {10}, 0.2, 0.01);
    torqueMode(gate);
    assert(gate.setCommand({1}, 0.5, 0.1));
    assert(gate.step(0.11, 0.01)[0] > 0);
    assert(gate.step(0.31, 0.01)[0] == 0);
    assert(gate.mode() == Gate::FAULT); // Feedback timeout.
    assert(gate.faultReason() == "qAct timeout");
    assert(!gate.resetAfterServoOff(0.31));
    feedback(gate, 0.31);
    assert(!gate.resetAfterServoOff(0.31)); // Fresh but ON.
    feedback(gate, 0.32, false);
    assert(gate.resetAfterServoOff(0.32));
    assert(gate.mode() == Gate::POSITION);
    assert(!gate.begin(true, 0.1, 0.32)); // Reset cleared previous angle samples.
    torqueMode(gate);
    assert(gate.step(0.11, 0.01)[0] == 0); // No replay of pre-fault torque.
  }
  {
    Gate gate({2}, {10}, 0.2, 0.01);
    torqueMode(gate);
    gate.updateActual({0}, 0.3);
    assert(gate.step(0.31, 0.01)[0] == 0);
    assert(gate.mode() == Gate::FAULT); // Servo feedback, not just angle, must be fresh.
    assert(gate.faultReason() == "servoState timeout");
  }
  {
    Gate gate({2}, {10}, 0.2, 0.01);
    torqueMode(gate);
    feedback(gate, 0.11, false);
    gate.step(0.11, 0.01);
    assert(gate.mode() == Gate::FAULT); // Servo OFF must invalidate torque mode.
  }
  {
    Gate gate({2}, {10}, 0.2, 0.01);
    torqueMode(gate);
    assert(!gate.setCommand({3}, 0.1, 0.1));
    assert(gate.mode() == Gate::FAULT); // Over-limit commands are rejected, not wrapped.
    assert(gate.faultReason() == "torque limit exceeded");
  }
  {
    Gate gate({0}, {10}, 0.2, 0.01);
    torqueMode(gate);
    assert(gate.setCommand({0}, 0.1, 0.1));
    assert(!gate.setCommand({0.01}, 0.1, 0.1));
  }
  for (double bad : {0.0, -1.0, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
    Gate gate({2}, {10}, 0.2, 0.01);
    torqueMode(gate);
    assert(!gate.setCommand({1}, bad, 0.1));
    assert(gate.step(0.11, 0.01)[0] == 0);
  }
  {
    Gate gate({2}, {10}, 0.2, 0.01);
    torqueMode(gate);
    assert(!gate.setCommand({}, 0.1, 0.1));
    assert(gate.mode() == Gate::FAULT);
  }
  {
    Gate gate({2}, {10}, 0.2, 0.01);
    torqueMode(gate);
    assert(!gate.setCommand({std::numeric_limits<double>::infinity()}, 0.1, 0.1));
  }
  {
    Gate gate({2}, {10}, 0.2, 0.01);
    gate.updateActual({std::numeric_limits<double>::quiet_NaN()}, 0);
    assert(gate.mode() == Gate::FAULT);
    assert(!gate.resetAfterServoOff(0));
  }
  {
    Gate gate({2}, {10}, 100, 0.01);
    feedback(gate, 0);
    assert(gate.begin(true, 0.1, 0));
    gate.step(5.2, 0.01); // hcf never acknowledged gain scheduling.
    assert(gate.mode() == Gate::FAULT);
  }
  {
    Gate gate({2}, {10}, 0.2, 0.01);
    feedback(gate, 0);
    assert(gate.begin(true, 0.3, 0));
    assert(gate.markGainsStarted(0));
    gate.updateActual({0}, 0.25);
    gate.updateServoState({true}, 0.25);
    gate.step(0.25, 0.01);
    assert(gate.mode() == Gate::FAULT); // Final qRef stream lost during transition.
  }
  bool rejected = false;
  try { Gate bad({2}, {0}, 0.2, 0.01); }
  catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected);
  std::cout << "DirectTorqueGate: all tests passed\n";
}
