// Offline integration of the actual IDC services/onExecute with a synthetic
// pendulum. No ROS, hardware, sensor, or RTC data-port connection is created.
#include "InverseDynamicsController.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <thread>
#include <sys/time.h>

namespace {
void require(bool condition, const char* description) {
  if (!condition) {
    std::cerr << "FAIL: " << description << std::endl;
    std::exit(EXIT_FAILURE);
  }
}
void near(double actual, double expected, const char* description) {
  require(std::isfinite(actual) && std::abs(actual - expected) < 1e-9, description);
}
double monotonic() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
double wall() {
  timeval now;
  gettimeofday(&now, nullptr);
  return now.tv_sec + now.tv_usec * 1e-6;
}

class ControllerHarness : public InverseDynamicsController {
public:
  explicit ControllerHarness(RTC::Manager* manager, double torque_limit = 20,
                             bool gravity_only = false, double slew_rate = 1000000)
      : InverseDynamicsController(manager) {
    dt_ = 0.002;
    loop_ = 0;
    direct_gate_.reset(new DirectTorqueGate({torque_limit}, {slew_rate}, 0.2, 0.01));
    cnoid::BodyPtr body = new cnoid::Body;
    auto base = new cnoid::Link;
    base->setName("base");
    base->setJointType(cnoid::Link::FIXED_JOINT);
    base->setInertia(cnoid::Matrix3::Zero());
    body->setRootLink(base);
    auto joint = new cnoid::Link;
    joint->setName("elbow");
    joint->setJointType(cnoid::Link::REVOLUTE_JOINT);
    joint->setJointId(0);
    joint->setJointAxis(cnoid::Vector3::UnitY());
    joint->setMass(2.0);
    joint->setCenterOfMass(cnoid::Vector3(0, 0, -0.4));
    joint->setInertia(0.05 * cnoid::Matrix3::Identity());
    base->appendChild(joint);
    body->updateLinkTree();
    TransparencyDynamics::Config dynamics;
    dynamics.sensor_link = "elbow";
    dynamics.gravity_only = gravity_only;
    transparency_gravity_only_ = gravity_only;
    dynamics.viscous = {0};
    dynamics.coulomb = {0};
    transparency_dynamics_.reset(new TransparencyDynamics(body, dynamics));
    TransparencyInputs::Config inputs;
    inputs.require_wrench = !gravity_only;
    inputs.joints = 1;
    inputs.timeout = 0.2;
    inputs.max_skew = 0.05;
    inputs.velocity_cutoff_hz = 10;
    inputs.acceleration_cutoff_hz = 10;
    inputs.max_velocity = {13.962634016};
    inputs.max_acceleration = {100};
    inputs.wrench_limits.fill(100);
    transparency_inputs_.reset(new TransparencyInputs(inputs));
    stamp_ = wall() - 0.01;
  }

  ~ControllerHarness() override {
    // RTObject_impl implicitly activates itself in its constructor, even when
    // initialize()/execution contexts are intentionally skipped. This fixture
    // has automatic storage, so revoke that POA activation before destruction.
    PortableServer::POA_var poa = m_pManager->getPOA();
    PortableServer::ObjectId_var id = poa->servant_to_id(this);
    poa->deactivate_object(id);
  }

  void enterTorque(bool include_force = true) {
    // Complete a synthetic gain ramp before exercising the public services.
    const double now = monotonic();
    direct_gate_->updateActual({0.4}, now - 0.02);
    direct_gate_->updatePositionReference({0.4}, now - 0.02);
    direct_gate_->updateServoState({true}, now - 0.02);
    require(direct_gate_->begin(true, 0.001, now - 0.02), "synthetic torque transition begins");
    require(direct_gate_->markGainsStarted(now - 0.019), "synthetic gain ramp acknowledged");
    direct_gate_->step(now - 0.017, dt_);
    require(getDirectControlMode() == DirectTorqueGate::TORQUE, "synthetic ramp reaches TORQUE");
    refresh(0, include_force);
    refresh(0, include_force); // Acceleration requires two independent velocity samples.
  }

  void refresh(double human_moment = 0.0, bool include_force = true) {
    const double now = monotonic();
    const double current_wall = wall();
    stamp_ = std::max(stamp_ + 0.001, current_wall - 0.001);
    direct_gate_->updateActual({0.4}, now);
    direct_gate_->updatePositionReference({0.4}, now);
    direct_gate_->updateServoState({true}, now);
    require(transparency_inputs_->position({0.4}, stamp_, now, current_wall), "position accepted");
    require(transparency_inputs_->velocity({0}, stamp_, now, current_wall), "velocity accepted");
    if (include_force)
      require(transparency_inputs_->wrench({0, 0, 0, 0, human_moment, 0}, stamp_, now, current_wall), "wrench accepted");
    ports_.m_qAct_.tm.sec = static_cast<CORBA::ULong>(stamp_);
    ports_.m_qAct_.tm.nsec = static_cast<CORBA::ULong>((stamp_ - ports_.m_qAct_.tm.sec) * 1e9);
    ports_.m_dqAct_.tm = ports_.m_qAct_.tm;
    ports_.m_elbowWrench_.tm = ports_.m_qAct_.tm;
  }

  void invalidForce() {
    stamp_ += 0.001;
    transparency_inputs_->wrench({0, 0, std::numeric_limits<double>::quiet_NaN(), 0, 0, 0},
                                stamp_, monotonic(), wall());
  }
  void excessiveVelocity() {
    stamp_ += 0.001;
    require(!transparency_inputs_->velocity({14}, stamp_, monotonic(), wall()),
            "raw velocity over 800 deg/s is rejected");
  }
  void staleForce() {
    // Only force has stopped: replenish actual/servo/velocity feedback after
    // the timeout, including two velocity samples to reinitialize acceleration.
    std::this_thread::sleep_for(std::chrono::milliseconds(210));
    refresh(0, false);
    refresh(0, false);
  }
  double output() const { return ports_.m_tau_.data[0]; }
  double modelOutput() const { return ports_.m_tauModel_.data[0]; }
  double humanOutput() const { return ports_.m_tauHuman_.data[0]; }
  size_t humanSamplesWidth() const { return ports_.m_tauHuman_.data.length(); }
  size_t wrenchSamplesWidth() const { return ports_.m_elbowWrenchOutData_.data.length(); }
  bool active() const { return transparency_active_; }
  void tick() { require(onExecute(0) == RTC::RTC_OK, "controller cycle completed"); }

private:
  double stamp_;
};

OpenHRP::InverseDynamicsControllerService::DirectTorqueSequence manual(double value) {
  OpenHRP::InverseDynamicsControllerService::DirectTorqueSequence command;
  command.length(1);
  command[0] = value;
  return command;
}
}  // namespace

int main() {
  // Naming and manager servant are disabled; this manager cannot discover or
  // connect to the running robot. The test never creates an execution context.
  char program[] = "test_transparency_control";
  char option[] = "-o";
  char no_names[] = "naming.enable:NO";
  char no_servant[] = "manager.corba_servant:NO";
  char keep_manager[] = "manager.shutdown_on_nortcs:NO";
  char endpoint[] = "corba.endpoints:127.0.0.1:0";
  char* arguments[] = {program, option, no_names, option, no_servant,
                       option, keep_manager, option, endpoint};
  RTC::Manager* manager = RTC::Manager::init(9, arguments);
  require(manager->activateManager(), "offline CORBA manager activates");
  // shutdown() synchronizes with the ORB runner. Without this runner OpenRTM
  // 1.1 waits for a nonexistent blocking runManager() caller indefinitely.
  manager->runManager(true);
  const double gravity = 2.0 * 9.80665 * 0.4 * std::sin(0.4);
  {
    ControllerHarness controller(manager);
    controller.enterTorque(false);
    require(!controller.startTransparency(), "full compensation still requires a force sensor");
    require(controller.getTransparencyStatus().find("elbowWrench: no sample") != std::string::npos,
            "missing required force is visible in status");
    controller.tick();
    near(controller.output(), 0, "missing required force cannot arm compensation");
  }
  {
    ControllerHarness controller(manager, 20, true, 1000);
    controller.refresh(0, false);
    controller.refresh(0, false);
    require(!controller.beginDirectControlTransition(true, 0.001),
            "gravity torque transition rejects absent producer before gain removal");
    require(controller.getDirectControlMode() == DirectTorqueGate::POSITION,
            "rejected gravity transition preserves POSITION");
    require(controller.startTransparency(), "gravity starts without force sensor");
    controller.tick(); // 2 Nm per cycle: below this pendulum's gravity target.
    require(controller.getTransparencyStatus().find("\"ready\":true") != std::string::npos,
            "input readiness is independent of torque output ramp");
    require(controller.getTransparencyStatus().find("\"output_ready\":false") != std::string::npos,
            "status reports incomplete gravity slew");
    require(!controller.beginDirectControlTransition(true, 0.001),
            "partial gravity output cannot authorize gain removal");
    require(controller.getDirectControlMode() == DirectTorqueGate::POSITION,
            "partial preload rejection preserves POSITION");
    require(controller.humanSamplesWidth() == 0 && controller.wrenchSamplesWidth() == 0,
            "missing force sensor does not publish fabricated contact measurements");
    controller.refresh(0, false);
    controller.tick();
    near(controller.output(), gravity, "bounded gravity output catches up");
    require(controller.getTransparencyStatus().find("\"output_ready\":true") != std::string::npos,
            "status reports completed gravity preload");
    require(controller.beginDirectControlTransition(true, 0.001),
            "complete sensorless gravity preload permits torque transition");
    require(controller.markDirectGainsStarted(), "supported torque gain ramp acknowledged");
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    controller.refresh(0, false);
    controller.tick();
    require(controller.getDirectControlMode() == DirectTorqueGate::TORQUE,
            "sensorless gravity reaches TORQUE");
    controller.invalidForce();
    controller.tick();
    require(controller.active(), "invalid optional wrench cannot disable gravity");
    near(controller.output(), gravity, "invalid optional wrench does not affect compensation");
    require(controller.humanSamplesWidth() == 0 && controller.wrenchSamplesWidth() == 0,
            "invalid optional wrench is not published as a measurement");
  }
  {
    ControllerHarness controller(manager, 20, true);
    controller.refresh();
    controller.refresh();
    require(controller.startTransparency(), "gravity can preload in POSITION");
    controller.tick();
    near(controller.output(), gravity, "gravity preloads before gain removal");
    require(!controller.setDirectTorqueCommand(manual(1), 0.1), "manual cannot replace preload");
    require(controller.beginDirectControlTransition(true, 0.001), "supported torque transition starts");
    require(controller.markDirectGainsStarted(), "torque gains acknowledged");
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    controller.refresh();
    controller.tick();
    require(controller.getDirectControlMode() == DirectTorqueGate::TORQUE, "gravity reaches TORQUE");
    require(controller.active(), "gravity stays active across gain removal");
    near(controller.output(), gravity, "no gravity gap on TORQUE entry");
    require(controller.beginDirectControlTransition(false, 0.001), "supported position transition starts");
    require(controller.directPositionReferenceAligned(), "aligned return accepts gravity feedforward");
    require(controller.markDirectGainsStarted(), "position gains acknowledged");
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    controller.refresh();
    controller.tick();
    require(controller.getDirectControlMode() == DirectTorqueGate::POSITION, "gravity returns to POSITION");
    near(controller.output(), gravity, "no gravity gap on POSITION entry");
    controller.staleForce();
    controller.tick();
    require(controller.getDirectControlMode() == DirectTorqueGate::POSITION,
            "stale optional force leaves gravity POSITION support active");
    near(controller.output(), gravity, "stale optional force does not remove gravity");
    controller.excessiveVelocity();
    controller.tick();
    require(controller.getDirectControlMode() == DirectTorqueGate::FAULT,
            "gravity preload still monitors joint velocity");
    near(controller.output(), 0, "invalid motion feedback clears gravity torque");
  }
  {
    ControllerHarness controller(manager, 20, true);
    controller.refresh(0, false);
    controller.refresh(0, false);
    require(controller.startTransparency(), "gravity starts before stop-versus-transition check");
    controller.tick();
    require(controller.stopTransparency(), "stop revokes gravity producer");
    require(!controller.beginDirectControlTransition(true, 0.001),
            "gravity stopped after preload cannot authorize gain removal");
    require(controller.getDirectControlMode() == DirectTorqueGate::POSITION,
            "stopped gravity transition rejection preserves position gains mode");
  }
  {
    ControllerHarness controller(manager);
    require(!controller.startTransparency(), "POSITION mode cannot start transparency");
    controller.enterTorque();
    // A queued manual command has not yet produced output. Starting transparency
    // must replace it atomically, rather than replaying it for the first cycle.
    require(controller.setDirectTorqueCommand(manual(8.0), 0.1), "manual torque accepted before start");
    require(controller.startTransparency(), "fresh TORQUE mode starts transparency");
    require(!controller.setDirectTorqueCommand(manual(8.0), 0.1), "manual writer excluded while active");
    controller.tick();
    near(controller.output(), gravity, "first motor output is measured-state gravity, not queued manual torque");
    near(controller.modelOutput(), gravity, "model diagnostic agrees with motor torque");
    near(controller.humanOutput(), 0, "zero contact diagnostic");
    controller.refresh(12.0);
    controller.tick();
    near(controller.humanOutput(), 12.0, "contact moment is monitored");
    near(controller.output(), gravity, "human moment does not alter motor compensation");
    require(controller.stopTransparency(), "stop accepted");
    controller.tick();
    near(controller.output(), 0, "stop revokes compensation command");
    require(!controller.active(), "stop disarms transparency");
    require(controller.setDirectTorqueCommand(manual(1.0), 0.1), "manual writer available after stop");
    controller.tick();
    near(controller.output(), 1, "manual command resumes after stop");
    require(!controller.startTransparency(), "nonzero manual output prevents handover");
  }
  {
    ControllerHarness controller(manager);
    controller.enterTorque();
    require(controller.startTransparency(), "start before invalid input");
    controller.tick();
    controller.invalidForce();
    controller.tick();
    require(controller.getDirectControlMode() == DirectTorqueGate::FAULT, "invalid force latches FAULT");
    require(!controller.active(), "fault disarms transparency");
    near(controller.output(), 0, "invalid input clears torque in the same cycle");
    controller.refresh();
    require(!controller.startTransparency(), "fresh input alone does not clear latched fault");
  }
  {
    ControllerHarness controller(manager);
    controller.enterTorque();
    require(controller.startTransparency(), "start before excessive velocity");
    controller.tick();
    controller.excessiveVelocity();
    controller.tick();
    require(controller.getDirectControlMode() == DirectTorqueGate::FAULT,
            "excessive velocity latches FAULT");
    require(!controller.active(), "velocity fault disarms transparency");
    near(controller.output(), 0, "excessive velocity clears torque in the same cycle");
    const std::string fault_status = controller.getTransparencyStatus();
    const std::string fault_reason = controller.getDirectControlFaultReason();
    require(fault_status.find("velocity limit exceeded") != std::string::npos,
            "status includes velocity fault details");
    controller.refresh();
    controller.refresh();
    controller.tick();
    require(controller.getDirectControlFaultReason() == fault_reason &&
            controller.getTransparencyStatus().find(fault_reason) != std::string::npos,
            "first velocity fault survives recovery of input values");
    require(!controller.startTransparency(), "fresh velocity does not clear latched fault");
    near(controller.output(), 0, "recovered velocity does not rearm torque");
  }
  {
    ControllerHarness controller(manager);
    controller.enterTorque();
    require(controller.startTransparency(), "start before stale input");
    controller.tick();
    controller.staleForce();
    controller.tick();
    require(controller.getDirectControlMode() == DirectTorqueGate::FAULT, "stale force faults control");
    near(controller.output(), 0, "stale force clears torque");
  }
  {
    ControllerHarness controller(manager);
    controller.enterTorque();
    require(controller.startTransparency(), "start before position transition");
    controller.tick();
    require(controller.beginDirectControlTransition(false, 0.01), "position transition accepted");
    require(!controller.active(), "position transition disarms producer");
    controller.tick();
    near(controller.output(), 0, "position transition clears old compensation");
    require(!controller.startTransparency(), "transition mode cannot rearm producer");
  }
  {
    ControllerHarness controller(manager);
    controller.enterTorque();
    require(controller.startTransparency(), "start before deactivation");
    controller.tick();
    controller.onDeactivated(0);
    near(controller.output(), 0, "RTC deactivation writes zero torque");
    require(!controller.active(), "RTC deactivation disarms producer");
    controller.onActivated(0);
    require(!controller.startTransparency(), "RTC reactivation cannot replay compensation");
  }
  {
    ControllerHarness zero_limit(manager, 0);
    zero_limit.enterTorque();
    require(!zero_limit.startTransparency(), "zero torque limit rejects nonzero model before arming");
    require(zero_limit.getDirectControlMode() == DirectTorqueGate::FAULT, "limit violation is latched");
    zero_limit.tick();
    near(zero_limit.output(), 0, "rejected start never produces torque");
  }
  std::cout << "Transparency controller integration checks passed" << std::endl;
  manager->shutdown();
  return EXIT_SUCCESS;
}
