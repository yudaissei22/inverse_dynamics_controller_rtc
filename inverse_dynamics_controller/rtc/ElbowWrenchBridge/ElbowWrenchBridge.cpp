#include "ElbowWrenchBridge.h"

#include <array>

namespace {
const char* specification[] = {
  "implementation_id", "ElbowWrenchBridge",
  "type_name", "ElbowWrenchBridge",
  "description", "Timestamp-preserving elbow WrenchStamped to RTC bridge",
  "version", "1.0",
  "vendor", "UronMan",
  "category", "Sensor",
  "activity_type", "PERIODIC",
  "kind", "DataFlowComponent",
  "max_instance", "1",
  "language", "C++",
  "lang_type", "compile",
  "wrench_topic", "/leptrino/wrench",
  "wrench_frame", "leptrino_elbow",
  ""
};
}  // namespace

ElbowWrenchBridge::ElbowWrenchBridge(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager), wrench_out_("wrenchOut", wrench_) {}

ElbowWrenchBridge::~ElbowWrenchBridge() {
  std::lock_guard<std::mutex> lock(execution_mutex_);
  stopSubscription();
}

RTC::ReturnCode_t ElbowWrenchBridge::onInitialize() {
  // This also supports loading the module into rtcd, whose main() does not
  // initialize ROS. Do not replace the manager's signal handlers.
  if (!ros::isInitialized()) {
    ros::M_string remappings;
    ros::init(remappings, "elbow_wrench_bridge",
              ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }
  topic_ = getProperties()["wrench_topic"];
  frame_ = getProperties()["wrench_frame"];
  if (topic_.empty() || frame_.empty()) {
    ROS_ERROR("ElbowWrenchBridge requires nonempty wrench_topic and wrench_frame profiles");
    return RTC::RTC_ERROR;
  }
  node_.reset(new ros::NodeHandle());
  node_->setCallbackQueue(&callback_queue_);
  wrench_.data.length(6);
  wrench_.tm.sec = 0;
  wrench_.tm.nsec = 0;
  addOutPort("wrenchOut", wrench_out_);
  ROS_INFO_STREAM("ElbowWrenchBridge: " << topic_ << " [" << frame_
                  << "] -> wrenchOut; source acquisition stamps are preserved");
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ElbowWrenchBridge::onActivated(RTC::UniqueId) {
  std::lock_guard<std::mutex> lock(execution_mutex_);
  stopSubscription();
  if (!node_ || !ros::ok()) return RTC::RTC_ERROR;
  // Subscribe only while active. Retain timestamp history over reactivation so
  // that a latched or replayed sample cannot reset the consumer's watchdog.
  subscriber_ = node_->subscribe(topic_, 1, &ElbowWrenchBridge::receiveWrench, this,
                                 ros::TransportHints().tcpNoDelay());
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ElbowWrenchBridge::onDeactivated(RTC::UniqueId) {
  std::lock_guard<std::mutex> lock(execution_mutex_);
  stopSubscription();
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ElbowWrenchBridge::onExecute(RTC::UniqueId) {
  std::lock_guard<std::mutex> lock(execution_mutex_);
  if (!ros::ok()) return RTC::RTC_ERROR;
  callback_queue_.callAvailable(ros::WallDuration(0.0));
  // No new ROS sample means no RTC write. In particular, never stamp or repeat
  // a cached value, which would hide a disconnected sensor from the controller.
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ElbowWrenchBridge::onFinalize() {
  std::lock_guard<std::mutex> lock(execution_mutex_);
  stopSubscription();
  node_.reset();
  return RTC::RTC_OK;
}

void ElbowWrenchBridge::stopSubscription() {
  subscriber_.shutdown();
  callback_queue_.clear();
}

void ElbowWrenchBridge::receiveWrench(
    const geometry_msgs::WrenchStamped::ConstPtr& message) {
  // Only onExecute invokes this callback, while holding execution_mutex_.
  const auto& value = message->wrench;
  std::array<double, 6> sample{{value.force.x, value.force.y, value.force.z,
                               value.torque.x, value.torque.y, value.torque.z}};
  const bool valid = validator_.validate(message->header.frame_id, frame_,
      message->header.stamp.sec, message->header.stamp.nsec, sample);
  // ROS and RTC both use unsigned 32-bit seconds and nanoseconds.
  wrench_.tm.sec = message->header.stamp.sec;
  wrench_.tm.nsec = message->header.stamp.nsec;
  for (std::size_t index = 0; index < sample.size(); ++index) wrench_.data[index] = sample[index];
  if (!valid) {
    ROS_WARN_THROTTLE(1.0, "ElbowWrenchBridge invalid frame, wrench, or acquisition stamp; forwarding invalid sample");
  }
  // Invalid packets must propagate as NaNs, instead of preserving a previously
  // valid force in the consumer until its timeout expires.
  wrench_out_.write();
}

extern "C" void ElbowWrenchBridgeInit(RTC::Manager* manager) {
  coil::Properties profile(specification);
  manager->registerFactory(profile, RTC::Create<ElbowWrenchBridge>, RTC::Delete<ElbowWrenchBridge>);
}
