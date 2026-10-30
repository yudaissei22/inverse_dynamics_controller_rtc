#ifndef INVERSE_DYNAMICS_CONTROLLER_ELBOW_WRENCH_BRIDGE_H
#define INVERSE_DYNAMICS_CONTROLLER_ELBOW_WRENCH_BRIDGE_H

#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/Manager.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <memory>
#include <mutex>
#include <string>
#include "WrenchSampleValidator.h"

// A separate PeriodicExecutionContext services the private ROS callback queue.
// No asynchronous spinner accesses the RTC port or component state.
class ElbowWrenchBridge : public RTC::DataFlowComponentBase {
public:
  explicit ElbowWrenchBridge(RTC::Manager* manager);
  ~ElbowWrenchBridge() override;

  RTC::ReturnCode_t onInitialize() override;
  RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id) override;
  RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id) override;
  RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id) override;
  RTC::ReturnCode_t onFinalize() override;

private:
  void receiveWrench(const geometry_msgs::WrenchStamped::ConstPtr& message);
  void stopSubscription();

  RTC::TimedDoubleSeq wrench_;
  RTC::OutPort<RTC::TimedDoubleSeq> wrench_out_;
  std::string topic_;
  std::string frame_;
  std::mutex execution_mutex_;
  ros::CallbackQueue callback_queue_;
  std::unique_ptr<ros::NodeHandle> node_;
  ros::Subscriber subscriber_;
  inverse_dynamics_controller::WrenchSampleValidator validator_;
};

extern "C" void ElbowWrenchBridgeInit(RTC::Manager* manager);

#endif
