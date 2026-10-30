#include "ElbowWrenchBridge.h"

namespace {
void initializeModule(RTC::Manager* manager) {
  ElbowWrenchBridgeInit(manager);
  manager->createComponent("ElbowWrenchBridge?instance_name=elbow_wrench_bridge");
}
}  // namespace

int main(int argc, char** argv) {
  RTC::Manager* manager = RTC::Manager::init(argc, argv);
  ros::init(argc, argv, "elbow_wrench_bridge", ros::init_options::NoSigintHandler);
  manager->setModuleInitProc(initializeModule);
  manager->activateManager();
  manager->runManager();
  ros::shutdown();
  return 0;
}
