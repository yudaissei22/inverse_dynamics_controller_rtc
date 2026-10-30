#include "InverseDynamicsControllerService_impl.h"
#include "InverseDynamicsController.h"

InverseDynamicsControllerService_impl::InverseDynamicsControllerService_impl()
{
}

InverseDynamicsControllerService_impl::~InverseDynamicsControllerService_impl()
{
}

CORBA::Boolean InverseDynamicsControllerService_impl::startInverseDynamicsController()
{
  return this->comp_->startInverseDynamicsController();
};

CORBA::Boolean InverseDynamicsControllerService_impl::stopInverseDynamicsController()
{
  return this->comp_->stopInverseDynamicsController();
};

CORBA::Boolean InverseDynamicsControllerService_impl::setInverseDynamicsControllerParam(const OpenHRP::InverseDynamicsControllerService::InverseDynamicsControllerParam& i_param)
{
  return this->comp_->setInverseDynamicsControllerParam(i_param);
};

CORBA::Boolean InverseDynamicsControllerService_impl::getInverseDynamicsControllerParam(OpenHRP::InverseDynamicsControllerService::InverseDynamicsControllerParam_out i_param)
{
  i_param = new OpenHRP::InverseDynamicsControllerService::InverseDynamicsControllerParam();
  return this->comp_->getInverseDynamicsControllerParam(*i_param);
};

void InverseDynamicsControllerService_impl::setComp(InverseDynamicsController *i_comp)
{
  this->comp_ = i_comp;
}

CORBA::Boolean InverseDynamicsControllerService_impl::beginDirectControlTransition(
    CORBA::Boolean torque, CORBA::Double duration) {
  return comp_->beginDirectControlTransition(torque, duration);
}
CORBA::Boolean InverseDynamicsControllerService_impl::markDirectGainsStarted() {
  return comp_->markDirectGainsStarted();
}
CORBA::Boolean InverseDynamicsControllerService_impl::setDirectTorqueCommand(
    const OpenHRP::InverseDynamicsControllerService::DirectTorqueSequence& tau,
    CORBA::Double ttl) {
  return comp_->setDirectTorqueCommand(tau, ttl);
}
CORBA::Long InverseDynamicsControllerService_impl::getDirectControlMode() {
  return comp_->getDirectControlMode();
}
CORBA::Long InverseDynamicsControllerService_impl::getDirectServoStateMask() {
  return comp_->getDirectServoStateMask();
}
char* InverseDynamicsControllerService_impl::getDirectControlFaultReason() {
  return CORBA::string_dup(comp_->getDirectControlFaultReason().c_str());
}
CORBA::Boolean InverseDynamicsControllerService_impl::directTorqueIsZero() {
  return comp_->directTorqueIsZero();
}
CORBA::Boolean InverseDynamicsControllerService_impl::directPositionReferenceAligned() {
  return comp_->directPositionReferenceAligned();
}
void InverseDynamicsControllerService_impl::abortDirectControl() {
  comp_->abortDirectControl();
}
CORBA::Boolean InverseDynamicsControllerService_impl::resetDirectControlAfterServoOff() {
  return comp_->resetDirectControlAfterServoOff();
}
CORBA::Boolean InverseDynamicsControllerService_impl::startTransparency() {
  return comp_->startTransparency();
}
CORBA::Boolean InverseDynamicsControllerService_impl::stopTransparency() {
  return comp_->stopTransparency();
}
char* InverseDynamicsControllerService_impl::getTransparencyStatus() {
  return CORBA::string_dup(comp_->getTransparencyStatus().c_str());
}
