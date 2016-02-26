#include "iri_base_driver/iri_base_driver.h"

namespace iri_base_driver
{

IriBaseDriver::IriBaseDriver() : driver_id_("none")
{
  pthread_mutex_init(&this->access_,NULL);
}

void IriBaseDriver::lock(void)
{
  pthread_mutex_lock(&this->access_);
}

void IriBaseDriver::unlock(void)
{
  pthread_mutex_unlock(&this->access_);
}

bool IriBaseDriver::try_enter(void)
{
  if(pthread_mutex_trylock(&this->access_)==0)
    return true;
  else
    return false;
}

void IriBaseDriver::setDriverId(const std::string & id)
{
  driver_id_ = id;
}

void IriBaseDriver::doOpen(void)
{
  if(openDriver()) this->state_ = OPENED;
}

void IriBaseDriver::doClose(void)
{
  this->preCloseHook();
  if(closeDriver()) this->state_ = CLOSED;
}

void IriBaseDriver::doStart(void)
{
  if(startDriver()) this->state_ = RUNNING;
}

void IriBaseDriver::doStop(void)
{
  if(stopDriver()) this->state_ = OPENED;
}

std::string IriBaseDriver::getID(void)
{
  return driver_id_;
}

void IriBaseDriver::setPreCloseHook(hookFunction f)
{
  preCloseHook = f;
}

IriBaseDriver::~IriBaseDriver()
{
  pthread_mutex_destroy(&this->access_);
}

}
