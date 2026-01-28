#include "agnocast/agnocast_timer.hpp"

#include "agnocast/agnocast_timer_info.hpp"

namespace agnocast
{

TimerBase::~TimerBase()
{
  unregister_timer_info(timer_id_);
}

}  // namespace agnocast
