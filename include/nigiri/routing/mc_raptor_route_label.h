#pragma once

#include "nigiri/types.h"
#include "nigiri/routing/routing_time.h"

#include <cinttypes>

namespace nigiri::routing {

struct mc_raptor_route_label {
  mc_raptor_route_label()
      : transport_(transport{transport_idx_t::invalid(), day_idx_t::invalid()}),
        departure_(routing_time::max()),
        walking_time_(minutes_after_midnight_t::max())
  {}

  mc_raptor_route_label(transport transport,
                        routing_time departure,
                        minutes_after_midnight_t walking_time)
      : transport_(transport),
        departure_(departure),
        walking_time_(walking_time)
  {}

  inline bool equals(const mc_raptor_route_label& other) const noexcept {
    return  departure_ == other.departure_ &&
            transport_.t_idx_ == other.transport_.t_idx_ &&
            transport_.day_ == other.transport_.day_ &&
            walking_time_ == other.walking_time_;
  }

  inline bool dominates(const mc_raptor_route_label& other) const noexcept {
    return  departure_ >= other.departure_ &&
            (transport_.day_ < other.transport_.day_ || (transport_.day_ == other.transport_.day_ && transport_.t_idx_ <= other.transport_.t_idx_)) &&
            walking_time_ <= other.walking_time_;
  }

  transport transport_;
  routing_time departure_;
  minutes_after_midnight_t walking_time_;
};

}