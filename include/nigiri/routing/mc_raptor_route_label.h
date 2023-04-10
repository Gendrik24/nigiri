#pragma once

#include "nigiri/types.h"
#include "nigiri/routing/routing_time.h"

#include <cinttypes>

namespace nigiri::routing {

struct mc_raptor_route_label {
  mc_raptor_route_label()
      : transport_(transport{transport_idx_t::invalid(), day_idx_t::invalid()}),
        departure_(routing_time::max())
  {}

  mc_raptor_route_label(transport transport,
                        routing_time departure)
      : transport_(transport),
        departure_(departure)
  {}

  inline bool equals(const mc_raptor_route_label& other) const noexcept {
    return departure_ == other.departure_ && transport_.t_idx_ == other.transport_.t_idx_ && transport_.day_ == other.transport_.day_;
  }

  inline bool dominates(const mc_raptor_route_label& other) const noexcept {
    return departure_ >= other.departure_ &&
           (transport_.day_ < other.transport_.day_ || (transport_.day_ == other.transport_.day_ && transport_.t_idx_ <= other.transport_.t_idx_));
  }

  transport transport_;
  routing_time departure_;
};

}