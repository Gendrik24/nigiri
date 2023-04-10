#pragma once

#include "nigiri/types.h"
#include "nigiri/routing/routing_time.h"

#include <cinttypes>

namespace nigiri::routing {

struct mc_raptor_label {
  mc_raptor_label()
      : arrival_(routing_time::min()),
        departure_(routing_time::max())
  {}

  mc_raptor_label(routing_time arrival,
                  routing_time departure)
      : arrival_(arrival),
        departure_(departure)
  {}

  inline bool equals(const mc_raptor_label& other) const noexcept {
    return arrival_ == other.arrival_ && departure_ == other.departure_;
  }

  inline bool dominates(const mc_raptor_label& other) const noexcept {
    return arrival_ <= other.arrival_ && departure_ >= other.departure_;
  }

  routing_time arrival_;
  routing_time departure_;
};

}
