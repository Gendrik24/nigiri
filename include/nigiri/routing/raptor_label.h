#pragma once

#include "nigiri/types.h"

namespace nigiri::routing {

struct raptor_label {
  raptor_label() = delete;
  raptor_label(const raptor_label& other) = default;
  raptor_label(minutes_after_midnight_t arrival,
               minutes_after_midnight_t departure,
               bitfield traffic_day_bitfield_)
                : arrival_(arrival),
                  departure_(departure),
                  traffic_day_bitfield_(traffic_day_bitfield_){};

  friend bool operator==(const raptor_label& lhs, const raptor_label& rhs) {
    return lhs.arrival_ == rhs.arrival_
           && lhs.departure_ == rhs.departure_
           && lhs.traffic_day_bitfield_ == rhs.traffic_day_bitfield_;
  }

  inline bool dominates(const raptor_label& other) const noexcept {
    return arrival_ <= other.arrival_
           && departure_ >= other.departure_
           && (traffic_day_bitfield_ | other.traffic_day_bitfield_) == traffic_day_bitfield_
           && ! (*this == other);
  }

  minutes_after_midnight_t arrival_;
  minutes_after_midnight_t departure_;
  bitfield traffic_day_bitfield_;
};

} // namespace nigiri
