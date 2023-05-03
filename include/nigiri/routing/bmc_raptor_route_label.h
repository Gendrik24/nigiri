#pragma once

#include "nigiri/types.h"

#include <cinttypes>

namespace nigiri::routing {

struct bmc_raptor_route_label {
  bmc_raptor_route_label()
      : transport_(relative_transport{transport_idx_t::invalid(), relative_day_idx_t::invalid()}),
        departure_(minutes_after_midnight_t::max()),
        current_stop_time_(long_minutes_after_midnight_t::max()),
        walking_time_(minutes_after_midnight_t::max())
  {}

  bmc_raptor_route_label(relative_transport transport,
                         minutes_after_midnight_t departure,
                         long_minutes_after_midnight_t current_stop_time,
                         minutes_after_midnight_t walking_time)
      : transport_(transport),
        departure_(departure),
        current_stop_time_(current_stop_time),
        walking_time_(walking_time)
  {}

  inline bool equals(const bmc_raptor_route_label& other) const noexcept {
    return  departure_ == other.departure_ &&
            transport_.t_idx_ == other.transport_.t_idx_ &&
            transport_.day_ == other.transport_.day_ &&
            walking_time_ == other.walking_time_;
  }

  inline bool dominates(const bmc_raptor_route_label& other) const noexcept {
    return  departure_ >= other.departure_ &&
            current_stop_time_ <= other.current_stop_time_ &&
            walking_time_ <= other.walking_time_;
  }

  inline bmc_raptor_route_label add_day_offset(std::size_t o) {
    return bmc_raptor_route_label{
        relative_transport{
            transport_.t_idx_,
            transport_.day_ + relative_day_idx_t {o}
        },
        departure_ + minutes_after_midnight_t{1440 * o},
        current_stop_time_ + minutes_after_midnight_t{1440 * o},
        walking_time_};
  }

  relative_transport transport_;
  minutes_after_midnight_t departure_;
  long_minutes_after_midnight_t current_stop_time_;
  minutes_after_midnight_t walking_time_;
};

}