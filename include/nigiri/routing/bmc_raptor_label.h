#pragma once

#include "nigiri/types.h"

#include <cinttypes>

namespace nigiri::routing {

struct bmc_raptor_label {
  bmc_raptor_label()
      : arrival_(long_minutes_after_midnight_t ::min()),
        departure_(long_minutes_after_midnight_t ::max()),
        walking_time_(minutes_after_midnight_t::max())
      {}

  bmc_raptor_label(long_minutes_after_midnight_t arrival,
                          long_minutes_after_midnight_t departure,
                          minutes_after_midnight_t walking_time)
      : arrival_(arrival),
        departure_(departure),
        walking_time_(walking_time)
      {}

  inline bool equals(const bmc_raptor_label& other) const noexcept {
    return arrival_ == other.arrival_ &&
           departure_ == other.departure_ &&
           walking_time_ == other.walking_time_;
  }

  inline bool dominates(const bmc_raptor_label& other) const noexcept {
    return arrival_ <= other.arrival_ &&
           departure_ >= other.departure_ &&
           walking_time_ <= other.walking_time_;
  }

  inline bmc_raptor_label add_day_offset(std::size_t o) const {
    return bmc_raptor_label{
        arrival_ + long_minutes_after_midnight_t {1440 * o},
        departure_ + long_minutes_after_midnight_t {1440 * o},
        walking_time_};
  }

  long_minutes_after_midnight_t arrival_;
  long_minutes_after_midnight_t departure_;
  minutes_after_midnight_t walking_time_;
};

}
