#pragma once

#include "nigiri/types.h"

#include <cinttypes>

namespace nigiri::routing {

struct arrival_departure_label {
  arrival_departure_label()
      : arrival_(minutes_after_midnight_t::min()),
        departure_(minutes_after_midnight_t::max())
      {}

  arrival_departure_label(minutes_after_midnight_t arrival,
                          minutes_after_midnight_t departure)
      : arrival_(arrival),
        departure_(departure)
      {}

  inline bool equals(const arrival_departure_label& other) const noexcept {
    return arrival_ == other.arrival_ && departure_ == other.departure_;
  }

  inline bool dominates(const arrival_departure_label& other) const noexcept {
    return arrival_ <= other.arrival_ && departure_ >= other.departure_;
  }

  inline arrival_departure_label add_day_offset(std::size_t o) {
    return arrival_departure_label{
        arrival_ + minutes_after_midnight_t{1440 * o},
        departure_ + minutes_after_midnight_t{1440 * o}};
  }

  minutes_after_midnight_t arrival_;
  minutes_after_midnight_t departure_;
};

}
