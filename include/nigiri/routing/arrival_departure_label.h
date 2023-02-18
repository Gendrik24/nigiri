#pragma once

#include "nigiri/types.h"
#include "nigiri/footpath.h"

#include <cinttypes>
#include <optional>

namespace nigiri::routing {

struct arrival_departure_label {

  arrival_departure_label()
      : arrival_(long_minutes_after_midnight_t ::min()),
        departure_(long_minutes_after_midnight_t ::max())
      {}

  arrival_departure_label(long_minutes_after_midnight_t arrival,
                          long_minutes_after_midnight_t departure)
      : arrival_(arrival),
        departure_(departure)
      {}

  arrival_departure_label(long_minutes_after_midnight_t arrival,
                          long_minutes_after_midnight_t departure,
                          std::optional<reconstruction_leg> leg)
      : arrival_(arrival),
        departure_(departure),
        leg_(leg)
      {}

  inline bool equals(const arrival_departure_label& other) const noexcept {
    return arrival_ == other.arrival_ && departure_ == other.departure_;
  }

  inline bool dominates(const arrival_departure_label& other) const noexcept {
    return arrival_ <= other.arrival_ && departure_ >= other.departure_;
  }

  inline arrival_departure_label add_day_offset(std::size_t o) const {
    if (leg_.has_value()) {
      reconstruction_leg rec_leg = leg_.value();
      rec_leg.uses_ = relative_transport{rec_leg.uses_.t_idx_,
                                         rec_leg.uses_.day_ + relative_day_idx_t{o}};
      return arrival_departure_label{
          arrival_ + long_minutes_after_midnight_t {1440 * o},
          departure_ + long_minutes_after_midnight_t {1440 * o},
          rec_leg
      };
    }
    return arrival_departure_label{
        arrival_ + long_minutes_after_midnight_t {1440 * o},
        departure_ + long_minutes_after_midnight_t {1440 * o},
        std::nullopt};
  }

  inline long_minutes_after_midnight_t get_travel_time() const {
    return arrival_ - departure_;
  }

  long_minutes_after_midnight_t arrival_;
  long_minutes_after_midnight_t departure_;
  std::optional<reconstruction_leg> leg_;
};

}
