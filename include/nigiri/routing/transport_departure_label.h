#pragma once

#include "nigiri/types.h"

#include <cinttypes>

namespace nigiri::routing {

struct transport_departure_label {
  transport_departure_label()
      : transport_(relative_transport{transport_idx_t::invalid(), relative_day_idx_t::invalid()}),
        departure_(minutes_after_midnight_t::max())
  {}

  transport_departure_label(relative_transport transport,
                          minutes_after_midnight_t departure)
      : transport_(transport),
        departure_(departure)
  {}

  inline bool equals(const transport_departure_label& other) const noexcept {
    return departure_ == other.departure_ && transport_.t_idx_ == other.transport_.t_idx_ && transport_.day_ == other.transport_.day_;
  }

  inline bool dominates(const transport_departure_label& other) const noexcept {
    return departure_ >= other.departure_ &&
           (transport_.day_ < other.transport_.day_ || (transport_.day_ == other.transport_.day_ && transport_.t_idx_ <= other.transport_.t_idx_));
  }

  inline transport_departure_label add_day_offset(std::size_t o) {
    return transport_departure_label{
        relative_transport{
            transport_.t_idx_,
            transport_.day_ + relative_day_idx_t {o}
        },
        departure_ + minutes_after_midnight_t{1440 * o}};
  }

  relative_transport transport_;
  minutes_after_midnight_t departure_;
};

}