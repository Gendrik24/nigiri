#pragma once

#include "nigiri/types.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/journey.h"
#include "nigiri/footpath.h"
#include "nigiri/dynamic_bitfield.h"
#include "nigiri/timetable.h"
#include "nigiri/routing/routing_time.h"

#include "cista/reflection/printable.h"

#include <variant>
#include <sstream>

namespace nigiri::routing {

struct raptor_route_label {
  raptor_route_label() = default;
  raptor_route_label(minutes_after_midnight_t departure,
                     relative_transport transport,
                     dynamic_bitfield traffic_day_bitfield)
      : departure_(departure),
        traffic_day_bitfield_(traffic_day_bitfield),
        t_(transport) {}


  friend bool operator==(const raptor_route_label& lhs,
                         const raptor_route_label& rhs) {
    return lhs.departure_ == rhs.departure_ &&  lhs.t_.day_ == rhs.t_.day_ &&
           lhs.t_.t_idx_ == rhs.t_.t_idx_ && lhs.traffic_day_bitfield_ == rhs.traffic_day_bitfield_;
  }

  std::string to_string() const noexcept {
    std::stringstream ss;
    ss << "[dep=" << departure_ << ", trip=(" << t_.t_idx_ << " + " << t_.day_ << ")"
       << ", tdb=" << traffic_day_bitfield_.to_string() << "]";
    return ss.str();
  }

  inline bool dominates(const raptor_route_label& other) const noexcept {
    return departure_ >= other.departure_ && (t_.day_ < other.t_.day_ || (t_.day_ == other.t_.day_ && t_.t_idx_ <= other.t_.t_idx_)) &&
           (traffic_day_bitfield_ | other.traffic_day_bitfield_) ==
               traffic_day_bitfield_;
  }

  minutes_after_midnight_t departure_;
  dynamic_bitfield traffic_day_bitfield_;
  relative_transport t_;
};

}