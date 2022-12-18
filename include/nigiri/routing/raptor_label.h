#pragma once

#include "nigiri/types.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/journey.h"
#include "nigiri/footpath.h"
#include "nigiri/dynamic_bitfield.h"

#include "cista/reflection/printable.h"

#include <variant>
#include <sstream>

namespace nigiri::routing {

struct raptor_label {
  raptor_label() = default;
  raptor_label(minutes_after_midnight_t arrival,
               minutes_after_midnight_t departure,
               dynamic_bitfield traffic_day_bitfield)
                : arrival_(arrival),
                  departure_(departure),
                  traffic_day_bitfield_(traffic_day_bitfield){};

  friend bool operator==(const raptor_label& lhs, const raptor_label& rhs) {
    return lhs.arrival_ == rhs.arrival_
           && lhs.departure_ == rhs.departure_
           && lhs.traffic_day_bitfield_ == rhs.traffic_day_bitfield_;
  }

  std::string to_string() const noexcept {
    std::stringstream ss;
    ss << "[arr=" << arrival_ << ", dep=" << departure_ << ", tdb=" << traffic_day_bitfield_.to_string() << "]";
    return ss.str();
  }

  inline bool dominates(const raptor_label& other) const noexcept {
    return arrival_ <= other.arrival_
           && departure_ >= other.departure_
           && (traffic_day_bitfield_ | other.traffic_day_bitfield_) == traffic_day_bitfield_
           && ! (*this == other);
  }

  minutes_after_midnight_t arrival_;
  minutes_after_midnight_t departure_;
  dynamic_bitfield traffic_day_bitfield_;
};

struct raptor_route_label : public raptor_label {
  raptor_route_label(minutes_after_midnight_t arrival,
                     minutes_after_midnight_t departure,
                     dynamic_bitfield traffic_day_bitfield,
                     transport t) : raptor_label(arrival, departure, traffic_day_bitfield), t_{t} {};
  transport t_;
};

typedef pareto_set<raptor_label> raptor_bag;

typedef pareto_set<raptor_route_label> route_bag;
} // namespace nigiri::routing
