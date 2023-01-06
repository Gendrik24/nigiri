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

struct raptor_label {
  raptor_label() = default;
  raptor_label(minutes_after_midnight_t arrival,
               minutes_after_midnight_t departure,
               dynamic_bitfield traffic_day_bitfield)
                : arrival_(arrival),
                  departure_(departure),
                  traffic_day_bitfield_(traffic_day_bitfield){};

  raptor_label(minutes_after_midnight_t arrival,
               minutes_after_midnight_t departure,
               dynamic_bitfield traffic_day_bitfield,
               transport t)
      : arrival_(arrival),
        departure_(departure),
        traffic_day_bitfield_(traffic_day_bitfield),
        t_(t){};

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

  std::vector<std::pair<routing_time, routing_time>> resolve_label(const uint32_t day_offset) const noexcept {
    std::vector<std::pair<routing_time, routing_time>> res;
    auto tdb = traffic_day_bitfield_;
    for (std::size_t i = 0U; i < tdb.size(); ++i) {
      if (!tdb.any()) {
        break;
      }
      if (tdb[0U]) {
        const day_idx_t base{day_offset + i};
        res.push_back({
            routing_time(base, departure_),
            routing_time(base, arrival_)
        });
      }

      tdb >>= 1U;
    }

    return res;
  }

  inline void assign_transport(const transport t) noexcept {
    this->t_ = t;
  }

  minutes_after_midnight_t arrival_;
  minutes_after_midnight_t departure_;
  dynamic_bitfield traffic_day_bitfield_;
  transport t_;
};

typedef pareto_set<raptor_label> raptor_bag;

} // namespace nigiri::routing
