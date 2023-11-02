#pragma once

#include <cstdint>
#include "nigiri/types.h"
#include <algorithm>
#include <limits>

namespace nigiri {
struct timetable;
struct footpath;
}  // namespace nigiri

namespace nigiri::routing {

struct query;

struct lower_bound {
  using value_type = std::uint16_t;

  static constexpr auto const kTotalBits = 16;
  static constexpr auto const kTravelTimeBits = 12;
  static constexpr auto const kTransportBits = kTotalBits - kTravelTimeBits;

  static constexpr auto const kTravelTimeUnreachable = 
    std::numeric_limits<value_type>::max() >> kTransportBits;

  static constexpr auto const kTransportsUnreachable = 
    std::numeric_limits<value_type>::max() >> kTravelTimeBits;

  bool unreachable() const noexcept {
    return travel_time_ == kTravelTimeUnreachable || transports_ == kTransportsUnreachable;
  }

  value_type travel_time_ : kTravelTimeBits;
  value_type transports_ : kTransportBits;

};

void dijkstra(timetable const&,
              query const&,
              vecvec<location_idx_t, footpath> const& lb_graph,
              std::vector<lower_bound>& dists);

void dijkstra(timetable const&,
              query const&,
              vecvec<component_idx_t, component_idx_t> const& lb_graph,
              std::vector<lower_bound>& dists);


}  // namespace nigiri::routing
