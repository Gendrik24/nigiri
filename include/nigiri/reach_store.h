#pragma once

#include "nigiri/types.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/journey.h"

namespace nigiri {

struct reach_t {
  std::uint8_t transport_reach_;
  std::uint16_t travel_time_reach_;
};

struct reach_stats {
  CISTA_PRINTABLE(reach_stats,
                  "compute_time_s",
                  "n_journeys_computed")
  std::uint64_t compute_time_s_{0ULL};
  std::uint64_t n_journeys_computed_{0ULL};
};

struct reach_store {
  interval<unixtime_t> valid_range_;

  // [ loc_0_reach | loc_1_reach | loc_2_reach | ... ]
  vector<reach_t> location_reach_;

  // Route 1:
  //   stop-1: [route-reach-in, route-reach-out, trip1-reach, trip2-reach, ..., tripN-reach]
  //   stop-2: [route-reach-in, route-reach-out, trip1-reach, trip2-reach, ..., tripN-reach]
  //   ...
  // Route 2:
  //  stop-1: [...]
  // ...
  // RouteN: ...
  vector_map<route_idx_t, interval<std::uint32_t>> route_reach_value_ranges_;
  vector<reach_t> reach_values_;

  reach_stats stats_;
};

} // namespace nigiri
