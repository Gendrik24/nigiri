#pragma once

#include "nigiri/types.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/journey.h"

namespace nigiri {

struct reach_t {
  std::uint8_t transport_reach_;
  std::uint16_t travel_time_reach_;
};

struct reach_store {
  interval<unixtime_t> valid_range_;

  // [ loc_0_reach | loc_1_reach | loc_2_reach | ... ]
  vector<reach_t> location_reach_;

  vector_map<route_idx_t, interval<std::uint32_t>> route_reach_value_ranges_;
  vector<reach_t> reach_values_;
};

} // namespace nigiri
