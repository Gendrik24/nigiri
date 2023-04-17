#pragma once

#include <set>
#include <vector>

#include "cista/containers/flat_matrix.h"

#include "nigiri/routing/bmc_raptor_bag.h"
#include "nigiri/routing/bmc_raptor_label.h"
#include "nigiri/routing/journey.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/routing_time.h"
#include "nigiri/types.h"

namespace nigiri {
struct timetable;
}

namespace nigiri::routing {

struct bmc_raptor_search_state {
  void reset(timetable const& tt);

  std::vector<bmc_raptor_bag<bmc_raptor_label>> best_bags_;
  matrix<bmc_raptor_bag<bmc_raptor_label>> round_bags_;
  std::vector<bool> station_mark_;
  std::vector<bool> prev_station_mark_;
  std::vector<bool> route_mark_;
  std::vector<bool> is_destination_;
  std::vector<std::set<location_idx_t>> destinations_;
  std::vector<pareto_set<journey>> results_;
  interval<unixtime_t> search_interval_;
  std::vector<duration_t> travel_time_lower_bound_;
};

}  // namespace nigiri::routing
