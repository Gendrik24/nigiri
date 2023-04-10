#pragma once

#include <set>
#include <vector>

#include "cista/containers/flat_matrix.h"

#include "nigiri/routing/journey.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/mc_raptor_bag.h"
#include "nigiri/routing/mc_raptor_label.h"
#include "nigiri/types.h"
#include "nigiri/routing/start_times.h"


namespace nigiri {
struct timetable;
}

namespace nigiri::routing {

struct mc_raptor_search_state {
  void reset(timetable const& tt);

  std::vector<mc_raptor_bag<mc_raptor_label>> best_bags_;
  matrix<mc_raptor_bag<mc_raptor_label>> round_bags_;
  std::vector<bool> station_mark_;
  std::vector<bool> prev_station_mark_;
  std::vector<bool> route_mark_;
  std::vector<bool> is_destination_;
  std::vector<std::set<location_idx_t>> destinations_;
  std::vector<pareto_set<journey>> results_;
  interval<unixtime_t> search_interval_;
  std::vector<duration_t> travel_time_lower_bound_;
  std::vector<start> starts_;
};

}  // namespace nigiri::routing
