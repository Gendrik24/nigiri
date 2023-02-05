#pragma once

#include <set>
#include <vector>

#include "cista/containers/flat_matrix.h"

#include "nigiri/routing/journey.h"
#include "nigiri/routing/pareto_set.h"
#include "nigiri/routing/routing_time.h"
#include "nigiri/routing/start_times.h"
#include "nigiri/routing/raptor_label.h"
#include "nigiri/routing/bmc_raptor_bag.h"
#include "nigiri/routing/arrival_departure_label.h"
#include "nigiri/types.h"


namespace nigiri {
struct timetable;
}

namespace nigiri::routing {

struct destination_comparator {
  explicit destination_comparator(timetable const&);
  bool operator()(location_idx_t, location_idx_t);
  timetable const& tt_;
};

struct search_state {
  void reset(timetable const& tt, routing_time init);

  std::vector<duration_t> travel_time_lower_bound_;
  std::vector<routing_time> best_;
  matrix<routing_time> round_times_;
  std::vector<bool> station_mark_;
  std::vector<bool> prev_station_mark_;
  std::vector<bool> route_mark_;
  std::vector<bool> is_destination_;
  std::vector<start> starts_;
  std::vector<std::set<location_idx_t>> destinations_;
  std::vector<pareto_set<journey>> results_;
  interval<unixtime_t> search_interval_;
};

struct profile_search_state {
  void reset(timetable const& tt);

  std::vector<bmc_raptor_bag<arrival_departure_label>> best_bag_;
  matrix<bmc_raptor_bag<arrival_departure_label>> round_bags_;
  std::vector<bool> station_mark_;
  std::vector<bool> prev_station_mark_;
  std::vector<bool> route_mark_;
  std::vector<bool> is_destination_;
  std::vector<start> starts_;
  std::vector<std::set<location_idx_t>> destinations_;
  std::vector<pareto_set<journey>> results_;
  interval<unixtime_t> search_interval_;
};

}  // namespace nigiri::routing
