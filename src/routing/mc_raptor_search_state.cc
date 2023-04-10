#include "nigiri/routing/mc_raptor_search_state.h"

#include "nigiri/routing/limits.h"
#include "nigiri/timetable.h"
#include "nigiri/routing/bmc_raptor_bag.h"
#include "nigiri/routing/mc_raptor_label.h"

namespace nigiri::routing {

void mc_raptor_search_state::reset(const timetable& tt) {
  is_destination_.resize(tt.n_locations());
  std::fill(begin(is_destination_), end(is_destination_), false);

  station_mark_.resize(tt.n_locations());
  std::fill(begin(station_mark_), end(station_mark_), false);

  prev_station_mark_.resize(tt.n_locations());
  std::fill(begin(prev_station_mark_), end(prev_station_mark_), false);

  route_mark_.resize(tt.n_routes());
  std::fill(begin(route_mark_), end(route_mark_), false);

  best_bags_.resize(tt.n_locations());
  std::fill(begin(best_bags_), end(best_bags_), mc_raptor_bag<mc_raptor_label>());

  round_bags_.resize(kMaxTransfers + 1U, tt.n_locations());
  round_bags_.reset(mc_raptor_bag<mc_raptor_label>());

  travel_time_lower_bound_.resize(tt.n_locations());
  std::fill(begin(travel_time_lower_bound_), end(travel_time_lower_bound_),
            duration_t{0});

  starts_.clear();
  destinations_.clear();
  results_.clear();
}

}  // namespace nigiri::routing
