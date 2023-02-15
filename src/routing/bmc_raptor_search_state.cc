#include "nigiri/routing/bmc_raptor_search_state.h"

#include "nigiri/routing/limits.h"
#include "nigiri/timetable.h"
#include "nigiri/routing/bmc_raptor_bag.h"
#include "nigiri/routing/arrival_departure_label.h"

namespace nigiri::routing {

void bmc_raptor_search_state::reset(const timetable& tt) {
  is_destination_.resize(tt.n_locations());
  std::fill(begin(is_destination_), end(is_destination_), false);

  station_mark_.resize(tt.n_locations());
  std::fill(begin(station_mark_), end(station_mark_), false);

  prev_station_mark_.resize(tt.n_locations());
  std::fill(begin(prev_station_mark_), end(prev_station_mark_), false);

  route_mark_.resize(tt.n_routes());
  std::fill(begin(route_mark_), end(route_mark_), false);

  best_bags_.resize(tt.n_locations());
  std::fill(begin(best_bags_), end(best_bags_), bmc_raptor_bag<arrival_departure_label>());

  round_bags_.resize(kMaxTransfers + 1U, tt.n_locations());
  round_bags_.reset(bmc_raptor_bag<arrival_departure_label>());

  destinations_.clear();
  results_.clear();
}

}  // namespace nigiri::routing
