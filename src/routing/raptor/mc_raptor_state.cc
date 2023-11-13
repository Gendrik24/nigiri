#include "nigiri/routing/raptor/mc_raptor_state.h"

#include "fmt/core.h"

#include "utl/helpers/algorithm.h"

#include "nigiri/routing/limits.h"
#include "nigiri/timetable.h"

namespace nigiri::routing {

void mc_raptor_state::resize(unsigned const n_locations,
                             unsigned const n_routes) {
  station_mark_.resize(n_locations);
  prev_station_mark_.resize(n_locations);
  route_mark_.resize(n_routes);
  best_.resize(n_locations);
  round_bags_.resize(kMaxTransfers + 1U, n_locations);
  results_.resize(n_locations);
}
}  // namespace nigiri::routing
