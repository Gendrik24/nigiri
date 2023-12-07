#include "nigiri/loader/build_conflict_graph.h"

#include <limits>
#include <vector>

#include "nigiri/timetable.h"
#include "nigiri/types.h"

namespace nigiri::loader {

void build_conflict_graph(timetable& tt) {
  const auto kGroupInvalid = std::numeric_limits<std::size_t>::max();

  vector_map<route_idx_t, std::size_t> route_groups(
    tt.n_routes(),
    kGroupInvalid);

  std::vector<std::size_t> num_of_routes_in_group;

  for (auto r_id = 0U; r_id < tt.n_routes(); ++r_id) {
    const auto& stop_sequence = tt.route_location_seq_[route_idx_t{r_id}];
    std::vector<bool> groups_conflicting(num_of_routes_in_group.size(), false);

    for (auto i = 0U; i < stop_sequence.size(); ++i) {
      auto const stop_idx = static_cast<stop_idx_t>(i);
      auto const stp = stop{stop_sequence[stop_idx]};

      const auto& conflicting_routes_at_stop =
        tt.location_routes_[stp.location_idx()];
      for (const auto& conflict_route : conflicting_routes_at_stop) {
        if (conflict_route == route_idx_t{r_id}) {
          continue;
        }

        if (route_groups[conflict_route] != kGroupInvalid) {
          groups_conflicting[route_groups[conflict_route]] = true;
        }
      }
    }

    std::size_t group = 0U;
    for (; group < groups_conflicting.size(); ++group) {
      if (! groups_conflicting[group]) {
        route_groups[route_idx_t{r_id}] = group;
        num_of_routes_in_group[group]++;
        break;
      }
    }

    if (group >= groups_conflicting.size()) {
      route_groups[route_idx_t{r_id}] = groups_conflicting.size();
      num_of_routes_in_group.emplace_back(1U);
    } 
  }

  tt.conflict_groups_last_index_.resize(
    num_of_routes_in_group.size(),
    std::numeric_limits<std::size_t>::max());

  if (num_of_routes_in_group.size() > 0) {
    tt.conflict_groups_last_index_[0U] = num_of_routes_in_group[0U];
  }
  for (std::size_t i = 1; i < num_of_routes_in_group.size(); ++i) {
    tt.conflict_groups_last_index_[i] = 
      num_of_routes_in_group[i] + tt.conflict_groups_last_index_[i-1];
  }
  
  std::vector<std::size_t> last_idx_used(
    tt.conflict_groups_last_index_.begin(), 
    tt.conflict_groups_last_index_.end());
  tt.routes_conflict_ordered_.resize(tt.n_routes(), route_idx_t::invalid());
  for (auto r_id = 0U; r_id < tt.n_routes(); ++r_id) {
    const auto group_idx = route_groups[route_idx_t{r_id}];
    const auto next_idx = --last_idx_used[group_idx];
    tt.routes_conflict_ordered_[next_idx] = route_idx_t{r_id};
  }
}

} // namespace nigiri::loader
