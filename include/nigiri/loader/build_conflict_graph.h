#pragma once

#include <cinttypes>
#include <limits>

#include "nigiri/timetable.h"
#include "fmt/core.h"

namespace nigiri::loader {

void build_conflict_graph(timetable& tt) {

  vector_map<route_idx_t, std::size_t> route_grouping(tt.n_routes(), std::numeric_limits<std::size_t>::max());
  vector<std::size_t> num_of_routes_in_group;

  for (auto r_id = 0U; r_id != tt.n_routes(); ++r_id) {
    auto const stop_sequence = tt.route_location_seq_[route_idx_t{r_id}];
    vector<bool> conflicting_groups(num_of_routes_in_group.size(), false);

    for (auto i = 0U; i != stop_sequence.size(); ++i) {
      auto const stop_idx = static_cast<unsigned>(i);
      auto const stop = timetable::stop{stop_sequence[stop_idx]};

      const auto& conflicting_routes_at_stop =
          tt.location_routes_.at(stop.location_idx());
      for (auto const& conflicting_r : conflicting_routes_at_stop) {
        if (conflicting_r == route_idx_t{r_id}) {
          continue;
        }

        if (route_grouping[conflicting_r] != std::numeric_limits<std::size_t>::max()) {
          conflicting_groups[route_grouping[conflicting_r]] = true;
        }
      }
    }

    std::size_t i;
    for (i = 0U; i < conflicting_groups.size(); ++i) {
      if (! conflicting_groups[i]) {
        route_grouping[route_idx_t{r_id}] = i;
        num_of_routes_in_group[i]++;
        break;
      }
    }
    if (i >= conflicting_groups.size()) {
      route_grouping[route_idx_t{r_id}] = conflicting_groups.size();
      num_of_routes_in_group.push_back(1U);
    }
  }

  tt.conflict_group_upper_bound.resize(num_of_routes_in_group.size(), std::numeric_limits<std::size_t>::max());
  for (std::size_t i = 0U; i < num_of_routes_in_group.size(); ++i) {
    tt.conflict_group_upper_bound[i] = i == 0U ?
      num_of_routes_in_group[i] : num_of_routes_in_group[i] + tt.conflict_group_upper_bound[i-1];
  }

  vector<std::size_t> last_idx_used(tt.conflict_group_upper_bound);
  tt.ordered_conflict_routes_.resize(tt.n_routes(), route_idx_t::invalid());
  for (auto r_id = 0U; r_id != tt.n_routes(); ++r_id) {
    const auto group_idx = route_grouping[route_idx_t{r_id}];
    const auto next_idx = --last_idx_used[group_idx];
    tt.ordered_conflict_routes_[next_idx] = route_idx_t{r_id};
  }
}
}