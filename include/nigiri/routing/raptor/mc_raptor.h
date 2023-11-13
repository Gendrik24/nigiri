#pragma once

#include <cstdint>
#include "nigiri/routing/pareto_set.h"
#include "nigiri/timetable.h"
#include "nigiri/routing/raptor/mc_raptor_state.h"
#include "nigiri/routing/query.h"

namespace nigiri::routing {

using dep_arr_t = std::pair<routing_time, routing_time>;

struct mc_raptor_stats {
    std::uint64_t n_routing_time_{0ULL};
    std::uint64_t n_footpaths_visited_{0ULL};
    std::uint64_t n_routes_visited_{0ULL};
    std::uint64_t n_earliest_trip_calls_{0ULL};
    std::uint64_t n_earliest_arrival_updated_by_route_{0ULL};
    std::uint64_t n_earliest_arrival_updated_by_footpath_{0ULL};
    std::uint64_t fp_update_prevented_by_lower_bound_{0ULL};
    std::uint64_t route_update_prevented_by_lower_bound_{0ULL};
};

struct mc_raptor {

    mc_raptor(timetable const& tt,
              mc_raptor_state& state,
              interval<unixtime_t> const search_interval,
              location_match_mode const start_match_mode,
              std::vector<offset> const start);

    void route();
    mc_raptor_stats const& get_stats() const;

private:

  bool is_better(auto a, auto b);
  bool is_better_or_eq(auto a, auto b);
  auto get_best(auto a, auto b);
  void rounds();
  bool update_route(unsigned const k, route_idx_t route);
  transport get_earliest_transport(const mc_raptor_label& current,
                                   route_idx_t const r,
                                   stop_idx_t stop_idx);
  void update_footpaths(unsigned const k);

  day_idx_t start_day_offset() const;
  day_idx_t number_of_days_in_search_interval() const;
  unsigned end_k() const;

  timetable const& tt_;
  std::uint16_t n_tt_days_;
  mc_raptor_state& state_;
  interval<unixtime_t> const search_interval_;
  mc_raptor_stats stats_;
  std::uint32_t n_locations_, n_routes_;
  std::vector<offset> const start_;
  location_match_mode start_match_mode_;
  unsigned int n_days_to_iterate_;
};
}