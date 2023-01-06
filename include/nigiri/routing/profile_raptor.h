#pragma once

#include <vector>

#include "nigiri/routing/query.h"
#include "nigiri/routing/routing_time.h"
#include "nigiri/routing/raptor.h"
#include "nigiri/routing/raptor_label.h"
#include "nigiri/types.h"

namespace nigiri {
struct timetable;
}

namespace nigiri::routing {

using dep_arr_t = std::pair<routing_time, routing_time>;

struct profile_search_state;

struct profile_raptor {
  profile_raptor(timetable const& tt, profile_search_state& state, query q);
  void init_starts();
  void route();
  void rounds();
  bool is_dominated_by_best_bags(const raptor_label& l);

  bool update_route(unsigned const k, route_idx_t route);
  void update_footpaths(unsigned const k);

  void get_earliest_sufficient_transports(const raptor_label label,
                                          route_idx_t const r,
                                          unsigned const stop_idx,
                                          std::vector<transport>& transports);

  void force_print_state(char const* comment = "");
  void print_state(char const* comment = "");

  day_idx_t start_day_offset() const;
  day_idx_t number_of_days_in_search_interval() const;
  unsigned end_k() const;

  void reconstruct();
  void reconstruct_for_destination(std::size_t dest_idx,
                                   location_idx_t dest,
                                   std::vector<routing_time> const& best_times,
                                   cista::raw::matrix<routing_time> const& round_times,
                                   const unixtime_t start_at_start,
                                   std::vector<pareto_set<journey>>& results);

  timetable const& tt_;
  std::uint16_t n_tt_days_;
  query q_;
  interval<unixtime_t> search_interval_;
  profile_search_state& state_;
  stats stats_;
};

}