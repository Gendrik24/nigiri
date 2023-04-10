#pragma once

#include <vector>

#include "nigiri/routing/mc_raptor_search_state.h"
#include "nigiri/routing/query.h"
#include "nigiri/routing/routing_time.h"
#include "nigiri/routing/raptor.h"
#include "nigiri/routing/mc_raptor_label.h"
#include "nigiri/routing/mc_raptor_route_label.h"
#include "nigiri/routing/transport_departure_label.h"
#include "nigiri/routing/mc_raptor_bag.h"
#include "nigiri/types.h"

#define MC_RAPTOR_GLOBAL_PRUNING
#define MC_RAPTOR_LOCAL_PRUNING

#ifdef MC_RAPTOR_GLOBAL_PRUNING
#define MC_RAPTOR_LOWER_BOUNDS
#endif

namespace nigiri {
struct timetable;
}

namespace nigiri::routing {

using dep_arr_t = std::pair<routing_time, routing_time>;
using raptor_bag = mc_raptor_bag<mc_raptor_label>;
using raptor_route_bag = mc_raptor_bag<mc_raptor_route_label>;

struct mc_raptor_search_state;

struct mc_raptor_stats {
  std::uint64_t n_routing_time_{0ULL};
  std::uint64_t n_footpaths_visited_{0ULL};
  std::uint64_t n_routes_visited_{0ULL};
  std::uint64_t n_earliest_trip_calls_{0ULL};
  std::uint64_t n_earliest_arrival_updated_by_route_{0ULL};
  std::uint64_t n_earliest_arrival_updated_by_footpath_{0ULL};
  std::uint64_t n_reconstruction_time{0ULL};
  std::uint64_t fp_update_prevented_by_lower_bound_{0ULL};
  std::uint64_t route_update_prevented_by_lower_bound_{0ULL};
  std::uint64_t lb_time_{0ULL};
};

struct mc_raptor {
  mc_raptor(timetable const& tt,
           mc_raptor_search_state& state,
           query q);

  bool is_better(auto a, auto b);
  bool is_better_or_eq(auto a, auto b);
  auto get_best(auto a, auto b);

  void route();
  void rounds();
  bool update_route(unsigned const k, route_idx_t route);
  transport get_earliest_transport(const mc_raptor_label& current,
                                   route_idx_t const r,
                                   unsigned const stop_idx,
                                   location_idx_t const l_idx);
  void update_footpaths(unsigned const k);
  void reconstruct();

  day_idx_t start_day_offset() const;
  day_idx_t number_of_days_in_search_interval() const;
  unsigned end_k() const;

  void update_destination_bag(unsigned long k);
  timetable const& tt_;
  std::uint16_t n_tt_days_;
  query q_;
  raptor_bag best_destination_bag;
  interval<unixtime_t> search_interval_;
  mc_raptor_search_state& state_;
  mc_raptor_stats stats_;
  unsigned int n_days_to_iterate_;
};

}