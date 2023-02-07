#pragma once

#include <vector>

#include "nigiri/routing/query.h"
#include "nigiri/routing/routing_time.h"
#include "nigiri/routing/raptor.h"
#include "nigiri/routing/raptor_label.h"
#include "nigiri/routing/raptor_route_label.h"
#include "nigiri/routing/arrival_departure_label.h"
#include "nigiri/routing/transport_departure_label.h"
#include "nigiri/routing/bmc_raptor_bag.h"
#include "nigiri/types.h"

#define PROFILE_RAPTOR_GLOBAL_PRUNING
#define PROFILE_RAPTOR_LOCAL_PRUNING

namespace nigiri {
struct timetable;
}

namespace nigiri::routing {

using dep_arr_t = std::pair<routing_time, routing_time>;
using raptor_bag = bmc_raptor_bag<arrival_departure_label>;
using raptor_route_bag = bmc_raptor_bag<transport_departure_label>;

struct profile_search_state;

struct profile_raptor {
  profile_raptor(timetable const& tt, profile_search_state& state, query q, stats s);
  void init_starts();
  void init_location_with_offset(minutes_after_midnight_t time_to_arrive,
                                 location_idx_t location);
  void route();
  void rounds();
  void update_destination_bag(unsigned long k);

  bool update_route(unsigned const k, route_idx_t route);
  void update_footpaths(unsigned const k);

  void get_earliest_sufficient_transports(const arrival_departure_label label,
                                          label_bitfield lbl_tdb,
                                          route_idx_t const r,
                                          unsigned const stop_idx,
                                          raptor_route_bag& bag);

  void force_print_state(char const* comment = "");
  void print_state(char const* comment = "");

  day_idx_t start_day_offset() const;
  day_idx_t number_of_days_in_search_interval() const;
  unsigned end_k() const;

  void reconstruct();
  void reconstruct_for_destination(std::size_t dest_idx,
                                   location_idx_t dest,
                                   std::vector<routing_time> const& best_times,
                                   matrix<routing_time> const& round_times,
                                   const unixtime_t start_at_start,
                                   std::vector<pareto_set<journey>>& results);

  timetable const& tt_;
  std::uint16_t n_tt_days_;
  query q_;
  raptor_bag best_destination_bag;
  interval<unixtime_t> search_interval_;
  profile_search_state& state_;
  stats stats_;
};

}