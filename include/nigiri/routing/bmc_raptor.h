#pragma once

#include <vector>

#include "nigiri/routing/bmc_raptor_bag.h"
#include "nigiri/routing/bmc_raptor_label.h"
#include "nigiri/routing/bmc_raptor_route_label.h"
#include "nigiri/routing/bmc_raptor_search_state.h"
#include "nigiri/routing/query.h"
#include "nigiri/routing/raptor.h"
#include "nigiri/routing/routing_time.h"
#include "nigiri/types.h"

//#define NIGIRI_OPENMP
#define BMC_RAPTOR_GLOBAL_PRUNING
#define BMC_RAPTOR_LOCAL_PRUNING

#ifdef BMC_RAPTOR_GLOBAL_PRUNING
#define BMC_RAPTOR_LOWER_BOUNDS
#endif

namespace nigiri {
struct timetable;
}

namespace nigiri::routing {

using dep_arr_t = std::pair<routing_time, routing_time>;
using bmc_raptor_bag_t = bmc_raptor_bag<bmc_raptor_label>;
using bmc_raptor_route_bag_t = bmc_raptor_bag<bmc_raptor_route_label>;

struct bmc_raptor_search_state;

struct bmc_raptor_stats {
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
  std::uint64_t max_round_arrival_labels_{0ULL};
};

template <criteria crit>
struct bmc_raptor {
  bmc_raptor(timetable const& tt,
             bmc_raptor_search_state& state,
             query q);
  void route();

  bmc_raptor_stats const& get_stats() const;

private:
  static constexpr auto const kBiCrit = (crit == criteria::biCriteria);
  static constexpr auto const kMultiCrit = (crit == criteria::multiCriteria);

  void init_starts();
  void rounds();
  bool update_route(unsigned const k, route_idx_t route);
  void get_earliest_sufficient_transports(const bmc_raptor_label label,
                                          label_bitfield lbl_tdb,
                                          route_idx_t const r,
                                          unsigned const stop_idx,
                                          bmc_raptor_route_bag_t& bag);
  void update_footpaths(unsigned const k);
  void reconstruct();
  void force_print_state(char const* comment = "");
  void print_state(char const* comment = "");

  day_idx_t start_day_offset() const;
  day_idx_t number_of_days_in_search_interval() const;
  unsigned end_k() const;
  void init_location_with_offset(minutes_after_midnight_t time_to_arrive,
                                 location_idx_t location);
  void update_destination_bag(unsigned long k);
  timetable const& tt_;
  std::uint16_t n_tt_days_;
  query q_;
  bmc_raptor_bag_t best_destination_bag;
  interval<unixtime_t> search_interval_;
  bmc_raptor_search_state& state_;
  bmc_raptor_stats stats_;
  unsigned int n_days_to_iterate_;
};

}