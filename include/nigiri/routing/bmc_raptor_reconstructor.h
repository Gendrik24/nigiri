#pragma once

#include <vector>
#include <set>
#include <optional>

#include "nigiri/types.h"
#include "nigiri/routing/routing_time.h"
#include "nigiri/types.h"
#include "nigiri/routing/journey.h"

namespace nigiri {
struct timetable;
}

namespace nigiri::routing {

struct query;
struct bmc_raptor_search_state;
struct journey;

using uncompressed_round_times_t = std::vector<dep_arr_t>;
using uncompressed_round_time_iterator = std::vector<dep_arr_t>::iterator;

struct bmc_raptor_reconstructor {

  struct departure_arrival_comparator {
    bool operator() (const dep_arr_t& el1, const dep_arr_t& el2) const {
      if (el1.first > el2.first) {
        return true;
      } else if (el1.first < el2.first) {
        return false;
      }

      if (el1.second < el2.second) {
        return true;
      }

      return false;
    };
  };

  struct departure_comparator {
    bool operator() (const routing_time& rt1, const routing_time& rt2) const {
      if (rt1 > rt2) {
        return true;
      }

      return false;
    };
  };

  bmc_raptor_reconstructor() = delete;

  bmc_raptor_reconstructor(timetable const&,
                           query const&,
                           bmc_raptor_search_state&,
                           interval<unixtime_t> search_interval);

  void reconstruct();

private:
  routing_time get_routing_time(const unsigned long k,
                                const location_idx_t loc,
                                const routing_time departure);
  void uncompress_round_bags();
  void get_departure_events();
  void reconstruct_for_destination(std::size_t dest_idx,
                                   location_idx_t dest,
                                   const routing_time departure_time);

  void reconstruct_journey(journey& j,
                           const routing_time departure_time);

  std::optional<journey::leg> find_start_footpath(journey const& j,
                                                  const routing_time departure);

  timetable const& tt_;
  query const& q_;
  bmc_raptor_search_state& state_;
  unsigned int n_rounds_;
  const interval<unixtime_t> search_interval_;
  matrix<uncompressed_round_times_t> round_times_;
  matrix<uncompressed_round_time_iterator> round_time_iters;
  std::set<routing_time, departure_comparator> departure_events_;
};

}  // namespace nigiri::routing
