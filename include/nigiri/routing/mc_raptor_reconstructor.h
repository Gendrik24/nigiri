#pragma once

#include <vector>
#include <set>
#include <optional>
#include <variant>

#include "nigiri/types.h"
#include "nigiri/routing/routing_time.h"
#include "nigiri/types.h"
#include "nigiri/routing/journey.h"
#include "nigiri/routing/bmc_raptor_search_state.h"
#include "nigiri/routing/mc_raptor_search_state.h"
#include "nigiri/routing/mc_raptor_label.h"

namespace nigiri {
struct timetable;
}

namespace nigiri::routing {

struct query;
struct bmc_raptor_search_state;
struct journey;

using uncompressed_round_times_t = std::vector<mc_raptor_label>;
using uncompressed_round_time_iterator = std::vector<mc_raptor_label>::iterator;

struct mc_raptor_reconstructor {

  struct departure_arrival_comparator {
    bool operator() (const mc_raptor_label& el1, const mc_raptor_label& el2) const {
      if (el1.departure_ > el2.departure_) {
        return true;
      } else if (el1.departure_ < el2.departure_) {
        return false;
      }

      if (el1.arrival_ < el2.arrival_) {
        return true;
      }

      return false;
    }
  };

  struct departure_comparator {
    bool operator() (const routing_time& rt1, const routing_time& rt2) const {
      if (rt1 > rt2) {
        return true;
      }

      return false;
    }
  };

  mc_raptor_reconstructor() = delete;

  mc_raptor_reconstructor(timetable const&,
                          query const&,
                          matrix<uncompressed_round_times_t>& round_times,
                          interval<unixtime_t> search_interval,
                          std::vector<std::set<location_idx_t>> const& destinations,
                          std::vector<pareto_set<journey>>& results);

  void reconstruct();

private:
  routing_time get_routing_time(const unsigned long k,
                                const location_idx_t loc,
                                const routing_time departure);
  void init();
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
  unsigned int n_rounds_;
  const interval<unixtime_t> search_interval_;
  matrix<uncompressed_round_times_t>& round_times_;
  matrix<uncompressed_round_time_iterator> round_time_iters;
  std::set<routing_time, departure_comparator> departure_events_;
  std::vector<std::set<location_idx_t>> const& destinations_;
  std::vector<pareto_set<journey>>& results_;
};

}  // namespace nigiri::routing
