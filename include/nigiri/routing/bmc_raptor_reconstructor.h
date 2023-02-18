#pragma once

#include <vector>
#include <set>
#include <optional>

#include "nigiri/types.h"
#include "nigiri/routing/routing_time.h"
#include "nigiri/routing/journey.h"
#include "nigiri/routing/arrival_departure_label.h"

namespace nigiri {
struct timetable;
}

namespace nigiri::routing {

struct query;
struct bmc_raptor_search_state;
struct journey;

using uncompressed_round_times_t = std::vector<arrival_departure_label>;
using uncompressed_round_time_iterator = std::vector<arrival_departure_label>::iterator;

struct bmc_raptor_reconstructor {

  struct departure_arrival_comparator {
    bool operator() (const arrival_departure_label& el1, const arrival_departure_label& el2) const {
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
    bool operator() (const long_minutes_after_midnight_t & rt1, const long_minutes_after_midnight_t & rt2) const {
      if (rt1 > rt2) {
        return true;
      }

      return false;
    }
  };

  bmc_raptor_reconstructor() = delete;

  bmc_raptor_reconstructor(timetable const&,
                           query const&,
                           bmc_raptor_search_state&,
                           interval<unixtime_t> search_interval);

  void reconstruct();

private:
  std::pair<routing_time, std::optional<reconstruction_leg>> get_routing_information(const unsigned long k,
                                                                                     const location_idx_t loc,
                                                                                     long_minutes_after_midnight_t departure);

  std::pair<journey::leg, journey::leg> get_legs(unsigned const k,
                                                 location_idx_t const l,
                                                 long_minutes_after_midnight_t departure,
                                                 journey& j);

  void uncompress_round_bags();
  void get_departure_events();
  void reconstruct_for_destination(std::size_t dest_idx,
                                   location_idx_t dest,
                                   const long_minutes_after_midnight_t departure_time);

  void reconstruct_journey(journey& j,
                           const long_minutes_after_midnight_t departure_time);

  std::optional<journey::leg> find_start_footpath(journey const& j,
                                                  const long_minutes_after_midnight_t departure);

  timetable const& tt_;
  query const& q_;
  bmc_raptor_search_state& state_;
  unsigned int n_rounds_;
  const interval<unixtime_t> search_interval_;
  matrix<uncompressed_round_times_t> round_times_;
  matrix<uncompressed_round_time_iterator> round_time_iters;
  std::set<long_minutes_after_midnight_t , departure_comparator> departure_events_;
};

}  // namespace nigiri::routing
