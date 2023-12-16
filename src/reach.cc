#include "nigiri/reach.h"

#include <filesystem>
#include <algorithm>

#include "utl/progress_tracker.h"
#include "utl/verify.h"

#include "nigiri/logging.h"
#include "nigiri/routing/raptor/raptor_search.h"
#include "nigiri/timetable.h"
#include "nigiri/common/parse_time.h"
#include "nigiri/reach_store.h"
#include "nigiri/logging.h"

#include <omp.h>

using namespace date;
using namespace nigiri;
using namespace nigiri::routing;

namespace nigiri {

reach_t get_location_reach(reach_store const& rs, location_idx_t loc) {
  return rs.location_reach_[to_idx(loc)];
}

reach_t get_route_reach(reach_store const& rs,
                        stop_idx_t s,
                        route_idx_t r) {
  return rs.route_location_reach_[r][s];
}

void update_location_reach(reach_store& rs,
                           location_idx_t loc,
                           reach_t const& reach) {
  reach_t& r = rs.location_reach_[to_idx(loc)];
  r.transport_reach_ = std::max(r.transport_reach_, reach.transport_reach_);
  r.travel_time_reach_ = std::max(r.travel_time_reach_, reach.travel_time_reach_);
}


void update_route_reach(reach_store& rs,
                        stop_idx_t s,
                        route_idx_t r,
                        reach_t const& reach) {
  reach_t& rch = rs.route_location_reach_[r][s];
  rch.transport_reach_ = std::max(rch.transport_reach_, reach.transport_reach_);
  rch.travel_time_reach_ = std::max(rch.travel_time_reach_, reach.travel_time_reach_);
}

void update_transport_reach(reach_store& rs,
                            timetable const& tt,
                            stop_idx_t s,
                            transport_idx_t t,
                            reach_t const& reach) {
  const auto r = tt.transport_route_[t];
  const auto& range = rs.route_reach_value_ranges_[r];
  const auto& trip_range = tt.route_transport_ranges_[r];
  const auto r_idx = range.from_ + s * to_idx(trip_range.size()) + to_idx(t - tt.route_transport_ranges_[r].from_);
  reach_t& rch = rs.reach_values_[r_idx];
  rch.transport_reach_ = std::max(rch.transport_reach_, reach.transport_reach_);
  rch.travel_time_reach_ = std::max(rch.travel_time_reach_, reach.travel_time_reach_);
}

void compute_reach(reach_store& rs,
                   timetable const& tt,
                   const nigiri::routing::journey& j) {
  using namespace std;

  const uint8_t n_transports_total = j.transfers_ + 1U;
  uint8_t n_transports_seen{0U};
  for (auto const& leg : j.legs_) {


    uint16_t travel_time_reach = std::min(
        (leg.arr_time_- j.start_time_).count(),
        (j.dest_time_ - leg.arr_time_).count());

    uint8_t transport_reach = std::min(
        n_transports_seen,
        static_cast<uint8_t>(n_transports_total - n_transports_seen));


    if (holds_alternative<nigiri::routing::journey::run_enter_exit>(leg.uses_)) {
      const auto& run = std::get<nigiri::routing::journey::run_enter_exit>(leg.uses_);
      const auto t_idx = run.r_.t_.t_idx_;

      n_transports_seen++;
      transport_reach = std::min(
          n_transports_seen,
          static_cast<uint8_t>(n_transports_total - n_transports_seen));


      update_route_reach(rs,
                         run.stop_range_.to_ - 1U,
                         tt.transport_route_[t_idx],
                         {transport_reach, travel_time_reach});

      update_transport_reach(rs,
                             tt,
                             run.stop_range_.to_ - 1U,
                             t_idx,
                             {transport_reach, travel_time_reach});
    }

    update_location_reach(rs,
                          leg.to_,
                          {transport_reach, travel_time_reach});
  }
}

void compute_reach(reach_store& rs,
                   timetable const& tt,
                   const pareto_set<nigiri::routing::journey>& journeys) {
  for (const auto& j : journeys) {
    compute_reach(rs, tt, j);
  }
}

void init_reach_store(reach_store& rs, timetable const& tt) {

  rs.location_reach_.resize(tt.n_locations(), {0U,0U});

  rs.route_location_reach_.resize(0U);

  rs.route_reach_value_ranges_.resize(0U);
  rs.reach_values_.resize(0U);

  for (auto r = route_idx_t{0}; r < tt.n_routes(); r++) {
    const auto n_stops = tt.route_location_seq_[r].size();
    std::size_t l = n_stops;
    std::vector<reach_t> v;
    v.resize(l);
    std::fill(v.begin(), v.end(), reach_t{0U, 0U});
    rs.route_location_reach_.emplace_back(v);

    const auto n_trips = tt.route_transport_ranges_[r].size();
    std::vector<reach_t> f;
    f.resize(to_idx(n_trips) * n_stops);
    std::fill(f.begin(), f.end(), reach_t{0U, 0U});
    rs.route_reach_value_ranges_.emplace_back(rs.reach_values_.size(), rs.reach_values_.size() + f.size());
    rs.reach_values_.insert(rs.reach_values_.end(), f.begin(), f.end());
  }
}

void add_reach_store_for(interval<nigiri::unixtime_t> const& time_range, timetable& tt) {
  reach_store rs;
  rs.valid_range_ = time_range;
  init_reach_store(rs, tt);

#if defined(_OPENMP)
  std::size_t loc_start;
  std::size_t n_locations_finished = 9U;

#pragma omp parallel for default(none) private(loc_start) shared(rs, n_locations_finished, tt, time_range)
  for (loc_start = 9U; loc_start < tt.n_locations(); ++loc_start) {

    const auto& results = nigiri::routing::mc_raptor_search(tt,
                                                            location{tt, location_idx_t{loc_start}}.id_,
                                                            rs.valid_range_);
#pragma omp critical
    {
      for (std::size_t loc_tgt = 0U; loc_tgt < tt.n_locations(); ++loc_tgt) {
        compute_reach(rs, tt,  results[loc_tgt]);
      }
      n_locations_finished++;
      nigiri::log(nigiri::log_lvl::info,
                  "reach_store",
                  "Progress {} %",
                  fmt::format("{:.2f}", (double) n_locations_finished / (double) tt.n_locations() * 100));
    }
  }

#else
  for (std::size_t loc_start = 9U; loc_start < n_locations(); ++loc_start) {
    nigiri::log(nigiri::log_lvl::info,
                "reach_store",
                "Starting to calculate reach values for journeys starting from location {}/{}",
                loc_start, n_locations());
    const auto& results = nigiri::routing::mc_raptor_search(tt,
                                                            location{tt, location_idx_t{loc_start}}.id_,
                                                            rs.valid_range_);


    for (std::size_t loc_tgt = 0U; loc_tgt < tt.n_locations(); ++loc_tgt) {
      compute_reach(rs, tt, results[loc_tgt]);
    }
  }
#endif

  tt.reach_stores_.push_back(rs);
}

}