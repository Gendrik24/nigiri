#include "nigiri/reach.h"

#include <filesystem>
#include <algorithm>

#include "utl/verify.h"

#include "nigiri/logging.h"
#include "nigiri/routing/raptor/raptor_search.h"
#include "nigiri/timetable.h"
#include "nigiri/common/parse_time.h"
#include "nigiri/reach_store.h"
#include "nigiri/logging.h"

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/pthread/mutex.hpp>

namespace nigiri {

bool uses_transport(const routing::journey::leg& l) {
  return holds_alternative<routing::journey::run_enter_exit>(l.uses_);
}

void update_location_reach(reach_store& rs,
                           location_idx_t loc,
                           reach_t const& reach) {
  reach_t& r = rs.location_reach_[to_idx(loc)];
  r.transport_reach_ = std::max(r.transport_reach_, reach.transport_reach_);
  r.travel_time_reach_ = std::max(r.travel_time_reach_, reach.travel_time_reach_);
}

void update_route_reach(reach_store& rs,
                        timetable const& tt,
                        stop_idx_t s,
                        transport_idx_t t,
                        reach_t const& reach) {
  const route_idx_t route_idx = tt.transport_route_[t];
  const auto range = rs.route_reach_value_ranges_[route_idx];
  const auto n_trips = tt.route_transport_ranges_[route_idx].size();
  const auto reach_idx = range.from_ + s * to_idx(n_trips + 2);
  reach_t& max_rch = rs.reach_values_[reach_idx];
  max_rch.transport_reach_ = std::max(max_rch.transport_reach_, reach.transport_reach_);
  max_rch.travel_time_reach_ = std::max(max_rch.travel_time_reach_, reach.travel_time_reach_);
}

void update_route_reach_out(reach_store& rs,
                            timetable const& tt,
                            stop_idx_t s,
                            transport_idx_t t,
                            reach_t const& reach) {
  const route_idx_t route_idx = tt.transport_route_[t];
  const auto range = rs.route_reach_value_ranges_[route_idx];
  const auto n_trips = tt.route_transport_ranges_[route_idx].size();
  const auto reach_idx = range.from_ + s * to_idx(n_trips + 2) + 1;
  reach_t& max_rch = rs.reach_values_[reach_idx];
  max_rch.transport_reach_ = std::max(max_rch.transport_reach_, reach.transport_reach_);
  max_rch.travel_time_reach_ = std::max(max_rch.travel_time_reach_, reach.travel_time_reach_);
}

void update_transport_reach(reach_store& rs,
                            timetable const& tt,
                            stop_idx_t s,
                            transport_idx_t t,
                            reach_t const& reach) {
  const auto r = tt.transport_route_[t];
  const auto range = rs.route_reach_value_ranges_[r];
  const auto trip_range = tt.route_transport_ranges_[r];
  const auto r_idx = range.from_ + s * to_idx(trip_range.size() + 2) + to_idx(t - trip_range.from_ + 2);
  reach_t& rch = rs.reach_values_[r_idx];
  rch.transport_reach_ = std::max(rch.transport_reach_, reach.transport_reach_);
  rch.travel_time_reach_ = std::max(rch.travel_time_reach_, reach.travel_time_reach_);
}

void compute_reach(reach_store& rs,
                   timetable const& tt,
                   const nigiri::routing::journey& j) {
  const uint8_t n_transports_total = j.transfers_ + 1U;
  uint8_t n_transports_seen{0U};
  for (auto const& leg : j.legs_) {

    uint16_t travel_time_reach = std::min(
        (leg.dep_time_- j.start_time_).count(),
        (j.dest_time_ - leg.dep_time_).count());

    uint8_t transport_reach = std::min(
        n_transports_seen,
        static_cast<uint8_t>(n_transports_total - n_transports_seen));

    if (uses_transport(leg)) {
      const auto& run = std::get<nigiri::routing::journey::run_enter_exit>(leg.uses_);
      const auto t_idx = run.r_.t_.t_idx_;
      update_route_reach_out(rs,
                             tt,
                             run.stop_range_.from_,
                             t_idx,
                             {transport_reach, travel_time_reach});
    }

    travel_time_reach = std::min(
        (leg.arr_time_- j.start_time_).count(),
        (j.dest_time_ - leg.arr_time_).count());


    if (uses_transport(leg)) {
      const auto& run = std::get<nigiri::routing::journey::run_enter_exit>(leg.uses_);
      const auto t_idx = run.r_.t_.t_idx_;

      n_transports_seen++;
      transport_reach = std::min(
          n_transports_seen,
          static_cast<uint8_t>(n_transports_total - n_transports_seen));

      update_transport_reach(rs,
                             tt,
                             run.stop_range_.to_ - 1U,
                             t_idx,
                             {transport_reach, travel_time_reach});

      update_route_reach(rs,
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

void compute_reach(reach_store& rs,
                   timetable const& tt,
                   interval<unixtime_t> const& time_range,
                   unsigned int num_threads) {
  boost::mutex mtx_;
  std::size_t n_locations_finished = 9U;

  auto const reach_from_loc = [&] (std::size_t loc_start) -> void {
    const auto& results = nigiri::routing::mc_raptor_search(tt,
                                                            location{tt, location_idx_t{loc_start}}.id_,
                                                            time_range);

    mtx_.lock();
    for (std::size_t loc_tgt = 0U; loc_tgt < tt.n_locations(); ++loc_tgt) {
      compute_reach(rs, tt,  results[loc_tgt]);
    }
    n_locations_finished++;
    nigiri::log(nigiri::log_lvl::info,
                "reach_store",
                "Progress {} %",
                fmt::format("{:.2f}", (double) n_locations_finished / (double) tt.n_locations() * 100));
    mtx_.unlock();

  };

  boost::asio::thread_pool pool(num_threads);
  for (std::size_t loc_start = 9U; loc_start < tt.n_locations(); ++loc_start) {
    boost::asio::post(pool, [&,loc_start] {reach_from_loc(loc_start);});
  }
  pool.join();
  }

void init_reach_store(reach_store& rs, timetable const& tt) {

  rs.location_reach_.resize(tt.n_locations(), {0U,0U});

  rs.route_reach_value_ranges_.resize(0U);
  rs.reach_values_.resize(0U);

  for (auto r = route_idx_t{0}; r < tt.n_routes(); r++) {
    const auto n_stops = tt.route_location_seq_[r].size();
    const auto n_trips = tt.route_transport_ranges_[r].size();
    std::vector<reach_t> f;
    f.resize(to_idx(n_trips + 2) * n_stops);
    std::fill(f.begin(), f.end(), reach_t{0U, 0U});
    rs.route_reach_value_ranges_.emplace_back(rs.reach_values_.size(), rs.reach_values_.size() + f.size());
    rs.reach_values_.insert(rs.reach_values_.end(), f.begin(), f.end());
  }
}

void add_reach_store_for(interval<nigiri::unixtime_t> const& time_range,
                         timetable& tt,
                         unsigned int num_threads) {
  reach_store rs;
  rs.valid_range_ = time_range;
  init_reach_store(rs, tt);

  compute_reach(rs, tt, time_range, num_threads);

  tt.reach_stores_.push_back(rs);
}

void extend_reach_store_by(reach_store_idx_t rs_idx,
                           duration_t const duration,
                           timetable& tt,
                           unsigned int num_threads) {
  if (rs_idx == reach_store_idx_t::invalid()) {
    return;
  }

  reach_store& rs = tt.reach_stores_[rs_idx];
  if (duration < duration_t::zero()) {
    compute_reach(rs, tt,
                  {rs.valid_range_.from_ + duration, rs.valid_range_.from_}, num_threads);
    rs.valid_range_ = {rs.valid_range_.from_ + duration, rs.valid_range_.to_};
  } else {
    compute_reach(rs, tt,
                  {rs.valid_range_.to_, rs.valid_range_.to_ + duration}, num_threads);
    rs.valid_range_ = {rs.valid_range_.from_, rs.valid_range_.to_ + duration};
  }
}

}