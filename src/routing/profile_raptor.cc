#include "nigiri/routing/profile_raptor.h"

#include <string>
#include <algorithm>
#include <set>
#include <chrono>
#include <utility>

#include "nigiri/routing/bmc_raptor_search_state.h"
#include "nigiri/routing/bmc_raptor_reconstructor.h"
#include "nigiri/routing/start_times.h"
#include "nigiri/types.h"
#include "nigiri/location.h"
#include "nigiri/timetable.h"
#include "nigiri/tracer.h"
#include "nigiri/special_stations.h"
#include "nigiri/routing/for_each_meta.h"

#include "utl/overloaded.h"
#include "utl/enumerate.h"
#include "utl/erase_if.h"

#include "fmt/core.h"

#define NIGIRI_PROFILE_RAPTOR_COUNTING
#ifdef NIGIRI_PROFILE_RAPTOR_COUNTING
#define NIGIRI_PROFILE_COUNT(s) ++stats_.s
#define NIGIRI_PROFILE_COUNT_BY(s,a) stats_.s += a
#else
#define NIGIRI_PROFILE_COUNT(s)
#define NIGIRI_PROFILE_COUNT_BY(s,a)
#endif

//#define NIGIRI_OPENMP

namespace nigiri::routing {

profile_raptor::profile_raptor(const timetable& tt,
                               bmc_raptor_search_state& state,
                               query q)
    : tt_{tt},
      n_tt_days_{static_cast<std::uint16_t>(tt_.date_range_.size().count())},
      q_(std::move(q)),
      search_interval_(q_.start_time_.apply(utl::overloaded{
          [](interval<unixtime_t> const& start_interval) { return start_interval; },
          [this](unixtime_t const start_time) {
            return interval<unixtime_t>{start_time, tt_.end()};
          }})),
      state_{state},
      n_days_to_iterate_{std::min(kMaxTravelTime / 1440U + 1,
                                  static_cast<unsigned int>(n_tt_days_ - to_idx(start_day_offset())))} {}

day_idx_t profile_raptor::start_day_offset() const {
  return tt_.day_idx_mam(this->search_interval_.from_).first;
}

day_idx_t profile_raptor::number_of_days_in_search_interval() const {
  return tt_.day_idx_mam(this->search_interval_.to_).first
         - tt_.day_idx_mam(this->search_interval_.from_).first + 1;
}

unsigned profile_raptor::end_k() const {
  return std::min(kMaxTransfers, q_.max_transfers_) + 1U;
}

void profile_raptor::init_location_with_offset(minutes_after_midnight_t time_to_arrive,
                                               location_idx_t location) {
  for (auto const& r : tt_.location_routes_.at(location)) {

    auto const location_seq = tt_.route_location_seq_.at(r);
    for (auto const [i, s] : utl::enumerate(location_seq)) {
      if (timetable::stop{s}.location_idx() != location) {
        continue;
      }

      auto const& transport_range = tt_.route_transport_ranges_[r];
      for (auto transport_idx = transport_range.from_;
           transport_idx != transport_range.to_; ++transport_idx) {

        auto const stop_time =
            tt_.event_mam(transport_idx, i,
                         event_type::kDep);

        auto const stop_time_day_offset = stop_time.count() / 1440;
        auto const stop_time_mam = stop_time % 1440;

        auto start_time = stop_time_mam - time_to_arrive;
        if (start_time < minutes_after_midnight_t::zero()) {
          start_time = (1_days - minutes_after_midnight_t{std::abs(start_time.count()) % 1440});
        }
        const auto at_stop = start_time + time_to_arrive;
        const int trip_tdb_bias_shift = start_day_offset().v_
                                        - stop_time_day_offset
                                        + (at_stop.count() / 1440);

        auto trip_tdb = tt_.bitfields_[tt_.transport_traffic_days_[transport_idx]];
        if (trip_tdb_bias_shift >= 0) {
          trip_tdb >>= static_cast<unsigned long>(trip_tdb_bias_shift);
        } else {
          trip_tdb <<= static_cast<unsigned long>(std::abs(trip_tdb_bias_shift));
        }

        auto mask = std::string(number_of_days_in_search_interval().v_, '1');
        const auto dep_lower_limit = search_interval_.from_.time_since_epoch();
        if (start_time < minutes_after_midnight_t{dep_lower_limit.count() % 1440}) {
          mask.back() = '0';
        }

        const auto dep_upper_limit = search_interval_.to_.time_since_epoch();
        if (start_time >= minutes_after_midnight_t{dep_upper_limit.count() % 1440}) {
          mask.front() = '0';
        }
        const auto traffic_day_bitfield = trip_tdb & bitfield{mask};

        if (traffic_day_bitfield.any() && (at_stop-start_time) <= minutes_after_midnight_t{kMaxTravelTime}) {
          bool merged = state_.round_bags_[0U][to_idx(location)]
                            .merge(arrival_departure_label{
                                at_stop,
                                start_time},
                                label_bitfield{traffic_day_bitfield.to_string()});

          state_.station_mark_[to_idx(location)] = merged || state_.station_mark_[to_idx(location)];
        }
      }
    }
    auto const first_out_of_interval = search_interval_.to_.time_since_epoch() % 1440;

    label_bitfield traffic_day_bitfield{
        std::string(number_of_days_in_search_interval().v_, '0').replace(0U, 1U, "1"),
    };
    bool merged = state_.round_bags_[0U][to_idx(location)]
                      .merge(arrival_departure_label{
                          first_out_of_interval + time_to_arrive,
                          first_out_of_interval},
                          traffic_day_bitfield);

    state_.station_mark_[to_idx(location)] = merged || state_.station_mark_[to_idx(location)];
  }

}

void profile_raptor::update_destination_bag(unsigned long k) {
  for (auto const dest : state_.destinations_.front()) {
    for (const auto& rl : state_.round_bags_[k-1][to_idx(dest)]) {
      best_destination_bag.merge(rl.label_, rl.tdb_);
    }
  }
}

void profile_raptor::init_starts() {
  std::set<location_idx_t> seen;
  for (const auto& o : q_.start_) {
    seen.clear();
    for_each_meta(tt_, q_.start_match_mode_, o.target_, [&](location_idx_t const l) {
      if (!seen.emplace(l).second) {
        return;
      }
      init_location_with_offset(o.duration_,l);

      // It is possible to transfer to another location and start from there
      if (q_.use_start_footpaths_) {

        // First get all outgoing footpaths
        auto const footpaths_out = tt_.locations_.footpaths_out_[l];
        for (const auto& fp : footpaths_out) {
          init_location_with_offset(o.duration_ + fp.duration_,fp.target_);
        }
      }
    });
  }
}


void profile_raptor::route() {
  state_.reset(tt_);
  state_.search_interval_ = search_interval_;

  collect_destinations(tt_, q_.destinations_, q_.dest_match_mode_,
                       state_.destinations_, state_.is_destination_);
  init_starts();
  rounds();
  reconstruct();
  for (auto& r : state_.results_) {
    utl::erase_if(r, [&](journey const& j) {
      return !search_interval_.contains(
          j.start_time_);
    });
  }
}

void profile_raptor::rounds() {
  print_state();
  for (auto k = 1U; k != end_k(); ++k) {
    trace_always("┊ round k={}\n", k);

    #ifdef PROFILE_RAPTOR_GLOBAL_PRUNING
    update_destination_bag(k);
    #endif

    // Round k
    auto any_marked = false;
    for (auto l_idx = location_idx_t{0U};
         l_idx != static_cast<cista::base_t<location_idx_t>>(
                      state_.station_mark_.size()); ++l_idx) {
      
      if (state_.station_mark_[to_idx(l_idx)]) {
        #ifdef PROFILE_RAPTOR_LOCAL_PRUNING
          for (const auto& l : state_.round_bags_[k-1][to_idx(l_idx)]) {
            state_.best_bags_[to_idx(l_idx)].merge(l.label_, l.tdb_);
          }
        #endif
        any_marked = true;
        for (auto const& r : tt_.location_routes_[l_idx]) {
          state_.route_mark_[to_idx(r)] = true;
        }
      }
    }
    
    std::swap(state_.prev_station_mark_, state_.station_mark_);
    std::fill(begin(state_.station_mark_), end(state_.station_mark_), false);
    
    if (!any_marked) {
      trace_always("┊ ╰ no routes marked, exit\n\n");
      return;
    }

    any_marked = false;
    #ifdef NIGIRI_OPENMP
    std::size_t routes_visited = 0;
    #pragma omp parallel default(none) reduction(|| : any_marked) reduction(+ : routes_visited) shared(state_, tt_, k)
    {
      for (std::size_t i = 0U; i < tt_.conflict_group_upper_bound.size(); ++i) {

        #pragma omp for
        for(std::size_t j = (i == 0U) ? 0 : tt_.conflict_group_upper_bound[i-1]; j < tt_.conflict_group_upper_bound[i]; ++j) {
          const auto route_idx = tt_.ordered_conflict_routes_[j];
          if (!state_.route_mark_[route_idx.v_]) {
            continue;
          }
          routes_visited++;
          any_marked |= update_route(k, route_idx);
        }
        #pragma omp barrier
      }
    }
    NIGIRI_PROFILE_COUNT_BY(n_routes_visited_, routes_visited);
    #else
    for (auto r_id = 0U; r_id != tt_.n_routes(); ++r_id) {
      if (!state_.route_mark_[r_id]) {
        continue;
      }
      trace("┊ ├ updating route {}\n", r_id);
      NIGIRI_PROFILE_COUNT(n_routes_visited_);
      any_marked |= update_route(k, route_idx_t{r_id});
    }
    #endif

    std::fill(begin(state_.route_mark_), end(state_.route_mark_), false);
    if (!any_marked) {
      trace_always("┊ ╰ no stations marked, exit\n\n");
      return;
    }
    update_footpaths(k);
    trace_always("┊ ╰ round {} done\n", k);
    print_state();
  }
}

bool profile_raptor::update_route(unsigned const k, route_idx_t route_idx) {

  auto any_marked = false;
  auto const stop_sequence = tt_.route_location_seq_[route_idx];

  raptor_route_bag r_b{};
  for (auto i = 0U; i != stop_sequence.size(); ++i) {
    auto const stop_idx =
        static_cast<unsigned>(i);
    auto const stop = timetable::stop{stop_sequence[stop_idx]};
    auto const l_idx = cista::to_idx(stop.location_idx());

    trace(
        "┊ │  stop_idx={}, location=(name={}, id={}, idx={})\n",
        stop_idx, tt_.locations_.names_[location_idx_t{l_idx}].view(),
        tt_.locations_.ids_[location_idx_t{l_idx}].view(), l_idx);

    trace(
        "┊ │    Update arrival times of labels in route bag:\n");
    auto const transfer_time_offset = tt_.locations_.transfer_time_[location_idx_t{l_idx}];
    auto const is_destination = state_.is_destination_[l_idx];
    for (const auto& active_label : r_b) {
      if (active_label.label_.transport_.t_idx_ == transport_idx_t::invalid()) {
        trace(
            "┊ │    │ current label={} has no active transport -> SKIP!\n",
            active_label.to_string());
        continue;
      }
      const auto new_arr = tt_.event_mam(route_idx,
                                         active_label.label_.transport_.t_idx_,
                                         stop_idx,
                                         event_type::kArr) + minutes_after_midnight_t{active_label.label_.transport_.day_.v_ * 1440};

      trace(
          "┊ │    │ current label={}, active transport={} -> updated arrival time={}, transfer time={}\n",
          active_label.to_string(), active_label.t_, new_arr, new_arr);
      const auto candidate_lbl = arrival_departure_label{
          new_arr + (is_destination ? minutes_after_midnight_t::zero() : transfer_time_offset),
          active_label.label_.departure_};
      auto candidate_tdb = active_label.tdb_;

      if (!stop.out_allowed() || (candidate_lbl.arrival_ - candidate_lbl.departure_).count() > kMaxTravelTime) {
        continue;
      }

      #ifdef PROFILE_RAPTOR_GLOBAL_PRUNING
      candidate_tdb = best_destination_bag.filter_dominated(candidate_lbl, candidate_tdb);
      #endif

      #ifdef PROFILE_RAPTOR_LOCAL_PRUNING
      candidate_tdb = state_.best_bags_[l_idx].filter_dominated(candidate_lbl, candidate_tdb);
      #endif

      if (candidate_tdb.none()) {
        continue;
      }

      if (state_.round_bags_[k][cista::to_idx(l_idx)].merge(candidate_lbl, candidate_tdb)) {
        trace("┊ │    │ label={}, not dominated -> new best bag label!\n", rl.to_string());
        NIGIRI_PROFILE_COUNT(n_earliest_arrival_updated_by_route_);
        state_.station_mark_[l_idx] = true;
        any_marked = true;
      } else {
        trace("┊ │    │ label={}, dominated by best local bag -> SKIP!\n", rl.to_string());
      }
    }

    if (i == stop_sequence.size()-1) {
      return any_marked;
    }

    if (stop.in_allowed() && state_.prev_station_mark_[l_idx]) {
      trace(
          "┊ │    Assign trips to labels from prev round:\n");
      for (const auto& l : state_.round_bags_[k-1][cista::to_idx(l_idx)]) {
        trace("┊ │    ├ search for transports serving label={}\n", l.to_string());
        get_earliest_sufficient_transports(l.label_,l.tdb_, route_idx, stop_idx, r_b);
      }
    }
  }
  return any_marked;
}


void profile_raptor::update_footpaths(unsigned const k) {
  trace("┊ ├ FOOTPATHS\n");
  std::vector<std::vector<cista::pair<arrival_departure_label, label_bitfield>>> buffered_labels{
      tt_.n_locations(), std::vector<cista::pair<arrival_departure_label, label_bitfield>>()};

  for (auto l_idx = location_idx_t{0U}; l_idx != tt_.n_locations(); ++l_idx) {
    trace("┊ ├ updating footpaths of {} ({} of {})\n", location{tt_, l_idx}, l_idx, tt_.n_locations());
    if (!state_.station_mark_[to_idx(l_idx)]) {
      trace("┊ │    ├ Not marked -> SKIP!\n");
      continue;
    }
    const auto& round_bag = state_.round_bags_[k][to_idx(l_idx)];
    auto const fps = tt_.locations_.footpaths_out_[l_idx];
    for (auto const& fp : fps) {
      NIGIRI_PROFILE_COUNT(n_footpaths_visited_);
      auto const target = to_idx(fp.target_);
      auto const fp_offset = fp.duration_ - ((state_.is_destination_[to_idx(l_idx)]) ? minutes_after_midnight_t::zero() : tt_.locations_.transfer_time_[l_idx]);
      trace("┊ │    ├ Footpath of duration {} to target {} merging {} labels\n", fp_offset, location{tt_, fp.target_}, round_bag.size());
      for (const auto & rl : round_bag) {
        const arrival_departure_label l_with_foot{
          rl.label_.arrival_ + fp_offset,
              rl.label_.departure_};

        if ((l_with_foot.arrival_-l_with_foot.departure_).count() > kMaxTravelTime) {
          continue;
        }

        auto tdb = rl.tdb_;

        #ifdef PROFILE_RAPTOR_GLOBAL_PRUNING
        tdb = best_destination_bag.filter_dominated(l_with_foot, tdb);
        #endif

        #ifdef PROFILE_RAPTOR_LOCAL_PRUNING
        tdb = state_.best_bags_[target].filter_dominated(l_with_foot, tdb);
        #endif

        if (tdb.none()) {
          continue;
        }

        buffered_labels[target].push_back({l_with_foot, tdb});
      }
    }
  }

  for (auto l_idx = location_idx_t{0U}; l_idx != tt_.n_locations(); ++l_idx) {
    for (const auto& l : buffered_labels[to_idx(l_idx)]) {
      if (state_.round_bags_[k][to_idx(l_idx)].merge(l.first, l.second)) {
        NIGIRI_PROFILE_COUNT(n_earliest_arrival_updated_by_footpath_);
        state_.station_mark_[to_idx(l_idx)] = true;
      }
    }
  }

}

void profile_raptor::get_earliest_sufficient_transports(const arrival_departure_label label,
                                                        label_bitfield lbl_tdb,
                                                        route_idx_t const r,
                                                        unsigned const stop_idx,
                                                        raptor_route_bag& bag) {
  NIGIRI_PROFILE_COUNT(n_earliest_trip_calls_);
  const auto lbl_arr_offset = (label.arrival_.count() / 1440U);

  auto const& stop_event_times = tt_.event_times_at_stop(
      r, stop_idx, event_type::kDep);

  for (auto i = day_idx_t::value_t{0U}; i != n_days_to_iterate_; ++i) {
    const auto day = lbl_arr_offset + i;
    if (lbl_tdb.none()) {
      return;
    }

    auto const time_range_to_scan = it_range{
        i == 0U ? std::lower_bound(
                      stop_event_times.begin(),
                      stop_event_times.end(), label.arrival_,
                      [&](auto&& a, auto&& b) { return (a % 1440U) < (b % 1440U); }) : stop_event_times.begin(),
        stop_event_times.end()};

    if (time_range_to_scan.empty()) {
      continue;
    }
    auto const base =
        static_cast<unsigned>(&*time_range_to_scan.begin_ - stop_event_times.data());

    auto const preliminary_tdb_bias_shit = to_idx(start_day_offset()) + day;
    for (auto const [t_offset, ev] : utl::enumerate(time_range_to_scan)) {
      auto const ev_mam = minutes_after_midnight_t{
          ev.count() < 1440 ? ev.count() : ev.count() % 1440};

      const auto travel_time_lb = ev_mam + minutes_after_midnight_t{day * 1440U} - label.departure_;
      if (travel_time_lb.count() > kMaxTravelTime || lbl_tdb.none()) {
        return;
      }

      auto const ev_day_offset = static_cast<day_idx_t::value_t>(
          ev.count() < 1440
              ? 0
              : ev.count() / 1440);

      auto const& t = tt_.route_transport_ranges_[r][base + t_offset];

      auto const& trip_traffic_day_bitfield = (preliminary_tdb_bias_shit >= ev_day_offset) ?
                                   tt_.bitfields_[tt_.transport_traffic_days_[t]] >> (preliminary_tdb_bias_shit - ev_day_offset) :
                                   tt_.bitfields_[tt_.transport_traffic_days_[t]] << (ev_day_offset - preliminary_tdb_bias_shit);
      auto trip_label_traffic_day_bitfield = label_bitfield{};
      truncate_to(trip_traffic_day_bitfield, trip_label_traffic_day_bitfield);

      if (trip_label_traffic_day_bitfield.none()) continue;

      const auto new_td_bitfield = lbl_tdb & (~trip_label_traffic_day_bitfield);
      if ((new_td_bitfield ^ lbl_tdb).any()) {
        bag.merge(transport_departure_label{
            relative_transport{t, relative_day_idx_t{day - ev_day_offset}},
            label.departure_},
            lbl_tdb & trip_label_traffic_day_bitfield
        );
        lbl_tdb = new_td_bitfield;
      }
    }
  }
}

void profile_raptor::force_print_state(const char* comment) {
  auto const empty_rounds = [&](std::uint32_t const l) {
    for (auto k = 0U; k != end_k(); ++k) {
      if (state_.round_bags_[k][l].size() != 0) {
        return false;
      }
    }
    return true;
  };

  fmt::print("PROFILE RAPTOR STATE INFO: {}\n", comment);

  for (auto l = 0U; l != tt_.n_locations(); ++l) {
    if (!is_special(location_idx_t{l}) && !state_.is_destination_[l] &&
        state_.best_bags_[l].size() == 0 && empty_rounds(l)) {
      continue;
    }

    std::string_view name, track;
    auto const type = tt_.locations_.types_.at(location_idx_t{l});
    if (type == location_type::kTrack) {
      name =
          tt_.locations_.names_.at(tt_.locations_.parents_[location_idx_t{l}])
              .view();
      track = tt_.locations_.names_.at(location_idx_t{l}).view();
    } else {
      name = tt_.locations_.names_.at(location_idx_t{l}).view();
      track = "---";
    }
    tt_.locations_.names_[location_idx_t{l}].view();
    auto const id = tt_.locations_.ids_[location_idx_t{l}].view();
    fmt::print(
        "[{}] {:8} [name={:48}, track={:10}, id={:16}]: ",
        state_.is_destination_[l] ? "X" : "_", l, name, track,
        id.substr(0, std::min(std::string_view ::size_type{16U}, id.size())));
    auto const b = state_.best_bags_[l];
    if (b.size() == 0) {
      fmt::print("best=_________, round_times: ");
    } else {
      fmt::print("best={:9}, round_times: ", b.size());
    }
    for (auto i = 0U; i != kMaxTransfers + 1U; ++i) {
      auto const rb = state_.round_bags_[i][l];
      if (rb.size() > 0) {
        fmt::print("{:9} ", rb.size());
      } else {
        fmt::print("_________ ");
      }
    }
    fmt::print("\n");
  }
}


void profile_raptor::reconstruct() {
  const auto t1 = std::chrono::steady_clock::now();
  state_.results_.resize(
      std::max(state_.results_.size(), state_.destinations_.size()));

  bmc_raptor_reconstructor reconstructor(tt_, q_, state_, search_interval_);
  reconstructor.reconstruct();
  const auto t2 = std::chrono::steady_clock::now();
  stats_.time_reconstruction_ += std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
}


#ifdef NIGIRI_RAPTOR_TRACING
void profile_raptor::print_state(char const* comment) {
  force_print_state(comment);
}
#else
void profile_raptor::print_state(char const*) {}
#endif

}
