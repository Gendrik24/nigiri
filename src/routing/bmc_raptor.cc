#include "nigiri/routing/bmc_raptor.h"

#include <string>
#include <algorithm>
#include <set>
#include <chrono>
#include <utility>
#include <array>

#include "nigiri/location.h"
#include "nigiri/routing/bmc_raptor_search_state.h"
#include "nigiri/routing/dijkstra.h"
#include "nigiri/routing/for_each_meta.h"
#include "nigiri/routing/mc_raptor_reconstructor.h"
#include "nigiri/routing/start_times.h"
#include "nigiri/special_stations.h"
#include "nigiri/timetable.h"
#include "nigiri/tracer.h"
#include "nigiri/types.h"

#include "utl/overloaded.h"
#include "utl/enumerate.h"
#include "utl/erase_if.h"
#include "utl/timing.h"

#include "fmt/core.h"

#define NIGIRI_PROFILE_RAPTOR_COUNTING
#ifdef NIGIRI_PROFILE_RAPTOR_COUNTING
#define NIGIRI_PROFILE_COUNT(s) ++stats_.s
#define NIGIRI_PROFILE_COUNT_BY(s,a) stats_.s += a
#else
#define NIGIRI_PROFILE_COUNT(s)
#define NIGIRI_PROFILE_COUNT_BY(s,a)
#endif

namespace nigiri::routing {

template <criteria crit>
bmc_raptor<crit>::bmc_raptor(const timetable& tt,
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

template <criteria crit>
bmc_raptor_stats const& bmc_raptor<crit>::get_stats() const {
  return stats_;
}

template <criteria crit>
day_idx_t bmc_raptor<crit>::start_day_offset() const {
  return tt_.day_idx_mam(this->search_interval_.from_).first;
}

template <criteria crit>
day_idx_t bmc_raptor<crit>::number_of_days_in_search_interval() const {
  return tt_.day_idx_mam(this->search_interval_.to_).first
         - tt_.day_idx_mam(this->search_interval_.from_).first + 1;
}

template <criteria crit>
unsigned bmc_raptor<crit>::end_k() const {
  return std::min(kMaxTransfers, q_.max_transfers_) + 1U;
}

template <criteria crit>
void bmc_raptor<crit>::init_location_with_offset(minutes_after_midnight_t time_to_arrive,
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

        auto const preliminary_tdb_bias_shit = to_idx(start_day_offset()) + (at_stop.count() / 1440);
        auto const& trip_tdb = (preliminary_tdb_bias_shit >= stop_time_day_offset) ?
           tt_.bitfields_[tt_.transport_traffic_days_[transport_idx]] >> (preliminary_tdb_bias_shit - stop_time_day_offset) :
           tt_.bitfields_[tt_.transport_traffic_days_[transport_idx]] << (stop_time_day_offset - preliminary_tdb_bias_shit);

        auto mask = std::string(to_idx(number_of_days_in_search_interval()), '1');
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
                            .merge(
              bmc_raptor_label{
                                at_stop,
                                start_time,
                                kBiCrit ? minutes_after_midnight_t::max() : time_to_arrive},
                                label_bitfield{traffic_day_bitfield.to_string()});

          state_.station_mark_[to_idx(location)] = merged || state_.station_mark_[to_idx(location)];
        }
      }
    }
    auto const first_out_of_interval = search_interval_.to_.time_since_epoch() % 1440;

    label_bitfield traffic_day_bitfield{
        std::string(to_idx(number_of_days_in_search_interval()), '0').replace(0U, 1U, "1"),
    };
    bool merged = state_.round_bags_[0U][to_idx(location)]
                      .merge(
        bmc_raptor_label{
                          first_out_of_interval + time_to_arrive,
                          first_out_of_interval,
                          kBiCrit ? minutes_after_midnight_t::max() : time_to_arrive},
                          traffic_day_bitfield);

    state_.station_mark_[to_idx(location)] = merged || state_.station_mark_[to_idx(location)];
  }
}

template <criteria crit>
void bmc_raptor<crit>::update_destination_bag(unsigned long k) {
  for (auto const dest : state_.destinations_.front()) {
    for (const auto& rl : state_.round_bags_[k-1][to_idx(dest)]) {
      best_destination_bag.merge(rl.label_, rl.tdb_);
    }
  }
}

template <criteria crit>
void bmc_raptor<crit>::init_starts() {
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

template <criteria crit>
void bmc_raptor<crit>::route() {
  state_.reset(tt_);
  state_.search_interval_ = search_interval_;

  collect_destinations(tt_, q_.destinations_, q_.dest_match_mode_,
                       state_.destinations_, state_.is_destination_);
  init_starts();
  #ifdef BMC_RAPTOR_LOWER_BOUNDS

  #ifdef NIGIRI_PROFILE_RAPTOR_COUNTING
  UTL_START_TIMING(lb);
  #endif

  dijkstra(tt_, q_, tt_.fwd_search_lb_graph_,
           state_.travel_time_lower_bound_);
  for (auto l = location_idx_t{0U}; l != tt_.locations_.children_.size(); ++l) {
    auto const lb = state_.travel_time_lower_bound_[to_idx(l)];
    for (auto const c : tt_.locations_.children_[l]) {
      state_.travel_time_lower_bound_[to_idx(c)] = lb;
    }
  }

  #ifdef NIGIRI_PROFILE_RAPTOR_COUNTING
  UTL_STOP_TIMING(lb);
  stats_.lb_time_ = static_cast<std::uint64_t>(UTL_TIMING_MS(lb));
  #endif
  #endif
  rounds();
  reconstruct();
  for (auto& r : state_.results_) {
    utl::erase_if(r, [&](journey const& j) {
      return !search_interval_.contains(
          j.start_time_);
    });
  }
}

template <criteria crit>
void bmc_raptor<crit>::rounds() {
  print_state();
  for (auto k = 1U; k != end_k(); ++k) {
    trace_always("┊ round k={}\n", k);

    #ifdef BMC_RAPTOR_GLOBAL_PRUNING
    update_destination_bag(k);
    #endif

    // Round k
    auto any_marked = false;
    for (auto l_idx = location_idx_t{0U};
         l_idx != static_cast<cista::base_t<location_idx_t>>(
                      state_.station_mark_.size()); ++l_idx) {
      
      if (state_.station_mark_[to_idx(l_idx)]) {
        #ifdef BMC_RAPTOR_LOCAL_PRUNING
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
          if (!state_.route_mark_[to_idx(route_idx)]) {
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

template <criteria crit>
bool bmc_raptor<crit>::update_route(unsigned const k, route_idx_t route_idx) {

  auto any_marked = false;
  auto const stop_sequence = tt_.route_location_seq_[route_idx];

  bmc_raptor_route_bag_t r_b{};
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
    for (auto& active_label : r_b) {
      if (active_label.label_.transport_.t_idx_ == transport_idx_t::invalid()) {
        trace(
            "┊ │    │ current label={} has no active transport -> SKIP!\n",
            active_label.to_string());
        continue;
      }
      const auto new_arr = tt_.event_mam(route_idx,
                                         active_label.label_.transport_.t_idx_,
                                         stop_idx,
                                         event_type::kArr) + minutes_after_midnight_t{to_idx(active_label.label_.transport_.day_) * 1440};

      active_label.label_.current_stop_time_ = new_arr;

      trace(
          "┊ │    │ current label={}, active transport={} -> updated arrival time={}, transfer time={}\n",
          active_label.to_string(), active_label.t_, new_arr, new_arr);
      const auto candidate_lbl = bmc_raptor_label{
          new_arr + (is_destination ? minutes_after_midnight_t::zero() : transfer_time_offset),
          active_label.label_.departure_,
          kBiCrit ? minutes_after_midnight_t::max() : (active_label.label_.walking_time_ + (is_destination ? minutes_after_midnight_t::zero() : transfer_time_offset))};
      auto candidate_tdb = active_label.tdb_;

      if (!stop.out_allowed() || (candidate_lbl.arrival_ - candidate_lbl.departure_).count() > kMaxTravelTime) {
        continue;
      }

      #ifdef BMC_RAPTOR_GLOBAL_PRUNING
      candidate_tdb = best_destination_bag.filter_dominated(candidate_lbl, candidate_tdb);
      if (candidate_tdb.none()) {
        continue;
      }
      #endif

      #ifdef BMC_RAPTOR_LOCAL_PRUNING
      candidate_tdb = state_.best_bags_[l_idx].filter_dominated(candidate_lbl, candidate_tdb);
      if (candidate_tdb.none()) {
        continue;
      }
      #endif

      #ifdef BMC_RAPTOR_LOWER_BOUNDS
      auto const lower_bound =
          state_.travel_time_lower_bound_[to_idx(stop.location_idx())];
      if (lower_bound.count() ==
          std::numeric_limits<duration_t::rep>::max()) {
        NIGIRI_PROFILE_COUNT(route_update_prevented_by_lower_bound_);
        continue;
      }

      candidate_tdb = best_destination_bag.filter_dominated(
          bmc_raptor_label{
              new_arr + lower_bound,
              active_label.label_.departure_,
              kBiCrit ? minutes_after_midnight_t::max() : (active_label.label_.walking_time_ + (is_destination ? minutes_after_midnight_t::zero() : transfer_time_offset))},
            candidate_tdb);

      if (candidate_tdb.none()) {
        NIGIRI_PROFILE_COUNT(route_update_prevented_by_lower_bound_);
        continue;
      }
      #endif

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

template <criteria crit>
void bmc_raptor<crit>::update_footpaths(unsigned const k) {
  trace("┊ ├ FOOTPATHS\n");
  std::vector<std::vector<cista::pair<bmc_raptor_label, label_bitfield>>> buffered_labels{
      tt_.n_locations(), std::vector<cista::pair<bmc_raptor_label, label_bitfield>>()};

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
        const bmc_raptor_label l_with_foot{
              rl.label_.arrival_ + fp_offset,
              rl.label_.departure_,
              kBiCrit ? minutes_after_midnight_t::max() : rl.label_.walking_time_ + fp_offset};

        if ((l_with_foot.arrival_-l_with_foot.departure_).count() > kMaxTravelTime) {
          continue;
        }

        auto tdb = rl.tdb_;

        #ifdef BMC_RAPTOR_GLOBAL_PRUNING
        tdb = best_destination_bag.filter_dominated(l_with_foot, tdb);
        if (tdb.none()) {
          continue;
        }
        #endif

        #ifdef BMC_RAPTOR_LOCAL_PRUNING
        tdb = state_.best_bags_[target].filter_dominated(l_with_foot, tdb);
        if (tdb.none()) {
          continue;
        }
        #endif

        #ifdef BMC_RAPTOR_LOWER_BOUNDS
        auto const lower_bound =
            state_.travel_time_lower_bound_[target];
        if (lower_bound.count() ==
            std::numeric_limits<duration_t::rep>::max()) {
          NIGIRI_PROFILE_COUNT(fp_update_prevented_by_lower_bound_);
          continue;
        }

        tdb = best_destination_bag.filter_dominated(
            bmc_raptor_label{
                      rl.label_.arrival_ + fp_offset + lower_bound,
                      rl.label_.departure_,
                      kBiCrit ? minutes_after_midnight_t::max() : rl.label_.walking_time_ + fp_offset},
                  tdb);

        if (tdb.none()) {
          NIGIRI_PROFILE_COUNT(fp_update_prevented_by_lower_bound_);
          continue;
        }
        #endif

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

template <criteria crit>
void bmc_raptor<crit>::get_earliest_sufficient_transports(const bmc_raptor_label label,
                                                          label_bitfield lbl_tdb,
                                                          route_idx_t const r,
                                                          unsigned const stop_idx,
                                                          bmc_raptor_route_bag_t& bag) {
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
        const auto offset = day - ev_day_offset;
        bag.merge(
            bmc_raptor_route_label{
            relative_transport{t, relative_day_idx_t{offset}},
            label.departure_,
            (stop_idx == 0 ? tt_.event_mam(t,stop_idx,event_type::kDep) : tt_.event_mam(t,stop_idx,event_type::kArr)) + long_minutes_after_midnight_t{offset * 1440},
            kBiCrit ? minutes_after_midnight_t::max() : label.walking_time_},
            lbl_tdb & trip_label_traffic_day_bitfield
        );
        lbl_tdb = new_td_bitfield;
      }
    }
  }
}

template <criteria crit>
void bmc_raptor<crit>::force_print_state(const char* comment) {
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

template <criteria crit>
void bmc_raptor<crit>::reconstruct() {
  state_.results_.resize(
      std::max(state_.results_.size(), state_.destinations_.size()));

  #ifdef NIGIRI_PROFILE_RAPTOR_COUNTING
  UTL_START_TIMING(rc);
  #endif

  matrix<uncompressed_round_times_t> round_times =
      make_flat_matrix<uncompressed_round_times_t>(end_k(), tt_.n_locations());

  const auto first_day_offset =
      tt_.day_idx_mam(search_interval_.from_).first;

  for (auto r = 0U; r < end_k(); ++r) {
    for (auto l = 0U; l < tt_.n_locations(); ++l) {
      const auto& bag = state_.round_bags_[r][l];
      auto& ert = round_times[r][l];

      const auto& uncompressed_labels = bag.uncompress();
      ert.reserve(uncompressed_labels.size());
      for (const auto& arr_dep_l : uncompressed_labels) {
        ert.push_back(mc_raptor_label{
            routing_time{first_day_offset + arr_dep_l.arrival_.count() / 1440, arr_dep_l.arrival_ % 1440},
            routing_time{first_day_offset + arr_dep_l.departure_.count() / 1440, arr_dep_l.departure_ % 1440},
            kBiCrit ? minutes_after_midnight_t::max() : arr_dep_l.walking_time_
        });
      }
    }
  }

  mc_raptor_reconstructor reconstructor(tt_,
                                        q_,
                                        round_times,
                                        search_interval_,
                                        state_.destinations_,
                                        state_.results_);
  reconstructor.reconstruct();

  #ifdef NIGIRI_PROFILE_RAPTOR_COUNTING
  UTL_STOP_TIMING(rc);
  stats_.n_reconstruction_time = static_cast<std::uint64_t>(UTL_TIMING_MS(rc));
  #endif
}


#ifdef NIGIRI_RAPTOR_TRACING
void profile_raptor::print_state(char const* comment) {
  force_print_state(comment);
}
#else
template <criteria crit>
void bmc_raptor<crit>::print_state(char const*) {}
#endif

template struct bmc_raptor<criteria::biCriteria>;
template struct bmc_raptor<criteria::multiCriteria>;

}
