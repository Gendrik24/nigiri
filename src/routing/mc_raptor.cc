#include "nigiri/routing/mc_raptor.h"

#include <string>
#include <algorithm>
#include <set>
#include <chrono>
#include <utility>
#include <array>
#include <iterator>

#include "nigiri/location.h"
#include "nigiri/routing/dijkstra.h"
#include "nigiri/routing/for_each_meta.h"
#include "nigiri/routing/mc_raptor_reconstructor.h"
#include "nigiri/routing/mc_raptor_search_state.h"
#include "nigiri/routing/start_times.h"
#include "nigiri/special_stations.h"
#include "nigiri/timetable.h"
#include "nigiri/types.h"

#include "utl/overloaded.h"
#include "utl/enumerate.h"
#include "utl/erase_if.h"
#include "utl/equal_ranges_linear.h"
#include "utl/timing.h"

#include "fmt/core.h"

#define NIGIRI_MC_RAPTOR_COUNTING
#ifdef NIGIRI_MC_RAPTOR_COUNTING
#define NIGIRI_MC_RAPTOR_COUNT(s) ++stats_.s
#define NIGIRI_MC_RAPTOR_COUNT_BY(s,a) stats_.s += a
#else
#define NIGIRI_MC_RAPTOR_COUNT(s)
#define NIGIRI_MC_RAPTOR_COUNT_BY(s,a)
#endif

namespace nigiri::routing {

template <criteria crit>
bool mc_raptor<crit>::is_better(auto a, auto b) {
  return a < b;
}

template <criteria crit>
bool mc_raptor<crit>::is_better_or_eq(auto a, auto b) {
  return a <= b;
}

template <criteria crit>
auto mc_raptor<crit>::get_best(auto a, auto b) {
  return is_better(a, b) ? a : b;
}

template <criteria crit>
mc_raptor<crit>::mc_raptor(const timetable& tt,
                     mc_raptor_search_state& state,
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
mc_raptor_stats const& mc_raptor<crit>::get_stats() const {
  return stats_;
}

template <criteria crit>
day_idx_t mc_raptor<crit>::start_day_offset() const {
  return tt_.day_idx_mam(this->search_interval_.from_).first;
}

template <criteria crit>
day_idx_t mc_raptor<crit>::number_of_days_in_search_interval() const {
  return tt_.day_idx_mam(this->search_interval_.to_).first
         - tt_.day_idx_mam(this->search_interval_.from_).first + 1;
}

template <criteria crit>
unsigned mc_raptor<crit>::end_k() const {
  return std::min(kMaxTransfers, q_.max_transfers_) + 1U;
}

template <criteria crit>
void mc_raptor<crit>::update_destination_bag(unsigned long k) {
  for (auto const dest : state_.destinations_.front()) {
    for (const auto& rl : state_.round_bags_[k-1][to_idx(dest)]) {
      best_destination_bag.merge(rl);
    }
  }
}

template <criteria crit>
void mc_raptor<crit>::route() {
  state_.reset(tt_);
  state_.search_interval_ = search_interval_;

  collect_destinations(tt_, q_.destinations_, q_.dest_match_mode_,
                       state_.destinations_, state_.is_destination_);
  state_.results_.resize(
      std::max(state_.results_.size(), state_.destinations_.size()));
  get_starts<direction::kForward>(tt_, q_.start_time_, q_.start_, q_.start_match_mode_,
                        q_.use_start_footpaths_, state_.starts_);
  #ifdef MC_RAPTOR_LOWER_BOUNDS

  #ifdef NIGIRI_MC_RAPTOR_COUNTING
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

  #ifdef NIGIRI_MC_RAPTOR_COUNTING
    UTL_STOP_TIMING(lb);
    stats_.lb_time_ = static_cast<std::uint64_t>(UTL_TIMING_MS(lb));
  #endif
  #endif
    utl::equal_ranges_linear(
        state_.starts_,
        [](start const& a, start const& b) {
          return a.time_at_start_ == b.time_at_start_;
        },
        [&](auto&& from_it, auto&& to_it) {
          for (auto const& s : it_range{from_it, to_it}) {
            state_.round_bags_[0U][to_idx(s.stop_)].merge(mc_raptor_label(
                {tt_, s.time_at_stop_},
                {tt_, from_it->time_at_start_},
                kBiCrit ? minutes_after_midnight_t::max() :
                          minutes_after_midnight_t{routing_time{tt_, s.time_at_stop_}.offset_ - routing_time{tt_, from_it->time_at_start_}.offset_}
            ));
            state_.best_bags_[to_idx(s.stop_)].merge(mc_raptor_label(
                {tt_, s.time_at_stop_},
                {tt_, from_it->time_at_start_},
                kBiCrit ? minutes_after_midnight_t::max() :
                        minutes_after_midnight_t{routing_time{tt_, s.time_at_stop_}.offset_ - routing_time{tt_, from_it->time_at_start_}.offset_}
            ));
            state_.station_mark_[to_idx(s.stop_)] = true;
          }
        });
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
void mc_raptor<crit>::rounds() {
  for (auto k = 1U; k != end_k(); ++k) {

#ifdef MC_RAPTOR_GLOBAL_PRUNING
    update_destination_bag(k);
#endif

    // Round k
    auto any_marked = false;
    for (auto l_idx = location_idx_t{0U};
         l_idx != static_cast<cista::base_t<location_idx_t>>(
                      state_.station_mark_.size()); ++l_idx) {

      if (state_.station_mark_[to_idx(l_idx)]) {
#ifdef MC_RAPTOR_LOCAL_PRUNING
        for (const auto& l : state_.round_bags_[k-1][to_idx(l_idx)]) {
          state_.best_bags_[to_idx(l_idx)].merge(l);
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
    NIGIRI_MC_RAPTOR_COUNT_BY(n_routes_visited_, routes_visited);
#else
    for (auto r_id = 0U; r_id != tt_.n_routes(); ++r_id) {
      if (!state_.route_mark_[r_id]) {
        continue;
      }
      NIGIRI_MC_RAPTOR_COUNT(n_routes_visited_);
      any_marked |= update_route(k, route_idx_t{r_id});
    }
#endif

    std::fill(begin(state_.route_mark_), end(state_.route_mark_), false);
    if (!any_marked) {
      return;
    }
    update_footpaths(k);
  }
}

template <criteria crit>
bool mc_raptor<crit>::update_route(unsigned const k, route_idx_t route_idx) {

  auto any_marked = false;
  auto const stop_sequence = tt_.route_location_seq_[route_idx];

  mc_raptor_route_bag_t r_b{};
  for (auto i = 0U; i != stop_sequence.size(); ++i) {
    auto const stop_idx =
        static_cast<unsigned>(i);
    auto const stop = timetable::stop{stop_sequence[stop_idx]};
    auto const l_idx = cista::to_idx(stop.location_idx());

    auto const transfer_time_offset = tt_.locations_.transfer_time_[location_idx_t{l_idx}];
    auto const is_destination = state_.is_destination_[l_idx];
    for (const auto& active_label : r_b) {
      if (active_label.transport_.t_idx_ == transport_idx_t::invalid()) {
        continue;
      }
      const routing_time new_arr(active_label.transport_.day_, tt_.event_mam(route_idx,
                                                  active_label.transport_.t_idx_,
                                                  stop_idx,
                                                  event_type::kArr));

      const auto candidate_lbl = mc_raptor_label{
          new_arr + (is_destination ? minutes_after_midnight_t::zero() : transfer_time_offset),
          active_label.departure_,
          kBiCrit ? minutes_after_midnight_t::max() : (active_label.walking_time_ + (is_destination ? minutes_after_midnight_t::zero() : transfer_time_offset))};

      if (!stop.out_allowed() || (candidate_lbl.arrival_.offset_ - candidate_lbl.departure_.offset_) > kMaxTravelTime) {
        continue;
      }

#ifdef MC_RAPTOR_GLOBAL_PRUNING
      if (best_destination_bag.is_dominated(candidate_lbl)) {
        continue;
      }
#endif

#ifdef MC_RAPTOR_LOCAL_PRUNING
      if (state_.best_bags_[l_idx].is_dominated(candidate_lbl)) {
        continue;
      }
#endif

#ifdef MC_RAPTOR_LOWER_BOUNDS
      auto const lower_bound =
          state_.travel_time_lower_bound_[to_idx(stop.location_idx())];
      if (lower_bound.count() ==
          std::numeric_limits<duration_t::rep>::max()) {
        NIGIRI_MC_RAPTOR_COUNT(route_update_prevented_by_lower_bound_);
        continue;
      }

      const auto dominated = best_destination_bag.is_dominated(mc_raptor_label{
            new_arr + lower_bound,
            active_label.departure_,
            kBiCrit ? minutes_after_midnight_t::max() : (active_label.walking_time_ + (is_destination ? minutes_after_midnight_t::zero() : transfer_time_offset))});

      if (dominated) {
        NIGIRI_MC_RAPTOR_COUNT(route_update_prevented_by_lower_bound_);
        continue;
      }
#endif

      if (state_.round_bags_[k][cista::to_idx(l_idx)].merge(candidate_lbl).first) {
        NIGIRI_MC_RAPTOR_COUNT(n_earliest_arrival_updated_by_route_);
        state_.station_mark_[l_idx] = true;
        any_marked = true;
      }
    }

    if (i == stop_sequence.size()-1) {
      return any_marked;
    }

    if (stop.in_allowed() && state_.prev_station_mark_[l_idx]) {
      for (const auto& l : state_.round_bags_[k-1][cista::to_idx(l_idx)]) {
        auto const new_et =
          get_earliest_transport(l, route_idx, stop_idx, location_idx_t{l_idx});
        if (new_et.is_valid()) {
          r_b.merge(mc_raptor_route_label(new_et, l.departure_, l.walking_time_));
        }
      }
    }
  }
  return any_marked;
}

template <criteria crit>
void mc_raptor<crit>::update_footpaths(unsigned const k) {
  std::vector<std::vector<mc_raptor_label>> buffered_labels{
      tt_.n_locations(), std::vector<mc_raptor_label>()};

  for (auto l_idx = location_idx_t{0U}; l_idx != tt_.n_locations(); ++l_idx) {
    if (!state_.station_mark_[to_idx(l_idx)]) {
      continue;
    }
    const auto& round_bag = state_.round_bags_[k][to_idx(l_idx)];
    auto const fps = tt_.locations_.footpaths_out_[l_idx];
    for (auto const& fp : fps) {
      NIGIRI_MC_RAPTOR_COUNT(n_footpaths_visited_);
      auto const target = to_idx(fp.target_);
      auto const fp_offset = fp.duration_ - ((state_.is_destination_[to_idx(l_idx)]) ? minutes_after_midnight_t::zero() : tt_.locations_.transfer_time_[l_idx]);
      for (const auto & rl : round_bag) {
        const mc_raptor_label l_with_foot{
            rl.arrival_ + fp_offset,
            rl.departure_,
            kBiCrit ? minutes_after_midnight_t::max() : rl.walking_time_ + fp_offset};

        if ((l_with_foot.arrival_.offset_ - l_with_foot.departure_.offset_) > kMaxTravelTime) {
          continue;
        }

#ifdef MC_RAPTOR_GLOBAL_PRUNING
        if (best_destination_bag.is_dominated(l_with_foot)) {
          continue;
        }
#endif

#ifdef MC_RAPTOR_LOCAL_PRUNING
        if (state_.best_bags_[target].is_dominated(l_with_foot)) {
          continue;
        }
#endif

#ifdef MC_RAPTOR_LOWER_BOUNDS
        auto const lower_bound =
            state_.travel_time_lower_bound_[target];
        if (lower_bound.count() ==
            std::numeric_limits<duration_t::rep>::max()) {
          NIGIRI_MC_RAPTOR_COUNT(fp_update_prevented_by_lower_bound_);
          continue;
        }

        const auto dominated = best_destination_bag.is_dominated(mc_raptor_label{
                                                        rl.arrival_ + fp_offset + lower_bound,
                                                        rl.departure_,
                                                        kBiCrit ? minutes_after_midnight_t::max() : rl.walking_time_ + fp_offset});

        if (dominated) {
          NIGIRI_MC_RAPTOR_COUNT(fp_update_prevented_by_lower_bound_);
          continue;
        }
#endif

        buffered_labels[target].push_back(l_with_foot);
      }
    }
  }

  for (auto l_idx = location_idx_t{0U}; l_idx != tt_.n_locations(); ++l_idx) {
    for (const auto& l : buffered_labels[to_idx(l_idx)]) {
      if (state_.round_bags_[k][to_idx(l_idx)].merge(l).first) {
        NIGIRI_MC_RAPTOR_COUNT(n_earliest_arrival_updated_by_footpath_);
        state_.station_mark_[to_idx(l_idx)] = true;
      }
    }
  }

}

template <criteria crit>
transport mc_raptor<crit>::get_earliest_transport(
    const mc_raptor_label& current,
    route_idx_t const r,
    unsigned const stop_idx,
    location_idx_t const l_idx) {
  NIGIRI_MC_RAPTOR_COUNT(n_earliest_trip_calls_);

  auto time = current.arrival_;
  if (time == kInvalidTime<direction::kForward>) {
    return {transport_idx_t::invalid(), day_idx_t::invalid()};
  }

  if (state_.is_destination_[to_idx(l_idx)]) {
    time = time + tt_.locations_.transfer_time_[l_idx];
  }
  auto const [day_at_stop, mam_at_stop] = time.day_idx_mam();

  auto const n_days_to_iterate =
      std::min(kMaxTravelTime / 1440U + 1,
               n_tt_days_ - to_idx(day_at_stop) + 1U);

  auto const event_times = tt_.event_times_at_stop(
      r, stop_idx, event_type::kDep);

  auto const seek_first_day = [&, mam_at_stop = mam_at_stop]() {
    return std::lower_bound(
        event_times.begin(),
        event_times.end(), mam_at_stop,
        [&](auto&& a, auto&& b) { return is_better(a, b); });
  };


  for (auto i = day_idx_t::value_t{0U}; i != n_days_to_iterate; ++i) {
    auto const day = day_at_stop + i;
    auto const ev_time_range = it_range{
        i == 0U ? seek_first_day() : event_times.begin(),
        event_times.end()};
    if (ev_time_range.empty()) {
      continue;
    }
    auto const base =
        static_cast<unsigned>(&*ev_time_range.begin_ - event_times.data());
    for (auto const [t_offset, ev] : utl::enumerate(ev_time_range)) {
      auto const ev_mam = minutes_after_midnight_t{
          ev.count() < 1440 ? ev.count() : ev.count() % 1440};

      if (best_destination_bag.is_dominated(mc_raptor_label(routing_time{day, ev_mam},
                                                            current.departure_,
                                                            kBiCrit ? minutes_after_midnight_t::max() : current.walking_time_))) {
        return {transport_idx_t::invalid(), day_idx_t::invalid()};
      }

      auto const t = tt_.route_transport_ranges_[r][base + t_offset];
      if (day == day_at_stop && !is_better_or_eq(mam_at_stop, ev_mam)) {
        continue;
      }

      auto const ev_day_offset = static_cast<day_idx_t::value_t>(
          ev.count() < 1440
              ? 0
              : static_cast<cista::base_t<day_idx_t>>(ev.count() / 1440));
      if (!tt_.bitfields_[tt_.transport_traffic_days_[t]].test(
              static_cast<std::size_t>(to_idx(day) - ev_day_offset))) {
        continue;
      }
      return {t, static_cast<day_idx_t>(day - ev_day_offset)};
    }
  }
  return {transport_idx_t::invalid(), day_idx_t::invalid()};
}

template <criteria crit>
void mc_raptor<crit>::reconstruct() {
  state_.results_.resize(
      std::max(state_.results_.size(), state_.destinations_.size()));

#ifdef NIGIRI_MC_RAPTOR_COUNTING
  UTL_START_TIMING(rc);
#endif

  matrix<uncompressed_round_times_t> round_times =
      make_flat_matrix<uncompressed_round_times_t>(end_k(), tt_.n_locations());


  for (auto r = 0U; r < end_k(); ++r) {
    for (auto l = 0U; l < tt_.n_locations(); ++l) {
      const auto& bag = state_.round_bags_[r][l];
      auto& ert = round_times[r][l];

      std::copy(bag.begin(), bag.end(), std::back_inserter(ert));
    }
  }

  mc_raptor_reconstructor reconstructor(tt_,
                                        q_,
                                        round_times,
                                        search_interval_,
                                        state_.destinations_,
                                        state_.results_);
  reconstructor.reconstruct();

#ifdef NIGIRI_MC_RAPTOR_COUNTING
  UTL_STOP_TIMING(rc);
  stats_.n_reconstruction_time = static_cast<std::uint64_t>(UTL_TIMING_MS(rc));
#endif
}

template struct mc_raptor<criteria::biCriteria>;
template struct mc_raptor<criteria::multiCriteria>;


}
