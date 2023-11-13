#include "nigiri/routing/raptor/mc_raptor.h"

#include "nigiri/routing/start_times.h"
#include "utl/equal_ranges_linear.h"
#include "nigiri/timetable.h"
#include "nigiri/routing/raptor/mc_raptor_state.h"
#include "nigiri/stop.h"
#include "utl/erase_if.h"
#include "utl/enumerate.h"

namespace nigiri::routing {

bool mc_raptor::is_better(auto a, auto b) {
  return a < b;
}

bool mc_raptor::is_better_or_eq(auto a, auto b) {
  return a <= b;
}

auto mc_raptor::get_best(auto a, auto b) {
  return is_better(a, b) ? a : b;
}


mc_raptor::mc_raptor(timetable const& tt,
                     mc_raptor_state& state,
                     interval<unixtime_t> const search_interval,
                     location_match_mode const start_match_mode,
                     std::vector<offset> const start)
    :   tt_{tt},
        n_tt_days_{static_cast<std::uint16_t>(tt_.date_range_.size().count())},
        state_{state},
        search_interval_{search_interval},
        n_locations_{tt_.n_locations()},
        n_routes_{tt.n_routes()},
        start_{start},
        start_match_mode_{start_match_mode},
        n_days_to_iterate_{std::min(kMaxTravelTime.count() / 1440U + 1,
                                  static_cast<unsigned int>(n_tt_days_ - to_idx(start_day_offset())))}
{
  state_.resize(n_locations_, n_routes_);
  state_.round_bags_.reset(pareto_set<mc_raptor_label>{});
};

mc_raptor_stats const& mc_raptor::get_stats() const {
  return stats_;
}

day_idx_t mc_raptor::start_day_offset() const {
  return tt_.day_idx_mam(this->search_interval_.from_).first;
}

day_idx_t mc_raptor::number_of_days_in_search_interval() const {
  return tt_.day_idx_mam(this->search_interval_.to_).first
         - tt_.day_idx_mam(this->search_interval_.from_).first + 1;
}

unsigned mc_raptor::end_k() const {
  return kMaxTransfers + 1U;
}

void mc_raptor::route() {
  std::vector<start> starts;

  get_starts(direction::kForward, tt_, nullptr,
             search_interval_, start_, start_match_mode_,
             true, starts, true);
  utl::equal_ranges_linear(
      starts,
      [](start const& a, start const& b) {
        return a.time_at_start_ == b.time_at_start_;
      },
      [&](auto&& from_it, auto&& to_it) {
        for (auto const& s : it_range{from_it, to_it}) {
          state_.round_bags_[0U][to_idx(s.stop_)].add(mc_raptor_label(
              {tt_, s.time_at_stop_},
              {tt_, from_it->time_at_start_}));
          state_.best_[to_idx(s.stop_)].add(mc_raptor_label(
              {tt_, s.time_at_stop_},
              {tt_, from_it->time_at_start_}));
          state_.station_mark_[to_idx(s.stop_)] = true;
        }
      });
  rounds();

  for (auto i = 0U; i != n_locations_; ++i) {

    for (auto k = 1U; k != end_k(); ++k) {
      auto const round_bag = state_.round_bags_[k][i];
      if (round_bag.size() != 0) {
        for (const auto& j : round_bag) {
          state_.results_[i].add(
              journey{.legs_ = {},
                      .start_time_ = j.departure_.to_unixtime(tt_),
                      .dest_time_ = j.arrival_.to_unixtime(tt_),
                      .dest_ = location_idx_t{i},
                      .transfers_ = static_cast<std::uint8_t>(k - 1)});
        }
      }
    }
  }

  for (auto& r : state_.results_) {
    utl::erase_if(r, [&](journey const& j) {
      return !search_interval_.contains(
          j.start_time_);
    });
  }
}

void mc_raptor::rounds() {
  for (auto k = 1U; k != end_k(); ++k) {

    // Round k
    auto any_marked = false;
    for (auto l_idx = location_idx_t{0U};
         l_idx != static_cast<cista::base_t<location_idx_t>>(
                      state_.station_mark_.size()); ++l_idx) {

      if (state_.station_mark_[to_idx(l_idx)]) {
        for (auto& l : state_.round_bags_[k-1][to_idx(l_idx)]) {
          state_.best_[to_idx(l_idx)].add(std::forward<mc_raptor_label>(l));
        }
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
    for (auto r_id = 0U; r_id != tt_.n_routes(); ++r_id) {
      if (!state_.route_mark_[r_id]) {
        continue;
      }
      any_marked |= update_route(k, route_idx_t{r_id});
    }

    std::fill(begin(state_.route_mark_), end(state_.route_mark_), false);
    if (!any_marked) {
      return;
    }
    update_footpaths(k);
  }
}

bool mc_raptor::update_route(unsigned const k, route_idx_t route_idx) {

  auto any_marked = false;
  auto const stop_sequence = tt_.route_location_seq_[route_idx];

  pareto_set<mc_raptor_route_label> r_b{};
  for (auto i = 0U; i != stop_sequence.size(); ++i) {
    auto const stop_idx =
        static_cast<stop_idx_t>(i);
    auto const stp = stop{stop_sequence[stop_idx]};
    auto const l_idx = cista::to_idx(stp.location_idx());

    auto const transfer_time_offset = tt_.locations_.transfer_time_[location_idx_t{l_idx}];
    for (const auto& active_label : r_b) {
      if (active_label.transport_.t_idx_ == transport_idx_t::invalid()) {
        continue;
      }
      const routing_time new_arr(active_label.transport_.day_, tt_.event_mam(route_idx,
                                                                             active_label.transport_.t_idx_,
                                                                             stop_idx,
                                                                             event_type::kArr).as_duration());

      auto candidate_lbl = mc_raptor_label{
          new_arr + transfer_time_offset,
          active_label.departure_};

      if (!stp.out_allowed() || (candidate_lbl.arrival_.offset_ - candidate_lbl.departure_.offset_) > kMaxTravelTime.count()) {
        continue;
      }

      for (const auto& el : state_.best_[l_idx]) {
        if (el.dominates(candidate_lbl)) {
          continue;
        }
      }

      if (std::get<0>(state_.round_bags_[k][cista::to_idx(l_idx)].add(std::forward<mc_raptor_label>(candidate_lbl)))) {
        state_.station_mark_[l_idx] = true;
        any_marked = true;
      }
    }

    if (i == stop_sequence.size()-1) {
      return any_marked;
    }

    if (stp.in_allowed() && state_.prev_station_mark_[l_idx]) {
      for (const auto& l : state_.round_bags_[k-1][cista::to_idx(l_idx)]) {
        auto const new_et =
            get_earliest_transport(l, route_idx, stop_idx);
        if (new_et.is_valid()) {
          r_b.add(mc_raptor_route_label(new_et, l.departure_));
        }
      }
    }
  }
  return any_marked;
}

void mc_raptor::update_footpaths(unsigned const k) {
  std::vector<std::vector<mc_raptor_label>> buffered_labels{
      tt_.n_locations(), std::vector<mc_raptor_label>()};

  for (auto l_idx = location_idx_t{0U}; l_idx != tt_.n_locations(); ++l_idx) {
    if (!state_.station_mark_[to_idx(l_idx)]) {
      continue;
    }
    const auto& round_bag = state_.round_bags_[k][to_idx(l_idx)];
    auto const fps = tt_.locations_.footpaths_out_[l_idx];
    for (auto const& fp : fps) {
      auto const target = fp.target_;
      auto const fp_offset = fp.duration_ - tt_.locations_.transfer_time_[l_idx].count();
      for (const auto & rl : round_bag) {
        const mc_raptor_label l_with_foot{
            rl.arrival_ + duration_t {fp_offset},
            rl.departure_};

        if ((l_with_foot.arrival_.offset_ - l_with_foot.departure_.offset_) > kMaxTravelTime.count()) {
          continue;
        }

        for (const auto& el : state_.best_[target]) {
          if (el.dominates(l_with_foot)) {
            continue;
          }
        }

        buffered_labels[target].push_back(l_with_foot);
      }
    }
  }

  for (auto l_idx = location_idx_t{0U}; l_idx != tt_.n_locations(); ++l_idx) {
    for (auto& l : buffered_labels[to_idx(l_idx)]) {
      if (std::get<0>(state_.round_bags_[k][to_idx(l_idx)].add(std::forward<mc_raptor_label>(l)))) {
        state_.station_mark_[to_idx(l_idx)] = true;
      }
    }
  }

}

transport mc_raptor::get_earliest_transport(
    const mc_raptor_label& current,
    route_idx_t const r,
    stop_idx_t stop_idx) {

  auto time = current.arrival_;
  if (time == kInvalidTime<direction::kForward>) {
    return {transport_idx_t::invalid(), day_idx_t::invalid()};
  }

  auto const [day_at_stop, mam_at_stop] = time.day_idx_mam();

  auto const n_days_to_iterate =
      std::min(kMaxTravelTime.count() / 1440U + 1,
               n_tt_days_ - to_idx(day_at_stop) + 1U);

  auto const event_times = tt_.event_times_at_stop(
      r, stop_idx, event_type::kDep);

  auto const seek_first_day = [&, m_at_stop = mam_at_stop]() {
    return std::lower_bound(
        event_times.begin(),
        event_times.end(), m_at_stop,
        [&](auto&& a, auto&& b) { return is_better(a.as_duration(), b); });
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
}

