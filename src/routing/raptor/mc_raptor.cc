#include "nigiri/routing/raptor/mc_raptor.h"

#include "utl/equal_ranges_linear.h"
#include "utl/erase_if.h"
#include "utl/enumerate.h"
#include "utl/helpers/algorithm.h"

#include "nigiri/stop.h"
#include "nigiri/timetable.h"
#include "nigiri/logging.h"
#include "nigiri/routing/start_times.h"
#include "nigiri/routing/raptor/mc_raptor_state.h"
#include "nigiri/routing/for_each_meta.h"

namespace nigiri::routing {

bool mc_raptor::is_better(auto a, auto b) { return a < b; }

bool mc_raptor::is_better_or_eq(auto a, auto b) { return a <= b; }

mc_raptor::mc_raptor(timetable const& tt,
                     mc_raptor_state& state,
                     interval<unixtime_t> const search_interval,
                     location_match_mode const start_match_mode,
                     std::vector<offset> const& start)
    : tt_{tt},
      n_tt_days_{tt_.internal_interval_days().size().count()},
      state_{state},
      search_interval_{search_interval},
      n_locations_{tt_.n_locations()},
      n_routes_{tt.n_routes()},
      start_{start},
      start_match_mode_{start_match_mode} {
  state_.resize(n_locations_, n_routes_);
  state_.round_bags_.reset(pareto_set<mc_raptor_label>{});
};

mc_raptor_stats const& mc_raptor::get_stats() const { return stats_; }

day_idx_t mc_raptor::start_day_offset() const {
  return tt_.day_idx_mam(this->search_interval_.from_).first;
}

unsigned mc_raptor::end_k() { return kMaxTransfers + 1U; }

void mc_raptor::route() {
  std::vector<start> starts;

  get_starts(direction::kForward, tt_, nullptr, search_interval_, start_,
             start_match_mode_, true, starts, true);
  utl::equal_ranges_linear(
      starts,
      [](start const& a, start const& b) {
        return a.time_at_start_ == b.time_at_start_;
      },
      [&](auto&& from_it, auto&& to_it) {
        for (auto const& s : it_range{from_it, to_it}) {
          state_.round_bags_[0U][to_idx(s.stop_)].add(mc_raptor_label(
              {tt_, s.time_at_stop_}, 0_minutes, {tt_, from_it->time_at_start_}));
          state_.best_[to_idx(s.stop_)].add(mc_raptor_label(
              {tt_, s.time_at_stop_}, 0_minutes, {tt_, from_it->time_at_start_}));
          state_.station_mark_[to_idx(s.stop_)] = true;
        }
      });
  rounds();
  reconstruct();
  for (auto& r : state_.results_) {
    utl::erase_if(r, [&](journey const& j) {
      return !search_interval_.contains(j.start_time_);
    });
  }
}

void mc_raptor::rounds() {
  for (auto k = 1U; k != end_k(); ++k) {

    // Round k
    auto any_marked = false;
    for (auto l_idx = location_idx_t{0U};
         l_idx != static_cast<cista::base_t<location_idx_t>>(
                      state_.station_mark_.size());
         ++l_idx) {

      if (state_.station_mark_[to_idx(l_idx)]) {
        for (auto& l : state_.round_bags_[k - 1][to_idx(l_idx)]) {
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
    auto const stop_idx = static_cast<stop_idx_t>(i);
    auto const stp = stop{stop_sequence[stop_idx]};
    auto const l_idx = cista::to_idx(stp.location_idx());

    auto const transfer_time_offset =
        tt_.locations_.transfer_time_[location_idx_t{l_idx}];
    for (const auto& active_label : r_b) {
      if (active_label.transport_.t_idx_ == transport_idx_t::invalid()) {
        continue;
      }
      const auto& trip = active_label.transport_;

      const routing_time new_arr(
          trip.day_,
          tt_.event_mam(route_idx, trip.t_idx_, stop_idx, event_type::kArr)
              .as_duration());

      auto candidate_lbl = mc_raptor_label{new_arr, transfer_time_offset,
                            active_label.departure_, active_label.prev_};

      candidate_lbl.with_ = mc_raptor_label::transport_leg{
          active_label.entered_, trip, stp.location_idx()};
      candidate_lbl.transfer_ = mc_raptor_label::footpath_leg{
          transfer_time_offset, stp.location_idx()};

      if (!stp.out_allowed() ||
          (get<0>(candidate_lbl.arrival_).offset_- candidate_lbl.departure_.offset_) >
              kMaxTravelTime.count()) {
        continue;
      }

      for (const auto& el : state_.best_[l_idx]) {
        if (el.dominates(candidate_lbl)) {
          continue;
        }
      }

      if (std::get<0>(state_.round_bags_[k][cista::to_idx(l_idx)].add(
              std::forward<mc_raptor_label>(candidate_lbl)))) {
        state_.station_mark_[l_idx] = true;
        any_marked = true;
      }
    }

    if (i == stop_sequence.size() - 1) {
      return any_marked;
    }

    if (stp.in_allowed() && state_.prev_station_mark_[l_idx]) {
      const auto& rb = state_.round_bags_[k - 1][cista::to_idx(l_idx)];
      for (auto rb_it = rb.begin(); rb_it != rb.end(); ++rb_it) {

        auto const new_et = get_earliest_transport(*rb_it, route_idx, stop_idx);
        if (new_et.is_valid()) {
          r_b.add(mc_raptor_route_label{new_et, stp.location_idx(),rb_it->departure_, rb_it});
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
      for (const auto& rl : round_bag) {
        mc_raptor_label l_with_foot{get<0>(rl.arrival_) + fp.duration(), 0_minutes,
                                    rl.departure_, rl.prev_};

        l_with_foot.with_ = rl.with_;
        l_with_foot.transfer_ =
            mc_raptor_label::footpath_leg{fp.duration(), fp.target()};

        if ((get<0>(l_with_foot.arrival_).offset_ - l_with_foot.departure_.offset_) >
            kMaxTravelTime.count()) {
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
      if (std::get<0>(state_.round_bags_[k][to_idx(l_idx)].add(
              std::forward<mc_raptor_label>(l)))) {
        state_.station_mark_[to_idx(l_idx)] = true;
      }
    }
  }
}

transport mc_raptor::get_earliest_transport(const mc_raptor_label& current,
                                            route_idx_t const r,
                                            stop_idx_t const stop_idx) {

  auto time = get<1>(current.arrival_);

  if (time == kInvalidTime<direction::kForward>) {
    return {transport_idx_t::invalid(), day_idx_t::invalid()};
  }

  auto const [day_at_stop, mam_at_stop] = time.day_idx_mam();

  auto const n_days_to_iterate =
      std::min(kMaxTravelTime.count() / 1440 + 1,
               n_tt_days_ - static_cast<int>(day_at_stop.v_) + 1);

  auto const event_times =
      tt_.event_times_at_stop(r, stop_idx, event_type::kDep);

  auto const seek_first_day = [&, m_at_stop = mam_at_stop]() {
    return std::lower_bound(
        event_times.begin(), event_times.end(), m_at_stop,
        [&](auto&& a, auto&& b) { return is_better(a.as_duration(), b); });
  };

  for (auto i = day_idx_t::value_t{0U}; i != n_days_to_iterate; ++i) {
    auto const day = day_at_stop + i;
    auto const ev_time_range = it_range{
        i == 0U ? seek_first_day() : event_times.begin(), event_times.end()};
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

void mc_raptor::reconstruct() {
  for (auto loc = 0U; loc != n_locations_; ++loc) {

    const auto fastest_direct = get_fastest_direct(location_idx_t{loc});

    for (auto k = 0U; k != end_k(); ++k) {
      auto const& round_bag = state_.round_bags_[k][loc];
      if (round_bag.size() == 0) {
        continue;
      }
      for (auto journey_it = round_bag.begin(); journey_it != round_bag.end();
           ++journey_it) {

        const duration_t travel_time{std::abs((get<0>(journey_it->arrival_) - journey_it->departure_).count())};
        if (travel_time >= fastest_direct) {
          continue;
        }

        auto const [optimal, it, _] = state_.results_[loc].add(
            journey{.legs_ = {},
                    .start_time_ = journey_it->departure_.to_unixtime(tt_),
                    .dest_time_ = get<0>(journey_it->arrival_).to_unixtime(tt_),
                    .dest_ = location_idx_t{loc},
                    .transfers_ = static_cast<std::uint8_t>(k - 1)});

        if (!optimal) {
          continue;
        }

        auto current_label = journey_it;

        for (auto i = k; i > 0; i--) {

          const auto& curr_opt_transfer = current_label->transfer_;
          const auto& curr_opt_transport = current_label->with_;

          if (!curr_opt_transfer.has_value() ||
              !curr_opt_transport.has_value()) {
            throw utl::fail(
                "No Transfer or Transport leg given for the current label!\n");
          }

          const auto target_time = get<0>(current_label->arrival_);

          const auto& curr_transfer_leg = *curr_opt_transfer;
          const auto& curr_transport_leg = *curr_opt_transport;

          const auto& prev_label = current_label->prev_;

          if (curr_transport_leg.exit_ == curr_transfer_leg.target_ && i == k) {

            it->add(journey::leg{direction::kForward, curr_transfer_leg.target_,
                curr_transfer_leg.target_,
                target_time.to_unixtime(tt_), target_time.to_unixtime(tt_),
                 footpath{curr_transfer_leg.target_, 0_minutes}});

          } else {

            if (kVerifyReconstruction) {
              const auto& fps = tt_.locations_.footpaths_out_[curr_transport_leg.exit_];
              bool hit = false;
              for (const auto& fp : fps) {
                if (fp.target() == curr_transfer_leg.target_ && fp.duration() == curr_transfer_leg.duration_) {
                  hit = true;
                  break;
                }
              }
              if (curr_transport_leg.exit_ == curr_transfer_leg.target_ &&
                  tt_.locations_.transfer_time_[curr_transfer_leg.target_] == curr_transfer_leg.duration_) {
                hit = true;
              }
              if (!hit) {
                nigiri::log(log_lvl::error,
                            "mc_raptor.reconstruction",
                            "Footpath {} -- {} --> {}, used in routing does not exist!",
                            location{tt_, curr_transport_leg.exit_},
                            curr_transfer_leg.duration_,
                            location{tt_, curr_transfer_leg.target_});
              }
            }

            it->add(journey::leg{direction::kForward, curr_transport_leg.exit_,
                                 curr_transfer_leg.target_,
                                 (target_time +
                                    (curr_transport_leg.exit_ == curr_transfer_leg.target_ ? 0 : -1)
                                    * curr_transfer_leg.duration_).to_unixtime(tt_),
                                 (target_time +
                                  (curr_transport_leg.exit_ == curr_transfer_leg.target_ ? 1 : 0)
                                      * curr_transfer_leg.duration_).to_unixtime(tt_),
                                 footpath{curr_transfer_leg.target_,
                  curr_transfer_leg.duration_}});
          }

          nigiri::rt::run r;
          r.t_ = current_label->with_.value().via_;
          r.stop_range_ = interval<stop_idx_t>{0, 0};
          const auto from_to = find_enter_exit(
              curr_transport_leg.via_, curr_transport_leg.enter_,
              get<1>(prev_label->arrival_), curr_transport_leg.exit_);

          if (kVerifyReconstruction) {
            const auto& stop_sequence = tt_.route_location_seq_[tt_.transport_route_[curr_transport_leg.via_.t_idx_]];
            if (from_to.from_ >= from_to.to_ || from_to.from_ == stop_sequence.size() || from_to.to_ == stop_sequence.size()) {
              nigiri::log(log_lvl::error,
                          "mc_raptor.reconstruction",
                          "Invalid transport ride used while routing. Not possible to enter transport {} at {} and exit at {}",
                          curr_transport_leg.via_,
                          location{tt_, curr_transport_leg.enter_},
                          location{tt_, curr_transport_leg.exit_});
            }
          }

          const auto trans_dep_time =
              tt_.event_time(r.t_, from_to.from_, event_type::kDep);

          const auto trans_arr_time =
              tt_.event_time(r.t_, from_to.to_, event_type::kArr);

          if (kVerifyReconstruction) {
            if (get<1>(prev_label->arrival_).to_unixtime(tt_) > trans_dep_time) {
              nigiri::log(log_lvl::error,
                          "mc_raptor.reconstruction",
                          "Not possible to enter transport {} (Route {}) at location {}, when arriving not earlier than {}",
                          curr_transport_leg.via_,
                          tt_.transport_route_[curr_transport_leg.via_.t_idx_],
                          curr_transport_leg.enter_,
                          get<1>(prev_label->arrival_).to_unixtime(tt_));
            }
          }

          it->add(journey::leg{
              direction::kForward, curr_transport_leg.enter_,
              curr_transport_leg.exit_, trans_dep_time,
              trans_arr_time,
              journey::run_enter_exit{r, from_to.from_, from_to.to_}});

          if (i == 1) {
            break;
          }
          current_label = prev_label;
        }
        const auto target_after_init_transfer =
            k == 0 ? location_idx_t{loc} : current_label->with_->enter_;

        if (k > 0) {
          current_label = current_label->prev_;
        }
        if (!is_journey_start(target_after_init_transfer)) {
          auto init_fp = find_start_footpath(target_after_init_transfer,
                                             get<1>(current_label->arrival_),
                                             current_label->departure_);
          if (!init_fp.has_value()) {
            throw utl::fail("No initial footpath found!\n");
          }

          it->add(std::move(*init_fp));
        }
        std::reverse(it->legs_.begin(), it->legs_.end());
      }
    }
  }
}

interval<stop_idx_t> mc_raptor::find_enter_exit(transport const via,
                                                location_idx_t const enter,
                                                routing_time const enter_after,
                                                location_idx_t const exit) {

  auto const stop_sequence = tt_.route_location_seq_[tt_.transport_route_[via.t_idx_]];
  interval<stop_idx_t> enter_exit{
      static_cast<unsigned short>(stop_sequence.size()),
      static_cast<unsigned short>(stop_sequence.size())};

  bool entered = false;
  for (auto i = 0U; i != stop_sequence.size(); ++i) {
    auto const stop_idx = static_cast<stop_idx_t>(i);
    auto const stp = stop{stop_sequence[stop_idx]};

    if (!entered &&
        stp.location_idx() == enter && stp.in_allowed() &&
        enter_after.to_unixtime(tt_) <= tt_.event_time(via, stop_idx, event_type::kDep)) {
      enter_exit.from_ = stop_idx;
      entered = true;
    }

    if (stp.location_idx() == exit && entered && stp.out_allowed()) {
      enter_exit.to_ = stop_idx;
      break;
    }
  }
  return enter_exit;
}

bool mc_raptor::is_journey_start(location_idx_t const l) {
  return utl::any_of(start_, [&](offset const& o) {
    return matches(tt_, start_match_mode_, o.target(), l);
  });
}

std::optional<journey::leg> mc_raptor::find_start_footpath(
    location_idx_t const leg_start_location,
    routing_time const leg_start_time,
    routing_time const journey_start_time) {

  auto const start_matches = [&](routing_time const a, routing_time const b) {
    return a == b;
  };

  if (is_journey_start(leg_start_location) &&
      is_better_or_eq(journey_start_time, leg_start_time)) {
    return std::nullopt;
  }

  auto const& footpaths = tt_.locations_.footpaths_in_[leg_start_location];
  for (auto const& fp : footpaths) {
    if (is_journey_start(fp.target()) &&
        leg_start_time != routing_time::max() &&
        start_matches(journey_start_time + fp.duration(), leg_start_time)) {
      return journey::leg{direction::kForward,
                          fp.target(),
                          leg_start_location,
                          journey_start_time.to_unixtime(tt_),
                          leg_start_time.to_unixtime(tt_),
                          fp};
    }
  }

  return std::nullopt;
}

duration_t mc_raptor::get_fastest_start_dest_overlap(location_idx_t dest) {
  auto min = duration_t{std::numeric_limits<duration_t::rep>::max()};
  for (auto const& s : start_) {
    for_each_meta(tt_, start_match_mode_, s.target_,
                  [&](location_idx_t const start) {
                      if (start == dest) {
                        min = std::min(min, s.duration_);
                      }
                  });
  }
  return min;
}

duration_t mc_raptor::get_fastest_direct_with_foot(location_idx_t dest) {
  auto min = duration_t{std::numeric_limits<duration_t::rep>::max()};
  for (auto const& start : start_) {
    auto const& footpaths = tt_.locations_.footpaths_out_;
    for (auto const& fp : footpaths[start.target_]) {
        if (dest == fp.target()) {
          min = std::min(min, start.duration_ + fp.duration());
        }
    }
  }
  return min;
}

duration_t mc_raptor::get_fastest_direct(location_idx_t const dest) {
  return std::min(get_fastest_direct_with_foot(dest),
                  get_fastest_start_dest_overlap(dest));
}


}