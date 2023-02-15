#include "nigiri/routing/bmc_raptor_reconstructor.h"

#include <set>

#include "nigiri/routing/limits.h"
#include "nigiri/routing/query.h"
#include "nigiri/types.h"
#include "nigiri/routing/bmc_raptor_search_state.h"
#include "nigiri/routing/for_each_meta.h"
#include "utl/helpers/algorithm.h"
#include "nigiri/special_stations.h"


namespace nigiri::routing {

bmc_raptor_reconstructor::bmc_raptor_reconstructor(const timetable& tt,
                                                   const query& q,
                                                   bmc_raptor_search_state& state,
                                                   interval<unixtime_t> search_interval)
      : tt_{tt},
        q_{q},
        state_{state},
        n_rounds_{std::min(kMaxTransfers, q_.max_transfers_) + 1U},
        search_interval_(search_interval),
        round_times_(make_flat_matrix<uncompressed_round_times_t>(n_rounds_, tt_.n_locations())),
        round_time_iters(make_flat_matrix<uncompressed_round_time_iterator>(n_rounds_, tt_.n_locations())) {}

void bmc_raptor_reconstructor::uncompress_round_bags() {
  const auto first_day_offset =
      tt_.day_idx_mam(search_interval_.from_).first;

  for (auto r = 0U; r < n_rounds_; ++r) {
    for (auto l = 0U; l < tt_.n_locations(); ++l) {
      const auto& bag = state_.round_bags_[r][l];
      auto& ert = round_times_[r][l];

      const auto& uncompressed_labels = bag.uncompress();
      ert.reserve(uncompressed_labels.size());
      for (const auto& arr_dep_l : uncompressed_labels) {
        ert.push_back(dep_arr_t{
            routing_time{first_day_offset + arr_dep_l.departure_.count() / 1440, arr_dep_l.departure_ % 1440},
            routing_time{first_day_offset + arr_dep_l.arrival_.count() / 1440, arr_dep_l.arrival_ % 1440}
        });
      }
      std::sort(ert.begin(), ert.end(), departure_arrival_comparator());
      round_time_iters[r][l] = ert.begin();
    }
  }
}

void bmc_raptor_reconstructor::get_departure_events() {
  for (auto l = 0U; l < tt_.n_locations(); ++l) {
    const auto& dep_arr_times = round_times_[0][l];
    for (const auto& dat : dep_arr_times) {
      departure_events_.insert(dat.first);
    }
  }
}

routing_time bmc_raptor_reconstructor::get_routing_time(const unsigned long k,
                                                        const location_idx_t loc,
                                                        const routing_time departure) {
  auto& iter = round_time_iters[k][to_idx(loc)];
  const auto& dep_arr_times = round_times_[k][to_idx(loc)];

  routing_time routing_time = kInvalidTime<direction::kForward>;
  while (iter != dep_arr_times.end() && iter->first > departure) {
    iter++;
  }
  if (iter != dep_arr_times.end() && iter->first == departure) {
    routing_time = iter->second;
  }
  return routing_time;
}

std::optional<journey::leg> bmc_raptor_reconstructor::find_start_footpath(journey const& j,
                                                                          const routing_time departure) {
  constexpr auto const kFwd = true;

  auto const is_better_or_eq = [](auto a, auto b) {
    return kFwd ? a <= b : a >= b;
  };

  auto const is_journey_start = [&](location_idx_t const candidate_l) {
    return utl::any_of(q_.start_, [&](offset const& o) {
      return matches(tt_, q_.start_match_mode_, o.target_, candidate_l);
    });
  };

  auto const leg_start_location =
      kFwd ? j.legs_.back().from_ : j.legs_.back().to_;
  auto const leg_start_time =
      kFwd ? j.legs_.back().dep_time_ : j.legs_.back().arr_time_;

  if (q_.start_match_mode_ != location_match_mode::kIntermodal &&
      is_journey_start(leg_start_location) &&
      is_better_or_eq(j.start_time_, leg_start_time)) {
    return std::nullopt;
  }

  auto const& footpaths =
      kFwd ? tt_.locations_.footpaths_in_[leg_start_location]
           : tt_.locations_.footpaths_out_[leg_start_location];
  auto const j_start_time = routing_time{tt_, j.start_time_};
  auto const fp_target_time = get_routing_time(0, leg_start_location, departure);

  if (q_.start_match_mode_ == location_match_mode::kIntermodal) {
    for (auto const& o : q_.start_) {
      if (matches(tt_, q_.start_match_mode_, o.target_, leg_start_location) &&
          is_better_or_eq(j.start_time_,
                          leg_start_time - (kFwd ? 1 : -1) * o.duration_)) {
        return journey::leg{direction::kForward,
                            get_special_station(special_station::kStart),
                            leg_start_location,
                            j.start_time_,
                            j.start_time_ + (kFwd ? 1 : -1) * o.duration_,
                            o};
      }

      for (auto const& fp : footpaths) {
        if (matches(tt_, q_.start_match_mode_, o.target_, fp.target_) &&
            is_better_or_eq(
                j.start_time_,
                leg_start_time -
                    (kFwd ? 1 : -1) * (o.duration_ + fp.duration_))) {
          return journey::leg{direction::kForward,
                              get_special_station(special_station::kStart),
                              leg_start_location,
                              j.start_time_,
                              j.start_time_ + (kFwd ? 1 : -1) * o.duration_,
                              o};
        }
      }
    }
  } else {
    for (auto const& fp : footpaths) {
      if (is_journey_start(fp.target_) &&
          fp_target_time != kInvalidTime<direction::kForward> &&
          std::abs((j_start_time - fp_target_time).count()) ==
              fp.duration_.count()) {
        return journey::leg{direction::kForward,
                            fp.target_,
                            leg_start_location,
                            j.start_time_,
                            fp_target_time.to_unixtime(tt_),
                            fp};
      }
    }
  }

  throw utl::fail("no valid journey start found");
}

void bmc_raptor_reconstructor::reconstruct_journey(journey& j,
                                                   const routing_time departure_time) {

  constexpr auto const kFwd = true;
  auto const is_better_or_eq = [](auto a, auto b) {
    return kFwd ? a <= b : a >= b;
  };

  auto const find_entry_in_prev_round =
      [&](unsigned const k, transport const& t, route_idx_t const r,
          std::size_t const from_stop_idx,
          routing_time const time) -> std::optional<journey::leg> {
    auto const& stop_seq = tt_.route_location_seq_[r];

    auto const n_stops =
        kFwd ? from_stop_idx + 1 : stop_seq.size() - from_stop_idx;
    for (auto i = 1U; i != n_stops; ++i) {
      auto const stop_idx =
          static_cast<unsigned>(kFwd ? from_stop_idx - i : from_stop_idx + i);
      auto const stop = timetable::stop{stop_seq[stop_idx]};
      auto const l = stop.location_idx();

      if ((kFwd && !stop.in_allowed()) || (!kFwd && !stop.out_allowed())) {
        continue;
      }

      auto const event_time = routing_time{
          t.day_, tt_.event_mam(t.t_idx_, stop_idx,
                               kFwd ? event_type::kDep : event_type::kArr)};

      const auto prev_routing_time = get_routing_time(k-1, l, departure_time);
      if (is_better_or_eq(prev_routing_time, event_time)) {
        return journey::leg{
            direction::kForward,
            timetable::stop{stop_seq[stop_idx]}.location_idx(),
            timetable::stop{stop_seq[from_stop_idx]}.location_idx(),
            event_time.to_unixtime(tt_),
            time.to_unixtime(tt_),
            journey::transport_enter_exit{
                t, stop_idx, static_cast<unsigned>(from_stop_idx)}};
      }

      // special case: first stop with meta stations
      if (k == 1 && q_.start_match_mode_ == location_match_mode::kEquivalent) {
        for (auto const& eq : tt_.locations_.equivalences_[l]) {
          const auto meta_prev_routing_time = get_routing_time(k-1, eq, departure_time);
          if (is_better_or_eq(meta_prev_routing_time,
                              event_time)) {
            return journey::leg{
                direction::kForward,
                timetable::stop{stop_seq[stop_idx]}.location_idx(),
                timetable::stop{stop_seq[from_stop_idx]}.location_idx(),
                event_time.to_unixtime(tt_),
                time.to_unixtime(tt_),
                journey::transport_enter_exit{
                    t, stop_idx, static_cast<unsigned>(from_stop_idx)}};
          }
        }
      }
    }

    return std::nullopt;
  };

  auto const get_route_transport =
      [&](unsigned const k, routing_time const time, route_idx_t const r,
          std::size_t const stop_idx) -> std::optional<journey::leg> {
    for (auto const t : tt_.route_transport_ranges_[r]) {
      auto const event_mam =
          tt_.event_mam(t, stop_idx, kFwd ? event_type::kArr : event_type::kDep);
      if (event_mam.count() % 1440 != time.mam().count()) {
        continue;
      }

      auto const day_offset =
          static_cast<cista::base_t<day_idx_t>>(event_mam.count() / 1440);
      auto const day = time.day() - day_offset;
      if (!tt_.bitfields_[tt_.transport_traffic_days_[t]].test(to_idx(day))) {
        continue;
      }

      auto leg =
          find_entry_in_prev_round(k, transport{t, day}, r, stop_idx, time);
      if (leg.has_value()) {
        return leg;
      }
    }
    return std::nullopt;
  };

  auto const get_transport =
      [&](unsigned const k, location_idx_t const l,
          routing_time const time) -> std::optional<journey::leg> {
    for (auto const& r : tt_.location_routes_[l]) {
      auto const location_seq = tt_.route_location_seq_[r];
      for (auto const [i, stop] : utl::enumerate(location_seq)) {
        auto const s = timetable::stop{stop};
        if (s.location_idx() != l ||  //
            (kFwd && (i == 0U || !s.out_allowed())) ||
            (!kFwd && (i == location_seq.size() - 1 || !s.in_allowed()))) {
          continue;
        }

        auto leg = get_route_transport(k, time, r, i);
        if (leg.has_value()) {
          return leg;
        }
      }
    }
    return std::nullopt;
  };

  auto const check_fp = [&](unsigned const k, location_idx_t const l,
                            routing_time const curr_time, footpath const fp)
      -> std::optional<std::pair<journey::leg, journey::leg>> {
    auto const fp_start = curr_time - (kFwd ? fp.duration_ : -fp.duration_);

    auto const transport_leg = get_transport(k, fp.target_, fp_start);

    if (transport_leg.has_value()) {
      auto const fp_leg = journey::leg{direction::kForward,
                                       fp.target_,
                                       l,
                                       fp_start.to_unixtime(tt_),
                                       curr_time.to_unixtime(tt_),
                                       fp};
      return std::pair{fp_leg, *transport_leg};
    }
    return std::nullopt;
  };

  auto const get_legs =
      [&](unsigned const k,
          location_idx_t const l) -> std::pair<journey::leg, journey::leg> {
    auto const curr_time = get_routing_time(k, l, departure_time);
    if (q_.dest_match_mode_ == location_match_mode::kIntermodal &&
        k == j.transfers_ + 1U) {
      for (auto const& dest_offset : q_.destinations_[0]) {
        std::optional<std::pair<journey::leg, journey::leg>> ret;
        for_each_meta(
            tt_, location_match_mode::kIntermodal, dest_offset.target_,
            [&](location_idx_t const eq) {
              auto const transfer_time = tt_.locations_.transfer_time_[eq];
              auto intermodal_dest = check_fp(
                  k, l, curr_time, {eq, dest_offset.duration_ + transfer_time});
              if (intermodal_dest.has_value()) {
                (kFwd ? intermodal_dest->first.dep_time_ += transfer_time
                      : intermodal_dest->first.arr_time_ -= transfer_time);
                intermodal_dest->first.uses_ =
                    offset{eq, dest_offset.duration_ - transfer_time,
                           dest_offset.type_};
                ret = std::move(intermodal_dest);
              }

              for (auto const& fp : kFwd ? tt_.locations_.footpaths_in_[eq]
                                         : tt_.locations_.footpaths_out_[eq]) {
                auto fp_intermodal_dest = check_fp(
                    k, l, curr_time,
                    {fp.target_, dest_offset.duration_ + fp.duration_});
                if (fp_intermodal_dest.has_value()) {
                  fp_intermodal_dest->first.uses_ = offset{
                      eq, fp.duration_ - transfer_time, dest_offset.type_};
                  ret = std::move(fp_intermodal_dest);
                }
              }
            });
        if (ret.has_value()) {
          return std::move(*ret);
        }
      }

      throw utl::fail(
          "intermodal destination reconstruction failed at k={}, t={}, "
          "stop=(name={}, id={}), time={}",
          k, j.transfers_, tt_.locations_.names_[l].view(),
          tt_.locations_.ids_[l].view(), curr_time);
    }

    auto transfer_at_same_stop =
        check_fp(k, l, curr_time,
                 footpath{l, (k == j.transfers_ + 1U)
                                 ? 0_minutes
                                 : tt_.locations_.transfer_time_[l]});
    if (transfer_at_same_stop.has_value()) {
      return std::move(*transfer_at_same_stop);
    }

    auto const fps =
        kFwd ? tt_.locations_.footpaths_in_[l] : tt_.locations_.footpaths_out_[l];
    for (auto const& fp : fps) {
      auto fp_legs = check_fp(k, l, curr_time, fp);
      if (fp_legs.has_value()) {
        return std::move(*fp_legs);
      }
    }

    throw utl::fail(
        "reconstruction failed at k={}, t={}, stop=(name={}, id={}), time={}",
        k, j.transfers_, tt_.locations_.names_[l].view(),
        tt_.locations_.ids_[l].view(), curr_time);
  };

  auto l = j.dest_;
  for (auto i = 0U; i <= j.transfers_; ++i) {
    auto const k = j.transfers_ + 1 - i;
    auto [fp_leg, transport_leg] = get_legs(k, l);
    l = kFwd ? transport_leg.from_ : transport_leg.to_;
    j.add(std::move(fp_leg));
    j.add(std::move(transport_leg));
  }

  auto init_fp = find_start_footpath(j, departure_time);
  if (init_fp.has_value()) {
    j.add(std::move(*init_fp));
  }

  if constexpr (kFwd) {
    std::reverse(begin(j.legs_), end(j.legs_));
  }
}

void bmc_raptor_reconstructor::reconstruct_for_destination(std::size_t dest_idx,
                                                           location_idx_t dest,
                                                           const routing_time departure_time) {
  for (auto k = 1U; k != n_rounds_; ++k) {
    auto& dest_iter = round_time_iters[k][to_idx(dest)];
    const auto& dest_dep_arr_events = round_times_[k][to_idx(dest)];

    routing_time dest_time = kInvalidTime<direction::kForward>;
    while (dest_iter != dest_dep_arr_events.end() && dest_iter->first > departure_time) {
      dest_iter++;
    }
    if (dest_iter != dest_dep_arr_events.end() && dest_iter->first == departure_time) {
      dest_time = dest_iter->second;
    }

    if (dest_time == kInvalidTime<direction::kForward>) {
      continue;
    }

    auto const [optimal, it] = state_.results_[dest_idx].add(journey{
        .legs_ = {},
        .start_time_ = departure_time.to_unixtime(tt_),
        .dest_time_ = dest_time.to_unixtime(tt_),
        .dest_ = dest,
        .transfers_ = static_cast<std::uint8_t>(k - 1)});
    if (optimal) {
      auto const outside_interval =
          !search_interval_.contains(it->start_time_);
      if (!outside_interval) {
        try {
          reconstruct_journey(*it, departure_time);
        } catch (std::exception const& e) {
          state_.results_[dest_idx].erase(it);
          log(log_lvl::error, "routing", "reconstruction failed: {}", e.what());
        }
      }
    }
  }
}

void bmc_raptor_reconstructor::reconstruct() {
  uncompress_round_bags();
  get_departure_events();

  for (const auto& dep_time : departure_events_) {

    for (auto const [i, t] : utl::enumerate(q_.destinations_)) {
      for (auto const dest : state_.destinations_[i]) {
        reconstruct_for_destination(i, dest, dep_time);
      }
    }
  }

}

}
