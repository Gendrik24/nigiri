#include "nigiri/timetable.h"

#include "cista/mmap.h"
#include "cista/serialization.h"

#include "utl/overloaded.h"

#include "nigiri/common/day_list.h"
#include "nigiri/rt/frun.h"
#include "nigiri/logging.h"
#include "nigiri/routing/raptor/raptor_search.h"

#include <sstream>

#include <omp.h>

namespace nigiri {

constexpr auto const kMode =
    cista::mode::WITH_INTEGRITY | cista::mode::WITH_STATIC_VERSION;

std::string reverse(std::string s) {
  std::reverse(s.begin(), s.end());
  return s;
}

std::string format(double d, unsigned short precision) {
  std::stringstream ss;
  ss.setf(std::ios::fixed);
  ss.precision(precision);
  ss << d;
  return ss.str();
}

inline bool contains(interval<unixtime_t> i1, interval<unixtime_t> i2) {
  return i1.from_ <= i2.from_ && i1.to_ >= i2.to_;
}

void timetable::locations::resolve_timezones() {
  for (auto& tz : timezones_) {
    if (holds_alternative<pair<string, void const*>>(tz)) {
      auto& [name, ptr] = tz.as<pair<string, void const*>>();
      ptr = date::locate_zone(name);
    }
  }
}

std::ostream& operator<<(std::ostream& out, timetable const& tt) {
  for (auto const [id, idx] : tt.trip_id_to_idx_) {
    auto const str = tt.trip_id_strings_[id].view();
    out << str << ":\n";
    for (auto const& t : tt.trip_transport_ranges_.at(idx)) {
      out << "  " << t.first << ": " << t.second << " active="
          << day_list{tt.bitfields_[tt.transport_traffic_days_[t.first]],
                      tt.internal_interval_days().from_}
          << "\n";
    }
  }

  auto const internal = tt.internal_interval_days();
  auto const num_days =
      static_cast<size_t>((internal.to_ - internal.from_ + 1_days) / 1_days);
  for (auto i = 0U; i != tt.transport_traffic_days_.size(); ++i) {
    auto const transport_idx = transport_idx_t{i};
    auto const num_stops =
        tt.route_location_seq_[tt.transport_route_[transport_idx]].size();
    auto const traffic_days =
        tt.bitfields_.at(tt.transport_traffic_days_.at(transport_idx));
    out << "TRANSPORT=" << transport_idx << ", TRAFFIC_DAYS="
        << reverse(traffic_days.to_string().substr(kMaxDays - num_days))
        << "\n";
    for (auto d = internal.from_; d != internal.to_;
         d += std::chrono::days{1}) {
      auto const day_idx = day_idx_t{
          static_cast<day_idx_t::value_t>((d - internal.from_) / 1_days)};
      if (traffic_days.test(to_idx(day_idx))) {
        date::to_stream(out, "%F", d);
        out << " (day_idx=" << day_idx << ")\n";
        out << rt::frun{
            tt,
            nullptr,
            {.t_ = transport{transport_idx, day_idx},
             .stop_range_ = {0U, static_cast<stop_idx_t>(num_stops)}}};
        out << "\n";
      }
    }
    out << "---\n\n";
  }
  return out;
}

cista::wrapped<timetable> timetable::read(cista::memory_holder&& mem) {
  return std::visit(
      utl::overloaded{
          [&](cista::buf<cista::mmap>& b) {
            auto const ptr =
                reinterpret_cast<timetable*>(&b[cista::data_start(kMode)]);
            return cista::wrapped{std::move(mem), ptr};
          },
          [&](cista::buffer& b) {
            auto const ptr = cista::deserialize<timetable, kMode>(b);
            return cista::wrapped{std::move(mem), ptr};
          },
          [&](cista::byte_buf& b) {
            auto const ptr = cista::deserialize<timetable, kMode>(b);
            return cista::wrapped{std::move(mem), ptr};
          }},
      mem);
}

void timetable::write(std::filesystem::path const& p) const {
  auto mmap = cista::mmap{p.string().c_str(), cista::mmap::protection::WRITE};
  auto writer = cista::buf<cista::mmap>(std::move(mmap));

  {
    auto const timer = scoped_timer{"timetable.write"};
    cista::serialize<kMode>(writer, *this);
  }
}

void timetable::write(cista::memory_holder& mem) const {
  std::visit(utl::overloaded{[&](cista::buf<cista::mmap>& writer) {
                               cista::serialize<kMode>(writer, *this);
                             },
                             [&](cista::buffer&) {
                               throw std::runtime_error{"not supported"};
                             },
                             [&](cista::byte_buf& b) {
                               auto writer = cista::buf{std::move(b)};
                               cista::serialize<kMode>(writer, *this);
                               b = std::move(writer.buf_);
                             }},
             mem);
}

void timetable::add_reach_store_for(const interval<nigiri::unixtime_t> time_range) {
  if (! contains(internal_interval(), time_range)) {
    return;
  }

  reach_store rs;
  rs.valid_range_ = time_range;
  init_reach_store(rs);

  #if defined(_OPENMP)
  std::size_t loc_start;
  std::size_t n_locations_finished = 9U;

  #pragma omp parallel for default(none) private(loc_start) shared(rs, n_locations_finished)
  for (loc_start = 9U; loc_start < n_locations(); ++loc_start) {

    const auto& results = nigiri::routing::mc_raptor_search(*this,
                                                            location{*this, location_idx_t{loc_start}}.id_,
                                                            rs.valid_range_);

    #pragma omp critical
    {
      for (std::size_t loc_tgt = 0U; loc_tgt < n_locations(); ++loc_tgt) {
        compute_reach_and_update(rs, results[loc_tgt]);
      }
      n_locations_finished++;
      nigiri::log(nigiri::log_lvl::info,
            "reach_store",
            "Progress {} %",
                  format((double) n_locations_finished / (double) n_locations() * 100, 2U));
    }
  }

  #else

  for (std::size_t loc_start = 9U; loc_start < n_locations(); ++loc_start) {
    nigiri::log(nigiri::log_lvl::info,
                "reach_store",
                "Starting to calculate reach values for journeys starting from location {}/{}",
                loc_start, n_locations());
    const auto& results = nigiri::routing::mc_raptor_search(*this,
                                                            location{*this, location_idx_t{loc_start}}.id_,
                                                            rs.valid_range_);


    for (std::size_t loc_tgt = 0U; loc_tgt < n_locations(); ++loc_tgt) {
      compute_reach_and_update(rs, results[loc_tgt]);
    }
  }
  #endif

  reach_stores_.push_back(rs);
}

void timetable::init_reach_store(reach_store& rs) {

    rs.location_reach_.resize(n_locations(), 0U);

    rs.route_location_reach_.resize(0U);

    rs.route_reach_value_ranges_.resize(0U);
    rs.reach_values_.resize(0U);

    for (auto r = route_idx_t{0}; r < n_routes(); r++) {
      const auto n_stops = route_location_seq_[r].size();
      std::size_t l = n_stops;
      std::vector<reach_t> v;
      v.resize(l);
      std::fill(v.begin(), v.end(), 0U);
      rs.route_location_reach_.emplace_back(v);

      const auto n_trips = route_transport_ranges_[r].size();
      std::vector<reach_t> f;
      f.resize(to_idx(n_trips) * n_stops);
      std::fill(f.begin(), f.end(), 0U);
      rs.route_reach_value_ranges_.emplace_back(rs.reach_values_.size(), rs.reach_values_.size() + f.size());
      rs.reach_values_.insert(rs.reach_values_.end(), f.begin(), f.end());
    }
}

reach_t timetable::get_location_reach(reach_store const& rs, location_idx_t loc) const {
    return rs.location_reach_[to_idx(loc)];
}

reach_t timetable::get_route_location_reach(reach_store const& rs,
                                            location_idx_t loc,
                                            route_idx_t r) const {
    return rs.route_location_reach_[r][to_idx(loc)];
}


reach_t timetable::get_trip_location_reach(reach_store const& rs,
                                           stop_idx_t s,
                                           transport_idx_t t) const {
    const auto r = transport_route_[t];
    const auto& range = rs.route_reach_value_ranges_[r];
    const auto& trip_range = route_transport_ranges_[r];
    const auto t_offset = t - route_transport_ranges_[r].from_;

    return rs.reach_values_[range.from_ + s * to_idx(trip_range.size()) + to_idx(t_offset)];
}


void timetable::attempt_update_location_reach(reach_store& rs,
                                              location_idx_t loc,
                                              reach_t reach) const {
    rs.location_reach_[to_idx(loc)] = std::max(reach, get_location_reach(rs, loc));
}


void timetable::attempt_update_route_location_reach(reach_store& rs,
                                                    location_idx_t loc,
                                                    route_idx_t r,
                                                    reach_t reach) const {
    rs.route_location_reach_[r][to_idx(loc)] = std::max(reach, get_route_location_reach(rs, loc, r));
}

void timetable::attempt_update_trip_location_reach(reach_store& rs,
                                                   stop_idx_t s,
                                                   transport_idx_t t,
                                                   reach_t reach) {
    const auto r = transport_route_[t];
    const auto& range = rs.route_reach_value_ranges_[r];
    const auto& trip_range = route_transport_ranges_[r];
    const auto r_idx = range.from_ + s * to_idx(trip_range.size()) + to_idx(t - route_transport_ranges_[r].from_);
    rs.reach_values_[r_idx] = std::max(reach, rs.reach_values_[r_idx]);
}

void timetable::compute_reach_and_update(reach_store& rs,
                                         const nigiri::routing::journey& j) {
    const std::uint8_t n_transports = j.transfers_ + 1U;
    auto n_transports_left = n_transports;
    for (auto const& leg : j.legs_) {
      std::uint8_t n_transports_right = n_transports - n_transports_left;
      reach_t reach = 0U;

      reach = std::min(n_transports_left, n_transports_right);
      attempt_update_location_reach(rs, leg.to_, reach);

      if (holds_alternative<nigiri::routing::journey::run_enter_exit>(leg.uses_)) {
      const auto& run = std::get<nigiri::routing::journey::run_enter_exit>(leg.uses_);
      const auto t_idx = run.r_.t_.t_idx_;

      attempt_update_route_location_reach(rs, leg.to_, transport_route_[t_idx], reach);
      attempt_update_trip_location_reach(rs, run.stop_range_.to_ - 1U, t_idx, reach);

      n_transports_left--;
      n_transports_right++;

      reach = std::min(n_transports_left, n_transports_right);

      attempt_update_location_reach(rs, leg.from_, reach);
      }
    }
}

void timetable::compute_reach_and_update(reach_store& rs,
                                         const pareto_set<nigiri::routing::journey>& journeys) {
    for (const auto& j : journeys) {
      compute_reach_and_update(rs, j);
    }
}

bool timetable::reach_store::valid_for(interval<unixtime_t> const inter) const {
  return valid_range_.from_ <= inter.from_ && inter.to_ <= valid_range_.to_;
}

}  // namespace nigiri
