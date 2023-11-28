#include <vector>
#include <sstream>
#include <filesystem>

#include "date/date.h"

#include "utl/progress_tracker.h"
#include "utl/helpers/algorithm.h"
#include "utl/verify.h"

#include "nigiri/loader/gtfs/loader.h"
#include "nigiri/loader/hrd/loader.h"
#include "nigiri/loader/init_finish.h"
#include "nigiri/logging.h"
#include "nigiri/routing/dijkstra.h"
#include "nigiri/routing/raptor/raptor_search.h"
#include "nigiri/timetable.h"

#include "cista/targets/file.h"
#include <omp.h>

using namespace date;
using namespace nigiri;
using namespace nigiri::loader;
using namespace nigiri::routing;

int main(int ac, char** av) {

  if (ac != 2) {
    fmt::print("usage: {} [TIMETABLE_PATH]\n",
               ac == 0U ? "nigiri-server" : av[0]);
    return 1;
  }

  utl::activate_progress_tracker("import");

  timetable tt;
  if (std::filesystem::is_regular_file(av[1])) {
    cista::file f{av[1], "r"};
    tt = *timetable::read(f.content());
  } else {
    auto loaders = std::vector<std::unique_ptr<loader_interface>>{};
    loaders.emplace_back(std::make_unique<gtfs::gtfs_loader>());
    loaders.emplace_back(std::make_unique<hrd::hrd_5_00_8_loader>());
    loaders.emplace_back(std::make_unique<hrd::hrd_5_20_26_loader>());
    loaders.emplace_back(std::make_unique<hrd::hrd_5_20_39_loader>());
    loaders.emplace_back(std::make_unique<hrd::hrd_5_20_avv_loader>());

    auto const src = source_idx_t{0U};
    auto const tt_path = std::filesystem::path{av[1]};
    auto const d = make_dir(tt_path);

    auto const c =
        utl::find_if(loaders, [&](auto&& l) { return l->applicable(*d); });
    utl::verify(c != end(loaders), "no loader applicable to {}", tt_path);
    log(log_lvl::info, "main", "loading nigiri timetable with configuration {}",
        (*c)->name());

    tt.date_range_ = {date::sys_days{January / 11 / 2021},
                      date::sys_days{January / 17 / 2021}};
    register_special_stations(tt);
    (*c)->load({}, src, *d, tt);
    finalize(tt);
  }

  const auto routing_result = raptor_search(tt, nullptr, location{tt, location_idx_t{4000U}}.id_, location{tt, location_idx_t{2500}}.id_,
                                            "2021-1-14 13:00 UTC", "2021-1-14 17:00 UTC", direction::kForward, false);

  fmt::print("{}\n", routing_result.journeys_->size());
}