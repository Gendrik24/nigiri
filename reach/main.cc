#include <fstream>
#include <filesystem>

#include "utl/progress_tracker.h"
#include "utl/helpers/algorithm.h"
#include "utl/verify.h"

#include "nigiri/loader/gtfs/loader.h"
#include "nigiri/loader/hrd/loader.h"
#include "nigiri/loader/init_finish.h"
#include "nigiri/logging.h"
#include "nigiri/routing/raptor/raptor_search.h"
#include "nigiri/timetable.h"
#include "nigiri/common/parse_time.h"

using namespace date;
using namespace nigiri;
using namespace nigiri::loader;
using namespace nigiri::routing;


int main(int argc, char** argv) {

  if (argc != 5) {
    fmt::print("usage: {} [TIMETABLE_PATH] [START_TIME] [END_TIME] [TIMETABLE_OUT]\n",
               argc == 0U ? "nigiri-server" : argv[0]);
    return 1;
  }

  utl::activate_progress_tracker("import");

  timetable tt;
  auto loaders = std::vector<std::unique_ptr<loader_interface>>{};
  loaders.emplace_back(std::make_unique<gtfs::gtfs_loader>());
  loaders.emplace_back(std::make_unique<hrd::hrd_5_00_8_loader>());
  loaders.emplace_back(std::make_unique<hrd::hrd_5_20_26_loader>());
  loaders.emplace_back(std::make_unique<hrd::hrd_5_20_39_loader>());
  loaders.emplace_back(std::make_unique<hrd::hrd_5_20_avv_loader>());

  auto const src = source_idx_t{0U};
  auto const tt_path = std::filesystem::path{argv[1]};
  auto const d = make_dir(tt_path);

  auto const c =
      utl::find_if(loaders, [&](auto&& l) { return l->applicable(*d); });
  utl::verify(c != end(loaders), "no loader applicable to {}", tt_path);
  log(log_lvl::info, "main", "loading nigiri timetable with configuration {}",
      (*c)->name());

  const unixtime_t from_time = parse_time(argv[2], "%Y-%m-%d %H:%M %Z");
  const unixtime_t to_time = parse_time(argv[3], "%Y-%m-%d %H:%M %Z");

  sys_days tt_from = date::sys_days{std::chrono::floor<std::chrono::days>(from_time)};
  tt_from -= std::chrono::days(2);

  sys_days tt_to = date::sys_days{std::chrono::ceil<std::chrono::days>(to_time)};
  tt_to += std::chrono::days(2);
  tt.date_range_ = {tt_from, tt_to};
  register_special_stations(tt);
  (*c)->load({}, src, *d, tt);
  finalize(tt);

  if (!std::filesystem::is_regular_file(argv[4])) {
    std::fstream fs;
    fs.open(argv[4], std::ios::binary);
    fs.close();
  }


  tt.add_reach_store_for({from_time, to_time});

  tt.write(argv[4]);
  return 0;
}