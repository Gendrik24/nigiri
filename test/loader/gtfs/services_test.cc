#include "gtest/gtest.h"

#include "nigiri/loader/gtfs/calendar.h"
#include "nigiri/loader/gtfs/calendar_date.h"
#include "nigiri/loader/gtfs/files.h"
#include "nigiri/loader/gtfs/services.h"
#include "nigiri/types.h"

#include "./test_data.h"

using namespace nigiri;
using namespace nigiri::loader;
using namespace nigiri::loader::gtfs;
using namespace date;

/*
 * -- BASE --
 * WE:       11         WD:       00
 *      0000011              1111100
 *      0000011              1111100
 *      0000011              1111100
 *      0000011              1111100
 *      0                    1
 */

/*
 * -- DATES --
 * WE:       11         WD:       00
 *      1100011              0011100
 *      0000011              1111100
 *      0000011              1111100
 *      0000011              1111100
 *      0                    1
 */

TEST(gtfs, service_dates) {
  auto dates =
      read_calendar_date(example_files().get_file(kCalendarDatesFile).data());
  auto calendar = read_calendar(example_files().get_file(kCalenderFile).data());
  auto traffic_days = merge_traffic_days(calendar, dates);

  auto we_bit_str = std::string{"1111000110000011000001100000110"};
  auto wd_bit_str = std::string{"0000111001111100111110011111001"};
  std::reverse(begin(we_bit_str), end(we_bit_str));
  std::reverse(begin(wd_bit_str), end(wd_bit_str));
  auto const we_traffic_days = bitfield{we_bit_str};
  auto const wd_traffic_days = bitfield{wd_bit_str};

  EXPECT_EQ(July / 1 / 2006, traffic_days.interval_.from_);
  EXPECT_EQ(August / 1 / 2006, traffic_days.interval_.to_);

  EXPECT_EQ(we_traffic_days, *traffic_days.traffic_days_["WE"]);
  EXPECT_EQ(wd_traffic_days, *traffic_days.traffic_days_["WD"]);
}
