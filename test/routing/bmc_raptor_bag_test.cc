#include "doctest/doctest.h"

#include "nigiri/routing/arrival_departure_label.h"
#include "nigiri/routing/bmc_raptor_bag.h"
#include "nigiri/types.h"

#include "cista/containers/pair.h"

#include <vector>
#include <string>

using namespace nigiri;
using namespace nigiri::routing;
using label = typename nigiri::routing::arrival_departure_label;
using control_bag_t = typename std::vector<cista::pair<label, label_bitfield>>;
using bag_t = typename nigiri::routing::bmc_raptor_bag<label>;

void is_equal(const bag_t& b, const control_bag_t& c) {
  CHECK_EQ(b.size(), c.size());
  auto b_iter = b.begin();
  auto c_iter = c.begin();
  while (b_iter != b.end()) {
    CHECK(b_iter->label_.equals(c_iter->first));
    const auto same_tdb = b_iter->tdb_ == c_iter->second;
    CHECK(same_tdb);

    b_iter++;
    c_iter++;
  }
}

TEST_CASE("bmc-raptor-bag") {
  bag_t bag;
  control_bag_t control_bag;

  const label l1{minutes_after_midnight_t{180}, minutes_after_midnight_t{150}};
  const label l2{minutes_after_midnight_t{180}, minutes_after_midnight_t{140}};
  const label l3{minutes_after_midnight_t{200}, minutes_after_midnight_t{130}};
  const std::string tdb1{"1001"};
  const std::string tdb2{"0110"};
  const std::string tdb3{"0011"};
  bag.merge(l1,
            label_bitfield{tdb1});
  bag.merge(l2,
            label_bitfield{tdb2});
  bag.merge(l3,
            label_bitfield{tdb3});

  control_bag.push_back({l1, label_bitfield{tdb1}});
  control_bag.push_back({l2, label_bitfield{tdb2}});
  is_equal(bag, control_bag);

  bag.merge(l1, label_bitfield{"1110"});
  control_bag.pop_back();
  control_bag[0].second = label_bitfield{"1111"};

  is_equal(bag, control_bag);

  const label l4{minutes_after_midnight_t{170}, minutes_after_midnight_t{150}};
  bag.merge(l4, label_bitfield{"0011"});

  control_bag[0].second = label_bitfield{"1100"};
  control_bag.push_back({l4, label_bitfield{"0011"}});

  is_equal(bag, control_bag);

  const label l5{minutes_after_midnight_t{200}, minutes_after_midnight_t{140}};
  bag.merge(l5, label_bitfield{"0110"});
  is_equal(bag, control_bag);

  bag.clear();
  control_bag.clear();
  CHECK_EQ(bag.size(), 0U);

  const label l6{minutes_after_midnight_t{240}, minutes_after_midnight_t{120}};
  const label l7{minutes_after_midnight_t {300}, minutes_after_midnight_t{120}};
  const label l8{minutes_after_midnight_t {300}, minutes_after_midnight_t {180}};
  const label l9{minutes_after_midnight_t {360}, minutes_after_midnight_t {240}};
  label_bitfield tdb{"1"};
  bag.merge(l6, tdb);
  bag.merge(l7, tdb << 1);
  bag.merge(l8, tdb << 2);
  bag.merge(l9, tdb << 3);

  control_bag.push_back({l6, tdb});
  control_bag.push_back({l7, tdb << 1});
  control_bag.push_back({l8, tdb << 2});
  control_bag.push_back({l9, tdb << 3});

  is_equal(bag, control_bag);
  const label l10{minutes_after_midnight_t{300}, minutes_after_midnight_t {240}};
  bag.merge(l10, label_bitfield{"1111"});

  control_bag.erase(control_bag.begin()+1, control_bag.end());
  control_bag.push_back({l10, label_bitfield{"1111"}});

  is_equal(bag, control_bag);

  bag.merge(l6, label_bitfield{"1111"});
  control_bag[0].second = label_bitfield {"1111"};

  is_equal(bag, control_bag);

  const label l11{minutes_after_midnight_t {300}, minutes_after_midnight_t {250}};
  bag.merge(l11, label_bitfield{"1001"});

  control_bag[1].second = label_bitfield{"0110"};
  control_bag.push_back({l11, label_bitfield{"1001"}});

  is_equal(bag, control_bag);
};