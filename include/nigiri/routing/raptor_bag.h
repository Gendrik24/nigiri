#pragma once

#include <vector>

#include "nigiri/routing/raptor_label.h"

namespace nigiri::routing {

using labels_t = std::vector<raptor_label>;
using const_labels_iterator = labels_t::const_iterator;
using bag_size_t = std::size_t;

struct raptor_bag {
  raptor_bag() {};
  raptor_bag(const labels_t& from_labels);

  bag_size_t size() const noexcept;

  raptor_label& operator[](const std::size_t i);
  const raptor_label& operator[](const std::size_t i) const;

  const_labels_iterator begin() const noexcept;
  const_labels_iterator end() const noexcept;

  bool merge(const raptor_label& label) noexcept;
  std::vector<bool> mergeAll(const labels_t& labels) noexcept;
  void clear() noexcept;

private:
  labels_t labels_;
};

} // namespace nigiri::routing
