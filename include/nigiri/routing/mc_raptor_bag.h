#pragma once

#include "nigiri/types.h"

#include <vector>
#include <cinttypes>

namespace nigiri::routing {

template <typename T>
struct mc_raptor_bag {

  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  std::pair<bool, iterator> merge(const T& label) {
    auto n_removed = std::size_t{0};
    for (auto i = 0U; i < labels_.size(); ++i) {
      if (labels_[i].dominates(label)) {
        return {false, end()};
      }
      if (label.dominates(labels_[i])) {
        n_removed++;
        continue;
      }
      labels_[i - n_removed] = labels_[i];
    }
    labels_.resize(labels_.size() - n_removed + 1);
    labels_.back() = label;
    return {true, std::next(begin(), static_cast<unsigned>(labels_.size() - 1))};
  }

  bool is_dominated(const T& label) const noexcept {
    for (const auto& l : labels_) {
      if (l.dominates(label)) {
        return true;
      }
    }
    return false;
  }

  friend const_iterator begin(mc_raptor_bag const& s) { return s.begin(); }
  friend const_iterator end(mc_raptor_bag const& s) { return s.end(); }
  friend iterator begin(mc_raptor_bag& s) { return s.begin(); }
  friend iterator end(mc_raptor_bag& s) { return s.end(); }
  iterator begin() { return labels_.begin(); }
  iterator end() { return labels_.end(); }
  const_iterator begin() const { return labels_.begin(); }
  const_iterator end() const { return labels_.end(); }
  iterator erase(iterator const& it) { return labels_.erase(it); }
  iterator erase(iterator const& from, iterator const& to) {return labels_.erase(from, to);}
  void clear() { labels_.clear(); }
  std::size_t size() const { return labels_.size(); }

private:
  std::vector<T> labels_;
};

}
