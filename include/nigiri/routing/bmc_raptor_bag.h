#pragma once

#include "cista/containers/pair.h"

#include "nigiri/types.h"

#include <vector>
#include <cinttypes>

namespace nigiri::routing {

template <typename T>
struct bmc_raptor_bag {

  struct indexed_label {
    indexed_label() = default;
    indexed_label(T label, label_bitfield tdb)
        : label_(label),
          tdb_(tdb) {}

    T label_;
    label_bitfield tdb_;
  };

  using iterator = typename std::vector<indexed_label>::iterator;
  using const_iterator = typename std::vector<indexed_label>::const_iterator;

  bool merge(const T& label, label_bitfield tdb) {
    auto n_removed = std::size_t{0};
    auto insert_at = size();
    for (auto i = 0U; i < labels_.size(); ++i) {
      auto& el = labels_[i];
      if (el.label_.dominates(label)) {
        tdb &= ~el.tdb_;
        if (!tdb.any()) {
          return false;
        }
      }
      else if (label.dominates(el.label_)) {
        el.tdb_ &= ~tdb;
        if (!el.tdb_.any()) {
          n_removed++;
          continue;
        }
      }
      if (label.equals(el.label_)) {
        insert_at = i - n_removed;
      }
      labels_[i - n_removed] = labels_[i];
    }
    if (insert_at < size()) {
      labels_[insert_at].tdb_ |= tdb;
      labels_.resize(labels_.size() - n_removed);
      return true;
    }
    labels_.resize(labels_.size() - n_removed + 1);
    labels_.back() = indexed_label{label, tdb};
    return true;
  }

  std::vector<T> uncompress() const noexcept {
    std::vector<T> uncompressed;
    for (const auto& l : labels_) {
      const auto& tdb = l.tdb_;
      for (std::size_t i = 0U; i < tdb.size(); ++i) {
        if (tdb.none()) {
          break;
        }
        if (tdb[0]) {
          uncompressed.push_back(l.add_day_offset(i));
        }

        tdb >>= 1U;
      }
    }
    return uncompressed;
  }

  friend const_iterator begin(bmc_raptor_bag const& s) { return s.begin(); }
  friend const_iterator end(bmc_raptor_bag const& s) { return s.end(); }
  friend iterator begin(bmc_raptor_bag& s) { return s.begin(); }
  friend iterator end(bmc_raptor_bag& s) { return s.end(); }
  iterator begin() { return labels_.begin(); }
  iterator end() { return labels_.end(); }
  const_iterator begin() const { return labels_.begin(); }
  const_iterator end() const { return labels_.end(); }
  iterator erase(iterator const& it) { return labels_.erase(it); }
  iterator erase(iterator const& from, iterator const& to) {return labels_.erase(from, to);}
  void clear() { labels_.clear(); }
  std::size_t size() const { return labels_.size(); }

private:
  std::vector<indexed_label> labels_;
};

}
