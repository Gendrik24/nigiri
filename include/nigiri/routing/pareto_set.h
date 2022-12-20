#pragma once

#include "utl/enumerate.h"
#include "fmt/core.h"

#include <cinttypes>
#include <vector>

namespace nigiri {

template <typename T>
struct pareto_set {
  using iterator = typename std::vector<T>::iterator;
  using const_iterator = typename std::vector<T>::const_iterator;

  size_t size() const { return els_.size(); }

  std::pair<bool, iterator> add(const T& el) {
    auto n_removed = std::size_t{0};
    for (auto i = 0U; i < els_.size(); ++i) {
      if (els_[i].dominates(el)) {
        return {false, end()};
      }
      if (el.dominates(els_[i])) {
        n_removed++;
        continue;
      }
      els_[i - n_removed] = els_[i];
    }
    els_.resize(els_.size() - n_removed + 1);
    els_.back() = std::move(el);
    return {true, std::next(begin(), static_cast<unsigned>(els_.size() - 1))};
  }

  bool dominates(const T& el) {
    for (const auto& l : els_) {
      if (l.dominates(el)) return true;
    }
    return false;
  }

  std::vector<std::pair<bool, iterator>> merge(const pareto_set<T>& set) {
    std::vector<std::pair<bool, iterator>> res;
    for (const auto& el : set) {
      auto n_removed = std::size_t{0};
      bool dominated = false;
      for (auto i = 0U; i < els_.size(); ++i) {
        if (els_[i].dominates(el)) {
          res.push_back({false, end()});
          dominated = true;
          break;
        }
        if (el.dominates(els_[i])) {
          n_removed++;
          continue;
        }
        els_[i - n_removed] = els_[i];
      }
      if (dominated) {
        continue;
      }
      els_.resize(els_.size() - n_removed + 1);
      els_.back() = std::move(el);
      res.push_back({true, std::next(begin(), static_cast<unsigned>(els_.size() - 1))});
    }

    return res;
  }

  friend const_iterator begin(pareto_set const& s) { return s.begin(); }
  friend const_iterator end(pareto_set const& s) { return s.end(); }
  friend iterator begin(pareto_set& s) { return s.begin(); }
  friend iterator end(pareto_set& s) { return s.end(); }
  iterator begin() { return els_.begin(); }
  iterator end() { return els_.end(); }
  const_iterator begin() const { return els_.begin(); }
  const_iterator end() const { return els_.end(); }
  iterator erase(iterator const& it) { return els_.erase(it); }
  iterator erase(iterator const& from, iterator const& to) {
    return els_.erase(from, to);
  }
  void clear() { els_.clear(); }

private:
  std::vector<T> els_;
};

}  // namespace nigiri
