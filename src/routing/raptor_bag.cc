#include "nigiri/routing/raptor_bag.h"

using namespace nigiri::routing;
using namespace std;

raptor_bag::raptor_bag(const labels_t& from_labels) {
  for (size_t i=0; i<from_labels.size(); ++i) {
    merge(from_labels[i]);
  }
}

bag_size_t raptor_bag::size() const noexcept {
  return labels_.size();
}

raptor_label& raptor_bag::operator[](const size_t i) {
  return labels_[i];
}

const raptor_label& raptor_bag::operator[](const size_t i) const {
  return labels_[i];
}

const_labels_iterator raptor_bag::begin() const noexcept {
  return labels_.begin();
}

const_labels_iterator raptor_bag::end() const noexcept {
  return labels_.end();
}

bool raptor_bag::merge(const raptor_label& label) noexcept {
  size_t removed = 0;
  for (size_t i=0; i<labels_.size(); ++i) {
    if (labels_[i].dominates(label)) return false;
    if (label.dominates(labels_[i])) {
      removed++;
    } else {
      labels_[i - removed] = labels_[i];
    }
  }
  labels_.resize(labels_.size() - removed + 1);
  labels_.back() = label;
  return true;
}

void raptor_bag::clear() noexcept {
  labels_.clear();
}

std::vector<bool> raptor_bag::mergeAll(const labels_t& labels) noexcept {
  std::vector<bool> individual_merge_results(labels.size());
  for (size_t i=0; i<labels.size(); ++i) {
    individual_merge_results[i] = merge(labels[i]);
  }

  return individual_merge_results;
}


