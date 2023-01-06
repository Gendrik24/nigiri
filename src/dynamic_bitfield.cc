#include "nigiri/dynamic_bitfield.h"

#include <string>
#include <string_view>

namespace nigiri {

dynamic_bitfield::dynamic_bitfield(bitfield bf, std::size_t size)
  : size_(size),
    mask_(std::string(size_, '1')),
    b_(bf & mask_) {};

dynamic_bitfield::dynamic_bitfield(std::size_t size)
  : dynamic_bitfield(bitfield{}, size) {};

dynamic_bitfield::dynamic_bitfield(std::string_view s, std::size_t size)
  : dynamic_bitfield(bitfield{s}, size) {};

bool dynamic_bitfield::any() const noexcept {
  return b_.any();
}

std::size_t dynamic_bitfield::size() const noexcept {
  return this->size_;
}

bool dynamic_bitfield::operator[](std::size_t i) const noexcept {
  return b_.test(i);
}

bool operator==(dynamic_bitfield const& a, dynamic_bitfield const& b) noexcept {
  return a.b_ == b.b_ && a.size_ == b.size_;
}

dynamic_bitfield& dynamic_bitfield::operator&=(dynamic_bitfield const& o) noexcept {
  b_ &= o.b_;
  b_ &= mask_;
  return *this;
}

dynamic_bitfield& dynamic_bitfield::operator|=(dynamic_bitfield const& o) noexcept {
  b_ |= o.b_;
  b_ &= mask_;
  return *this;
}

dynamic_bitfield& dynamic_bitfield::operator^=(dynamic_bitfield const& o) noexcept {
  b_ ^= o.b_;
  b_ &= mask_;
  return *this;
}

dynamic_bitfield dynamic_bitfield::operator~() const noexcept {
  auto copy = *this;
  copy.b_ = ~copy.b_;
  return copy;
}

dynamic_bitfield operator&(dynamic_bitfield const& lhs, dynamic_bitfield const& rhs) noexcept {
  auto copy = lhs;
  copy &= rhs;
  return copy;
}

dynamic_bitfield operator|(dynamic_bitfield const& lhs, dynamic_bitfield const& rhs) noexcept {
  auto copy = lhs;
  copy |= rhs;
  return copy;
}

dynamic_bitfield operator^(dynamic_bitfield const& lhs, dynamic_bitfield const& rhs) noexcept {
  auto copy = lhs;
  copy ^= rhs;
  return copy;
}

dynamic_bitfield& dynamic_bitfield::operator>>=(std::size_t const shift) noexcept {
  b_ >>= shift;
  return *this;
}

dynamic_bitfield& dynamic_bitfield::operator<<=(std::size_t const shift) noexcept {
  b_ <<= shift;
  b_ &= mask_;
  return *this;
}

dynamic_bitfield dynamic_bitfield::operator>>(std::size_t const i) const noexcept {
  auto copy = *this;
  copy >>= i;
  return copy;
}

dynamic_bitfield dynamic_bitfield::operator<<(std::size_t const i) const noexcept {
  auto copy = *this;
  copy <<= i;
  return copy;
}

std::string dynamic_bitfield::to_string() const {
  std::string e_str = b_.to_string();
  return e_str.substr(e_str.size() - size_, size_);
}

};
