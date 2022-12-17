#pragma once

#include "nigiri/types.h"

#include <string_view>
#include <string>

namespace nigiri {

struct dynamic_bitfield {
  dynamic_bitfield() = default;
  dynamic_bitfield(bitfield bf, std::size_t size);
  dynamic_bitfield(std::size_t size);
  dynamic_bitfield(std::string_view s, std::size_t size);

  bool any() const noexcept;

  friend bool operator==(dynamic_bitfield const& a, dynamic_bitfield const& b) noexcept;

  dynamic_bitfield& operator&=(dynamic_bitfield const& o) noexcept;
  dynamic_bitfield& operator|=(dynamic_bitfield const& o) noexcept;
  dynamic_bitfield& operator^=(dynamic_bitfield const& o) noexcept;
  dynamic_bitfield operator~() const noexcept;

  friend dynamic_bitfield operator&(dynamic_bitfield const& lhs, dynamic_bitfield const& rhs) noexcept;
  friend dynamic_bitfield operator|(dynamic_bitfield const& lhs, dynamic_bitfield const& rhs) noexcept;
  friend dynamic_bitfield operator^(dynamic_bitfield const& lhs, dynamic_bitfield const& rhs) noexcept;


  dynamic_bitfield& operator>>=(std::size_t const shift) noexcept;
  dynamic_bitfield& operator<<=(std::size_t const shift) noexcept;

  dynamic_bitfield operator>>(std::size_t const i) const noexcept;
  dynamic_bitfield operator<<(std::size_t const i) const noexcept;

  std::string to_string() const;

private:
  std::size_t size_;
  bitfield mask_;
  bitfield b_;
};

};
