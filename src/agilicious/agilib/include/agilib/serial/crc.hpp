#pragma once

#include <cstdint>
#include <limits>
#include <utility>

namespace agi {

// This provides compile-time generated CRC table and runtime checking.
template<typename crc_t, crc_t polynomial>
class CRC {
 public:
  using Type = crc_t;
  static constexpr int SIZE = sizeof(crc_t);

  constexpr CRC() : CRC(std::make_integer_sequence<int, 256>()) {}

  constexpr crc_t compute(const char* const buffer, const int length) const {
    crc_t crc = 0;
    for (int i = 0; i < length; ++i)
      crc = table[(crc ^ buffer[i]) & 0xFF] ^ (crc >> 8);
    return (~crc) ^ std::numeric_limits<crc_t>::max();
  }

 private:
  template<int... pack>
  constexpr CRC(std::integer_sequence<int, pack...>)
    : table{prepare(pack)...} {}
  static constexpr crc_t prepare(const crc_t val, const int iter = 8) {
    return iter <= 0
             ? val
             : prepare((val >> 1) ^ (val & 1 ? polynomial : 0), iter - 1);
  }
  constexpr crc_t operator[](const int i) const { return table[i]; }

  crc_t table[256];
};

// Predefined CRC tables for 16, 32, 64 bit checksums.
// Polynomials according to ANSI/ISO.
// https://en.wikipedia.org/wiki/Cyclic_redundancy_check
using CRC16 = CRC<uint16_t, 0xA001>;
using CRC32 = CRC<uint32_t, 0xEDB88320>;
using CRC64 = CRC<uint64_t, 0xC96C5795D7870F42>;

}  // namespace agi
