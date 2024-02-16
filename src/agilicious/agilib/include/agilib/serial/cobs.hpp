#pragma once


namespace agi {

template<char Delimiter>
struct COBS {
  static bool stuff(const char* const src, const int src_length,
                    char* const dst, int* const dst_length) {
    if (src == nullptr || dst == nullptr) return false;
    if (src_length < 1 || *dst_length < src_length + 1) return false;

    const char* src_ptr = src;
    char* dst_ptr = dst;

    const char* const end = src + src_length;
    char* last = dst_ptr++;
    int count = 1;

    do {
      if (*src_ptr != Delimiter) {
        *dst_ptr++ = *src_ptr;
        ++count;
      } else {
        if (count == Delimiter) count = 0;
        *last = count;
        last = dst_ptr++;
        count = 1;
      }
    } while (++src_ptr < end);

    if (count == Delimiter) count = 0;
    *last = count;
    *dst_length = src_length + 1;

    return true;
  }

  static int unstuff(const char* const src, const int src_length,
                     char* const dst, int* const dst_length) {
    if (src == nullptr || dst == nullptr) return false;
    if (src_length < 1 || *dst_length < src_length) return false;

    const char* src_ptr = src;
    char* dst_ptr = dst;

    const char* end = src + src_length;
    char count = *src_ptr++;
    if (count == 0) count = Delimiter;
    --count;

    do {
      if (*src_ptr == Delimiter) return false;
      if (count != 0) {
        *dst_ptr++ = *src_ptr;
        --count;
      } else {
        count = *src_ptr;
        *dst_ptr++ = Delimiter;
        if (count == 0) count = Delimiter;
        --count;
      }
    } while (++src_ptr < end);
    *dst_length = src_length - 1;

    return count == 0;
  }
};


}  // namespace agi
