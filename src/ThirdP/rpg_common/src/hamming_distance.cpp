#include "rpg_common/hamming_distance.h"

#if __SSE2__
#include <emmintrin.h>
#endif

#include <glog/logging.h>

namespace rpg_common {

#if __SSE2__

// Courtesy of
// http://stackoverflow.com/questions/17354971/fast-counting-the-number-of-set-bits-in-m128i-register
const __m128i popcount_mask1 = _mm_set1_epi8(0x77);
const __m128i popcount_mask2 = _mm_set1_epi8(0x0F);
inline __m128i popcnt8(__m128i x) {
    __m128i n;
    // Count bits in each 4-bit field.
    n = _mm_srli_epi64(x, 1);
    n = _mm_and_si128(popcount_mask1, n);
    x = _mm_sub_epi8(x, n);
    n = _mm_srli_epi64(n, 1);
    n = _mm_and_si128(popcount_mask1, n);
    x = _mm_sub_epi8(x, n);
    n = _mm_srli_epi64(n, 1);
    n = _mm_and_si128(popcount_mask1, n);
    x = _mm_sub_epi8(x, n);
    x = _mm_add_epi8(x, _mm_srli_epi16(x, 4));
    x = _mm_and_si128(popcount_mask2, x);
    return x;
}

inline __m128i popcnt64(__m128i n) {
    const __m128i cnt8 = popcnt8(n);
    return _mm_sad_epu8(cnt8, _mm_setzero_si128());
}

inline int popcnt128(__m128i n) {
    const __m128i cnt64 = popcnt64(n);
    const __m128i cnt64_hi = _mm_unpackhi_epi64(cnt64, cnt64);
    const __m128i cnt128 = _mm_add_epi32(cnt64, cnt64_hi);
    return _mm_cvtsi128_si32(cnt128);
}

#endif  // __SSE2__

// Using bit twiddling hack from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
// , just as ORB_SLAM_2.
int hammingDistance(
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>& a,
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>& b)
{
  CHECK_EQ(a.size(), b.size());

  int result = 0;

#if __SSE2__
  CHECK_EQ(a.size() % (sizeof(__m128) / sizeof(unsigned char)), 0);

  for (int i = 0; i < a.size(); i += sizeof(__m128) / sizeof(unsigned char))
  {
    const __m128i* ai = (const __m128i*)(a.data() + i);
    const __m128i* bi = (const __m128i*)(b.data() + i);

    __m128i v = _mm_xor_si128(*ai, *bi);
    result += popcnt128(v);
  }
#else
  for (int i = 0; i < a.size(); ++i)
  {
    unsigned int v = a(i) ^ b(i);
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    result += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }
#endif

  return result;
}

int hammingDistance(
    const Eigen::Matrix<
    unsigned char, Eigen::Dynamic, Eigen::Dynamic>::ConstColXpr& a,
    const Eigen::Matrix<
    unsigned char, Eigen::Dynamic, Eigen::Dynamic>::ConstColXpr& b)
{
  CHECK_EQ(a.size(), b.size());

  int result = 0;

#if __SSE2__
  CHECK_EQ(a.size() % (sizeof(__m128) / sizeof(unsigned char)), 0);

  for (int i = 0; i < a.size(); i += sizeof(__m128) / sizeof(unsigned char))
  {
    const __m128i* ai = (const __m128i*)(a.data() + i);
    const __m128i* bi = (const __m128i*)(b.data() + i);

    __m128i v = _mm_xor_si128(*ai, *bi);
    result += popcnt128(v);
  }
#else
  for (int i = 0; i < a.size(); ++i)
  {
    unsigned int v = a(i) ^ b(i);
    v = v - ((v >> 1) & 0x55555555);
    v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    result += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
  }
#endif

  return result;
}

}  // namespace rpg_common
