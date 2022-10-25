#include "rpg_common/cv_type.h"

#include <opencv2/core/core.hpp>

namespace rpg_common {

#define CV_TYPE(type, _value) \
template <> \
const int CvType<type>::value = _value;

CV_TYPE(unsigned char, CV_8U);
CV_TYPE(char, CV_8S);
CV_TYPE(uint16_t, CV_16U);
CV_TYPE(int16_t, CV_16S);
CV_TYPE(int32_t, CV_32S);
CV_TYPE(float, CV_32F);
CV_TYPE(double, CV_64F);

}  // namespace rpg_common
