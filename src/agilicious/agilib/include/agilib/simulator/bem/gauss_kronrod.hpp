#pragma once

#include "agilib/math/math.hpp"

namespace agi {

template<int rows>
struct GaussKronrodResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GaussKronrodResult(const ArrayVector<rows> result,
                     const ArrayVector<rows> error)
    : result_{result}, error_{error} {}
  const ArrayVector<rows> result_;
  const ArrayVector<rows> error_;
};


template<int rows>
struct GaussKronrodFunction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual ~GaussKronrodFunction() = default;
  virtual Array<rows, 15> evaluate(const Ref<const ArrayVector<15>>,
                                   const void* const = nullptr) const = 0;
  static constexpr int SIZE = rows;
};


template<typename Function>
class GaussKronrod {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int NK = 15;

  GaussKronrodResult<Function::SIZE> integrate(
    const Function& fcn, const Scalar range_min, const Scalar range_max,
    const void* const param = nullptr) const {
    const Scalar scale = 0.5 * (range_max - range_min);
    const Scalar offset = 0.5 * (range_max + range_min);
    const ArrayVector<NK> kronrod_abscissae =
      this->kronrod_abscissae_.array() * scale + offset;

    const Array<Function::SIZE, NK> kronrod_res =
      fcn.evaluate(kronrod_abscissae, param);
    const ArrayVector<Function::SIZE> gauss =
      kronrod_res.matrix() * gauss_weights_.matrix();
    const ArrayVector<Function::SIZE> kronrod =
      kronrod_res.matrix() * kronrod_weights_.matrix();
    return GaussKronrodResult<Function::SIZE>(kronrod * scale,
                                              (kronrod - gauss).abs() * scale);
  }

 private:
  // https://www.advanpix.com/2011/11/07/gauss-kronrod-quadrature-nodes-weights/

  // Note the 0's to get rid of every second value
  const ArrayVector<NK> gauss_weights_ =
    (ArrayVector<NK>() << 0, 1.294849661688696932706114326790820e-01, 0,
     2.797053914892766679014677714237796e-01, 0,
     3.818300505051189449503697754889751e-01, 0,
     4.179591836734693877551020408163265e-01, 0,
     3.818300505051189449503697754889751e-01, 0,
     2.797053914892766679014677714237796e-01, 0,
     1.294849661688696932706114326790820e-01, 0)
      .finished();

  const ArrayVector<NK> kronrod_abscissae_ =
    (ArrayVector<NK>() << -9.914553711208126392068546975263285e-01,
     -9.491079123427585245261896840478513e-01,
     -8.648644233597690727897127886409262e-01,
     -7.415311855993944398638647732807884e-01,
     -5.860872354676911302941448382587296e-01,
     -4.058451513773971669066064120769615e-01,
     -2.077849550078984676006894037732449e-01,
     0.000000000000000000000000000000000e+00,
     2.077849550078984676006894037732449e-01,
     4.058451513773971669066064120769615e-01,
     5.860872354676911302941448382587296e-01,
     7.415311855993944398638647732807884e-01,
     8.648644233597690727897127886409262e-01,
     9.491079123427585245261896840478513e-01,
     9.914553711208126392068546975263285e-01)
      .finished();


  const ArrayVector<NK> kronrod_weights_ =
    (ArrayVector<NK>() << 2.293532201052922496373200805896959e-02,
     6.309209262997855329070066318920429e-02,
     1.047900103222501838398763225415180e-01,
     1.406532597155259187451895905102379e-01,
     1.690047266392679028265834265985503e-01,
     1.903505780647854099132564024210137e-01,
     2.044329400752988924141619992346491e-01,
     2.094821410847278280129991748917143e-01,
     2.044329400752988924141619992346491e-01,
     1.903505780647854099132564024210137e-01,
     1.690047266392679028265834265985503e-01,
     1.406532597155259187451895905102379e-01,
     1.047900103222501838398763225415180e-01,
     6.309209262997855329070066318920429e-02,
     2.293532201052922496373200805896959e-02)
      .finished();
};


}  // namespace agi
