#pragma once

#include "avoidlib/objects/static_object.hpp"

namespace avoidlib {
class Cylinder : public StaticObject {
 public:
  Cylinder(const std::string& id, const std::string& prefab_id = "Cylinder")
    : StaticObject(id, prefab_id) {}
  ~Cylinder() {}
  void setOpacity(const float& opa) {opacity = opa;};
  float getOpacity() {return opacity;};
protected:
  Scalar opacity;
};

}  // namespace avoidlib
