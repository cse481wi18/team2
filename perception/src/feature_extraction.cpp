#include "perception/feature_extraction.h"

#include <algorithm> // std::min and std::max

#include "perception/object.h"
#include "perception_msgs/ObjectFeatures.h"

namespace perception {
void ExtractSizeFeatures(const Object& object,
                         perception_msgs::ObjectFeatures* features) {
  // "x" dimension is always the smallest of x and y to account for rotations.
  // z always points up.
  double x = std::min(object.dimensions.x, object.dimensions.y);
  double y = std::max(object.dimensions.y, object.dimensions.x);
  double z = object.dimensions.z;
  features->names.push_back("box_dim_x");
  features->values.push_back(x);
  features->names.push_back("box_dim_y");
  features->values.push_back(y);
  features->names.push_back("box_dim_z");
  features->values.push_back(z);
}
}  // namespace perception