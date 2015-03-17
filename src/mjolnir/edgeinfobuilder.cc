#include <algorithm>
#include <ostream>
#include <iostream>

#include <valhalla/midgard/util.h>
#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/edgeinfo.h>
#include "mjolnir/edgeinfobuilder.h"

namespace valhalla {
namespace mjolnir {

// Set the indexes to names used by this edge.
void EdgeInfoBuilder::set_text_name_offset_list(
    const std::vector<uint32_t>& text_name_offset_list) {
  text_name_offset_list_ = text_name_offset_list;
}

// Set the shape of the edge.
void EdgeInfoBuilder::set_shape(const std::vector<PointLL>& shape) {
  // Set the shape
  encoded_shape_ = midgard::encode<std::vector<PointLL> >(shape);
}

std::size_t EdgeInfoBuilder::SizeOf() const {
  std::size_t size = 0;
  size += sizeof(baldr::EdgeInfo::PackedItem);
  size += (text_name_offset_list_.size() * sizeof(uint32_t));
  size += (encoded_shape_.size() * sizeof(std::string::value_type));
  return size;
}

std::ostream& operator<<(std::ostream& os, const EdgeInfoBuilder& eib) {
  // Make packeditem
  // TODO - protect against exceeding sizes!
  baldr::EdgeInfo::PackedItem item;
  item.fields.name_count = static_cast<uint32_t>(eib.text_name_offset_list_.size());

  // Check if we are exceeding the max encoded size
  if (eib.encoded_shape_.size() > kMaxEncodedShapeSize) {
    LOG_ERROR("Exceeding max encoded shape size: " +
              std::to_string(eib.encoded_shape_.size()));
    item.fields.encoded_shape_size = static_cast<uint32_t>(kMaxEncodedShapeSize);
  } else {
    item.fields.encoded_shape_size = static_cast<uint32_t>(eib.encoded_shape_.size());
  }

  // Write out the bytes
  os.write(reinterpret_cast<const char*>(&item), sizeof(baldr::EdgeInfo::PackedItem));
  os.write(reinterpret_cast<const char*>(&eib.text_name_offset_list_[0]),
            (eib.text_name_offset_list_.size() * sizeof(uint32_t)));
  os << eib.encoded_shape_;

  return os;
}

}
}
