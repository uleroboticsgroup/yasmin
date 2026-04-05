// Copyright (C) 2026 Maik Knof
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef YASMIN_PCL__COMMON__FILTER_STATE_UTILS_HPP_
#define YASMIN_PCL__COMMON__FILTER_STATE_UTILS_HPP_

#include <string>

#include <pcl/types.h>
#include <yasmin/blackboard.hpp>

#include "yasmin_pcl/common/cloud_types.hpp"

namespace yasmin_pcl::common {

template <class FilterT>
inline void
set_optional_input_indices(FilterT &filter,
                           const yasmin::Blackboard::SharedPtr &blackboard,
                           const std::string &key = "input_indices") {
  if (!blackboard->contains(key)) {
    return;
  }

  const auto input_indices = blackboard->get<Indices>(key);
  pcl::IndicesPtr input_indices_ptr(new pcl::Indices(input_indices));
  filter.setIndices(input_indices_ptr);
}

template <class FilterT>
inline void
store_removed_indices(FilterT &filter,
                      const yasmin::Blackboard::SharedPtr &blackboard,
                      const std::string &key = "removed_indices") {
  const auto removed_indices_ptr = filter.getRemovedIndices();
  if (removed_indices_ptr) {
    blackboard->set<Indices>(key, *removed_indices_ptr);
  } else {
    blackboard->set<Indices>(key, Indices{});
  }
}

template <class FilterT>
inline void
store_output_indices(FilterT &filter,
                     const yasmin::Blackboard::SharedPtr &blackboard,
                     const std::string &key = "output_indices") {
  Indices output_indices;
  filter.filter(output_indices);
  blackboard->set<Indices>(key, output_indices);
}

} // namespace yasmin_pcl::common

#endif // YASMIN_PCL__COMMON__FILTER_STATE_UTILS_HPP_
