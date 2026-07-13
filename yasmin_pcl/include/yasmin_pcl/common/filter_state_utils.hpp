// Copyright (C) 2026 Maik Knof
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef YASMIN_PCL__COMMON__FILTER_STATE_UTILS_HPP_
#define YASMIN_PCL__COMMON__FILTER_STATE_UTILS_HPP_

#include <exception>
#include <string>
#include <unordered_set>

#include "yasmin/blackboard.hpp"
#include "yasmin/logs.hpp"
#include "yasmin_pcl/common/cloud_types.hpp"
#include "yasmin_pcl/common/pcl_compat.hpp"

namespace yasmin_pcl::common {

/** @brief Build a dense index vector [0, 1, ..., n-1] from cloud dimensions. */
inline Indices make_domain_indices(const PclPointCloud2Ptr &input_cloud) {
  const auto num_points = static_cast<std::size_t>(input_cloud->width) *
                          static_cast<std::size_t>(input_cloud->height);
  Indices indices;
  indices.reserve(num_points);
  for (std::size_t idx = 0; idx < num_points; ++idx) {
    indices.push_back(static_cast<int>(idx));
  }
  return indices;
}

/** @brief Return the set of indices present in domain but absent from kept. */
inline Indices compute_removed_indices(const Indices &domain_indices,
                                       const Indices &output_indices) {
  std::unordered_set<int> selected(output_indices.begin(),
                                   output_indices.end());
  Indices removed;
  removed.reserve(domain_indices.size());
  for (const int index : domain_indices) {
    if (selected.count(index) == 0U) {
      removed.push_back(index);
    }
  }
  return removed;
}

/**
 * @brief Set the input indices for a PCL filter if they exist in the
 * blackboard.
 * @tparam FilterT Type of the PCL filter.
 * @param filter Reference to the PCL filter.
 * @param blackboard Shared pointer to the blackboard.
 * @param key Key to look up input indices in the blackboard (default:
 * "input_indices").
 *
 * This function checks if the specified key exists in the blackboard. If it
 * does, it retrieves the input indices and sets them on the provided PCL
 * filter.
 */
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

/**
 * @brief Store the removed indices from a PCL filter into the blackboard.
 * @tparam FilterT Type of the PCL filter.
 * @param filter Reference to the PCL filter.
 * @param blackboard Shared pointer to the blackboard.
 * @param key Key to store removed indices in the blackboard (default:
 * "removed_indices").
 *
 * This function retrieves the removed indices from the provided PCL filter and
 * stores them in the blackboard under the specified key. If no removed indices
 * are available, an empty vector is stored instead.
 */
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

/**
 * @brief Execute a PCL filter and store the results in the blackboard.
 * @tparam FilterT Type of the PCL filter.
 * @tparam SetupFn Type of the setup function for the filter.
 * @param blackboard Shared pointer to the blackboard.
 * @param filter_name Name of the filter (used for logging).
 * @param extract_removed_indices Flag indicating whether to extract removed
 * indices.
 * @param setup Function to set up the filter before execution.
 * @return A string indicating the result of the filtering operation
 * ("succeeded" or "aborted").
 *
 * This function retrieves the input cloud from the blackboard, sets up and
 * executes the specified PCL filter, and stores the output cloud and optionally
 * removed indices back into the blackboard. If any exception occurs during
 * filtering, it logs an error message and returns "aborted".
 */
template <typename FilterT, typename SetupFn>
inline std::string
execute_filter(const yasmin::Blackboard::SharedPtr &blackboard,
               const std::string &filter_name, bool extract_removed_indices,
               SetupFn &&setup) {
  try {
    const auto input_cloud = blackboard->get<PclPointCloud2Ptr>("input_cloud");
    if (!input_cloud) {
      YASMIN_LOG_WARN("Input PCL point cloud pointer is null");
      return "aborted";
    }

    FilterT filter(extract_removed_indices);
    filter.setInputCloud(input_cloud);
    setup(filter);
    set_optional_input_indices(filter, blackboard);

    auto output_cloud = make_pcl_point_cloud2();
    filter.filter(*output_cloud);
    blackboard->set<PclPointCloud2Ptr>("output_cloud", output_cloud);

    if (extract_removed_indices) {
      store_removed_indices(filter, blackboard);
    }

    return "succeeded";
  } catch (const std::exception &e) {
    YASMIN_LOG_ERROR("%s filtering failed: %s", filter_name.c_str(), e.what());
    return "aborted";
  }
}

} // namespace yasmin_pcl::common

#endif // YASMIN_PCL__COMMON__FILTER_STATE_UTILS_HPP_
