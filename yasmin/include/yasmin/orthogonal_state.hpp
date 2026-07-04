// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#ifndef YASMIN__ORTHOGONAL_STATE_HPP_
#define YASMIN__ORTHOGONAL_STATE_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "yasmin/join_state.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"

namespace yasmin {

/**
 * @class OrthogonalState
 * @brief Represents a state that can execute multiple concurrent states in
 * parallel.
 *
 * The OrthogonalState class allows for the execution of multiple states
 * concurrently, each in its own region. It manages the synchronization of
 * these regions and determines the overall outcome based on the outcomes of
 * the individual regions.
 */
class OrthogonalState : public State {
public:
  /** @brief Shared pointer aliases for OrthogonalState. */
  YASMIN_PTR_ALIASES(OrthogonalState)

  /**
   * @struct RegionDescriptor
   * @brief Describes a region within the OrthogonalState.
   *
   * Each region has a name and a shared pointer to a StateMachine that
   * represents the state machine for that region.
   */
  struct RegionDescriptor {
    /// @brief Name of this region.
    std::string name;
    /// @brief The state machine running in this region.
    StateMachine::SharedPtr sm;
  };

  /**
   * @brief Constructs an OrthogonalState with a default outcome and an optional
   * outcome map.
   * @param default_outcome The default outcome to return if no specific outcome
   * is determined.
   * @param outcome_map A map of region names to their corresponding outcomes.
   */
  OrthogonalState(const std::string &default_outcome,
                  const OutcomeMap &outcome_map = {});

  /**
   * @brief Adds a region to the OrthogonalState.
   * @param name The name of the region.
   * @param sm A shared pointer to the StateMachine for the region.
   */
  void add_region(const std::string &name, StateMachine::SharedPtr sm);

  /**
   * @brief Gets the list of regions in the OrthogonalState.
   * @return A constant reference to the vector of RegionDescriptor.
   */
  const std::vector<RegionDescriptor> &get_regions() const noexcept;

  /**
   * @brief Gets the outcome map for the OrthogonalState.
   * @return A constant reference to the OutcomeMap.
   */
  const OutcomeMap &get_outcome_map() const noexcept;

  /**
   * @brief Gets the default outcome for the OrthogonalState.
   * @return A constant reference to the default outcome string.
   */
  const std::string &get_default_outcome() const noexcept;

  /// @brief Hook type for GIL management around thread fork/join
  using GilHook = std::function<void()>;

  /**
   * @brief Sets the hooks for GIL management before forking threads and after
   * joining threads.
   * @param before_fork A function to be called before forking threads.
   * @param after_join A function to be called after joining threads.
   */
  static void set_thread_hooks(GilHook before_fork, GilHook after_join);

  /**
   * @brief Configures the OrthogonalState.
   *
   * This method sets up the necessary synchronization mechanisms and prepares
   * the state for execution.
   */
  void configure() override;

  /**
   * @brief Executes the OrthogonalState.
   * @param blackboard A shared pointer to the Blackboard to use during
   * execution.
   * @return A string representing the outcome of the execution.
   *
   * This method executes all regions concurrently and waits for their
   * completion. It then evaluates the outcomes of the regions to determine the
   * overall outcome of the OrthogonalState.
   */
  std::string execute(Blackboard::SharedPtr blackboard) override;

  /**
   * @brief Cancels the execution of the OrthogonalState.
   *
   * This method cancels all regions and sets the status to CANCELED.
   */
  void cancel_state() override;

  /**
   * @brief Validates the region state machines of this orthogonal state.
   * @param strict_mode If true, performs strict validation.
   *
   * Recursively validates each region's StateMachine.
   */
  void validate(bool strict_mode = false);

  /**
   * @brief Converts the OrthogonalState to a string representation.
   * @return A string representation of the OrthogonalState.
   *
   * This method retrieves the demangled name of the class for a readable
   * string representation.
   */
  std::string to_string() const override;

private:
  /// @brief Hooks for GIL management before forking threads and after joining
  /// threads.
  static GilHook before_fork_hook_;
  /// @brief Hook for GIL management after joining threads.
  static GilHook after_join_hook_;

  /// @brief The list of regions in the OrthogonalState.
  std::vector<RegionDescriptor> regions_;
  /// @brief A map of region names to their corresponding RegionBarrier.
  std::unordered_map<std::string, RegionBarrier::SharedPtr> barriers_;
  /// @brief A map of region names to their corresponding outcomes.
  OutcomeMap outcome_map_;
  /// @brief The default outcome to return if no specific outcome is determined.
  std::string default_outcome_;
  /// @brief Indicates whether the OrthogonalState has been configured.
  std::atomic_bool configured_{false};
  /// @brief Precomputed region name -> index map for O(1) outcome lookups.
  std::unordered_map<std::string, size_t> region_name_to_index_;

  /**
   * @brief Evaluates the outcomes of the regions to determine the overall
   * outcome.
   * @param region_outcomes A vector of outcomes from each region.
   * @return A string representing the overall outcome.
   *
   * This method checks the outcomes of all regions against the outcome map and
   * determines the overall outcome based on the defined rules. If no specific
   * outcome is determined, it returns the default outcome.
   */
  std::string
  evaluate_outcomes(const std::vector<std::string> &region_outcomes) const;
};

} // namespace yasmin

#endif // YASMIN__ORTHOGONAL_STATE_HPP_
