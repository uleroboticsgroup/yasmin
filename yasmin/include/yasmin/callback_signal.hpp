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

#ifndef YASMIN__CALLBACK_SIGNAL_HPP_
#define YASMIN__CALLBACK_SIGNAL_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/types.hpp"

namespace yasmin {

/**
 * @class CallbackSignalFuture
 * @brief Handle returned by CallbackSignal::trigger_async().
 *
 * The future can be queried for completion, waited on, and inspected for a
 * captured exception. Calling wait() rethrows the first callback exception,
 * if any, after all callbacks have finished.
 */
class CallbackSignalFuture {
public:
  /**
   * @brief Shared pointer aliases for CallbackSignalFuture.
   */
  YASMIN_PTR_ALIASES(CallbackSignalFuture)

  /**
   * @brief Wait until the asynchronous trigger finishes.
   *
   * If one or more callbacks threw an exception, the first captured exception
   * is rethrown after all callbacks have completed.
   * @throws std::runtime_error if a callback raised an exception.
   */
  void wait() const;

  /**
   * @brief Check whether the asynchronous trigger has completed.
   * @return True if the trigger finished, false otherwise.
   */
  bool is_completed() const;

  /**
   * @brief Check whether the asynchronous trigger captured an exception.
   * @return True if a callback threw, false otherwise.
   */
  bool has_exception() const;

  /**
   * @brief Get the stored exception message if one exists.
   * @return Exception text or an empty string when no exception was captured.
   */
  std::string get_exception_message() const;

  /**
   * @brief Destructor for CallbackSignalFuture.
   *
   * Waits for the background thread to finish and rethrows any captured
   * exception if one exists.
   */
  ~CallbackSignalFuture();

private:
  /**
   * @brief Shared state for the asynchronous trigger.
   */
  struct SharedState {
    /// @brief Mutex protecting the shared state.
    mutable std::mutex mutex;
    /// @brief Condition variable to signal completion.
    std::condition_variable condition;
    /// @brief Whether the async trigger has finished.
    bool completed{false};
    /// @brief Captured exception from callback execution, if any.
    std::exception_ptr exception{};
  };

  /**
   * @brief Construct a CallbackSignalFuture with shared state.
   * @param state Shared state for the asynchronous trigger.
   */
  explicit CallbackSignalFuture(std::shared_ptr<SharedState> state);

  /**
   * @brief Mark the asynchronous trigger as completed.
   * @param exception Optional exception captured during callback execution.
   */
  void set_completed(std::exception_ptr exception = nullptr);

  /// @brief Shared state for the asynchronous trigger.
  std::shared_ptr<SharedState> state_;
  /// @brief Background thread executing the callbacks.
  std::thread thread_;

  // Allow CallbackSignal to access private members for setting completion.
  friend class CallbackSignal;
};

/**
 * @class CallbackSignal
 * @brief Thread-safe callback fan-out primitive for cross-state coordination.
 *
 * CallbackSignal stores a set of zero-argument callbacks. Any state can place a
 * shared CallbackSignal instance in the blackboard, register callbacks from C++
 * or Python, and later trigger all registered callbacks synchronously or
 * asynchronously.
 *
 * Triggering uses a snapshot of the currently registered callbacks. Callbacks
 * added or removed while a trigger is already running only affect future
 * triggers.
 */
class CallbackSignal {
public:
  /**
   * @brief Shared pointer aliases for CallbackSignal.
   */
  YASMIN_PTR_ALIASES(CallbackSignal)

  /// Identifier returned when a callback is registered.
  using CallbackId = std::uint64_t;
  /// Callback signature used by the signal.
  using Callback = std::function<void()>;

  /**
   * @brief Register a callback.
   * @param callback Zero-argument callback to register.
   * @return Identifier that can later be passed to remove_callback().
   * @throws std::invalid_argument if the callback is empty.
   */
  CallbackId add_callback(Callback callback);

  /**
   * @brief Register a callback that cancels a state when triggered.
   * @param state State to cancel.
   * @return Identifier that can later be passed to remove_callback().
   * @throws std::invalid_argument if the state pointer is null.
   */
  CallbackId add_cancel_callback(const State::SharedPtr &state);

  /**
   * @brief Remove a previously registered callback.
   * @param callback_id Identifier returned by add_callback().
   * @return True if the callback existed and was removed, false otherwise.
   */
  bool remove_callback(CallbackId callback_id);

  /**
   * @brief Remove all registered callbacks.
   */
  void clear_callbacks();

  /**
   * @brief Get the number of currently registered callbacks.
   * @return Number of callbacks.
   */
  std::size_t callback_count() const;

  /**
   * @brief Check whether the signal currently has no callbacks.
   * @return True when no callbacks are registered.
   */
  bool empty() const;

  /**
   * @brief Trigger all registered callbacks synchronously.
   *
   * All callbacks from the captured snapshot are executed before this function
   * returns. If one or more callbacks throw an exception, the first exception
   * is rethrown after the snapshot has been processed completely.
   * @throws Rethrows the first exception thrown by any callback.
   */
  void trigger() const;

  /**
   * @brief Trigger all registered callbacks on a background thread.
   * @return Future-like handle that can be waited on.
   */
  CallbackSignalFuture::SharedPtr trigger_async() const;

private:
  /**
   * @brief Internal structure to hold callback entries.
   */
  struct CallbackEntry {
    /// @brief Unique identifier for this callback entry.
    CallbackId id;
    /// @brief The callback function to execute.
    Callback callback;
  };

  /**
   * @brief Execute a snapshot of callbacks.
   * @param callbacks Vector of callback entries to execute.
   *
   * This function executes all callbacks in the provided snapshot. If any
   * callback throws an exception, the first exception is captured and
   * rethrown after all callbacks have been executed.
   */
  static void execute_snapshot(const std::vector<CallbackEntry> &callbacks);

  /**
   * @brief Capture a snapshot of the currently registered callbacks.
   * @return Vector of callback entries representing the snapshot.
   *
   * This function locks the internal mutex to safely copy the current
   * callbacks into a new vector, which is then returned for execution.
   */
  std::vector<CallbackEntry> snapshot_callbacks() const;

  /// @brief Mutex to protect access to the callbacks vector and
  /// next_callback_id_.
  mutable std::mutex mutex_;
  /// @brief Vector of registered callback entries.
  std::vector<CallbackEntry> callbacks_;
  /// @brief Atomic counter for generating unique callback identifiers.
  std::atomic<CallbackId> next_callback_id_{1};
};

} // namespace yasmin

#endif // YASMIN__CALLBACK_SIGNAL_HPP_
