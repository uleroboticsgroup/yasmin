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

private:
  struct SharedState {
    mutable std::mutex mutex;
    std::condition_variable condition;
    bool completed{false};
    std::exception_ptr exception{};
  };

  explicit CallbackSignalFuture(std::shared_ptr<SharedState> state);

  void set_completed(std::exception_ptr exception = nullptr);

  std::shared_ptr<SharedState> state_;

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
   */
  void trigger() const;

  /**
   * @brief Trigger all registered callbacks on a background thread.
   * @return Future-like handle that can be waited on.
   */
  CallbackSignalFuture::SharedPtr trigger_async() const;

private:
  struct CallbackEntry {
    CallbackId id;
    Callback callback;
  };

  static void execute_snapshot(const std::vector<CallbackEntry> &callbacks);

  std::vector<CallbackEntry> snapshot_callbacks() const;

  mutable std::mutex mutex_;
  std::vector<CallbackEntry> callbacks_;
  std::atomic<CallbackId> next_callback_id_{1};
};

} // namespace yasmin

#endif // YASMIN__CALLBACK_SIGNAL_HPP_
