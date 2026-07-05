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

#ifndef YASMIN__REGION_BARRIER_HPP_
#define YASMIN__REGION_BARRIER_HPP_

#include <condition_variable>
#include <mutex>

#include "yasmin/types.hpp"

namespace yasmin {

/**
 * @class RegionBarrier
 * @brief A synchronization primitive that allows multiple threads to wait for
 * each other at a barrier point.
 *
 * The RegionBarrier class is used to synchronize the execution of multiple
 * threads. It allows a specified number of threads (parties) to wait until all
 * have reached a certain point in their execution before proceeding. This is
 * useful in scenarios where multiple concurrent tasks need to be synchronized.
 */
class RegionBarrier {
public:
  /** @brief Shared pointer aliases for RegionBarrier. */
  YASMIN_PTR_ALIASES(RegionBarrier)

  /**
   * @brief Constructs a RegionBarrier with a specified number of parties.
   * @param party_count The number of threads that must call arrive_and_wait()
   * before any of them can proceed.
   */
  explicit RegionBarrier(int party_count);

  /** @brief Default destructor. */
  ~RegionBarrier() = default;

  /** @brief Deleted copy constructor (non-copyable). */
  RegionBarrier(const RegionBarrier &) = delete;

  /** @brief Deleted copy assignment (non-copyable). */
  RegionBarrier &operator=(const RegionBarrier &) = delete;

  /**
   * @brief Waits for all parties to arrive at the barrier.
   *
   * This method blocks the calling thread until the specified number of
   * threads have called arrive_and_wait(). Once all threads have arrived, they
   * are released to continue execution.
   */
  void arrive_and_wait();

  /**
   * @brief Cancels the barrier, releasing all waiting threads.
   *
   * This method sets the barrier to a canceled state, allowing all waiting
   * threads to proceed. Subsequent calls to arrive_and_wait() will not block.
   */
  void cancel();

  /**
   * @brief Resets the barrier to its initial state.
   *
   * This method resets the barrier, allowing it to be reused for another
   * synchronization point. It can only be called when no threads are waiting
   * at the barrier.
   */
  void reset();

  /**
   * @brief Gets the number of parties required to reach the barrier.
   * @return The number of parties as an integer.
   */
  int get_party_count() const;

private:
  /// @brief Mutex for synchronizing access to the barrier state.
  std::mutex mtx_;
  /// @brief Condition variable for blocking and waking up threads.
  std::condition_variable cv_;
  /// @brief The number of threads that must call arrive_and_wait() before the
  /// barrier is lifted.
  int party_count_;
  /// @brief Initial party count preserved for reset().
  int initial_count_;
  /// @brief The number of threads that have currently arrived at the barrier.
  int arrived_{0};
  /// @brief The current generation of the barrier, used to distinguish between
  /// consecutive barrier phases.
  int generation_{0};
  /// @brief Indicates whether the barrier has been canceled.
  bool canceled_{false};
};

} // namespace yasmin

#endif // YASMIN__REGION_BARRIER_HPP_
