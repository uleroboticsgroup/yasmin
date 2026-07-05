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

#include "yasmin/callback_signal.hpp"

#include <algorithm>
#include <thread>
#include <utility>

using namespace yasmin;

CallbackSignalFuture::CallbackSignalFuture(std::shared_ptr<SharedState> state)
    : state_(std::move(state)) {}

CallbackSignalFuture::~CallbackSignalFuture() {
  if (this->thread_.joinable()) {
    this->thread_.join();
  }
}

void CallbackSignalFuture::set_completed(std::exception_ptr exception) {
  {
    std::lock_guard<std::mutex> lock(this->state_->mutex);
    this->state_->exception = exception;
    this->state_->completed = true;
  }

  this->state_->condition.notify_all();
}

void CallbackSignalFuture::wait() const {
  std::unique_lock<std::mutex> lock(this->state_->mutex);
  this->state_->condition.wait(lock,
                               [this]() { return this->state_->completed; });

  if (this->state_->exception) {
    std::rethrow_exception(this->state_->exception);
  }
}

bool CallbackSignalFuture::is_completed() const {
  std::lock_guard<std::mutex> lock(this->state_->mutex);
  return this->state_->completed;
}

bool CallbackSignalFuture::has_exception() const {
  std::lock_guard<std::mutex> lock(this->state_->mutex);
  return static_cast<bool>(this->state_->exception);
}

std::string CallbackSignalFuture::get_exception_message() const {
  std::exception_ptr exception;

  {
    std::lock_guard<std::mutex> lock(this->state_->mutex);
    exception = this->state_->exception;
  }

  if (!exception) {
    return "";
  }

  try {
    std::rethrow_exception(exception);
  } catch (const std::exception &error) {
    return error.what();
  } catch (...) {
    return "Unknown exception";
  }
}

CallbackSignal::CallbackId CallbackSignal::add_callback(Callback callback) {
  if (!callback) {
    throw std::invalid_argument("CallbackSignal callback must not be empty");
  }

  std::lock_guard<std::mutex> lock(this->mutex_);

  const CallbackId callback_id = this->next_callback_id_.fetch_add(1);
  this->callbacks_.push_back({callback_id, std::move(callback)});
  return callback_id;
}

CallbackSignal::CallbackId
CallbackSignal::add_cancel_callback(const State::SharedPtr &state) {
  if (!state) {
    throw std::invalid_argument("CallbackSignal state must not be null");
  }

  State::WeakPtr weak_state = state;
  return this->add_callback([weak_state]() {
    if (const auto locked_state = weak_state.lock()) {
      locked_state->cancel_state();
    }
  });
}

bool CallbackSignal::remove_callback(CallbackId callback_id) {
  std::lock_guard<std::mutex> lock(this->mutex_);

  const auto callback_it =
      std::find_if(this->callbacks_.begin(), this->callbacks_.end(),
                   [callback_id](const CallbackEntry &entry) {
                     return entry.id == callback_id;
                   });

  if (callback_it == this->callbacks_.end()) {
    return false;
  }

  this->callbacks_.erase(callback_it);
  return true;
}

void CallbackSignal::clear_callbacks() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->callbacks_.clear();
}

std::size_t CallbackSignal::callback_count() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->callbacks_.size();
}

bool CallbackSignal::empty() const { return this->callback_count() == 0U; }

std::vector<CallbackSignal::CallbackEntry>
CallbackSignal::snapshot_callbacks() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->callbacks_;
}

void CallbackSignal::execute_snapshot(
    const std::vector<CallbackEntry> &callbacks) {
  std::exception_ptr first_exception;

  for (const auto &entry : callbacks) {
    try {
      entry.callback();
    } catch (...) {
      if (!first_exception) {
        first_exception = std::current_exception();
      }
    }
  }

  if (first_exception) {
    std::rethrow_exception(first_exception);
  }
}

void CallbackSignal::trigger() const {
  CallbackSignal::execute_snapshot(this->snapshot_callbacks());
}

CallbackSignalFuture::SharedPtr CallbackSignal::trigger_async() const {
  auto state = std::make_shared<CallbackSignalFuture::SharedState>();
  auto future =
      CallbackSignalFuture::SharedPtr(new CallbackSignalFuture(state));
  const auto callbacks = this->snapshot_callbacks();

  std::thread worker([state, callbacks]() {
    try {
      CallbackSignal::execute_snapshot(callbacks);
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->completed = true;
      }
      state->condition.notify_all();
    } catch (...) {
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->exception = std::current_exception();
        state->completed = true;
      }
      state->condition.notify_all();
    }
  });

  future->thread_ = std::move(worker);
  return future;
}
