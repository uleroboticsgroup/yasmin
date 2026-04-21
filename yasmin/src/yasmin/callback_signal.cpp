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

#include "yasmin/callback_signal.hpp"

#include <algorithm>
#include <thread>
#include <utility>

using namespace yasmin;

CallbackSignalFuture::CallbackSignalFuture(std::shared_ptr<SharedState> state)
    : state_(std::move(state)) {}

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
  auto future = CallbackSignalFuture::SharedPtr(new CallbackSignalFuture(
      std::make_shared<CallbackSignalFuture::SharedState>()));
  const auto callbacks = this->snapshot_callbacks();

  std::thread([future, callbacks]() {
    try {
      CallbackSignal::execute_snapshot(callbacks);
      future->set_completed();
    } catch (...) {
      future->set_completed(std::current_exception());
    }
  }).detach();

  return future;
}
