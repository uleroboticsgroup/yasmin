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

#include "yasmin_viewer/yasmin_viewer_node.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

#if __has_include("rclcpp/version.h")
#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(32, 0, 0)
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>

namespace {

std::string resolve_host(const std::string &host) {
  if (host == "0.0.0.0" || host == "::" || host == "localhost") {
    return "127.0.0.1";
  }
  return host;
}

} // namespace

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;

namespace yasmin_viewer {

struct AcceptorHolder {
  asio::io_context io_context;
  tcp::acceptor acceptor;

  AcceptorHolder(const std::string &host, unsigned short port)
      : io_context(1),
        acceptor(io_context, {asio::ip::make_address(host), port}) {}
};
namespace {

std::string mime_type(const std::string &path) {
  const std::filesystem::path file_path(path);
  const std::string extension = file_path.extension().string();

  if (extension == ".htm" || extension == ".html") {
    return "text/html";
  }
  if (extension == ".css") {
    return "text/css";
  }
  if (extension == ".txt") {
    return "text/plain";
  }
  if (extension == ".js") {
    return "application/javascript";
  }
  if (extension == ".json") {
    return "application/json";
  }
  if (extension == ".svg") {
    return "image/svg+xml";
  }
  if (extension == ".png") {
    return "image/png";
  }
  if (extension == ".jpg" || extension == ".jpeg") {
    return "image/jpeg";
  }
  if (extension == ".ico") {
    return "image/x-icon";
  }

  return "application/octet-stream";
}

std::string read_file(const std::filesystem::path &path) {
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + path.string());
  }

  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

std::filesystem::path resolve_file_path(const std::string &web_root,
                                        const std::string &request_target) {
  std::string target = request_target;
  if (target.empty() || target == "/") {
    target = "/index.html";
  }

  const auto query_separator = target.find('?');
  if (query_separator != std::string::npos) {
    target = target.substr(0, query_separator);
  }

  if (!target.empty() && target.front() == '/') {
    target.erase(target.begin());
  }

  std::error_code ec;
  std::filesystem::path web_root_normalized =
      std::filesystem::weakly_canonical(web_root, ec);

  if (ec) {
    throw std::runtime_error("Invalid web root");
  }

  std::filesystem::path resolved =
      (web_root_normalized / target).lexically_normal();

  std::string resolved_str = resolved.string();
  std::string web_root_str = web_root_normalized.string();

  if (resolved_str.find(web_root_str) != 0) {
    throw std::runtime_error("Path traversal detected");
  }

  if (!std::filesystem::exists(resolved, ec) || ec) {
    throw std::runtime_error("File not found");
  }

  return resolved;
}

http::response<http::string_body> make_response(http::status status,
                                                const std::string &content_type,
                                                const std::string &body,
                                                bool keep_alive) {
  http::response<http::string_body> response{status, 11};
  response.set(http::field::content_type, content_type);
  response.set(http::field::server, "yasmin_viewer");
  response.keep_alive(keep_alive);
  response.body() = body;
  response.prepare_payload();
  return response;
}

void handle_session(tcp::socket socket, YasminViewerNode *node) {
  beast::error_code error_code;
  beast::flat_buffer buffer;

  while (true) {
    http::request<http::string_body> request;
    http::read(socket, buffer, request, error_code);

    if (error_code == http::error::end_of_stream) {
      break;
    }

    if (error_code) {
      break;
    }

    http::response<http::string_body> response;

    if (request.method() != http::verb::get &&
        request.method() != http::verb::head) {
      response = make_response(http::status::method_not_allowed, "text/plain",
                               "Method not allowed", request.keep_alive());
    } else {
      const std::string target = std::string(request.target());

      if (target == "/api/fsms") {
        response = make_response(http::status::ok, "application/json",
                                 node->get_fsms_json(), request.keep_alive());
      } else {
        try {
          const auto file_path =
              resolve_file_path(node->get_web_root(), target);
          const std::string body = read_file(file_path);
          response =
              make_response(http::status::ok, mime_type(file_path.string()),
                            request.method() == http::verb::head ? "" : body,
                            request.keep_alive());
          if (request.method() == http::verb::head) {
            response.content_length(body.size());
          }
        } catch (const std::exception &) {
          response = make_response(http::status::not_found, "text/plain",
                                   "Not found", request.keep_alive());
        }
      }
    }

    http::write(socket, response, error_code);
    if (error_code || !request.keep_alive()) {
      break;
    }
  }

  socket.shutdown(tcp::socket::shutdown_send, error_code);
  node->on_connection_closed();
}

} // namespace

YasminViewerNode::YasminViewerNode()
    : Node("yasmin_viewer"), port_(5000), max_age_seconds_(3.0),
      server_running_(false) {
  this->declare_parameter<std::string>("host", "0.0.0.0");
  this->declare_parameter<int64_t>("port", 5000);
  this->declare_parameter<double>("max_age_seconds", 3.0);

  this->host_ = this->get_parameter("host").as_string();
  this->port_ = this->get_parameter("port").as_int();
  this->max_age_seconds_ = this->get_parameter("max_age_seconds").as_double();

  web_root_ =
#if __has_include("rclcpp/version.h")
#if RCLCPP_VERSION_GTE(32, 0, 0)
      (ament_index_cpp::get_package_share_path("yasmin_viewer") / "web")
          .string();
#else
      ament_index_cpp::get_package_share_directory("yasmin_viewer") + "/web";
#endif
#else
      ament_index_cpp::get_package_share_directory("yasmin_viewer") + "/web";
#endif

  this->fsm_sub_ = this->create_subscription<StateMachineMsg>(
      "/fsm_viewer", 10,
      std::bind(&YasminViewerNode::fsm_viewer_cb, this, std::placeholders::_1));

  this->start_server();

  RCLCPP_INFO(this->get_logger(), "Started YASMIN viewer on http://%s:%ld",
              this->host_.c_str(), static_cast<long>(this->port_));
}

YasminViewerNode::~YasminViewerNode() { this->stop_server(); }

std::string YasminViewerNode::get_fsms_json() {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(this->fsms_mutex_);
  this->prune_expired_locked(now);

  std::ostringstream stream;
  stream << "{";

  bool first_entry = true;
  for (const auto &[name, cached_fsm] : this->fsms_) {
    if (!first_entry) {
      stream << ",";
    }
    first_entry = false;
    stream << '"' << escape_json(name) << '"' << ":" << cached_fsm.json;
  }

  stream << "}";
  return stream.str();
}

std::string YasminViewerNode::get_web_root() const { return this->web_root_; }

void YasminViewerNode::fsm_viewer_cb(const StateMachineMsg::SharedPtr msg) {
  if (!msg || msg->states.empty()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const std::string fsm_name = msg->states.front().name;
  const std::string json = state_machine_to_json(*msg);

  std::lock_guard<std::mutex> lock(this->fsms_mutex_);
  this->prune_expired_locked(now);
  this->fsms_[fsm_name] = CachedFsm{json, now};
}

void YasminViewerNode::start_server() {
  this->acceptor_holder_ = std::make_unique<AcceptorHolder>(
      resolve_host(this->host_), static_cast<unsigned short>(this->port_));
  this->server_running_.store(true);
  this->server_thread_ = std::thread(&YasminViewerNode::run_server, this);
}

void YasminViewerNode::stop_server() {
  this->server_running_.store(false);

  if (this->acceptor_holder_) {
    beast::error_code ec;
    this->acceptor_holder_->acceptor.cancel(ec);
  }

  if (this->server_thread_.joinable()) {
    this->server_thread_.join();
  }
}

void YasminViewerNode::run_server() {
  try {
    if (!this->acceptor_holder_) {
      RCLCPP_ERROR(this->get_logger(), "Acceptor not initialized");
      return;
    }

    auto &acceptor = this->acceptor_holder_->acceptor;
    auto &io_context = this->acceptor_holder_->io_context;

    while (rclcpp::ok() && this->server_running_.load()) {
      tcp::socket socket(io_context);
      beast::error_code error_code;
      acceptor.accept(socket, error_code);

      if (error_code) {
        if (this->server_running_.load()) {
          RCLCPP_WARN(this->get_logger(), "Accept failed: %s",
                      error_code.message().c_str());
        }
        continue;
      }

      uint32_t prev =
          this->active_connections_.fetch_add(1, std::memory_order_relaxed);

      if (prev >= YasminViewerNode::kMaxConnections) {
        this->active_connections_.fetch_sub(1, std::memory_order_relaxed);
        RCLCPP_WARN(this->get_logger(),
                    "Too many connections, dropping client");
        continue;
      }

      std::thread(handle_session, std::move(socket), this).detach();
    }
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(this->get_logger(), "Viewer server stopped: %s",
                 exception.what());
  }
}

void YasminViewerNode::prune_expired_locked(
    const std::chrono::steady_clock::time_point &now) {
  const auto max_age = std::chrono::duration<double>(this->max_age_seconds_);

  for (auto iterator = this->fsms_.begin(); iterator != this->fsms_.end();) {
    if ((now - iterator->second.timestamp) > max_age) {
      iterator = this->fsms_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

std::string YasminViewerNode::escape_json(const std::string &value) {
  std::ostringstream stream;

  for (const char character : value) {
    switch (character) {
    case '\\':
      stream << "\\\\";
      break;
    case '"':
      stream << "\\\"";
      break;
    case '\b':
      stream << "\\b";
      break;
    case '\f':
      stream << "\\f";
      break;
    case '\n':
      stream << "\\n";
      break;
    case '\r':
      stream << "\\r";
      break;
    case '\t':
      stream << "\\t";
      break;
    default:
      if (static_cast<unsigned char>(character) < 0x20U) {
        stream << "\\u" << std::hex << std::uppercase
               << static_cast<int>(character);
      } else {
        stream << character;
      }
      break;
    }
  }

  return stream.str();
}

std::string YasminViewerNode::transitions_to_json(
    const std::vector<TransitionMsg> &transitions) {
  std::ostringstream stream;
  stream << "[";

  for (std::size_t index = 0; index < transitions.size(); ++index) {
    if (index > 0) {
      stream << ",";
    }

    stream << "[\"" << escape_json(transitions[index].outcome) << "\",\""
           << escape_json(transitions[index].state) << "\"]";
  }

  stream << "]";
  return stream.str();
}

std::string YasminViewerNode::state_to_json(const StateMsg &state) {
  std::ostringstream stream;
  stream << "{";
  stream << "\"id\":" << state.id << ",";
  stream << "\"parent\":" << state.parent << ",";
  stream << "\"name\":\"" << escape_json(state.name) << "\",";
  stream << "\"transitions\":" << transitions_to_json(state.transitions) << ",";
  stream << "\"outcomes\":[";

  for (std::size_t index = 0; index < state.outcomes.size(); ++index) {
    if (index > 0) {
      stream << ",";
    }
    stream << "\"" << escape_json(state.outcomes[index]) << "\"";
  }

  stream << "],";
  stream << "\"is_fsm\":" << (state.is_fsm ? "true" : "false") << ",";
  stream << "\"current_state\":" << state.current_state;
  stream << "}";
  return stream.str();
}

std::string
YasminViewerNode::state_machine_to_json(const StateMachineMsg &msg) {
  std::ostringstream stream;
  stream << "[";

  for (std::size_t index = 0; index < msg.states.size(); ++index) {
    if (index > 0) {
      stream << ",";
    }
    stream << state_to_json(msg.states[index]);
  }

  stream << "]";
  return stream.str();
}

} // namespace yasmin_viewer
