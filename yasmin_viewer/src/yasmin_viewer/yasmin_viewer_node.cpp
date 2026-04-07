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

#include "yasmin_viewer/yasmin_viewer_node.hpp"

#include <cctype>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

#if __has_include(<ament_index_cpp/get_package_share_path.hpp>)
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
using tcp = asio::ip::tcp;

namespace yasmin_viewer {
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

bool is_path_safe(const std::string &request_target) {
  return request_target.find("..") == std::string::npos;
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

  if (!is_path_safe(target)) {
    throw std::runtime_error("Unsafe path requested");
  }

  if (!target.empty() && target.front() == '/') {
    target.erase(target.begin());
  }

  return std::filesystem::path(web_root) / target;
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
}

} // namespace

YasminViewerNode::YasminViewerNode()
    : Node("yasmin_viewer"), port_(5000), max_age_seconds_(3.0),
      server_running_(false) {
  this->declare_parameter<std::string>("host", "0.0.0.0");
  this->declare_parameter<int64_t>("port", 5000);
  this->declare_parameter<double>("max_age_seconds", 3.0);

  host_ = this->get_parameter("host").as_string();
  port_ = this->get_parameter("port").as_int();
  max_age_seconds_ = this->get_parameter("max_age_seconds").as_double();
  web_root_ =
#if __has_include(<ament_index_cpp/get_package_share_path.hpp>)
      (ament_index_cpp::get_package_share_path("yasmin_viewer") / "web")
          .string();
#else
      ament_index_cpp::get_package_share_directory("yasmin_viewer") + "/web";
#endif

  fsm_sub_ = this->create_subscription<StateMachineMsg>(
      "/fsm_viewer", 10,
      std::bind(&YasminViewerNode::fsm_viewer_cb, this, std::placeholders::_1));

  this->start_server();

  RCLCPP_INFO(this->get_logger(), "Started YASMIN viewer on http://%s:%ld",
              host_.c_str(), static_cast<long>(port_));
}

YasminViewerNode::~YasminViewerNode() { this->stop_server(); }

std::string YasminViewerNode::get_fsms_json() const {
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(fsms_mutex_);
  prune_expired_locked(now);

  std::ostringstream stream;
  stream << "{";

  bool first_entry = true;
  for (const auto &[name, cached_fsm] : fsms_) {
    if (!first_entry) {
      stream << ",";
    }
    first_entry = false;
    stream << '"' << escape_json(name) << '"' << ":" << cached_fsm.json;
  }

  stream << "}";
  return stream.str();
}

std::string YasminViewerNode::get_web_root() const { return web_root_; }

void YasminViewerNode::fsm_viewer_cb(const StateMachineMsg::SharedPtr msg) {
  if (!msg || msg->states.empty()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const std::string fsm_name = msg->states.front().name;
  const std::string json = state_machine_to_json(*msg);

  std::lock_guard<std::mutex> lock(fsms_mutex_);
  prune_expired_locked(now);
  fsms_[fsm_name] = CachedFsm{json, now};
}

void YasminViewerNode::start_server() {
  server_running_.store(true);
  server_thread_ = std::thread(&YasminViewerNode::run_server, this);
}

void YasminViewerNode::stop_server() {
  server_running_.store(false);

  if (server_thread_.joinable()) {
    try {
      asio::io_context io_context;
      tcp::resolver resolver(io_context);
      const std::string wake_host = (host_ == "0.0.0.0" || host_ == "::")
                                        ? "127.0.0.1"
                                    : host_ == "localhost" ? "127.0.0.1"
                                                           : host_;
      auto endpoints = resolver.resolve(wake_host, std::to_string(port_));
      tcp::socket socket(io_context);
      asio::connect(socket, endpoints);
      socket.close();
    } catch (const std::exception &) {
    }
    server_thread_.join();
  }
}

void YasminViewerNode::run_server() {
  try {
    asio::io_context io_context(1);
    const auto address =
        asio::ip::make_address(host_ == "localhost" ? "127.0.0.1" : host_);
    tcp::acceptor acceptor(io_context,
                           {address, static_cast<unsigned short>(port_)});

    while (rclcpp::ok() && server_running_.load()) {
      tcp::socket socket(io_context);
      beast::error_code error_code;
      acceptor.accept(socket, error_code);

      if (error_code) {
        if (server_running_.load()) {
          RCLCPP_WARN(this->get_logger(), "Accept failed: %s",
                      error_code.message().c_str());
        }
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
    const std::chrono::steady_clock::time_point &now) const {
  const auto max_age = std::chrono::duration<double>(max_age_seconds_);

  for (auto iterator = fsms_.begin(); iterator != fsms_.end();) {
    if ((now - iterator->second.timestamp) > max_age) {
      iterator = fsms_.erase(iterator);
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
