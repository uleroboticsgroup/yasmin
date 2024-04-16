// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef YASMIN_VIEWER_PUB_HPP
#define YASMIN_VIEWER_PUB_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"

#include "yasmin_msgs/msg/state.hpp"
#include "yasmin_msgs/msg/state_machine.hpp"
#include "yasmin_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"

namespace yasmin_viewer {

class YasminViewerPub {

public:
  YasminViewerPub(rclcpp::Node *node, std::string fsm_name,
                  std::shared_ptr<yasmin::StateMachine> fsm);

  YasminViewerPub(std::string fsm_name,
                  std::shared_ptr<yasmin::StateMachine> fsm);

  std::vector<yasmin_msgs::msg::Transition>
  parse_transitions(std::map<std::string, std::string> transitions);

  void parse_state(std::string name, std::shared_ptr<yasmin::State> state,
                   std::map<std::string, std::string> transitions,
                   std::vector<yasmin_msgs::msg::State> &states_list,
                   int parent);

protected:
  void start_publisher();

private:
  rclcpp::Node *node;
  rclcpp::Publisher<yasmin_msgs::msg::StateMachine>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;

  std::string fsm_name;
  std::shared_ptr<yasmin::StateMachine> fsm;
};

} // namespace yasmin_viewer

#endif
