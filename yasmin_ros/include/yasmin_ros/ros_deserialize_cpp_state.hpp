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

#ifndef YASMIN_ROS__ROS_DESERIALIZE_CPP_STATE_HPP_
#define YASMIN_ROS__ROS_DESERIALIZE_CPP_STATE_HPP_

#include <string>

#include "yasmin/state.hpp"
#include "yasmin_ros/supported_interface_serialization.hpp"

namespace yasmin_ros {

class RosDeserializeCppState : public yasmin::State {
public:
  RosDeserializeCppState();
  ~RosDeserializeCppState() override = default;

  void configure() override;
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;

private:
  std::string interface_type_;
  const InterfaceSerializationHandler *handler_;
};

} // namespace yasmin_ros

#endif // YASMIN_ROS__ROS_DESERIALIZE_CPP_STATE_HPP_
