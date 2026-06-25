// Copyright (C) 2023 Miguel Ángel González Santamarta
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

#include "yasmin/cb_state.hpp"
#include "yasmin/types.hpp"

namespace yasmin {

CbState::CbState(const Outcomes &outcomes, CbStateCallback callback)
    : State(outcomes), callback(callback) {}

std::string CbState::execute(Blackboard::SharedPtr blackboard) {
  return this->callback(blackboard);
}

} // namespace yasmin
