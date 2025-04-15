// Copyright (C) 2025 Pedro Edom Nunes
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

#include <string>

#ifndef YASMIN_DEMOS_FOO_STATE_H
#define YASMIN_DEMOS_FOO_STATE_H

class FooState : public yasmin::State {
public:
  FooState();
  ~FooState();

  std::string
  execute(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard);
  int counter;
};

#endif // YASMIN_DEMOS_FOO_STATE_H