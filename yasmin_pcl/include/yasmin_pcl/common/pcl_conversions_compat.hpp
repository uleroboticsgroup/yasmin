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

#ifndef YASMIN_PCL__COMMON__PCL_CONVERSIONS_COMPAT_HPP_
#define YASMIN_PCL__COMMON__PCL_CONVERSIONS_COMPAT_HPP_

// pcl_conversions >= 2.7 (ROS 2 Kilted / Rolling) renamed the header to .hpp.
// Older releases use .h.
#if __has_include(<pcl_conversions/pcl_conversions.hpp>)
#include <pcl_conversions/pcl_conversions.hpp>
#else
#include <pcl_conversions/pcl_conversions.h>
#endif

#endif // YASMIN_PCL__COMMON__PCL_CONVERSIONS_COMPAT_HPP_
