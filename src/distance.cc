// Copyright (c) 2016, Joseph Mirabel
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-quadcopter.
// hpp-quadcopter is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-quadcopter is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-quadcopter. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/quadcopter/distance.hh>

namespace hpp {
  namespace quadcopter {
    value_type Distance::impl_distance (
        ConfigurationIn_t q1, ConfigurationIn_t q2) const
    {
      /*
      std::vector<value_type> segment_times(2);
      // The formula was taken from mav_planning_utils::estimateSegmentTimes
      value_type distance = (q1 - q2).head<3>().norm();
      value_type v_max = 2;
      value_type a_max = 2;
      value_type magic_fabian_constant = 6.5; // tuning parameter ... 6.5 is default.
      segment_times[1] = distance / v_max * 2 *
        (1.0 + magic_fabian_constant * v_max / a_max * exp(-distance / v_max * 2));
      if (segment_times[1] < 1e-5) return PathPtr_t();
      // This value is not important.
      segment_times[0] = segment_times[1] / 2;
      */
      value_type res =
          (q1 - q2).head<3>   (  ).squaredNorm() // Position
        + (q1 - q2).segment<3>( 7).squaredNorm() // Velocity 
        + (q1 - q2).segment<3>(10).squaredNorm() // Acceleration
        + (q1 - q2).segment<3>(13).squaredNorm();// Jerk
      return std::sqrt(res);
    }
  } // namespace quadcopter
} // namespace hpp
