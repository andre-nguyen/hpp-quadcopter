// Copyright (c) 2016 CNRS
// Authors: Joseph Mirabel
//
// This file is part of hpp-quadcopter
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
// hpp-quadcopter  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_QUADCOPTER_DISTANCE_HH
# define HPP_QUADCOPTER_DISTANCE_HH

# include <hpp/core/distance.hh>

# include <hpp/quadcopter/fwd.hh>
# include <hpp/quadcopter/config.hh>

namespace hpp {
  namespace quadcopter {
    /// \addtogroup steering_method
    /// \{
    class HPP_QUADCOPTER_DLLAPI Distance : public core::Distance {
    public:
      static DistancePtr_t create (const DevicePtr_t& robot)
      {
        Distance* ptr = new Distance (robot);
        DistancePtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      static core::DistancePtr_t createCopy (const DistancePtr_t& distance)
      {
        Distance* ptr = new Distance (*distance);
        DistancePtr_t shPtr (ptr);
        ptr->init (shPtr);
        return shPtr;
      }

      virtual core::DistancePtr_t clone () const
      {
        return createCopy (weak_.lock ());
      }

      /// Get robot
      const DevicePtr_t& robot () const
      {
	return robot_;
      }

      Distance (const DevicePtr_t& robot) : robot_ (robot) {}

    protected:
      Distance (const Distance& distance) : robot_ (distance.robot_) {}

      void init (DistanceWkPtr_t self) { weak_ = self; }

      /// Derived class should implement this function
      virtual value_type impl_distance (ConfigurationIn_t q1,
				    ConfigurationIn_t q2) const;
    private:
      DevicePtr_t robot_;
      DistanceWkPtr_t weak_;
    }; // class Distance
    /// \}
  } //   namespace quadcopter
} // namespace hpp
#endif // HPP_QUADCOPTER_DISTANCE_HH
