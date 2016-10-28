//
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

#ifndef HPP_QUADCOPTER_STEERING_METHOD_STRAIGHT_HH
# define HPP_QUADCOPTER_STEERING_METHOD_STRAIGHT_HH

# include <hpp/model/joint.hh>
# include <hpp/core/steering-method.hh>
# include <hpp/core/problem.hh>
# include <hpp/quadcopter/config.hh>
# include <hpp/quadcopter/flat-path.hh>
# include <hpp/util/debug.hh>
# include <hpp/util/pointer.hh>

namespace hpp {
  namespace quadcopter {
    /// \addtogroup steering_method
    /// \{

    /// Steering method that creates FlatPath instances
    ///
    class HPP_QUADCOPTER_DLLAPI SteeringMethod : public core::SteeringMethod
    {
    public:
      /// Create instance and return shared pointer
      static SteeringMethodPtr_t create (const ProblemPtr_t& problem)
      {
	SteeringMethod* ptr = new SteeringMethod (problem);
	SteeringMethodPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Copy instance and return shared pointer
      static SteeringMethodPtr_t createCopy
	(const SteeringMethodPtr_t& other)
      {
	SteeringMethod* ptr = new SteeringMethod (*other);
	SteeringMethodPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }
      /// Copy instance and return shared pointer
      virtual core::SteeringMethodPtr_t copy () const
      {
	return createCopy (weak_.lock ());
      }

      /// create a path between two configurations
      virtual PathPtr_t impl_compute (ConfigurationIn_t q1,
				      ConfigurationIn_t q2) const
      {
	PathPtr_t path;
	// PathPtr_t path = FlatPath::create (device_.lock (), q1, q2,
					   // constraints ());
	return path;
      }
    protected:
      /// Constructor with robot
      /// Weighed distance is created from robot
      SteeringMethod (const ProblemPtr_t& problem) :
	core::SteeringMethod (problem), device_ (problem->robot ()), weak_ ()
	{
	}
      /// Copy constructor
      SteeringMethod (const SteeringMethod& other) :
	core::SteeringMethod (other), device_ (other.device_),
	weak_ ()
	{
	}

      /// Store weak pointer to itself
      void init (SteeringMethodWkPtr_t weak)
      {
        core::SteeringMethod::init (weak);
	weak_ = weak;
      }
    private:
      DeviceWkPtr_t device_;
      SteeringMethodWkPtr_t weak_;
    }; // SteeringMethod
    /// \}
  } // namespace quadcopter
} // namespace hpp
#endif // HPP_QUADCOPTER_STEERING_METHOD_STRAIGHT_HH
