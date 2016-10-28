//
// Copyright (c) 2015 CNRS
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

#ifndef HPP_QUADCOPTER_FLAT_PATH_HH
# define HPP_QUADCOPTER_FLAT_PATH_HH

# include <hpp/model/device.hh>

# include <hpp/core/path.hh>

# include <mav_planning_utils/polynomial_trajectory.h>

# include <hpp/quadcopter/config.hh>
# include <hpp/quadcopter/fwd.hh>

namespace hpp {
  namespace quadcopter {
    /// Interpolation for a quadcopter.
    class HPP_QUADCOPTER_DLLAPI FlatPath : public core::Path
    {
    public:
      typedef core::Path parent_t;

      static const int N = 10;

      typedef mav_planning_utils::PolynomialTrajectory<N> Trajectory_t;
      typedef mav_planning_utils::PolynomialTrajectory<2> YawTrajectory_t;

      /// Destructor
      virtual ~FlatPath () throw () {}

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param distanceBetweenAxes Distance between front and rear wheel axes.
      static FlatPathPtr_t create (const model::DevicePtr_t& device,
				   const Trajectory_t& traj,
				   const YawTrajectory_t& yawTraj);

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param constraints the path is subject to
      static FlatPathPtr_t create (const DevicePtr_t& device,
				   const Trajectory_t& traj,
				   const YawTrajectory_t& yawTraj,
				   ConstraintSetPtr_t constraints);

      /// Create copy and return shared pointer
      /// \param path path to copy
      static FlatPathPtr_t createCopy (const FlatPathPtr_t& path)
      {
	FlatPath* ptr = new FlatPath (*path);
	FlatPathPtr_t shPtr (ptr);
	ptr->initCopy (shPtr);
	return shPtr;
      }

      /// Create copy and return shared pointer
      /// \param path path to copy
      /// \param constraints the path is subject to
      static FlatPathPtr_t createCopy
	(const FlatPathPtr_t& path, const ConstraintSetPtr_t& constraints)
      {
	FlatPath* ptr = new FlatPath (*path, constraints);
	FlatPathPtr_t shPtr (ptr);
	ptr->initCopy (shPtr);
	return shPtr;
      }

      /// Return a shared pointer to this
      ///
      /// As StaightPath are immutable, and refered to by shared pointers,
      /// they do not need to be copied.
      virtual PathPtr_t copy () const
      {
	return createCopy (weak_.lock ());
      }

      /// Return a shared pointer to a copy of this and set constraints
      ///
      /// \param constraints constraints to apply to the copy
      /// \precond *this should not have constraints.
      virtual PathPtr_t copy (const ConstraintSetPtr_t& constraints) const
      {
	return createCopy (weak_.lock (), constraints);
      }

      /// Return the internal robot.
      DevicePtr_t device () const;

      /// Get the initial configuration
      Configuration_t initial () const
      {
        Configuration_t out (outputSize());
        operator() (out, timeRange().first);
        return out;
      }

      /// Get the final configuration
      Configuration_t end () const
      {
        Configuration_t out (outputSize());
        operator() (out, timeRange().second);
        return out;
      }

    protected:
      /// Print path in a stream
      virtual std::ostream& print (std::ostream &os) const
      {
	os << "FlatPath:" << std::endl;
	os << "interval: [ " << timeRange ().first << ", "
	   << timeRange ().second << " ]" << std::endl;
	os << "initial configuration: " << initial().transpose () << std::endl;
	os << "final configuration:   " << end().transpose () << std::endl;
	return os;
      }

      /// Constructor
      FlatPath (const DevicePtr_t& robot,
          const Trajectory_t& traj,
          const YawTrajectory_t& yaw);

      /// Constructor with constraints
      FlatPath (const DevicePtr_t& robot,
          const Trajectory_t& traj,
          const YawTrajectory_t& yaw,
          ConstraintSetPtr_t constraints);

      /// Copy constructor
      FlatPath (const FlatPath& path);

      /// Copy constructor with constraints
      FlatPath (const FlatPath& path,
		    const ConstraintSetPtr_t& constraints);

      void init (FlatPathPtr_t self);

      void initCopy (FlatPathPtr_t self)
      {
	parent_t::initCopy (self);
	weak_ = self;
      }

      virtual bool impl_compute (ConfigurationOut_t result,
				 value_type param) const;

    private:
      DevicePtr_t device_;

      Trajectory_t traj_;
      YawTrajectory_t yawTraj_;

      FlatPathWkPtr_t weak_;
    }; // class FlatPath
  } //   namespace quadcopter
} // namespace hpp
#endif // HPP_QUADCOPTER_FLAT_PATH_HH
