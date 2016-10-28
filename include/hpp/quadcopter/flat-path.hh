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

# include <hpp/quadcopter/config.hh>
# include <hpp/quadcopter/fwd.hh>
# include <hpp/model/device.hh>
# include <hpp/core/path.hh>

namespace hpp {
  namespace quadcopter {
    /// Linear interpolation between two configurations
    ///
    /// Degrees of freedom are interpolated depending on the type of
    /// \link hpp::model::Joint joint \endlink
    /// they parameterize:
    ///   \li linear interpolation for translation joints, bounded rotation
    ///       joints, and translation part of freeflyer joints,
    ///   \li angular interpolation for unbounded rotation joints,
    ///   \li constant angular velocity for SO(3) part of freeflyer joints.
    class HPP_QUADCOPTER_DLLAPI FlatPath : public core::Path
    {
    public:
      typedef core::Path parent_t;
      /// Destructor
      virtual ~FlatPath () throw () {}

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param distanceBetweenAxes Distance between front and rear wheel axes.
      static FlatPathPtr_t create (const model::DevicePtr_t& device,
				   ConfigurationIn_t init,
				   ConfigurationIn_t end,
				   value_type distanceBetweenAxes);

      /// Create instance and return shared pointer
      /// \param device Robot corresponding to configurations
      /// \param init, end Start and end configurations of the path
      /// \param distanceBetweenAxes Distance between front and rear wheel axes.
      /// \param constraints the path is subject to
      static FlatPathPtr_t create (const DevicePtr_t& device,
				   ConfigurationIn_t init,
				   ConfigurationIn_t end,
				   value_type distanceBetweenAxes,
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

      /// Modify initial configuration
      /// \param initial new initial configuration
      /// \pre input configuration should be of the same size as current initial
      /// configuration
      void initialConfig (ConfigurationIn_t initial)
      {
	assert (initial.size () == initial_.size ());
	initial_ = initial;
      }

      /// Modify end configuration
      /// \param end new end configuration
      /// \pre input configuration should be of the same size as current end
      /// configuration
      void endConfig (ConfigurationIn_t end)
      {
	assert (end.size () == end_.size ());
	end_ = end;
      }

      /// Return the internal robot.
      DevicePtr_t device () const;

      /// Get the initial configuration
      Configuration_t initial () const
      {
        return initial_;
      }

      /// Get the final configuration
      Configuration_t end () const
      {
        return end_;
      }

    protected:
      /// Print path in a stream
      virtual std::ostream& print (std::ostream &os) const
      {
	os << "FlatPath:" << std::endl;
	os << "interval: [ " << timeRange ().first << ", "
	   << timeRange ().second << " ]" << std::endl;
	os << "initial configuration: " << initial_.transpose () << std::endl;
	os << "final configuration:   " << end_.transpose () << std::endl;
	return os;
      }
      /// Constructor
      FlatPath (const DevicePtr_t& robot, ConfigurationIn_t init,
		    ConfigurationIn_t end, value_type distanceBetweenAxes);

      /// Constructor with constraints
      FlatPath (const DevicePtr_t& robot, ConfigurationIn_t init,
		    ConfigurationIn_t end, value_type distanceBetweenAxes,
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
      /// Compute coefficient of polynomial defining the trajectory of the flat
      /// output.
      void computeCoefficients ();
      /// Check that steering angle does not get out of bounds along a path.
      void checkSteeringAngle ();
      DevicePtr_t device_;
      Configuration_t initial_;
      Configuration_t end_;
      value_type distanceBetweenAxes_;
      /// Coefficients of the polynomial flat output
      std::vector <vector2_t> P_;
      FlatPathWkPtr_t weak_;
    }; // class FlatPath
  } //   namespace quadcopter
} // namespace hpp
#endif // HPP_QUADCOPTER_FLAT_PATH_HH
