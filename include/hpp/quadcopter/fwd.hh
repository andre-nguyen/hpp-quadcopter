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

#ifndef HPP_QUADCOPTER_FWD_HH
# define HPP_QUADCOPTER_FWD_HH

# include <hpp/util/pointer.hh>
# include <hpp/core/fwd.hh>

namespace mav_planning_utils {
  template <int N> class PolynomialTrajectory;
}

namespace hpp {
  namespace quadcopter {
    HPP_PREDEF_CLASS (FlatPath);
    typedef boost::shared_ptr <FlatPath> FlatPathPtr_t;
    HPP_PREDEF_CLASS (SteeringMethod);
    typedef boost::shared_ptr <SteeringMethod> SteeringMethodPtr_t;
    HPP_PREDEF_CLASS (ConfigurationShooter);
    typedef boost::shared_ptr <ConfigurationShooter> ConfigurationShooterPtr_t;

    typedef core::value_type value_type;
    typedef model::Device Device_t;
    typedef model::DevicePtr_t DevicePtr_t;
    typedef model::DeviceWkPtr_t DeviceWkPtr_t;
    typedef model::Configuration_t Configuration_t;
    typedef model::ConfigurationIn_t ConfigurationIn_t;
    typedef model::ConfigurationOut_t ConfigurationOut_t;
    typedef model::ConfigurationPtr_t ConfigurationPtr_t;
    typedef core::ConstraintSet ConstraintSet;
    typedef core::ConstraintSetPtr_t ConstraintSetPtr_t;
    typedef core::Path Path;
    typedef core::PathPtr_t PathPtr_t;
    typedef core::ProblemPtr_t ProblemPtr_t;
    typedef core::interval_t interval_t;
    typedef core::value_type value_type;
    typedef core::Joint Joint;
    typedef core::JointPtr_t JointPtr_t;

    typedef Eigen::Matrix <value_type, 2, 1> vector2_t;
    typedef model::vector3_t vector3_t;
    typedef model::Transform3f Transform3f;
  } // namespace quadcopter
} // namespace hpp
#endif // HPP_QUADCOPTER_FWD_HH
