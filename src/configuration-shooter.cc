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

#include <hpp/quadcopter/configuration-shooter.hh>

#include <hpp/util/debug.hh>

#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/model/extra-config-space.hh>

#include <mav_msgs/conversions.h>
#include <mav_planning_utils/trajectory_sampling.h>

namespace hpp {
  namespace quadcopter {
    namespace {
      void generateRandomConfig (DevicePtr_t robot, ConfigurationOut_t config) {
        core::JointVector_t jv = robot->getJointVector ();
	assert (config.size() ==  robot->configSize ());

	for (core::JointVector_t::const_iterator itJoint = jv.begin ();
	     itJoint != jv.end (); itJoint++) {
	  std::size_t rank = (*itJoint)->rankInConfiguration ();
	  (*itJoint)->configuration ()->uniformlySample (rank, config);
	}
	// Shoot extra configuration variables
        core::size_type extraDim = robot->extraConfigSpace ().dimension ();
        core::size_type offset   = robot->configSize () - extraDim;
	for (core::size_type i=0; i<extraDim; ++i) {
	  value_type lower = robot->extraConfigSpace ().lower (i);
	  value_type upper = robot->extraConfigSpace ().upper (i);
	  value_type range = upper - lower;
	  if ((range < 0) ||
	      (range == std::numeric_limits<double>::infinity())) {
	    std::ostringstream oss
	      ("Cannot uniformy sample extra config variable ");
	    oss << i << ". min = " <<lower<< ", max = " << upper << std::endl;
	    throw std::runtime_error (oss.str ());
	  }
	  config [offset + i] = lower + (upper - lower) * rand ()/RAND_MAX;
	}
      }
    }

    ConfigurationPtr_t ConfigurationShooter::shoot () const
    {
      ConfigurationPtr_t config (new Configuration_t (robot_->configSize ()));
      generateRandomConfig (robot_, *config);

      mav_msgs::EigenTrajectoryPoint state;
      state.position_W     = config->head<3>();
      state.velocity_W     = config->segment<3> (7);
      state.acceleration_W = config->segment<3> (10);
      state.jerk_W         = config->segment<3> (13);
      // state.snap_W = config.segment<3> (7);

      mav_msgs::EigenMavState mav_state;

      mav_msgs::EigenMavStateFromEigenTrajectoryPoint(state, &mav_state);
      // std::cout << sampling_time << "\t: " << mav_state.toString() << std::endl;

      config->operator()( 3) = mav_state.orientation_W_B.w();
      config->segment<3>( 4) = mav_state.orientation_W_B.vec();

      return config;
    }
  } //   namespace quadcopter
} // namespace hpp
