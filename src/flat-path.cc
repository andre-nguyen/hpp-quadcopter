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

#include <hpp/util/debug.hh>
#include <hpp/model/device.hh>
#include <hpp/model/joint.hh>
#include <hpp/model/joint-configuration.hh>
#include <hpp/core/config-projector.hh>
#include <hpp/core/projection-error.hh>
#include <hpp/quadcopter/flat-path.hh>

namespace hpp {
  namespace quadcopter {

    FlatPathPtr_t FlatPath::create (const model::DevicePtr_t& device,
				    const Trajectory_t& traj,
				    const YawTrajectory_t& yawTraj)
    {
      FlatPath* ptr = new FlatPath (device, traj, yawTraj);
      FlatPathPtr_t shPtr (ptr);
      try {
	ptr->init (shPtr);
      } catch (const std::exception& exc) {
	shPtr.reset ();
      }
      return shPtr;
    }

    FlatPathPtr_t FlatPath::create (const DevicePtr_t& device,
				    const Trajectory_t& traj,
				    const YawTrajectory_t& yawTraj,
				    ConstraintSetPtr_t constraints)
    {
      FlatPath* ptr = new FlatPath (device, traj, yawTraj,
				    constraints);
      FlatPathPtr_t shPtr (ptr);
      try {
	ptr->init (shPtr);
	hppDout (info, "success");
      } catch (const std::exception& exc) {
	hppDout (info, "failure");
	shPtr.reset ();
      }
      return shPtr;
    }

    void FlatPath::init (FlatPathPtr_t self)
    {
      parent_t::init (self);
      weak_ = self;
    }

    FlatPath::FlatPath (const DevicePtr_t& device,
			const Trajectory_t& traj,
			const YawTrajectory_t& yawTraj) :
      parent_t (
          interval_t (0, 1.),
          // interval_t (traj.getMinTime(), traj.getMaxTime()),
          device->configSize (), device->numberDof ()),
      device_ (device),
      traj_ (traj), yawTraj_ (yawTraj)
    {
      assert (device);
      assert (!constraints ());
    }

    FlatPath::FlatPath (const DevicePtr_t& device,
			const Trajectory_t& traj,
			const YawTrajectory_t& yawTraj,
			ConstraintSetPtr_t constraints) :
      parent_t (
          interval_t (0, 1.),
          // interval_t (traj.getMinTime(), traj.getMaxTime()),
          device->configSize (), device->numberDof (), constraints),
      device_ (device),
      traj_ (traj), yawTraj_ (yawTraj)
    {
      assert (device);
      assert (!constraints || constraints->isSatisfied (initial()));
      if (constraints && !constraints->isSatisfied (end())) {
	hppDout (error, *constraints);
	hppDout (error, end().transpose ());
	abort ();
      }
    }

    FlatPath::FlatPath (const FlatPath& path) :
      parent_t (path), device_ (path.device_),
      traj_ (path.traj_), yawTraj_ (path.yawTraj_)
    {}

    FlatPath::FlatPath (const FlatPath& path,
			const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints), device_ (path.device_),
      traj_ (path.traj_), yawTraj_ (path.yawTraj_)
    {
      // assert (constraints->apply (initial()));
      // assert (constraints->apply (end()));
      assert (constraints->isSatisfied (initial()));
      assert (constraints->isSatisfied (end()));
    }

    bool FlatPath::impl_compute (ConfigurationOut_t result,
				 value_type param) const
    {
      /*
      mav_msgs::EigenTrajectoryPoint state;

      if (param == timeRange ().first || timeRange ().second == 0) {
        state = traj.
	result = initial_;
	return true;
      }
      if (param == timeRange ().second) {
	result = end_;
	return true;
      }

      // I get a SEGV when sampling_time == tmax
      // because some iterators get out of range.
      if (sampling_time >= tmax) sampling_time = tmax * 0.9999999;

      // mav_planning_utils::sampleTrajectory(trajectory, yaw_trajectory, sampling_time, &flat_state);
      mav_planning_utils::sampleTrajectory(trajectory, yaw_trajectory, sampling_time, &state);
      mav_msgs::EigenMavStateFromEigenTrajectoryPoint(state, &mav_state);
      // State orientation is always identity
      // std::cout << sampling_time << "\t: " << state.position_W.transpose()
      // << ", " << state.orientation_W_B.coeffs().transpose() << std::endl;

      std::cout << sampling_time << "\t: " << mav_state.toString() << std::endl;
      */
      return true;
    }

    DevicePtr_t FlatPath::device () const
    {
      return device_;
    }
  } //   namespace quadcopter
} // namespace hpp
