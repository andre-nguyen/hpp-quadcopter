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

#include <hpp/quadcopter/flat-path.hh>

#include <mav_msgs/conversions.h>
#include <mav_planning_utils/polynomial_trajectory.h>
#include <mav_planning_utils/trajectory_sampling.h>

#include <hpp/util/debug.hh>

#include <hpp/model/device.hh>

namespace hpp {
  namespace quadcopter {
    FlatPath::~FlatPath () throw ()
    {
      delete traj_;
      delete yawTraj_;
    }

    FlatPathPtr_t FlatPath::create (const model::DevicePtr_t& device,
				    Trajectory_t* traj,
				    YawTrajectory_t* yawTraj)
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
				    Trajectory_t* traj,
				    YawTrajectory_t* yawTraj,
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
			Trajectory_t* traj,
			YawTrajectory_t* yawTraj) :
      parent_t (
          interval_t (traj->getMinTime(), traj->getMaxTime()),
          device->configSize (), device->numberDof ()),
      device_ (device),
      traj_ (traj), yawTraj_ (yawTraj)
    {
      assert (device);
      assert (!constraints ());
    }

    FlatPath::FlatPath (const DevicePtr_t& device,
			Trajectory_t* traj,
			YawTrajectory_t* yawTraj,
			ConstraintSetPtr_t constraints) :
      parent_t (
          interval_t (traj->getMinTime(), traj->getMaxTime()),
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
      mav_msgs::EigenTrajectoryPoint state;
      mav_msgs::EigenMavState mav_state;

      // I get a SEGV when sampling_time == tmax
      // because some iterators get out of range.
      if (param < timeRange ().first)
        param = timeRange().first;
      else if (param >= timeRange ().second)
        param = timeRange ().second - (timeRange().second - timeRange().first) * 1e-5;

      mav_planning_utils::sampleTrajectory(*traj_, *yawTraj_, param, &state);
      mav_msgs::EigenMavStateFromEigenTrajectoryPoint(state, &mav_state);
      // std::cout << sampling_time << "\t: " << mav_state.toString() << std::endl;

      result.head<3>   (  ) = mav_state.position_W;
      result           ( 3) = mav_state.orientation_W_B.w();
      result.segment<3>( 4) = mav_state.orientation_W_B.vec();
      assert (state.velocity_W.isApprox(mav_state.velocity_W));
      result.segment<3>( 7) = mav_state.velocity_W;
      result.segment<3>(10) = state.acceleration_W;
      result.segment<3>(13) = state.jerk_W;

      return true;
    }

    DevicePtr_t FlatPath::device () const
    {
      return device_;
    }
  } //   namespace quadcopter
} // namespace hpp
