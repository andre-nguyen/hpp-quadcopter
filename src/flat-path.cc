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
				    ConfigurationIn_t init,
				    ConfigurationIn_t end,
				    value_type distanceBetweenAxes)
    {
      FlatPath* ptr = new FlatPath (device, init, end, distanceBetweenAxes);
      FlatPathPtr_t shPtr (ptr);
      try {
	ptr->init (shPtr);
      } catch (const std::exception& exc) {
	shPtr.reset ();
      }
      return shPtr;
    }

    FlatPathPtr_t FlatPath::create (const DevicePtr_t& device,
				    ConfigurationIn_t init,
				    ConfigurationIn_t end,
				    value_type distanceBetweenAxes,
				    ConstraintSetPtr_t constraints)
    {
      FlatPath* ptr = new FlatPath (device, init, end, distanceBetweenAxes,
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

    void FlatPath::computeCoefficients ()
    {
      // Position of initial configuration (x0,y0)
      vector2_t M0 (initial_.segment <2> (0));
      hppDout (info, "M0=" << M0.transpose ());
      // orientation of initial configuration (x0,y0)
      vector2_t u0 (initial_.segment <2> (2));
      hppDout (info, "u0=" << u0.transpose ());
      // curvature relative to initial configuration
      value_type kappa0 (tan (initial_ [4])/distanceBetweenAxes_);
      hppDout (info, "kappa0=" << kappa0);
      // normal acceleration
      vector2_t v0; v0 [0] = -kappa0 * u0 [1]; v0 [1] = kappa0 * u0 [0];
      hppDout (info, "v0=" << v0.transpose ());

      // Position of initial configuration (x0,y0)
      vector2_t M1 (end_.segment <2> (0));
      hppDout (info, "M1=" << M1.transpose ());
      // orientation of initial configuration (x0,y0)
      vector2_t u1 (end_.segment <2> (2));
      hppDout (info, "u1=" << u1.transpose ());
      // curvature relative to initial configuration
      value_type kappa1 (tan (end_ [4])/distanceBetweenAxes_);
      hppDout (info, "kappa1=" << kappa1);
      // normal acceleration
      vector2_t v1; v1 [0] = -kappa1 * u1 [1]; v1 [1] = kappa1 * u1 [0];
      hppDout (info, "v1=" << v1.transpose ());

      P_ [0] = M0;
      P_ [1] = u0;
      P_ [2] = .5*v0;
      P_ [3] = 10*(M1-M0) - 6*u0 - 4*u1 - 1.5*v0 + .5*v1;
      P_ [4] = -15*(M1-M0) + 8*u0 + 7*u1 + 1.5*v0 - v1;
      P_ [5] = 6*(M1-M0) - 3*u0 - 3*u1 - .5*v0 + .5*v1;
      hppDout (info, "P_ [0]=" << P_ [0].transpose ());
      hppDout (info, "P_ [1]=" << P_ [1].transpose ());
      hppDout (info, "P_ [2]=" << P_ [2].transpose ());
      hppDout (info, "P_ [3]=" << P_ [3].transpose ());
      hppDout (info, "P_ [4]=" << P_ [4].transpose ());
      hppDout (info, "P_ [5]=" << P_ [5].transpose ());
    }

    void FlatPath::checkSteeringAngle ()
    {
    }

    void FlatPath::init (FlatPathPtr_t self)
    {
      parent_t::init (self);
      weak_ = self;
      checkSteeringAngle ();
    }

    FlatPath::FlatPath (const DevicePtr_t& device,
			ConfigurationIn_t init,
			ConfigurationIn_t end,
			value_type distanceBetweenAxes) :
      parent_t (interval_t (0, 1.), device->configSize (),
		device->numberDof ()),
      device_ (device), initial_ (init), end_ (end),
      distanceBetweenAxes_ (distanceBetweenAxes), P_ (6)
    {
      computeCoefficients ();
      assert (device);
      assert (distanceBetweenAxes > 0);
      assert (!constraints ());
    }

    FlatPath::FlatPath (const DevicePtr_t& device,
			ConfigurationIn_t init,
			ConfigurationIn_t end,
			value_type distanceBetweenAxes,
			ConstraintSetPtr_t constraints) :
      parent_t (interval_t (0, 1.), device->configSize (),
		device->numberDof (), constraints),
      device_ (device), initial_ (init), end_ (end),
      distanceBetweenAxes_ (distanceBetweenAxes), P_ (6)
    {
      assert (device);
      assert (distanceBetweenAxes >= 0);
      assert (!constraints || constraints->isSatisfied (initial_));
      if (constraints && !constraints->isSatisfied (end_)) {
	hppDout (error, *constraints);
	hppDout (error, end_.transpose ());
	abort ();
      }
      computeCoefficients ();
    }

    FlatPath::FlatPath (const FlatPath& path) :
      parent_t (path), device_ (path.device_), initial_ (path.initial_),
      end_ (path.end_), distanceBetweenAxes_ (path.distanceBetweenAxes_),
      P_ (path.P_)
    {
    }

    FlatPath::FlatPath (const FlatPath& path,
			const ConstraintSetPtr_t& constraints) :
      parent_t (path, constraints), device_ (path.device_),
      initial_ (path.initial_), end_ (path.end_),
      distanceBetweenAxes_ (path.distanceBetweenAxes_), P_ (path.P_)
    {
      assert (constraints->apply (initial_));
      assert (constraints->apply (end_));
      assert (constraints->isSatisfied (initial_));
      assert (constraints->isSatisfied (end_));
    }

    bool FlatPath::impl_compute (ConfigurationOut_t result,
				 value_type param) const
    {
      if (param == timeRange ().first || timeRange ().second == 0) {
	result = initial_;
	return true;
      }
      if (param == timeRange ().second) {
	result = end_;
	return true;
      }
      // Compute value and derivatives of flat output
      value_type& t (param);
      value_type t2 (t*t);
      value_type t3 (t2*t);
      value_type t4 (t3*t);
      value_type t5 (t4*t);
      vector2_t P (P_ [0] + t*P_ [1] + t2*P_ [2] + t3*P_[3] + t4*P_[4] +
		   t5*P_[5]);
      vector2_t dP (P_ [1] + 2*t*P_ [2] + 3*t2*P_[3] + 4*t3*P_[4] + 5*t4*P_[5]);
      vector2_t d2P (2*P_ [2] + 6*t*P_[3] + 12*t2*P_[4] + 20*t3*P_[5]);

      value_type norm_dP (sqrt(dP.squaredNorm ()));
      result.segment <2> (0) = P;
      result.segment <2> (2) = dP/norm_dP;
      value_type kappa = (dP [0]*d2P [1] - dP [1]*d2P [0])/
	(norm_dP*norm_dP*norm_dP);
      result [4] = atan (distanceBetweenAxes_ * kappa);
      return true;
    }

    DevicePtr_t FlatPath::device () const
    {
      return device_;
    }
  } //   namespace quadcopter
} // namespace hpp
