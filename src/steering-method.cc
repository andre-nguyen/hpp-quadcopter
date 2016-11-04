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

# include <hpp/quadcopter/steering-method.hh>

# include <mav_planning_utils/polynomial_optimization.h>
# include <mav_planning_utils/polynomial_trajectory.h>

const int dimension = 3; // 3D path.
const size_t derivative_to_optimize = mav_planning_utils::derivative_order::SNAP;

namespace hpp {
  namespace quadcopter {
    PathPtr_t SteeringMethod::impl_compute (ConfigurationIn_t q1,
        ConfigurationIn_t q2) const
    {
      using namespace mav_planning_utils;
      Vertex::Vector vertices;
      Vertex start(3), middle(3), end(3);

      // TODO so far, only position and linear velocity are constrained.
      // We should also constrain orientation and angular velocity.

      start.addConstraint (derivative_order::POSITION,     q1.head   <3>(  ));
      start.addConstraint (derivative_order::VELOCITY,     q1.segment<3>( 7));
      start.addConstraint (derivative_order::ACCELERATION, q1.segment<3>(10));
      start.addConstraint (derivative_order::JERK,         q1.segment<3>(13));
      // start.addConstraint (derivative_order::ACCELERATION, Eigen::VectorXd::Zero(3));
      // start.addConstraint (derivative_order::JERK,         Eigen::VectorXd::Zero(3));
      vertices.push_back(start);

      // Without this line, I get an error saying I need strictly more than 2 vertex.
      vertices.push_back(middle);

      end.addConstraint (derivative_order::POSITION,     q2.head   <3>( ));
      end.addConstraint (derivative_order::VELOCITY,     q2.segment<3>(7));
      end.addConstraint (derivative_order::ACCELERATION, q2.segment<3>(10));
      end.addConstraint (derivative_order::JERK,         q2.segment<3>(13));
      // end.addConstraint (derivative_order::ACCELERATION, Eigen::VectorXd::Zero(3));
      // end.addConstraint (derivative_order::JERK,         Eigen::VectorXd::Zero(3));
      vertices.push_back(end);

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

      PolynomialOptimization<FlatPath::N> opt(dimension);
      opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
      opt.solveLinear();

      Segment<FlatPath::N>::Vector segments;
      opt.getSegments(&segments);

      // Sample the trajectory
      FlatPath::TrajectoryPtr_t trajectory (new FlatPath::Trajectory_t(dimension, segments));

      double tmin = trajectory->getMinTime();
      double tmax = trajectory->getMaxTime();
      // std::cout << tmin << ", " << tmax << std::endl;

      // Yaw trajectory is linear
      // TODO yaw velocity is not continuous here. Must use polynome of order 2.
      mav_planning_utils::Segment<2>::Vector yaw_segments;
      yaw_segments.push_back(mav_planning_utils::Segment<2>(1));
      Eigen::Vector2d coeffs (0,0);
      // Eigen::Vector2d coeffs (
          // yawFromQuaternion(Eigen::Quaternion<value_type>(q1[3],q1[4],q1[5],q1[6])),
          // yawFromQuaternion(Eigen::Quaternion<value_type>(q2[3],q2[4],q2[5],q2[6])));
      yaw_segments[0][0].setCoefficients(coeffs);
      yaw_segments[0].setTime(tmax - tmin); // or only tmax ?
      FlatPath::YawTrajectoryPtr_t yaw_trajectory (new FlatPath::YawTrajectory_t(1, yaw_segments));

      PathPtr_t path = FlatPath::create (device_.lock (), trajectory, yaw_trajectory);
      // std::cout << (path->initial() - q1).transpose() << std::endl;
      return path;
    }
  } // namespace quadcopter
} // namespace hpp
